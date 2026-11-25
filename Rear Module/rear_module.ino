// =============================================================================
// Rear Module firmware: LiDAR close-pass / approach detector, warning lights
// and camera trigger ("relay", v1.13)
// =============================================================================
//
// Board:      FireBeetle ESP32. In the Arduino IDE choose "ESP32 Dev Module"
//             (FQBN esp32:esp32:esp32), upload speed 115200.
// Core:       esp32 by Espressif, 3.x (built with 3.2.1). Needs the 3.x
//             ESP-NOW callback API, so 2.x cores will not compile it.
// Libraries:  Adafruit NeoPixel (seen working: 1.15.5). WiFi, ESP-NOW and
//             FreeRTOS come with the ESP32 core.
//
// What it does (details in SPECIFICATION.md in this folder):
//   - Reads STL-19P LiDAR data forwarded by a computer over USB (460800 baud),
//     or straight from the sensor on UART2 if LIDAR_RX_PIN is defined.
//   - Runs two detectors on every scan: close pass (within 1.5 m on the right)
//     and approaching vehicle (behind, 1.5-12 m).
//   - Approaching vehicle: pulses a 30-LED WS2813 strip red (GPIO2) for 2 s.
//   - Close pass: asks the Camera Module for 10 photos over ESP-NOW and sends
//     them to the computer over USB.
//
// Settings at a glance (search for the name to change them):
//   ledDataPin = 2 (FireBeetle label D9)  strip: 30 px, GRB, 800 kHz
//   ledBrightness = 20 / 255              pulseMs = 350
//   burstShots = 10, shotIntervalMs = 500
//   channel = 6 (ESP-NOW)                 cameraMac = 3C:8A:1F:D7:B1:24
//   USB serial 460800 baud                LIDAR_RX_PIN: not defined (USB input)
//
// KNOWN ISSUE (kept on purpose, see SPECIFICATION.md section 6): a close pass
// on its own takes photos but does NOT flash the lights. Only the approach
// detector triggers the lights.
//
// Provenance: this file is the five files of the v1.13 relay sketch
// (Detection.h, ScanInput.h, Stl19p.h, Radio.h, Relay.ino) merged in that
// order so the module is one file to copy and upload. No logic was changed;
// only the #include lines between those files were removed. The originals are
// in Archive/Firmware_Backup_v2_2026-11-25/LiDAR_v1.13_Complete_Setup/.
// The version string must stay "1.13": the LiDAR viewer in Tools/ checks for
// "READY 1.13 RADIO".
// =============================================================================

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <vector>
#include <utility>
#include <cstddef>
#include <cstring>
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_system.h>
#include <atomic>
#include <Adafruit_NeoPixel.h>

// ==========================================================================
// From Detection.h (v1.13, unchanged)
// ==========================================================================

namespace lidar {
struct Point {
  float x;
  float y;
  float angle;
  uint16_t index;
};

struct Result {
  bool close = false;
  bool newClose = false;
  bool incoming = false;
  bool renewWarning = false;
  float speed = 0;
  std::vector<uint16_t> closePoints;
  std::vector<uint16_t> approachPoints;
};

class Detector {
 public:
  void reset() {
    *this = Detector{};
  }

  Result processScan(const std::vector<Point> &points, double now) {
    if (lastScan >= 0 && now <= lastScan) reset();
    if (lastScan >= 0 && now - lastScan > 0.65) {
      closeRuns = 0;
      lastClose = -1;
      wasClose = false;
    }
    lastScan = now;
    Result result;
    detectClosePass(points, now, result);
    detectApproach(points, now, result);
    return result;
  }

 private:
  struct Shape {
    float left;
    float right;
    float y;
    unsigned count;
    float width() const { return right - left; }
    float center() const { return (left + right) / 2; }
    float weight() const {
      const float dx = left <= 0 && right >= 0 ? 0 : std::min(std::abs(left), std::abs(right));
      return 1.35f - 0.65f * std::min(dx / 5, 1.0f);
    }
  };

  struct Candidate {
    Shape shape;
    std::vector<uint16_t> indices;
  };

  struct ScanShapes {
    double time;
    std::vector<Shape> shapes;
  };

  struct Evidence {
    double time;
    float speed;
    float weight;
  };

  double lastScan = -1;
  double lastClose = -1;
  double warningUntil = -1;
  unsigned closeRuns = 0;
  bool wasClose = false;
  float speed = 0;
  std::deque<ScanShapes> history;
  std::deque<Evidence> evidence;

  void detectClosePass(const std::vector<Point> &points, double now, Result &result) {
    std::vector<Point> nearby;
    for (const Point &p : points) {
      const float radius = std::hypot(p.x, p.y);
      if (p.angle >= 55 && p.angle < 180 && radius >= 0.15f && radius <= 1.5f)
        nearby.push_back(p);
    }
    std::sort(nearby.begin(), nearby.end(), [](const Point &a, const Point &b) {
      return a.y < b.y;
    });
    float bestSpan = 0;
    for (size_t start = 0; start < nearby.size();) {
      size_t end = start + 1;
      while (end < nearby.size() && nearby[end].y - nearby[end - 1].y <= 0.55f) ++end;
      const float span = nearby[end - 1].y - nearby[start].y;
      if (end - start >= 12 && span >= 0.75f && span > bestSpan) {
        bestSpan = span;
        result.closePoints.clear();
        for (size_t i = start; i < end; ++i) result.closePoints.push_back(nearby[i].index);
      }
      start = end;
    }
    closeRuns = bestSpan ? std::min(closeRuns + 1, 2u) : 0;
    if (closeRuns == 2) lastClose = now;
    result.close = lastClose >= 0 && now - lastClose <= 2;
    result.newClose = result.close && !wasClose;
    wasClose = result.close;
  }

  static bool inApproachZone(const Point &p) {
    const float radius = std::hypot(p.x, p.y);
    const float angle = std::atan2(p.y, p.x) * 57.2957795f;
    return radius > 1.5f && radius <= 12 && angle >= -125 && angle < 0 && p.x <= 5;
  }

  static void collectGroups(const std::vector<Point> &band, std::vector<Candidate> &candidates) {
    for (size_t start = 0; start < band.size();) {
      size_t end = start + 1;
      while (end < band.size() && band[end].x - band[end - 1].x <= 0.85f) ++end;
      const float width = band[end - 1].x - band[start].x;
      if (end - start >= 3 && width >= 0.25f) {
        std::vector<float> ys;
        for (size_t i = start; i < end; ++i) ys.push_back(band[i].y);
        std::sort(ys.begin(), ys.end());
        if (width >= 0.85f * (ys.back() - ys.front())) {
          Shape shape{band[start].x, band[end - 1].x, ys[ys.size() / 2],
                      static_cast<unsigned>(ys.size())};
          Candidate candidate{shape, {}};
          for (size_t i = start; i < end; ++i) candidate.indices.push_back(band[i].index);
          candidates.push_back(std::move(candidate));
        }
      }
      start = end;
    }
  }

  static std::vector<Candidate> approachShapes(const std::vector<Point> &points) {
    std::vector<Point> filtered;
    float low = 0;
    float high = -12;
    for (const Point &p : points) {
      if (!inApproachZone(p)) continue;
      filtered.push_back(p);
      low = std::min(low, p.y);
      high = std::max(high, p.y);
    }
    if (filtered.size() < 3) return {};
    std::stable_sort(filtered.begin(), filtered.end(), [](const Point &a, const Point &b) {
      return a.x < b.x;
    });
    std::vector<Candidate> candidates;
    for (int bandIndex = static_cast<int>(std::floor(low * 5)); bandIndex / 5.0 <= high; ++bandIndex) {
      const double bottom = bandIndex / 5.0;
      std::vector<Point> band;
      for (const Point &p : filtered) {
        if (p.y >= bottom && p.y < bottom + 0.75) band.push_back(p);
      }
      collectGroups(band, candidates);
    }
    std::stable_sort(candidates.begin(), candidates.end(), [](const Candidate &a, const Candidate &b) {
      return a.shape.width() > b.shape.width();
    });
    std::vector<Candidate> unique;
    for (auto &candidate : candidates) {
      const Shape &shape = candidate.shape;
      const bool duplicate = std::any_of(unique.begin(), unique.end(), [&](const Candidate &prior) {
        return std::abs(shape.y - prior.shape.y) <= 0.35f &&
               std::abs(shape.center() - prior.shape.center()) <= 0.65f;
      });
      if (!duplicate) unique.push_back(std::move(candidate));
      if (unique.size() == 30) break;
    }
    return unique;
  }

  void detectApproach(const std::vector<Point> &points, double now, Result &result) {
    const auto candidates = approachShapes(points);
    while (!history.empty() && now - history.front().time > 0.85) history.pop_front();
    const Candidate *best = nullptr;
    float bestSpeed = 0;
    float bestScore = 0;
    for (const Candidate &candidate : candidates) {
      const Shape &shape = candidate.shape;
      if (shape.left > 1.5f || shape.right < -1.5f || shape.y >= 0) continue;
      for (const ScanShapes &scan : history) {
        const double elapsed = now - scan.time;
        if (elapsed < 0.04) continue;
        for (const Shape &prior : scan.shapes) {
          if (shape.left > prior.right + 0.7f || shape.right < prior.left - 0.7f) continue;
          const float velocity = (shape.y - prior.y) / elapsed;
          if (velocity < 0.6f || velocity > 35) continue;
          const float eta = (-1.5f - shape.y) / velocity;
          if (eta < -0.2f || eta > 8) continue;
          const float score = shape.weight() * shape.width();
          if (!best || score > bestScore || (score == bestScore && shape.count > best->shape.count)) {
            best = &candidate;
            bestScore = score;
            bestSpeed = velocity;
          }
        }
      }
    }
    ScanShapes current{now, {}};
    for (const Candidate &candidate : candidates) current.shapes.push_back(candidate.shape);
    history.push_back(std::move(current));
    if (history.size() > 48) history.pop_front();

    if (best) evidence.push_back({now, bestSpeed, best->shape.weight()});
    while (!evidence.empty() && now - evidence.front().time > 0.65) evidence.pop_front();
    if (evidence.size() > 48) evidence.pop_front();
    float weight = 0;
    for (const Evidence &item : evidence) weight += item.weight;
    if (evidence.size() >= 2 && evidence.back().time - evidence.front().time >= 0.08 && weight >= 2) {
      std::vector<float> speeds;
      for (const Evidence &item : evidence) speeds.push_back(item.speed);
      std::sort(speeds.begin(), speeds.end());
      const float median = speeds[speeds.size() / 2];
      speed = speed ? 0.65f * speed + 0.35f * median : median;
      warningUntil = now + 2;
      result.renewWarning = true;
      if (best) result.approachPoints = best->indices;
    }
    result.incoming = now < warningUntil;
    if (!result.incoming) speed = 0;
    result.speed = speed;
  }
};
}

// ==========================================================================
// From ScanInput.h (v1.13, unchanged)
// ==========================================================================

namespace lidar {
class ScanInput {
 public:
  // Frame marker 0x7e, escaping 0x7e/0x7d with 0x7d and XOR 0x20.
  // Payload: scan ID, elapsed milliseconds, count, then x/y mm and angle centidegrees.
  static constexpr uint8_t marker = 0x7e;
  static constexpr size_t maxPoints = 1024;

  const uint8_t *rawData() const { return bytes_ + header; }
  size_t rawSize() const { return rawSize_; }

  bool collecting() const { return collecting_; }

  bool push(uint8_t byte, uint32_t &id, uint32_t &stamp, std::vector<Point> &points) {
    rawSize_ = 0;
    if (byte == marker) {
      if (!collecting_) {
        collecting_ = true;
        escaped_ = false;
        size_ = 0;
        return false;
      }
      collecting_ = false;
      if (escaped_ || size_ < header || size_ > sizeof(bytes_)) return false;
      const size_t count = word(8);
      if (count == 65535) {
        rawSize_ = size_ - header;
        return false;
      }
      if (count > maxPoints || size_ != header + count * 6) return false;
      id = dword(0);
      stamp = dword(4);
      points.clear();
      for (size_t i = 0; i < count; ++i) {
        const size_t offset = header + i * 6;
        points.push_back({
          static_cast<int16_t>(word(offset)) / 1000.0f,
          static_cast<int16_t>(word(offset + 2)) / 1000.0f,
          word(offset + 4) / 100.0f,
          static_cast<uint16_t>(i)
        });
      }
      return true;
    }
    if (!collecting_) return false;
    if (!escaped_ && byte == 0x7d) {
      escaped_ = true;
      return false;
    }
    if (size_ < sizeof(bytes_)) bytes_[size_] = escaped_ ? byte ^ 0x20 : byte;
    if (size_ <= sizeof(bytes_)) ++size_;
    escaped_ = false;
    return false;
  }

  void discardPartial() { collecting_ = false; }

 private:
  static constexpr size_t header = 10;
  uint8_t bytes_[header + maxPoints * 6];
  size_t size_ = 0;
  size_t rawSize_ = 0;
  bool collecting_ = false;
  bool escaped_ = false;

  uint16_t word(size_t offset) const {
    return uint16_t(bytes_[offset]) | (uint16_t(bytes_[offset + 1]) << 8);
  }
  uint32_t dword(size_t offset) const {
    return uint32_t(word(offset)) | (uint32_t(word(offset + 2)) << 16);
  }
};
}

// ==========================================================================
// From Stl19p.h (v1.13, unchanged)
// ==========================================================================

namespace lidar {
class Stl19p {
 public:
  // Sensor stream: 230400 baud, 8N1, 47-byte packets, twelve beams per packet.
  bool push(uint8_t byte, uint32_t receivedAt) {
    if (used == 0 && byte != 0x54) return false;
    if (used == 1 && byte != 0x2c) {
      used = byte == 0x54 ? 1 : 0;
      return false;
    }
    packet[used++] = byte;
    if (used < sizeof(packet)) return false;
    if (crc() != packet[46]) {
      discardSweep();
      resync();
      return false;
    }
    used = 0;
    return assemble(receivedAt);
  }

  std::vector<Point> scan;
  uint32_t stamp = 0;

 private:
  uint8_t packet[47];
  size_t used = 0;
  std::vector<Point> points;
  bool collecting = false;
  bool overflow = false;
  bool havePacket = false;
  float lastAngle = -1;
  float previousSpan = 0;
  uint16_t previousEnd = 0;
  uint16_t previousTime = 0;
  uint32_t previousArrival = 0;
  uint32_t clock = 0;
  uint32_t lastBeam = 0;

  uint16_t word(size_t offset) const {
    return uint16_t(packet[offset]) | (uint16_t(packet[offset + 1]) << 8);
  }

  uint8_t crc() const {
    uint8_t value = 0;
    for (size_t i = 0; i < 46; ++i) {
      value ^= packet[i];
      for (int bit = 0; bit < 8; ++bit)
        value = (value << 1) ^ ((value & 0x80) ? 0x4d : 0);
    }
    return value;
  }

  void discardSweep() {
    collecting = false;
    overflow = false;
    lastAngle = -1;
    points.clear();
  }

  void resync() {
    for (size_t i = 1; i + 1 < sizeof(packet); ++i) {
      if (packet[i] == 0x54 && packet[i + 1] == 0x2c) {
        used = sizeof(packet) - i;
        std::memmove(packet, packet + i, used);
        return;
      }
    }
    used = packet[46] == 0x54 ? 1 : 0;
    if (used) packet[0] = 0x54;
  }

  bool assemble(uint32_t receivedAt) {
    const uint16_t speed = word(2);
    const uint16_t start = word(4);
    const uint16_t end = word(42);
    const uint16_t deviceTime = word(44);
    const float span = ((int(end) - start + 36000) % 36000) / 100.0f;
    if (speed < 360 || speed > 10800 || start >= 36000 || end >= 36000 ||
        deviceTime > 30000 || span <= 0 || span > 60) {
      discardSweep();
      return false;
    }
    const uint16_t time = deviceTime % 30000;
    const float duration = 1000 * span / speed;
    if (havePacket) {
      const unsigned delta = (int(time) - previousTime + 30000) % 30000;
      if (delta == 0 && end == previousEnd) return false;
      const float angularGap = ((int(start) - previousEnd + 36000) % 36000) / 100.0f;
      const uint32_t hostGap = receivedAt - previousArrival;
      if (delta == 0 || delta > 15000 || hostGap > 750 || angularGap <= 0 ||
          angularGap > std::max(previousSpan, span) / 11 * 1.85f + 0.04f ||
          delta > std::max(20.0f, duration * 4)) discardSweep();
      clock += delta > 0 && delta <= 15000 && hostGap <= 750 ? delta :
               std::max(hostGap, uint32_t(1));
    } else {
      clock = static_cast<uint32_t>(std::ceil(duration));
      havePacket = true;
    }
    previousTime = time;
    previousEnd = end;
    previousSpan = span;
    previousArrival = receivedAt;

    bool complete = false;
    for (unsigned i = 0; i < 12; ++i) {
      const float angle = std::fmod(start / 100.0f + span * i / 11, 360.0f);
      const uint32_t beamTime = static_cast<uint32_t>(std::max(0.0, clock - duration * (11 - i) / 11.0));
      if (lastAngle >= 0 && angle < lastAngle - 180) {
        if (collecting && !overflow) {
          scan.swap(points);
          stamp = lastBeam;
          complete = true;
        }
        points.clear();
        collecting = true;
        overflow = false;
      }
      lastAngle = angle;
      lastBeam = beamTime;
      const unsigned distance = word(6 + i * 3);
      if (!collecting || angle < 55 || angle > 215 || distance < 30 || distance > 12000)
        continue;
      if (points.size() >= 1024) {
        overflow = true;
        continue;
      }
      const float theta = angle * 0.01745329252f;
      points.push_back({distance / 1000.0f * std::sin(theta),
                        distance / 1000.0f * std::cos(theta), angle,
                        static_cast<uint16_t>(points.size())});
    }
    return complete;
  }
};
}

// ==========================================================================
// From Radio.h (v1.13, unchanged)
// ==========================================================================

constexpr uint8_t channel = 6;
constexpr size_t chunkSize = 224;
constexpr uint32_t maxJpeg = 128 * 1024;
constexpr uint32_t invalidSize = UINT32_MAX;
const uint8_t cameraMac[] = {0x3c, 0x8a, 0x1f, 0xd7, 0xb1, 0x24};
const uint8_t relayMac[] = {0xd4, 0x8c, 0x49, 0xfa, 0x75, 0x48};

struct Request {
  uint32_t token;
  uint32_t offset;
};

struct Reply {
  Request request;
  uint32_t total;
  uint8_t bytes[chunkSize];
};

static_assert(sizeof(Reply) <= 250, "ESP-NOW packet too large");

inline bool startRadio(const uint8_t *address, esp_now_recv_cb_t callback) {
  if (!WiFi.mode(WIFI_STA)) return false;
  WiFi.setSleep(false);
  if (esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE) != ESP_OK ||
      esp_now_init() != ESP_OK) return false;
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, address, 6);
  peer.channel = channel;
  peer.ifidx = WIFI_IF_STA;
  return esp_now_add_peer(&peer) == ESP_OK &&
         esp_now_register_recv_cb(callback) == ESP_OK;
}

// ==========================================================================
// From Relay.ino (v1.13, unchanged)
// ==========================================================================

QueueHandle_t replies;
bool radioReady;
TaskHandle_t photoTask;
QueueHandle_t photos;
SemaphoreHandle_t serialOutput;
uint32_t warningAt;
bool warningRequested;
bool lightTestPending;
bool lightOn;
bool pulseReportPending;
bool reportThisPulse;
bool versionPending;
uint32_t phaseAt;
lidar::ScanInput scanInput;
lidar::Detector detector;
lidar::Stl19p liveLidar;
bool liveScan;
#ifdef LIDAR_RX_PIN
HardwareSerial lidarUart(2);
#endif
std::vector<lidar::Point> scanPoints;
lidar::Result scanResult;
uint32_t scanId;
uint32_t lastInputAt;
bool scanPending;
bool approachWasActive;
unsigned closeEvents;
unsigned approachEvents;
bool cameraTaskError;
std::atomic<bool> burstActive{false};
std::atomic<TickType_t> burstStart{0};
constexpr uint32_t captureBurstBit = 1;
constexpr uint32_t testCameraBit = 2;
unsigned skippedBursts;
uint32_t burstSequence;
constexpr unsigned burstShots = 10;
constexpr unsigned shotIntervalMs = 500;
constexpr uint32_t pulseMs = 350;
constexpr uint8_t ledDataPin = 2; // FireBeetle D9
constexpr uint8_t ledBrightness = 20;

struct Photo {
  uint8_t *bytes;
  uint32_t size;
  const char *error;
  uint32_t burst;
  unsigned shot;

  Photo(uint8_t *data = nullptr, uint32_t length = 0, const char *message = nullptr)
      : bytes(data), size(length), error(message), burst(0), shot(0) {}
};

Photo capture();

Adafruit_NeoPixel strip(30, ledDataPin, NEO_GRB + NEO_KHZ800);

void extendWarning(bool manual) {
  warningAt = millis();
  warningRequested = true;
  lightTestPending |= manual;
}

void updateLights(uint32_t now) {
  if (lightOn) {
    if (now - phaseAt < pulseMs) return;
    strip.clear();
    strip.show();
    lightOn = false;
    phaseAt = now;
    pulseReportPending |= reportThisPulse;
    return;
  }

  if (warningRequested && now - warningAt >= 2000) warningRequested = false;
  if (!warningRequested || now - phaseAt < pulseMs) return;

  reportThisPulse = lightTestPending;
  lightTestPending = false;
  strip.fill(strip.Color(255, 0, 0));
  strip.show();
  lightOn = true;
  phaseAt = now;
}

void reportStatus() {
  if ((!versionPending && !pulseReportPending && !scanPending && !closeEvents &&
       !approachEvents && !cameraTaskError && !skippedBursts) || !serialOutput ||
      xSemaphoreTake(serialOutput, 0) != pdTRUE) return;
  if (versionPending) {
    Serial.println(radioReady ? "READY 1.13 RADIO OK" : "READY 1.13 RADIO ERROR");
    versionPending = false;
  }
  if (pulseReportPending) {
    Serial.println("OK LED PULSE GPIO2");
    pulseReportPending = false;
  }
  if (closeEvents || approachEvents) {
    Serial.printf("EVENT %u %u\n", closeEvents, approachEvents);
    closeEvents = approachEvents = 0;
  }
  if (skippedBursts) {
    Serial.printf("INFO %u close pass(es) during an active photo burst\n", skippedBursts);
    skippedBursts = 0;
  }
  if (cameraTaskError) {
    Serial.println("ERR Photo task failed to start");
    cameraTaskError = false;
  }
  if (scanPending && liveScan) {
    static uint8_t payload[lidar::ScanInput::maxPoints * 5];
    for (size_t i = 0; i < scanPoints.size(); ++i) {
      const auto &p = scanPoints[i];
      const uint16_t x = static_cast<int16_t>(std::lround(p.x * 1000));
      const uint16_t y = static_cast<int16_t>(std::lround(p.y * 1000));
      payload[5 * i] = x;
      payload[5 * i + 1] = x >> 8;
      payload[5 * i + 2] = y;
      payload[5 * i + 3] = y >> 8;
      payload[5 * i + 4] = 0;
    }
    for (uint16_t i : scanResult.closePoints) payload[5 * i + 4] = 1;
    for (uint16_t i : scanResult.approachPoints) payload[5 * i + 4] = 2;
    Serial.printf("LIVE %lu %d %d %.1f %u\n", static_cast<unsigned long>(liveLidar.stamp),
                  scanResult.close, scanResult.incoming, scanResult.speed * 3.6f,
                  unsigned(scanPoints.size()));
    Serial.write(payload, scanPoints.size() * 5);
    scanPending = false;
  }
  if (scanPending) {
    Serial.printf("SCAN %lu %d %d %.1f", static_cast<unsigned long>(scanId),
                  scanResult.close, scanResult.incoming, scanResult.speed * 3.6f);
    for (uint16_t i : scanResult.closePoints) Serial.printf(" +%u", unsigned(i));
    for (uint16_t i : scanResult.approachPoints) Serial.printf(" -%u", unsigned(i));
    Serial.println();
    scanPending = false;
  }
  xSemaphoreGive(serialOutput);
}

void receiveReply(const esp_now_recv_info_t *info, const uint8_t *data, int size) {
  if (size != sizeof(Reply) || memcmp(info->src_addr, cameraMac, 6)) return;
  Reply reply;
  memcpy(&reply, data, sizeof(reply));
  xQueueSend(replies, &reply, 0);
}

bool requestChunk(Request request, Reply &reply) {
  for (int attempt = 0; attempt < 4; ++attempt) {
    esp_now_send(cameraMac, reinterpret_cast<const uint8_t *>(&request), sizeof(request));
    const uint32_t wait = request.offset == 0 ? 600 : 50;
    const uint32_t start = millis();
    while (millis() - start < wait) {
      if (xQueueReceive(replies, &reply, pdMS_TO_TICKS(5)) == pdTRUE &&
          reply.request.token == request.token &&
          reply.request.offset == request.offset) return true;
    }
  }
  return false;
}

Photo capture() {
  if (!radioReady) return {nullptr, 0, "Relay radio failed to start"};
  xQueueReset(replies);
  Request request = {esp_random() | 1U, 0};
  Reply reply;
  if (!requestChunk(request, reply))
    return {nullptr, 0, "Camera did not answer ESP-NOW"};
  const uint32_t size = reply.total;
  if (!size || size > maxJpeg)
    return {nullptr, 0, "Camera could not capture the JPEG"};

  uint8_t *jpeg = static_cast<uint8_t *>(malloc(size));
  if (!jpeg) return {nullptr, 0, "Relay ran out of memory"};
  const uint32_t transferAt = millis();
  while (request.offset < size) {
    if (millis() - transferAt >= 8000 ||
        (request.offset && (!requestChunk(request, reply) || reply.total != size))) {
      free(jpeg);
      return {nullptr, 0, "Lost a JPEG packet between camera and relay"};
    }
    const size_t length = min(chunkSize, size_t(size - request.offset));
    memcpy(jpeg + request.offset, reply.bytes, length);
    request.offset += length;
  }
  return {jpeg, size, nullptr};
}

void captureBurst(TickType_t lastRequest) {
  const uint32_t burst = ++burstSequence;
  for (unsigned shot = 1; shot <= burstShots; ++shot) {
    vTaskDelayUntil(&lastRequest, pdMS_TO_TICKS(shotIntervalMs));
    lastRequest = xTaskGetTickCount();
    Photo photo = capture();
    photo.burst = burst;
    photo.shot = shot;
    xQueueSend(photos, &photo, portMAX_DELAY);
  }
  burstActive.store(false);
}

void capturePhotos(void *) {
  for (;;) {
    uint32_t commands;
    if (xTaskNotifyWait(0, UINT32_MAX, &commands, portMAX_DELAY) != pdTRUE) continue;
    if (commands & captureBurstBit) captureBurst(burstStart.load());
    if (commands & testCameraBit) {
      Photo photo = capture();
      photo.shot = 1;
      xQueueSend(photos, &photo, portMAX_DELAY);
    }
  }
}

void sendPhotos(void *) {
  for (;;) {
    Photo photo;
    xQueueReceive(photos, &photo, portMAX_DELAY);
    xSemaphoreTake(serialOutput, portMAX_DELAY);
    if (photo.error && !photo.burst) Serial.printf("ERR Camera test: %s\n", photo.error);
    else if (photo.error)
      Serial.printf("ERR Burst %lu photo %u/%u: %s\n",
                    static_cast<unsigned long>(photo.burst), photo.shot, burstShots, photo.error);
    else {
      Serial.printf("JPEG %u %lu %u\n", static_cast<unsigned>(photo.size),
                    static_cast<unsigned long>(photo.burst), photo.shot);
      Serial.write(photo.bytes, photo.size);
      free(photo.bytes);
    }
    xSemaphoreGive(serialOutput);
  }
}

void setup() {
  Serial.setRxBufferSize(8192);
  Serial.begin(460800);
#ifdef LIDAR_RX_PIN
  lidarUart.setRxBufferSize(8192);
  lidarUart.begin(230400, SERIAL_8N1, LIDAR_RX_PIN, -1);
#endif
  replies = xQueueCreate(8, sizeof(Reply));
  radioReady = replies && startRadio(cameraMac, receiveReply);
  photos = xQueueCreate(2, sizeof(Photo));
  serialOutput = xSemaphoreCreateMutex();
  strip.begin();
  strip.setBrightness(ledBrightness);
  strip.clear();
  strip.show();
  if (photos && serialOutput &&
      xTaskCreate(sendPhotos, "sendPhotos", 4096, nullptr, 1, nullptr) == pdPASS)
    xTaskCreate(capturePhotos, "capturePhotos", 4096, nullptr, 1, &photoTask);
}

void requestPhotoBurst() {
  if (!photoTask) {
    cameraTaskError = true;
    return;
  }
  if (burstActive.exchange(true)) {
    ++skippedBursts;
    return;
  }
  burstStart.store(xTaskGetTickCount());
  xTaskNotify(photoTask, captureBurstBit, eSetBits);
}

void processScan(uint32_t stamp) {
  scanResult = detector.processScan(scanPoints, stamp / 1000.0);
  scanPending = true;
  if (scanResult.newClose) {
    ++closeEvents;
    requestPhotoBurst();
  }
  if (scanResult.renewWarning) extendWarning(false);
  if (scanResult.incoming && !approachWasActive) ++approachEvents;
  approachWasActive = scanResult.incoming;
}

void processLidarByte(uint8_t byte, uint32_t now) {
  if (!liveLidar.push(byte, now)) return;
  if (!liveScan) detector.reset();
  liveScan = true;
  scanPoints.swap(liveLidar.scan);
  processScan(liveLidar.stamp);
}

void receiveInput(uint8_t byte) {
  const uint32_t now = millis();
  if (scanInput.collecting() && now - lastInputAt > 250) scanInput.discardPartial();
  lastInputAt = now;
  if (scanInput.collecting() || byte == lidar::ScanInput::marker) {
    uint32_t stamp;
    if (scanInput.push(byte, scanId, stamp, scanPoints)) {
      if (liveScan) detector.reset();
      liveScan = false;
      processScan(stamp);
    }
    for (size_t i = 0; i < scanInput.rawSize(); ++i)
      processLidarByte(scanInput.rawData()[i], now);
    return;
  }

  switch (byte) {
    case 'V': versionPending = true; break;
    case 'C': extendWarning(true); break;
    case 'P':
      if (photoTask) xTaskNotify(photoTask, testCameraBit, eSetBits);
      else cameraTaskError = true;
      break;
    case 'R':
      detector.reset();
      scanPending = false;
      approachWasActive = false;
      break;
  }
}

void loop() {
  for (unsigned i = 0; i < 1024 && Serial.available(); ++i)
    receiveInput(static_cast<uint8_t>(Serial.read()));
#ifdef LIDAR_RX_PIN
  for (unsigned i = 0; i < 1024 && lidarUart.available(); ++i)
    processLidarByte(static_cast<uint8_t>(lidarUart.read()), millis());
#endif
  updateLights(millis());
  reportStatus();
  delay(1);
}
