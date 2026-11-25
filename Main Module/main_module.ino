// Upload copy for the bike: LiDAR wired to GPIO16 (UART2 RX).
// To read the LiDAR from a computer over USB instead, delete the #define below.
#define LIDAR_RX_PIN 16

// =============================================================================
// Main Module firmware v1.25: LiDAR close-pass / approach detector, warning
// lights, camera trigger, and the Bluetooth link to the phone
//
// v1.25 = the reconstructed v1.21 relay + the phone link (Bluetooth LE) +
// forwarding the phone's handlebar screens to the Front Module. Not related
// to the v1.22-1.24 phone experiments on Oscar's Mac (not in this repo).
// =============================================================================
//
// Board:      FireBeetle ESP32. In the Arduino IDE choose "ESP32 Dev Module"
//             (FQBN esp32:esp32:esp32), upload speed 115200.
//             Tools -> Partition Scheme: "Huge APP (3MB No OTA/1MB SPIFFS)"
//             (FQBN esp32:esp32:esp32:PartitionScheme=huge_app). With
//             Bluetooth the sketch fills 92% of the default 1.3 MB.
// Core:       esp32 by Espressif, 3.x (compile-checked with 3.3.8; Oscar's
//             Mac had 3.2.0). Needs the 3.x ESP-NOW callback API.
// Libraries:  Adafruit NeoPixel (1.15.5) and NimBLE-Arduino (2.5.2), both in
//             the Arduino Library Manager. WiFi, ESP-NOW and FreeRTOS come
//             with the ESP32 core.
// LiDAR:      wired to GPIO16 on the bike: build with -DLIDAR_RX_PIN=16
//             (Tools/flash_module.command: LIDAR_RX_PIN=16 ...). Without it,
//             scans come from a computer over USB.
//
// What it does (details in SPECIFICATION.md in this folder):
//   - Reads STL-19P LiDAR data (UART2 on LIDAR_RX_PIN, or forwarded by a
//     computer over USB at 460800 baud) and runs two detectors on every scan:
//     close pass (within 1.5 m on the right) and approaching vehicle.
//   - Automatic actions (on from power-up since v1.25; 'A' on / 'M' off):
//     approach or close pass -> 30-LED WS2813 strip flashes red on GPIO2;
//     new close pass -> 5 photos from the Camera Module over ESP-NOW.
//   - Every photo goes to the computer (USB) and, as before, is broadcast
//     as JPEG packets to the Front Module (forwardDisplay).
//   - Phone link (Bluetooth LE, "BikeNav Main"): forwards the phone's
//     handlebar screens unchanged to the Front Module over ESP-NOW; on a close
//     pass sends the phone an alert at once, then a package with the photos
//     and a LiDAR clip (2 s before to 1.5 s after). Protocol: Phone App/
//     SPECIFICATION.md section 3. Code: the last two sections of this file.
//
// Settings at a glance (search for the name to change them):
//   ledDataPin = 2 (FireBeetle label D9)  strip: 30 px, GRB, brightness 80
//   LED on/off 500 ms; driven by directLedShow() (bit-banged, not RMT)
//   burstShots = 5, shotIntervalMs = 500  automaticActions = true at boot
//   channel = 6 (ESP-NOW)                 cameraMac = 3C:8A:1F:D7:B1:24
//   Front Module: JPEG broadcast to FF:FF:FF:FF:FF:FF (v1.21) and phone
//   screens to phone::frontMac = 8C:94:DF:6C:BC:80 (v1.25)
//   USB serial 460800 baud
//   Bluetooth name "BikeNav Main", service 6e3b2000-8f7a-4c1e-9b52-2d1c0b7a1f10
//
// Provenance: the first four sections are the relay's Detection.h,
// ScanInput.h, Stl19p.h and Radio.h (identical in v1.13 and v1.21), then
// TestImage.h and Relay.ino from Rear_v1.21_reconstructed in the team's
// "BikeNav_PreBluetooth_Source" bundle (25 Nov 2026). That v1.21 is a
// RECONSTRUCTION of Oscar's LED patch on the recovered v1.20 sketch; the
// original Mac source was not available. Every v1.25 change to that code is
// marked "v1.25:". The phone link is new code in the last two sections.
// =============================================================================

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <vector>
#include <utility>
#include <cstddef>
#include <cstring>
#include <cstdarg>
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_system.h>
#include <atomic>
#include <soc/gpio_reg.h>
#include <soc/soc.h>
#include <xtensa/core-macros.h>
#include <Adafruit_NeoPixel.h>
#include <NimBLEDevice.h>
#include <esp_idf_version.h>

// ==========================================================================
// From Detection.h (v1.13 = v1.21, unchanged)
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
// From ScanInput.h (v1.13 = v1.21, unchanged)
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
// From Stl19p.h (v1.13 = v1.21, unchanged)
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
// From Radio.h (v1.13 = v1.21; one v1.25 change, marked)
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
  // v1.25: was WiFi.setSleep(false). With Bluetooth running, ESP-IDF requires
  // Wi-Fi modem sleep and aborts otherwise. The module never joins an access
  // point, so modem sleep doesn't make it miss ESP-NOW packets.
  WiFi.setSleep(true);
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
// From TestImage.h (v1.20/v1.21, unchanged): a known JPEG for the 'T' display test
// ==========================================================================

const uint8_t debugJpeg[] = {
255,216,255,224,0,16,74,70,73,70,0,1,1,0,0,1,0,1,0,0,255,219,0,67,
0,8,6,6,7,6,5,8,7,7,7,9,9,8,10,12,20,13,12,11,11,12,25,18,
19,15,20,29,26,31,30,29,26,28,28,32,36,46,39,32,34,44,35,28,28,40,55,41,
44,48,49,52,52,52,31,39,57,61,56,50,60,46,51,52,50,255,219,0,67,1,9,9,
9,12,11,12,24,13,13,24,50,33,28,33,50,50,50,50,50,50,50,50,50,50,50,50,
50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,
50,50,50,50,50,50,50,50,50,50,50,50,50,50,255,192,0,17,8,0,240,1,64,3,
1,34,0,2,17,1,3,17,1,255,196,0,31,0,0,1,5,1,1,1,1,1,1,0,
0,0,0,0,0,0,0,1,2,3,4,5,6,7,8,9,10,11,255,196,0,181,16,0,
2,1,3,3,2,4,3,5,5,4,4,0,0,1,125,1,2,3,0,4,17,5,18,33,
49,65,6,19,81,97,7,34,113,20,50,129,145,161,8,35,66,177,193,21,82,209,240,36,
51,98,114,130,9,10,22,23,24,25,26,37,38,39,40,41,42,52,53,54,55,56,57,58,
67,68,69,70,71,72,73,74,83,84,85,86,87,88,89,90,99,100,101,102,103,104,105,106,
115,116,117,118,119,120,121,122,131,132,133,134,135,136,137,138,146,147,148,149,150,151,152,153,
154,162,163,164,165,166,167,168,169,170,178,179,180,181,182,183,184,185,186,194,195,196,197,198,
199,200,201,202,210,211,212,213,214,215,216,217,218,225,226,227,228,229,230,231,232,233,234,241,
242,243,244,245,246,247,248,249,250,255,196,0,31,1,0,3,1,1,1,1,1,1,1,1,
1,0,0,0,0,0,0,1,2,3,4,5,6,7,8,9,10,11,255,196,0,181,17,0,
2,1,2,4,4,3,4,7,5,4,4,0,1,2,119,0,1,2,3,17,4,5,33,49,
6,18,65,81,7,97,113,19,34,50,129,8,20,66,145,161,177,193,9,35,51,82,240,21,
98,114,209,10,22,36,52,225,37,241,23,24,25,26,38,39,40,41,42,53,54,55,56,57,
58,67,68,69,70,71,72,73,74,83,84,85,86,87,88,89,90,99,100,101,102,103,104,105,
106,115,116,117,118,119,120,121,122,130,131,132,133,134,135,136,137,138,146,147,148,149,150,151,
152,153,154,162,163,164,165,166,167,168,169,170,178,179,180,181,182,183,184,185,186,194,195,196,
197,198,199,200,201,202,210,211,212,213,214,215,216,217,218,226,227,228,229,230,231,232,233,234,
242,243,244,245,246,247,248,249,250,255,218,0,12,3,1,0,2,17,3,17,0,63,0,249,
254,138,43,208,62,25,124,50,255,0,133,141,253,169,255,0,19,127,236,255,0,176,121,95,
242,237,230,239,223,191,253,181,198,54,123,245,160,15,63,162,189,255,0,254,25,151,254,166,
239,252,166,255,0,246,218,63,225,153,127,234,110,255,0,202,111,255,0,109,160,15,0,162,
189,255,0,254,25,151,254,166,239,252,166,255,0,246,218,63,225,153,127,234,110,255,0,202,
111,255,0,109,160,15,0,162,189,255,0,254,25,151,254,166,239,252,166,255,0,246,218,63,
225,153,127,234,110,255,0,202,111,255,0,109,160,15,0,162,189,255,0,254,25,151,254,166,
239,252,166,255,0,246,218,63,225,153,127,234,110,255,0,202,111,255,0,109,160,15,0,162,
189,255,0,254,25,151,254,166,239,252,166,255,0,246,218,63,225,153,127,234,110,255,0,202,
111,255,0,109,160,15,0,162,189,255,0,254,25,151,254,166,239,252,166,255,0,246,218,63,
225,153,127,234,110,255,0,202,111,255,0,109,160,15,0,162,189,255,0,254,25,151,254,166,
239,252,166,255,0,246,218,63,225,153,127,234,110,255,0,202,111,255,0,109,160,15,0,162,
189,255,0,254,25,151,254,166,239,252,166,255,0,246,218,63,225,153,127,234,110,255,0,202,
111,255,0,109,160,15,0,162,189,255,0,254,25,151,254,166,239,252,166,255,0,246,218,63,
225,153,127,234,110,255,0,202,111,255,0,109,160,15,0,162,189,255,0,254,25,151,254,166,
239,252,166,255,0,246,218,63,225,153,127,234,110,255,0,202,111,255,0,109,160,15,0,162,
189,255,0,254,25,151,254,166,239,252,166,255,0,246,218,63,225,153,127,234,110,255,0,202,
111,255,0,109,160,15,0,162,189,255,0,254,25,151,254,166,239,252,166,255,0,246,218,63,
225,153,127,234,110,255,0,202,111,255,0,109,160,15,0,162,189,255,0,254,25,151,254,166,
239,252,166,255,0,246,218,63,225,153,127,234,110,255,0,202,111,255,0,109,160,15,0,162,
189,255,0,254,25,151,254,166,239,252,166,255,0,246,218,63,225,153,127,234,110,255,0,202,
111,255,0,109,160,15,0,162,189,255,0,254,25,151,254,166,239,252,166,255,0,246,218,63,
225,153,127,234,110,255,0,202,111,255,0,109,160,15,0,162,189,255,0,254,25,151,254,166,
239,252,166,255,0,246,218,63,225,153,127,234,110,255,0,202,111,255,0,109,160,15,0,162,
189,255,0,254,25,151,254,166,239,252,166,255,0,246,218,63,225,153,127,234,110,255,0,202,
111,255,0,109,160,15,0,162,189,255,0,254,25,151,254,166,239,252,166,255,0,246,218,63,
225,153,127,234,110,255,0,202,111,255,0,109,160,15,0,162,189,3,226,111,195,47,248,87,
63,217,127,241,55,254,208,251,127,155,255,0,46,222,86,205,155,63,219,108,231,127,183,74,
243,250,0,43,223,255,0,102,95,249,154,127,237,211,255,0,107,87,128,87,191,254,204,191,
243,52,255,0,219,167,254,214,160,15,64,248,155,241,55,254,21,207,246,95,252,74,63,180,
62,223,230,255,0,203,207,149,179,102,207,246,27,57,223,237,210,188,255,0,254,26,107,254,
165,31,252,169,127,246,170,63,105,175,249,149,191,237,239,255,0,104,215,128,80,7,191,255,
0,195,77,127,212,163,255,0,149,47,254,213,71,252,52,215,253,74,63,249,82,255,0,237,
85,224,20,80,7,191,255,0,195,77,127,212,163,255,0,149,47,254,213,71,252,52,215,253,
74,63,249,82,255,0,237,85,224,20,80,7,191,255,0,195,77,127,212,163,255,0,149,47,
254,213,71,252,52,215,253,74,63,249,82,255,0,237,85,224,20,80,7,191,255,0,195,77,
127,212,163,255,0,149,47,254,213,71,252,52,215,253,74,63,249,82,255,0,237,85,224,20,
80,7,191,255,0,195,77,127,212,163,255,0,149,47,254,213,71,252,52,215,253,74,63,249,
82,255,0,237,85,224,20,80,7,191,255,0,195,77,127,212,163,255,0,149,47,254,213,71,
252,52,215,253,74,63,249,82,255,0,237,85,224,20,80,7,191,255,0,195,77,127,212,163,
255,0,149,47,254,213,71,252,52,215,253,74,63,249,82,255,0,237,85,224,20,80,7,191,
255,0,195,77,127,212,163,255,0,149,47,254,213,71,252,52,215,253,74,63,249,82,255,0,
237,85,224,20,80,7,191,255,0,195,77,127,212,163,255,0,149,47,254,213,71,252,52,215,
253,74,63,249,82,255,0,237,85,224,20,80,7,191,255,0,195,77,127,212,163,255,0,149,
47,254,213,71,252,52,215,253,74,63,249,82,255,0,237,85,224,20,80,7,191,255,0,195,
77,127,212,163,255,0,149,47,254,213,71,252,52,215,253,74,63,249,82,255,0,237,85,224,
20,80,7,191,255,0,195,77,127,212,163,255,0,149,47,254,213,71,252,52,215,253,74,63,
249,82,255,0,237,85,224,20,80,7,191,255,0,195,77,127,212,163,255,0,149,47,254,213,
71,252,52,215,253,74,63,249,82,255,0,237,85,224,20,80,7,191,255,0,195,77,127,212,
163,255,0,149,47,254,213,71,252,52,215,253,74,63,249,82,255,0,237,85,224,20,80,7,
191,255,0,195,77,127,212,163,255,0,149,47,254,213,71,252,52,215,253,74,63,249,82,255,
0,237,85,224,20,80,7,191,255,0,195,77,127,212,163,255,0,149,47,254,213,71,252,52,
215,253,74,63,249,82,255,0,237,85,224,20,80,7,191,255,0,195,77,127,212,163,255,0,
149,47,254,213,94,129,240,203,226,111,252,44,111,237,79,248,148,127,103,253,131,202,255,0,
151,159,55,126,253,255,0,236,46,49,179,223,173,124,129,94,255,0,251,50,255,0,204,211,
255,0,110,159,251,90,128,15,218,107,254,101,111,251,123,255,0,218,53,224,21,239,255,0,
180,215,252,202,223,246,247,255,0,180,107,192,40,0,175,127,253,153,127,230,105,255,0,183,
79,253,173,94,1,94,255,0,251,50,255,0,204,211,255,0,110,159,251,90,128,15,218,107,
254,101,111,251,123,255,0,218,53,224,21,239,255,0,180,215,252,202,223,246,247,255,0,180,
107,128,248,37,255,0,37,123,66,255,0,183,143,253,39,146,128,60,254,138,251,254,138,0,
248,2,138,251,254,138,0,248,2,138,251,254,138,0,248,2,138,251,254,138,0,248,2,138,
251,254,138,0,248,2,138,251,254,138,0,248,2,138,251,254,138,0,248,2,138,251,127,199,
127,242,79,60,75,255,0,96,171,175,253,20,213,241,5,0,20,81,69,0,20,81,69,0,
20,81,69,0,20,81,69,0,20,81,69,0,20,81,69,0,20,81,69,0,20,81,69,0,
21,239,255,0,179,47,252,205,63,246,233,255,0,181,171,192,43,223,255,0,102,95,249,154,
127,237,211,255,0,107,80,1,251,77,127,204,173,255,0,111,127,251,70,188,2,189,255,0,
246,154,255,0,153,91,254,222,255,0,246,141,120,5,0,21,239,255,0,179,47,252,205,63,
246,233,255,0,181,171,192,43,223,255,0,102,95,249,154,127,237,211,255,0,107,80,1,251,
77,127,204,173,255,0,111,127,251,70,184,15,130,95,242,87,180,47,251,120,255,0,210,121,
43,191,253,166,191,230,86,255,0,183,191,253,163,92,7,193,47,249,43,218,23,253,188,127,
233,60,148,1,245,253,20,81,64,5,20,81,64,5,20,81,64,5,20,81,64,5,20,81,
64,5,20,81,64,5,20,81,64,28,255,0,142,255,0,228,158,120,151,254,193,87,95,250,
41,171,226,10,251,127,199,127,242,79,60,75,255,0,96,171,175,253,20,213,241,5,0,20,
81,69,0,20,81,69,0,20,81,69,0,20,81,69,0,20,81,69,0,20,81,69,0,20,
81,69,0,20,81,69,0,21,239,255,0,179,47,252,205,63,246,233,255,0,181,171,192,43,
223,255,0,102,95,249,154,127,237,211,255,0,107,80,1,251,77,127,204,173,255,0,111,127,
251,70,188,2,189,255,0,246,154,255,0,153,91,254,222,255,0,246,141,120,5,0,21,239,
255,0,179,47,252,205,63,246,233,255,0,181,171,192,43,223,255,0,102,95,249,154,127,237,
211,255,0,107,80,1,251,77,127,204,173,255,0,111,127,251,70,184,15,130,95,242,87,180,
47,251,120,255,0,210,121,43,191,253,166,191,230,86,255,0,183,191,253,163,92,7,193,47,
249,43,218,23,253,188,127,233,60,148,1,245,253,20,81,64,5,20,81,64,5,20,81,64,
5,20,81,64,5,20,81,64,5,20,81,64,5,20,81,64,28,255,0,142,255,0,228,158,
120,151,254,193,87,95,250,41,171,226,10,251,127,199,127,242,79,60,75,255,0,96,171,175,
253,20,213,241,5,0,20,81,69,0,20,81,69,0,20,81,69,0,20,81,69,0,20,81,
69,0,20,81,69,0,20,81,69,0,20,81,69,0,21,239,255,0,179,47,252,205,63,246,
233,255,0,181,171,192,43,223,255,0,102,95,249,154,127,237,211,255,0,107,80,1,251,77,
127,204,173,255,0,111,127,251,70,188,2,189,255,0,246,154,255,0,153,91,254,222,255,0,
246,141,120,5,0,21,239,255,0,179,47,252,205,63,246,233,255,0,181,171,192,43,223,255,
0,102,95,249,154,127,237,211,255,0,107,80,1,251,77,127,204,173,255,0,111,127,251,70,
184,15,130,95,242,87,180,47,251,120,255,0,210,121,43,191,253,166,191,230,86,255,0,183,
191,253,163,92,7,193,47,249,43,218,23,253,188,127,233,60,148,1,245,253,20,81,64,5,
20,81,64,5,20,81,64,5,20,81,64,5,20,81,64,5,20,81,64,5,20,81,64,28,
255,0,142,255,0,228,158,120,151,254,193,87,95,250,41,171,226,10,251,127,199,127,242,79,
60,75,255,0,96,171,175,253,20,213,241,5,0,20,81,69,0,20,81,69,0,20,81,69,
0,20,81,69,0,20,81,69,0,20,81,69,0,20,81,69,0,20,81,69,0,21,239,255,
0,179,47,252,205,63,246,233,255,0,181,171,192,43,223,255,0,102,95,249,154,127,237,211,
255,0,107,80,1,251,77,127,204,173,255,0,111,127,251,70,188,2,189,255,0,246,154,255,
0,153,91,254,222,255,0,246,141,120,5,0,21,239,255,0,179,47,252,205,63,246,233,255,
0,181,171,192,43,223,255,0,102,95,249,154,127,237,211,255,0,107,80,1,251,77,127,204,
173,255,0,111,127,251,70,184,15,130,95,242,87,180,47,251,120,255,0,210,121,43,191,253,
166,191,230,86,255,0,183,191,253,163,94,113,240,130,254,207,76,248,165,163,94,95,221,193,
105,107,31,159,190,105,228,17,162,230,9,0,203,30,6,73,3,241,160,15,177,232,174,127,
254,19,191,7,255,0,208,215,161,255,0,224,198,31,254,42,143,248,78,252,31,255,0,67,
94,135,255,0,131,24,127,248,170,0,232,40,174,127,254,19,191,7,255,0,208,215,161,255,
0,224,198,31,254,42,143,248,78,252,31,255,0,67,94,135,255,0,131,24,127,248,170,0,
232,40,174,127,254,19,191,7,255,0,208,215,161,255,0,224,198,31,254,42,143,248,78,252,
31,255,0,67,94,135,255,0,131,24,127,248,170,0,232,40,174,127,254,19,191,7,255,0,
208,215,161,255,0,224,198,31,254,42,143,248,78,252,31,255,0,67,94,135,255,0,131,24,
127,248,170,0,232,40,174,127,254,19,191,7,255,0,208,215,161,255,0,224,198,31,254,42,
143,248,78,252,31,255,0,67,94,135,255,0,131,24,127,248,170,0,232,40,174,127,254,19,
191,7,255,0,208,215,161,255,0,224,198,31,254,42,143,248,78,252,31,255,0,67,94,135,
255,0,131,24,127,248,170,0,232,40,174,127,254,19,191,7,255,0,208,215,161,255,0,224,
198,31,254,42,143,248,78,252,31,255,0,67,94,135,255,0,131,24,127,248,170,0,60,119,
255,0,36,243,196,191,246,10,186,255,0,209,77,95,16,87,216,126,52,241,167,133,110,188,
11,226,27,123,127,18,232,211,79,46,153,114,145,199,29,252,76,206,198,38,0,0,27,36,
147,198,43,227,202,0,40,162,138,0,40,162,138,0,40,162,138,0,40,162,138,0,40,162,
138,0,40,162,138,0,40,162,138,0,40,162,138,0,43,223,255,0,102,95,249,154,127,237,
211,255,0,107,87,128,87,191,254,204,191,243,52,255,0,219,167,254,214,160,3,246,154,255,
0,153,91,254,222,255,0,246,141,120,5,123,255,0,237,53,255,0,50,183,253,189,255,0,
237,26,240,10,0,43,223,255,0,102,95,249,154,127,237,211,255,0,107,87,128,87,191,254,
204,191,243,52,255,0,219,167,254,214,160,3,246,154,255,0,153,91,254,222,255,0,246,141,
120,5,123,255,0,237,53,255,0,50,183,253,189,255,0,237,26,240,10,0,40,162,138,0,
40,162,138,0,40,162,138,0,40,162,138,0,40,162,138,0,40,162,138,0,40,162,138,0,
40,162,138,0,40,162,138,0,40,162,138,0,40,162,138,0,40,162,138,0,40,162,138,0,
40,162,138,0,40,162,138,0,40,162,138,0,43,223,255,0,102,95,249,154,127,237,211,255,
0,107,87,128,87,191,254,204,191,243,52,255,0,219,167,254,214,160,3,246,154,255,0,153,
91,254,222,255,0,246,141,120,5,123,255,0,237,53,255,0,50,183,253,189,255,0,237,26,
240,10,0,43,223,255,0,102,95,249,154,127,237,211,255,0,107,87,128,87,191,254,204,191,
243,52,255,0,219,167,254,214,160,3,246,154,255,0,153,91,254,222,255,0,246,141,120,5,
123,255,0,237,53,255,0,50,183,253,189,255,0,237,26,240,10,0,40,162,138,0,40,162,
138,0,40,162,138,0,40,162,138,0,40,162,138,0,40,162,138,0,40,162,138,0,40,162,
138,0,43,232,255,0,133,191,11,124,27,226,63,135,26,78,173,171,104,223,104,190,159,206,
243,37,251,84,201,187,108,206,163,133,112,7,0,14,5,124,225,95,95,252,18,255,0,146,
67,161,127,219,199,254,148,73,64,7,252,41,47,135,159,244,47,127,228,237,199,255,0,28,
163,254,20,151,195,207,250,23,191,242,118,227,255,0,142,87,160,81,64,30,127,255,0,10,
75,225,231,253,11,223,249,59,113,255,0,199,40,255,0,133,37,240,243,254,133,239,252,157,
184,255,0,227,149,232,20,80,7,159,255,0,194,146,248,121,255,0,66,247,254,78,220,127,
241,202,63,225,73,124,60,255,0,161,123,255,0,39,110,63,248,229,122,5,20,1,231,255,
0,240,164,190,30,127,208,189,255,0,147,183,31,252,114,143,248,82,95,15,63,232,94,255,
0,201,219,143,254,57,94,129,69,0,121,255,0,252,41,47,135,159,244,47,127,228,237,199,
255,0,28,163,254,20,151,195,207,250,23,191,242,118,227,255,0,142,87,160,81,64,30,127,
255,0,10,75,225,231,253,11,223,249,59,113,255,0,199,40,255,0,133,37,240,243,254,133,
239,252,157,184,255,0,227,149,232,20,80,7,159,255,0,194,146,248,121,255,0,66,247,254,
78,220,127,241,202,63,225,73,124,60,255,0,161,123,255,0,39,110,63,248,229,122,5,20,
1,242,71,198,191,11,104,222,17,241,149,157,134,135,103,246,75,89,52,244,153,147,205,121,
50,230,73,1,57,114,79,69,31,149,118,255,0,179,47,252,205,63,246,233,255,0,181,171,
3,246,142,255,0,146,135,167,255,0,216,42,63,253,27,45,111,254,204,191,243,52,255,0,
219,167,254,214,160,3,246,154,255,0,153,91,254,222,255,0,246,141,120,5,123,255,0,237,
53,255,0,50,183,253,189,255,0,237,26,240,10,0,43,223,255,0,102,95,249,154,127,237,
211,255,0,107,87,128,87,191,254,204,191,243,52,255,0,219,167,254,214,160,3,246,154,255,
0,153,91,254,222,255,0,246,141,120,5,123,255,0,237,53,255,0,50,183,253,189,255,0,
237,26,240,10,0,40,174,219,193,247,139,101,225,141,81,223,87,191,210,145,245,75,4,107,
139,21,220,248,41,115,193,249,215,229,239,223,160,224,214,223,140,162,213,14,135,11,69,253,
179,105,110,177,93,249,214,118,86,236,214,72,126,221,115,185,89,195,128,48,56,229,15,1,
125,120,0,242,234,43,209,53,93,18,238,215,226,166,175,170,106,118,23,144,105,144,106,87,
87,134,103,182,59,36,84,118,112,23,56,13,184,133,3,158,119,117,169,82,123,43,221,126,
103,185,73,111,180,189,86,192,234,199,205,81,27,189,197,180,114,51,179,0,91,5,218,41,
148,242,120,151,62,148,1,230,212,87,97,225,61,127,87,127,17,234,87,3,81,186,141,238,
172,181,11,137,188,169,89,3,72,45,102,96,216,7,25,7,145,233,142,43,79,85,125,98,
251,192,154,45,207,219,252,64,241,190,159,41,156,71,11,201,108,231,237,115,228,203,38,254,
14,58,229,79,0,126,0,30,121,69,122,47,139,47,117,107,91,31,237,4,159,80,210,166,
91,205,145,219,52,185,64,25,27,38,210,85,193,242,120,0,168,227,6,62,77,115,255,0,
16,47,174,239,60,121,174,173,213,212,243,172,58,141,204,113,9,100,44,35,81,43,97,87,
61,7,176,160,14,106,138,40,160,2,138,40,160,2,138,40,160,2,138,40,160,2,190,191,
248,37,255,0,36,135,66,255,0,183,143,253,40,146,190,64,175,175,254,9,127,201,33,208,
191,237,227,255,0,74,36,160,15,64,162,138,40,0,162,138,40,0,162,138,40,0,162,138,
40,0,162,138,40,0,162,138,40,0,162,138,40,3,230,15,218,59,254,74,30,159,255,0,
96,168,255,0,244,108,181,191,251,50,255,0,204,211,255,0,110,159,251,90,176,63,104,239,
249,40,122,127,253,130,163,255,0,209,178,214,255,0,236,203,255,0,51,79,253,186,127,237,
106,0,63,105,175,249,149,191,237,239,255,0,104,215,128,87,191,254,211,95,243,43,127,219,
223,254,209,175,0,160,2,189,255,0,246,101,255,0,153,167,254,221,63,246,181,120,5,123,
255,0,236,203,255,0,51,79,253,186,127,237,106,0,63,105,175,249,149,191,237,239,255,0,
104,215,128,87,191,254,211,95,243,43,127,219,223,254,209,175,0,160,2,138,40,160,2,138,
40,160,2,138,40,160,2,138,40,160,2,138,40,160,2,138,40,160,2,138,40,160,2,138,
40,160,2,190,191,248,37,255,0,36,135,66,255,0,183,143,253,40,146,190,64,175,175,254,
9,127,201,33,208,191,237,227,255,0,74,36,160,15,64,162,138,40,0,162,138,40,0,162,
138,40,0,162,138,40,0,162,138,40,0,162,138,40,0,162,138,40,3,230,15,218,59,254,
74,30,159,255,0,96,168,255,0,244,108,181,191,251,50,255,0,204,211,255,0,110,159,251,
90,176,63,104,239,249,40,122,127,253,130,163,255,0,209,178,214,255,0,236,203,255,0,51,
79,253,186,127,237,106,0,63,105,175,249,149,191,237,239,255,0,104,215,128,87,191,254,211,
95,243,43,127,219,223,254,209,175,0,160,2,189,255,0,246,101,255,0,153,167,254,221,63,
246,181,120,5,123,255,0,236,203,255,0,51,79,253,186,127,237,106,0,63,105,175,249,149,
191,237,239,255,0,104,215,128,87,191,254,211,95,243,43,127,219,223,254,209,175,0,160,2,
138,40,160,2,138,40,160,2,138,40,160,2,138,40,160,2,138,40,160,2,138,40,160,2,
138,40,160,2,138,40,160,2,190,191,248,37,255,0,36,135,66,255,0,183,143,253,40,146,
190,64,175,175,254,9,127,201,33,208,191,237,227,255,0,74,36,160,15,64,162,138,40,0,
162,138,40,0,162,138,40,0,162,138,40,0,162,138,40,0,162,138,40,0,162,138,40,3,
230,15,218,59,254,74,30,159,255,0,96,168,255,0,244,108,181,191,251,50,255,0,204,211,
255,0,110,159,251,90,176,63,104,239,249,40,122,127,253,130,163,255,0,209,178,214,255,0,
236,203,255,0,51,79,253,186,127,237,106,0,63,105,175,249,149,191,237,239,255,0,104,215,
128,87,191,254,211,95,243,43,127,219,223,254,209,175,0,160,2,189,255,0,246,101,255,0,
153,167,254,221,63,246,181,120,5,123,255,0,236,203,255,0,51,79,253,186,127,237,106,0,
63,105,175,249,149,191,237,239,255,0,104,215,128,87,191,254,211,95,243,43,127,219,223,254,
209,175,0,160,2,138,40,160,2,138,40,160,2,138,40,160,2,138,40,160,2,138,40,160,
2,138,40,160,2,138,40,160,2,138,40,160,2,190,191,248,37,255,0,36,135,66,255,0,
183,143,253,40,146,190,64,173,139,31,22,120,147,76,179,142,206,195,196,26,173,165,172,121,
217,12,23,178,70,139,146,73,194,131,129,146,73,252,104,3,238,122,43,226,15,248,78,252,
97,255,0,67,94,185,255,0,131,25,191,248,170,63,225,59,241,135,253,13,122,231,254,12,
102,255,0,226,168,3,237,250,43,226,15,248,78,252,97,255,0,67,94,185,255,0,131,25,
191,248,170,63,225,59,241,135,253,13,122,231,254,12,102,255,0,226,168,3,237,250,43,226,
15,248,78,252,97,255,0,67,94,185,255,0,131,25,191,248,170,63,225,59,241,135,253,13,
122,231,254,12,102,255,0,226,168,3,237,250,43,226,15,248,78,252,97,255,0,67,94,185,
255,0,131,25,191,248,170,63,225,59,241,135,253,13,122,231,254,12,102,255,0,226,168,3,
237,250,43,226,15,248,78,252,97,255,0,67,94,185,255,0,131,25,191,248,170,63,225,59,
241,135,253,13,122,231,254,12,102,255,0,226,168,3,237,250,43,226,15,248,78,252,97,255,
0,67,94,185,255,0,131,25,191,248,170,63,225,59,241,135,253,13,122,231,254,12,102,255,
0,226,168,3,237,250,43,226,15,248,78,252,97,255,0,67,94,185,255,0,131,25,191,248,
170,63,225,59,241,135,253,13,122,231,254,12,102,255,0,226,168,3,208,63,104,239,249,40,
122,127,253,130,163,255,0,209,178,214,255,0,236,203,255,0,51,79,253,186,127,237,106,240,
205,75,86,212,181,155,133,184,213,53,11,187,233,213,2,44,151,83,52,172,23,36,224,22,
36,227,36,156,123,154,247,63,217,151,254,102,159,251,116,255,0,218,212,0,126,211,95,243,
43,127,219,223,254,209,175,0,175,127,253,166,191,230,86,255,0,183,191,253,163,94,1,64,
5,123,255,0,236,203,255,0,51,79,253,186,127,237,106,240,10,247,255,0,217,151,254,102,
159,251,116,255,0,218,212,0,126,211,95,243,43,127,219,223,254,209,175,0,175,175,254,38,
252,50,255,0,133,141,253,151,255,0,19,127,236,255,0,176,121,191,242,237,230,239,223,179,
253,181,198,54,123,245,175,63,255,0,134,101,255,0,169,187,255,0,41,191,253,182,128,60,
2,138,247,255,0,248,102,95,250,155,191,242,155,255,0,219,104,255,0,134,101,255,0,169,
187,255,0,41,191,253,182,128,60,2,138,247,255,0,248,102,95,250,155,191,242,155,255,0,
219,104,255,0,134,101,255,0,169,187,255,0,41,191,253,182,128,60,2,138,247,255,0,248,
102,95,250,155,191,242,155,255,0,219,104,255,0,134,101,255,0,169,187,255,0,41,191,253,
182,128,60,2,138,247,255,0,248,102,95,250,155,191,242,155,255,0,219,104,255,0,134,101,
255,0,169,187,255,0,41,191,253,182,128,60,2,138,247,255,0,248,102,95,250,155,191,242,
155,255,0,219,104,255,0,134,101,255,0,169,187,255,0,41,191,253,182,128,60,2,138,247,
255,0,248,102,95,250,155,191,242,155,255,0,219,104,255,0,134,101,255,0,169,187,255,0,
41,191,253,182,128,60,2,138,247,255,0,248,102,95,250,155,191,242,155,255,0,219,104,255,
0,134,101,255,0,169,187,255,0,41,191,253,182,128,60,2,138,247,255,0,248,102,95,250,
155,191,242,155,255,0,219,104,255,0,134,101,255,0,169,187,255,0,41,191,253,182,128,60,
2,138,247,255,0,248,102,95,250,155,191,242,155,255,0,219,104,255,0,134,101,255,0,169,
187,255,0,41,191,253,182,128,60,2,138,247,255,0,248,102,95,250,155,191,242,155,255,0,
219,104,255,0,134,101,255,0,169,187,255,0,41,191,253,182,128,60,2,138,247,255,0,248,
102,95,250,155,191,242,155,255,0,219,104,255,0,134,101,255,0,169,187,255,0,41,191,253,
182,128,60,2,138,247,255,0,248,102,95,250,155,191,242,155,255,0,219,104,255,0,134,101,
255,0,169,187,255,0,41,191,253,182,128,60,2,138,247,255,0,248,102,95,250,155,191,242,
155,255,0,219,104,255,0,134,101,255,0,169,187,255,0,41,191,253,182,128,60,2,138,247,
255,0,248,102,95,250,155,191,242,155,255,0,219,104,255,0,134,101,255,0,169,187,255,0,
41,191,253,182,128,60,2,138,247,255,0,248,102,95,250,155,191,242,155,255,0,219,104,255,
0,134,101,255,0,169,187,255,0,41,191,253,182,128,60,2,138,247,255,0,248,102,95,250,
155,191,242,155,255,0,219,104,255,0,134,101,255,0,169,187,255,0,41,191,253,182,128,60,
2,189,255,0,246,101,255,0,153,167,254,221,63,246,181,31,240,204,191,245,55,127,229,55,
255,0,182,215,160,124,50,248,101,255,0,10,231,251,83,254,38,255,0,218,31,111,242,191,
229,219,202,217,179,127,251,109,156,239,246,233,64,30,127,251,77,127,204,173,255,0,111,127,
251,70,188,2,189,255,0,246,154,255,0,153,91,254,222,255,0,246,141,120,5,0,21,232,
31,12,190,38,255,0,194,185,254,212,255,0,137,71,246,135,219,252,175,249,121,242,182,108,
223,254,195,103,59,253,186,87,159,209,64,30,255,0,255,0,13,53,255,0,82,143,254,84,
191,251,85,31,240,211,95,245,40,255,0,229,75,255,0,181,87,128,81,64,30,255,0,255,
0,13,53,255,0,82,143,254,84,191,251,85,31,240,211,95,245,40,255,0,229,75,255,0,
181,87,128,81,64,30,255,0,255,0,13,53,255,0,82,143,254,84,191,251,85,31,240,211,
95,245,40,255,0,229,75,255,0,181,87,128,81,64,30,255,0,255,0,13,53,255,0,82,
143,254,84,191,251,85,31,240,211,95,245,40,255,0,229,75,255,0,181,87,128,81,64,30,
255,0,255,0,13,53,255,0,82,143,254,84,191,251,85,31,240,211,95,245,40,255,0,229,
75,255,0,181,87,128,81,64,30,255,0,255,0,13,53,255,0,82,143,254,84,191,251,85,
31,240,211,95,245,40,255,0,229,75,255,0,181,87,128,81,64,30,255,0,255,0,13,53,
255,0,82,143,254,84,191,251,85,31,240,211,95,245,40,255,0,229,75,255,0,181,87,128,
81,64,30,255,0,255,0,13,53,255,0,82,143,254,84,191,251,85,31,240,211,95,245,40,
255,0,229,75,255,0,181,87,128,81,64,30,255,0,255,0,13,53,255,0,82,143,254,84,
191,251,85,31,240,211,95,245,40,255,0,229,75,255,0,181,87,128,81,64,30,255,0,255,
0,13,53,255,0,82,143,254,84,191,251,85,31,240,211,95,245,40,255,0,229,75,255,0,
181,87,128,81,64,30,255,0,255,0,13,53,255,0,82,143,254,84,191,251,85,31,240,211,
95,245,40,255,0,229,75,255,0,181,87,128,81,64,30,255,0,255,0,13,53,255,0,82,
143,254,84,191,251,85,31,240,211,95,245,40,255,0,229,75,255,0,181,87,128,81,64,30,
255,0,255,0,13,53,255,0,82,143,254,84,191,251,85,31,240,211,95,245,40,255,0,229,
75,255,0,181,87,128,81,64,30,255,0,255,0,13,53,255,0,82,143,254,84,191,251,85,
31,240,211,95,245,40,255,0,229,75,255,0,181,87,128,81,64,30,255,0,255,0,13,53,
255,0,82,143,254,84,191,251,85,31,240,211,95,245,40,255,0,229,75,255,0,181,87,128,
81,64,30,255,0,255,0,13,53,255,0,82,143,254,84,191,251,85,31,240,211,95,245,40,
255,0,229,75,255,0,181,87,128,81,64,30,255,0,255,0,13,53,255,0,82,143,254,84,
191,251,85,31,240,211,95,245,40,255,0,229,75,255,0,181,87,128,81,64,30,255,0,255,
0,13,53,255,0,82,143,254,84,191,251,85,31,240,211,95,245,40,255,0,229,75,255,0,
181,87,128,81,64,30,129,241,55,226,111,252,44,111,236,191,248,148,127,103,253,131,205,255,
0,151,159,55,126,253,159,236,46,49,179,223,173,121,253,20,80,7,255,217
};

// ==========================================================================
// From Relay.ino (reconstructed v1.21; v1.25 changes marked)
// ==========================================================================

QueueHandle_t replies;
QueueHandle_t displayAcks;
struct DisplayAck { uint32_t magic, token, size, status; };
constexpr uint32_t ackMagic = 0x44495350;
struct AckEvent { DisplayAck ack; uint8_t mac[6]; };
uint32_t lidarBytes=0, lidarScans=0, lastDebugAt=0;
bool radioReady;
bool displayRadioReady;
const uint8_t displayBroadcast[] = {255,255,255,255,255,255};
unsigned displayDrops = 0;
TaskHandle_t photoTask;
QueueHandle_t photos;
SemaphoreHandle_t serialOutput;
TaskHandle_t ledTaskHandle = nullptr;
std::atomic<bool> pulseReportPending{false};
bool versionPending;
// v1.25: was false (photos and lights only after the computer sent 'A').
// On the bike there is no computer, so they are on from power-up. 'M' still
// turns them off.
bool automaticActions = true;
uint32_t lastLiveOutput = 0;
std::atomic<uint32_t> loopCounter{0};
std::atomic<uint32_t> loopStage{0};
void debugLine(const char *message);
constexpr uint32_t ledManualBit = 1;
constexpr uint32_t ledWarningBit = 2;
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
constexpr uint32_t testDisplayBit = 4;
unsigned skippedBursts;
uint32_t burstSequence;
constexpr unsigned burstShots = 5;  // v1.25: was 10 (the phone gets the photos)
constexpr unsigned shotIntervalMs = 500;
constexpr uint32_t pulseMs = 500;
constexpr uint8_t ledDataPin = 2; // FireBeetle D9
constexpr uint8_t ledBrightness = 80;

struct Photo {
  uint8_t *bytes;
  uint32_t size;
  const char *error;
  uint32_t burst;
  unsigned shot;
  uint32_t takenAt = 0;  // v1.25: millis() when the photo was requested

  Photo(uint8_t *data = nullptr, uint32_t length = 0, const char *message = nullptr)
      : bytes(data), size(length), error(message), burst(0), shot(0) {}
};

Photo capture();
// v1.25: declared here because, merged into one file, the Arduino IDE would
// otherwise put its own prototype for this above struct Photo.
void forwardDisplay(const Photo &photo);

// v1.25: the phone link (defined in the "Phone link" section at the end).
namespace phone {
void begin();
void service();
void onScan(uint32_t now);
void onClosePass(uint32_t now, bool burstStarted, bool simulated);
void cameraResult(bool ok);
bool keepPhoto(const Photo &photo);
bool bluetoothStarted();
}

Adafruit_NeoPixel strip(30, ledDataPin, NEO_GRB + NEO_KHZ800);


// Drive the strip directly on GPIO2 as in the v1.21 local patch. Feed GRB
// bytes already scaled by NeoPixel. Interrupts are masked for one 30-pixel frame.
static portMUX_TYPE ledLock=portMUX_INITIALIZER_UNLOCKED;
void IRAM_ATTR directLedShow() {
  const uint32_t cyclesPerUs=getCpuFrequencyMhz();
  const uint32_t bitCycles=cyclesPerUs*5/4;
  const uint32_t highOne=cyclesPerUs*8/10, highZero=cyclesPerUs*35/100;
  const uint32_t mask=1u<<ledDataPin;
  const uint8_t *pixels=strip.getPixels();
  const unsigned size=strip.numPixels()*3;
  if(!pixels) return;
  delayMicroseconds(300);
  portENTER_CRITICAL(&ledLock);
  for(unsigned i=0;i<size;++i) {
    for(uint8_t bit=0x80;bit;bit>>=1) {
      uint32_t high=(pixels[i]&bit)?highOne:highZero;
      uint32_t t=XTHAL_GET_CCOUNT();
      REG_WRITE(GPIO_OUT_W1TS_REG,mask);
      while(uint32_t(XTHAL_GET_CCOUNT()-t)<high) {}
      REG_WRITE(GPIO_OUT_W1TC_REG,mask);
      while(uint32_t(XTHAL_GET_CCOUNT()-t)<bitCycles) {}
    }
  }
  portEXIT_CRITICAL(&ledLock);
  delayMicroseconds(300);
}

// LED output owned exclusively by the Arduino loop, like the known-good sketch.
uint32_t warningAt=0, ledChangedAt=0;
bool warning=false, ledOn=false;
unsigned manualCycles=0;
void extendWarning(bool manual) {
  if (manual) { manualCycles=3; ledChangedAt=millis()-500; }
  else { warning=true; warningAt=millis(); }
}
void serviceLed() {
  if (warning && millis()-warningAt>=2000) warning=false;
  if (millis()-ledChangedAt<500) return;
  if (!ledOn && !manualCycles && !warning) return;
  ledChangedAt=millis();
  if (!ledOn) { strip.fill(strip.Color(255,0,0)); directLedShow(); ledOn=true; }
  else { strip.clear(); directLedShow(); ledOn=false;
    if (manualCycles && --manualCycles==0) pulseReportPending.store(true);
  }
}
// The manual test uses the proven standalone sequence, not the warning state machine.
void manualLedTest() {
  debugLine("INFO LED TEST START GPIO2 30 PIXELS BRIGHTNESS80");
  warning = false; manualCycles = 0; ledOn = false;
  strip.setBrightness(80);
  for (unsigned i=0; i<3; ++i) {
    loopStage.store(10);
    debugLine("INFO LED BEFORE RED SHOW");
    strip.fill(strip.Color(255,0,0)); directLedShow();
    loopStage.store(11);
    debugLine("INFO LED AFTER RED SHOW");
    delay(500);
    loopStage.store(12);
    strip.clear(); directLedShow();
    loopStage.store(13);
    debugLine("INFO LED AFTER CLEAR SHOW");
    delay(500);
  }
  ledChangedAt=millis();
  pulseReportPending.store(true);
  loopStage.store(0);
}

// Independent heartbeat reveals a stalled Arduino loop rather than merely disabling buttons.
void diagnosticWatch(void *) {
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(2000));
    if (serialOutput && xSemaphoreTake(serialOutput, pdMS_TO_TICKS(50)) == pdTRUE) {
      Serial.printf("DEBUG WATCH %lu %lu\n", (unsigned long)loopCounter.load(), (unsigned long)loopStage.load());
      xSemaphoreGive(serialOutput);
    }
  }
}

void reportStatus() {
  if (serialOutput && xSemaphoreTake(serialOutput, 0) == pdTRUE) {
    AckEvent event;
    while (displayAcks && xQueueReceive(displayAcks, &event, 0) == pdTRUE) {
      Serial.printf("DEBUG DISPLAY_ACK %lu %lu %lu %02X:%02X:%02X:%02X:%02X:%02X\n", (unsigned long)event.ack.token, (unsigned long)event.ack.size, (unsigned long)event.ack.status, event.mac[0],event.mac[1],event.mac[2],event.mac[3],event.mac[4],event.mac[5]);
    }
    if (millis()-lastDebugAt >= 1000) { lastDebugAt=millis(); Serial.printf("DEBUG HEALTH %lu %lu %u %u %u %u\n",(unsigned long)lidarBytes,(unsigned long)lidarScans,(unsigned)ESP.getFreeHeap(), (unsigned)burstActive.load(), (unsigned)radioReady,(unsigned)displayRadioReady); }
    xSemaphoreGive(serialOutput);
  }
  if ((!versionPending && !pulseReportPending.load() && !scanPending && !closeEvents &&
       !approachEvents && !cameraTaskError && !skippedBursts) || !serialOutput ||
      xSemaphoreTake(serialOutput, 0) != pdTRUE) return;
  if (versionPending) {
    // The version line stays "1.20" (as in v1.21) so existing viewers still
    // recognise the board; v1.25 adds the phone-link line after it.
    Serial.println(radioReady ? "READY 1.20 RADIO OK" : "READY 1.20 RADIO ERROR");
    Serial.println(displayRadioReady ? "INFO DISPLAY RELAY READY" : "INFO DISPLAY RELAY ERROR");
    Serial.println(phone::bluetoothStarted() ? "INFO PHONE LINK 1.25 BLUETOOTH READY"  // v1.25
                                             : "INFO PHONE LINK 1.25 BLUETOOTH ERROR");
    Serial.println("USB IMAGE READY 1.26");
    versionPending = false;
  }
  if (pulseReportPending.exchange(false)) {
    Serial.println("OK LED PULSE GPIO2");
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
  if (scanPending && liveScan && millis()-lastLiveOutput >= 200) {
    lastLiveOutput=millis();
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
  if (scanPending && !liveScan) {
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
  if (size == sizeof(DisplayAck) && displayAcks) {
    AckEvent event; memcpy(&event.ack,data,sizeof(DisplayAck)); memcpy(event.mac,info->src_addr,6);
    if (event.ack.magic == ackMagic && memcmp(info->src_addr,cameraMac,6)) xQueueSend(displayAcks,&event,0);
    return;
  }
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

void debugLine(const char *message) {
  if (serialOutput && xSemaphoreTake(serialOutput, portMAX_DELAY) == pdTRUE) {
    Serial.println(message); xSemaphoreGive(serialOutput);
  }
}
Photo capture() {
  debugLine("DEBUG CAMERA_REQUEST");
  if (!radioReady) return {nullptr, 0, "Relay radio failed to start"};
  xQueueReset(replies);
  Request request = {esp_random() | 1U, 0};
  Reply reply;
  if (!requestChunk(request, reply))
    return {nullptr, 0, "Camera did not answer ESP-NOW"};
  debugLine("DEBUG CAMERA_HEADER_RECEIVED");
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
  debugLine("DEBUG CAMERA_JPEG_COMPLETE");
  return {jpeg, size, nullptr};
}

// Forward each complete JPEG as the same 236-byte packet format used by the camera.
// Runs in the capture task, never in the LiDAR loop or Wi-Fi callback.
void forwardDisplay(const Photo &photo) {
  if (photo.error || !photo.bytes || !photo.size) return;
  if (!displayRadioReady) { debugLine("DEBUG DISPLAY_RADIO_UNAVAILABLE"); return; }
  char msg[120]; unsigned sent=0;
  Reply packet = {};
  packet.request.token = esp_random() | 1U;
  packet.total = photo.size;
  snprintf(msg,sizeof(msg),"DEBUG DISPLAY_BEGIN %lu %lu %lu",(unsigned long)packet.request.token,(unsigned long)photo.size,(unsigned long)((photo.size+chunkSize-1)/chunkSize)); debugLine(msg);
  for (uint32_t offset = 0; offset < photo.size; offset += chunkSize) {
    packet.request.offset = offset;
    memset(packet.bytes, 0, sizeof(packet.bytes));
    memcpy(packet.bytes, photo.bytes + offset, min(chunkSize, size_t(photo.size - offset)));
    bool queued = false;
    for (int attempt = 0; attempt < 3; ++attempt) {
      if (esp_now_send(displayBroadcast, reinterpret_cast<const uint8_t *>(&packet), sizeof(packet)) == ESP_OK) {
        queued = true;
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(5) + 1);
    }
    if (!queued) { ++displayDrops; break; }
    ++sent;
    // Pace packets so the Wi-Fi queue can drain; broadcast has no delivery ACK.
    vTaskDelay(pdMS_TO_TICKS(4) + 1);
  }
  snprintf(msg,sizeof(msg),"DEBUG DISPLAY_END %lu %u %lu",(unsigned long)packet.request.token,sent,(unsigned long)((photo.size+chunkSize-1)/chunkSize)); debugLine(msg);
}

void captureBurst(TickType_t lastRequest) {
  const uint32_t burst = ++burstSequence;
  for (unsigned shot = 1; shot <= burstShots; ++shot) {
    vTaskDelayUntil(&lastRequest, pdMS_TO_TICKS(shotIntervalMs));
    lastRequest = xTaskGetTickCount();
    const uint32_t takenAt = millis();  // v1.25
    Photo photo = capture();
    photo.takenAt = takenAt;  // v1.25
    photo.burst = burst;
    photo.shot = shot;
    forwardDisplay(photo);
    xQueueSend(photos, &photo, portMAX_DELAY);
  }
  burstActive.store(false);
}

void capturePhotos(void *) {
  for (;;) {
    uint32_t commands;
    if (xTaskNotifyWait(0, UINT32_MAX, &commands, portMAX_DELAY) != pdTRUE) continue;
    if (commands & captureBurstBit) captureBurst(burstStart.load());
    if (commands & testDisplayBit) {
      debugLine("DEBUG DISPLAY_TEST_KNOWN_IMAGE");
      Photo test(const_cast<uint8_t*>(debugJpeg),sizeof(debugJpeg)); forwardDisplay(test);
    }
    if (commands & testCameraBit) {
      const uint32_t takenAt = millis();  // v1.25
      Photo photo = capture();
      photo.takenAt = takenAt;  // v1.25
      photo.shot = 1;
      forwardDisplay(photo);
      xQueueSend(photos, &photo, portMAX_DELAY);
    }
  }
}

void sendPhotos(void *) {
  for (;;) {
    Photo photo;
    xQueueReceive(photos, &photo, portMAX_DELAY);
    phone::cameraResult(!photo.error);  // v1.25
    xSemaphoreTake(serialOutput, portMAX_DELAY);
    if (photo.error && !photo.burst) Serial.printf("ERR Camera test: %s\n", photo.error);
    else if (photo.error)
      Serial.printf("ERR Burst %lu photo %u/%u: %s\n",
                    static_cast<unsigned long>(photo.burst), photo.shot, burstShots, photo.error);
    else {
      Serial.printf("JPEG %u %lu %u\n", static_cast<unsigned>(photo.size),
                    static_cast<unsigned long>(photo.burst), photo.shot);
      Serial.write(photo.bytes, photo.size);
    }
    xSemaphoreGive(serialOutput);
    // v1.25: burst photos (and burst errors, so every shot is counted) go to
    // the phone link, which then owns the bytes. Was: free(photo.bytes).
    if (!(photo.burst && phone::keepPhoto(photo))) free(photo.bytes);
  }
}

void setup() {
  Serial.setRxBufferSize(8192);
  Serial.begin(460800);
  strip.begin(); strip.setBrightness(80);
  strip.clear(); directLedShow();
#ifdef LIDAR_RX_PIN
  lidarUart.setRxBufferSize(8192);
  lidarUart.begin(230400, SERIAL_8N1, LIDAR_RX_PIN, -1);
#endif

  replies = xQueueCreate(8, sizeof(Reply));
  displayAcks = xQueueCreate(16,sizeof(AckEvent));
  radioReady = replies && startRadio(cameraMac, receiveReply);
  if (radioReady) {
    esp_now_peer_info_t displayPeer = {};
    memcpy(displayPeer.peer_addr, displayBroadcast, 6);
    displayPeer.channel = channel;
    displayPeer.ifidx = WIFI_IF_STA;
    displayRadioReady = esp_now_add_peer(&displayPeer) == ESP_OK;
  }
  photos = xQueueCreate(2, sizeof(Photo));
  serialOutput = xSemaphoreCreateMutex();
  if (serialOutput) xTaskCreate(diagnosticWatch, "diagWatch", 4096, nullptr, 1, nullptr);
  if (photos && serialOutput &&
      xTaskCreate(sendPhotos, "sendPhotos", 8192, nullptr, 1, nullptr) == pdPASS)
    xTaskCreate(capturePhotos, "capturePhotos", 8192, nullptr, 1, &photoTask);
  phone::begin();  // v1.25
}

// v1.25: returns whether a new burst started (was void).
bool requestPhotoBurst() {
  if (!photoTask) {
    cameraTaskError = true;
    return false;
  }
  if (burstActive.exchange(true)) {
    ++skippedBursts;
    return false;
  }
  burstStart.store(xTaskGetTickCount());
  xTaskNotify(photoTask, captureBurstBit, eSetBits);
  return true;
}

void processScan(uint32_t stamp) {
  scanResult = detector.processScan(scanPoints, stamp / 1000.0);
  scanPending = true;
  phone::onScan(millis());  // v1.25: LiDAR clip for the phone
  if (scanResult.newClose) {
    ++closeEvents;
    // v1.25: the phone is alerted even with automatic actions off; the
    // package then carries the LiDAR clip without photos.
    const bool started = automaticActions && requestPhotoBurst();
    phone::onClosePass(millis(), started, false);
  }
  if (automaticActions && (scanResult.renewWarning || scanResult.close)) extendWarning(false);
  if (scanResult.incoming && !approachWasActive) ++approachEvents;
  approachWasActive = scanResult.incoming;
}

void processLidarByte(uint8_t byte, uint32_t now) {
  ++lidarBytes;
  if (!liveLidar.push(byte, now)) return;
  ++lidarScans;
  if (!liveScan) detector.reset();
  liveScan = true;
  scanPoints.swap(liveLidar.scan);
  processScan(liveLidar.stamp);
}

// USB image debugger: each line is '@' followed by one hex-encoded Reply.
// Lowercase digits keep a partial line harmless to the legacy command parser.
static char imageHex[sizeof(Reply)*2];
static size_t imageUsed;
static bool imageLine;
static uint32_t imageLastByte;

int imageNibble(char c) {
  if (c >= '0' && c <= '9') return c-'0';
  if (c >= 'a' && c <= 'f') return c-'a'+10;
  return -1;
}

bool readImageByte(uint8_t byte) {
  if (imageLine && millis()-imageLastByte > 2000) imageLine=false;
  if (byte=='@') { imageLine=true; imageUsed=0; imageLastByte=millis(); return true; }
  if (!imageLine) return false;
  imageLastByte=millis();
  if (byte!='\n') {
    if (imageUsed<sizeof(imageHex) && imageNibble(byte)>=0) imageHex[imageUsed++]=byte;
    else imageLine=false;
    return true;
  }
  imageLine=false;
  if (imageUsed!=sizeof(imageHex)) return true;
  Reply packet={};
  auto *raw=reinterpret_cast<uint8_t*>(&packet);
  for (size_t i=0;i<sizeof(packet);++i)
    raw[i]=(imageNibble(imageHex[i*2])<<4)|imageNibble(imageHex[i*2+1]);
  if (!packet.request.token || packet.total>maxJpeg || packet.request.offset>=packet.total ||
      packet.request.offset%chunkSize) return true;
  esp_err_t result=displayRadioReady ? esp_now_send(displayBroadcast,raw,sizeof(packet)) : ESP_FAIL;
  char message[96];
  snprintf(message,sizeof(message),"USBFRAME %lu %lu %d",(unsigned long)packet.request.token,
           (unsigned long)packet.request.offset,(int)result);
  debugLine(message);
  return true;
}

void receiveInput(uint8_t byte) {
  const uint32_t now = millis();
  if (scanInput.collecting() && now - lastInputAt > 250) scanInput.discardPartial();
  if (!scanInput.collecting() && readImageByte(byte)) return;
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
    case 'A': automaticActions=true; debugLine("INFO AUTOMATIC ACTIONS ON"); break;
    case 'M': automaticActions=false; warning=false; debugLine("INFO AUTOMATIC ACTIONS OFF"); break;
    case 'T':
      if (photoTask) xTaskNotify(photoTask, testDisplayBit, eSetBits);
      else cameraTaskError = true;
      break;
    case 'P':
      if (photoTask) xTaskNotify(photoTask, testCameraBit, eSetBits);
      else cameraTaskError = true;
      break;
    case 'X':  // v1.25: pretend a close pass happened, to test the phone link
      ++closeEvents;
      phone::onClosePass(now, requestPhotoBurst(), true);
      break;
    case 'R':
      detector.reset();
      scanPending = false;
      approachWasActive = false;
      break;
  }
}

void loop() {
  static bool startupTest = true;
  if (startupTest) { startupTest=false; extendWarning(true); }
  ++loopCounter;
  loopStage.store(1);
  serviceLed();
  for (unsigned i = 0; i < 1024 && Serial.available(); ++i)
    receiveInput(static_cast<uint8_t>(Serial.read()));
  loopStage.store(2);
#ifdef LIDAR_RX_PIN
  for (unsigned i = 0; i < 1024 && lidarUart.available(); ++i)
    processLidarByte(static_cast<uint8_t>(lidarUart.read()), millis());
#endif
  loopStage.store(3);
  serviceLed();
  loopStage.store(4);
  reportStatus();
  loopStage.store(5);
  phone::service();  // v1.25
  loopStage.store(0);
  delay(1);
}

// ==========================================================================
// Phone protocol bytes (new in v1.25)
//
// Builds the exact bytes of Phone App/SPECIFICATION.md section 3. Plain C++
// with no Arduino or ESP-IDF calls, so Tools/Phone Link Test/ can compile this
// block on a computer and check it against the phone app's own decoder. Keep
// it that way, and keep the BEGIN/END marker lines.
// ==========================================================================
// BEGIN PHONE WIRE
namespace wire {

constexpr uint8_t msgDisplayChunk = 0x10;    // phone -> main, forwarded to the Front Module
constexpr uint8_t msgTransferAck = 0x30;     // phone -> main
constexpr uint8_t msgHello = 0x40;           // phone -> main
constexpr uint8_t msgClosePassAlert = 0xA1;  // main -> phone
constexpr uint8_t msgTransferStart = 0xB1;
constexpr uint8_t msgTransferData = 0xB2;
constexpr uint8_t msgTransferEnd = 0xB3;
constexpr uint8_t msgStatus = 0xC1;
constexpr uint8_t transferClosePass = 1;
constexpr uint8_t ackOk = 0;
constexpr uint16_t unknown16 = 0xFFFF;
constexpr size_t maxScanPoints = 1024;
constexpr size_t dataHeaderBytes = 7;        // [0xB2][id u16][offset u32]

inline void put16(uint8_t *p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v);
  p[1] = static_cast<uint8_t>(v >> 8);
}
inline void put32(uint8_t *p, uint32_t v) {
  put16(p, static_cast<uint16_t>(v));
  put16(p + 2, static_cast<uint16_t>(v >> 16));
}
inline uint16_t get16(const uint8_t *p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }

// Metres -> millimetres for a u16 field (0xFFFE at most; 0xFFFF means unknown).
inline uint16_t millimetres(float metres) {
  const long mm = std::lround(metres * 1000);
  return static_cast<uint16_t>(mm < 0 ? 0 : (mm > 0xFFFE ? 0xFFFE : mm));
}
// Metres per second -> km/h x 10 for a u16 field.
inline uint16_t kmh10(float metresPerSecond) {
  const long v = std::lround(metresPerSecond * 36);
  return static_cast<uint16_t>(v < 0 ? 0 : (v > 0xFFFE ? 0xFFFE : v));
}
// Metres -> signed millimetres for a LiDAR point (same as the USB LIVE output).
inline int16_t coordinate(float metres) {
  const long mm = std::lround(metres * 1000);
  return static_cast<int16_t>(mm < -32768 ? -32768 : (mm > 32767 ? 32767 : mm));
}

// Standard CRC-32 (zlib / Ethernet): crc32Update(0, "123456789", 9) ==
// 0xCBF43926. Chain calls to cover several pieces.
inline uint32_t crc32Update(uint32_t crc, const uint8_t *data, size_t length) {
  static uint32_t table[256];
  static bool ready = false;
  if (!ready) {
    for (uint32_t n = 0; n < 256; ++n) {
      uint32_t c = n;
      for (int k = 0; k < 8; ++k) c = (c & 1) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
      table[n] = c;
    }
    ready = true;
  }
  crc = ~crc;
  for (size_t i = 0; i < length; ++i) crc = table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
  return ~crc;
}

// ---- notifications to the phone (each returns its length) ----
inline size_t alert(uint8_t *out, uint16_t eventId, uint16_t minDistanceMm, uint16_t speedKmh10) {
  out[0] = msgClosePassAlert;
  put16(out + 1, eventId);
  put16(out + 3, minDistanceMm);
  put16(out + 5, speedKmh10);
  return 7;
}
inline size_t status(uint8_t *out, bool lidarOk, bool cameraOk, bool frontOk, uint8_t batteryPercent,
                     uint8_t firmwareMajor, uint8_t firmwareMinor) {
  out[0] = msgStatus;
  out[1] = (lidarOk ? 1 : 0) | (cameraOk ? 2 : 0) | (frontOk ? 4 : 0);
  out[2] = batteryPercent;  // 255 = unknown
  out[3] = firmwareMajor;
  out[4] = firmwareMinor;
  return 5;
}
inline size_t transferStart(uint8_t *out, uint16_t transferId, uint8_t kind, uint32_t totalLength) {
  out[0] = msgTransferStart;
  put16(out + 1, transferId);
  out[3] = kind;
  put32(out + 4, totalLength);
  return 8;
}
inline size_t transferDataHeader(uint8_t *out, uint16_t transferId, uint32_t offset) {
  out[0] = msgTransferData;
  put16(out + 1, transferId);
  put32(out + 3, offset);
  return dataHeaderBytes;
}
inline size_t transferEnd(uint8_t *out, uint16_t transferId, uint32_t crc) {
  out[0] = msgTransferEnd;
  put16(out + 1, transferId);
  put32(out + 3, crc);
  return 7;
}

// ---- the close-pass package ("CPK1", section 3.4) ----
inline size_t packageHeader(uint8_t *out, uint16_t eventId, uint32_t eventUptimeMs, uint16_t minDistanceMm,
                            uint16_t speedKmh10, uint8_t photoCount) {
  static const uint8_t magic[4] = {0x43, 0x50, 0x4B, 0x31};  // "CPK1"
  std::memcpy(out, magic, 4);
  put16(out + 4, eventId);
  put32(out + 6, eventUptimeMs);
  put16(out + 10, minDistanceMm);
  put16(out + 12, speedKmh10);
  out[14] = photoCount;
  out[15] = 0;
  return 16;
}
inline size_t photoHeader(uint8_t *out, int32_t offsetMs, uint32_t length) {
  put32(out, static_cast<uint32_t>(offsetMs));
  put32(out + 4, length);
  return 8;
}
inline size_t clipHeader(uint8_t *out, uint16_t scanCount) {
  put16(out, scanCount);
  put16(out + 2, 0);
  return 4;
}
inline size_t scanHeader(uint8_t *out, int32_t offsetMs, uint8_t flags, uint16_t speedKmh10, uint16_t pointCount) {
  put32(out, static_cast<uint32_t>(offsetMs));
  out[4] = flags;  // bit 0 close pass, bit 1 approaching vehicle
  put16(out + 5, speedKmh10);
  put16(out + 7, pointCount);
  return 9;
}
inline size_t point(uint8_t *out, int16_t xMm, int16_t yMm, uint8_t pointClass) {
  put16(out, static_cast<uint16_t>(xMm));
  put16(out + 2, static_cast<uint16_t>(yMm));
  out[4] = pointClass;  // 0 normal, 1 close pass, 2 approaching vehicle
  return 5;
}

// Picks at most `cap` of `n` points for the clip: every close-pass or
// approaching-vehicle point (class != 0) first, then evenly spaced normal
// points. Writes the chosen indices in their original order; returns how many.
inline size_t choosePoints(const uint8_t *classes, size_t n, size_t cap, uint16_t *keep) {
  size_t special = 0;
  for (size_t i = 0; i < n; ++i) special += classes[i] != 0;
  const size_t normal = n - special;
  size_t budget = special >= cap ? 0 : cap - special;
  if (budget > normal) budget = normal;
  size_t kept = 0, seen = 0, taken = 0;
  for (size_t i = 0; i < n && kept < cap; ++i) {
    if (classes[i]) {
      keep[kept++] = static_cast<uint16_t>(i);
    } else {
      if (taken < budget && seen * budget >= taken * normal) {
        keep[kept++] = static_cast<uint16_t>(i);
        ++taken;
      }
      ++seen;
    }
  }
  return kept;
}

// A package is sent from several separate buffers (header, photos, clip)
// without first copying them into one big block.
struct Segment {
  const uint8_t *data;
  size_t length;
};
inline size_t totalLength(const Segment *segments, size_t count) {
  size_t total = 0;
  for (size_t i = 0; i < count; ++i) total += segments[i].length;
  return total;
}
inline uint32_t crcOf(const Segment *segments, size_t count) {
  uint32_t crc = 0;
  for (size_t i = 0; i < count; ++i) crc = crc32Update(crc, segments[i].data, segments[i].length);
  return crc;
}
// Copies up to `length` bytes starting at `offset` of the joined segments.
inline size_t copyOut(const Segment *segments, size_t count, size_t offset, uint8_t *out, size_t length) {
  size_t copied = 0;
  for (size_t i = 0; i < count && copied < length; ++i) {
    const Segment &s = segments[i];
    if (offset >= s.length) {
      offset -= s.length;
      continue;
    }
    size_t n = s.length - offset;
    if (n > length - copied) n = length - copied;
    std::memcpy(out + copied, s.data + offset, n);
    copied += n;
    offset = 0;
  }
  return copied;
}

}  // namespace wire
// END PHONE WIRE

// ==========================================================================
// Phone link over Bluetooth LE (new in v1.25)
//
// The phone (Bike Nav app) connects to "BikeNav Main". It writes handlebar
// screens (0x10), which go straight to the Front Module over ESP-NOW, a hello
// (0x40), answered with a status (0xC1), and acks (0x30). On a close pass the
// phone gets an alert (0xA1) at once, then, once the 5 photos are in and the
// LiDAR has recorded 1.5 s more, a close-pass package (0xB1/0xB2/0xB3).
//
// Tasks: NimBLE callbacks run on the Bluetooth task and only forward screens
// and set flags; the photos come from the sendPhotos task through a queue;
// everything else runs from loop() via phone::service().
// ==========================================================================
namespace phone {

// ---- settings ----
const char *const deviceName = "BikeNav Main";
const char *const serviceUuid = "6e3b2000-8f7a-4c1e-9b52-2d1c0b7a1f10";
const char *const phoneToMainUuid = "6e3b2001-8f7a-4c1e-9b52-2d1c0b7a1f10";  // write
const char *const mainToPhoneUuid = "6e3b2002-8f7a-4c1e-9b52-2d1c0b7a1f10";  // notify
const uint8_t frontMac[] = {0x8c, 0x94, 0xdf, 0x6c, 0xbc, 0x80};  // Front Module (change if replaced)
constexpr uint8_t firmwareMajor = 1;
constexpr uint8_t firmwareMinor = 25;
constexpr uint32_t clipBeforeMs = 2000;     // LiDAR clip: this long before the close pass...
constexpr uint32_t clipAfterMs = 1500;      // ...to this long after
constexpr size_t clipSlots = 40;            // recent scans kept (4 s at 10 scans/s)
constexpr size_t clipMaxPoints = 200;       // points kept per scan (the phone allows 1024)
constexpr uint32_t photoWaitMs = 20000;     // send the package anyway after this long
constexpr uint32_t ackTimeoutMs = 5000;     // phone must confirm a package within this
constexpr unsigned maxResends = 2;          // then the package is dropped
constexpr uint32_t statusEveryMs = 10000;
constexpr uint32_t lidarFreshMs = 1000;     // "LiDAR OK" = a scan within this
constexpr uint32_t frontFreshMs = 15000;    // "Front OK" = last screen delivered within this
constexpr uint32_t approachMemoryMs = 5000; // approach speed reported with a close pass
constexpr size_t maxNotify = 512;           // largest notification we build
constexpr unsigned partsPerLoop = 16;       // notifications per loop() pass

// ---- shared with the Bluetooth task and the sendPhotos task ----
NimBLECharacteristic *toPhone;
bool bluetoothReady;
std::atomic<bool> connected{false};
std::atomic<uint32_t> connection{0};  // +1 on every new connection
std::atomic<uint16_t> mtu{23};
std::atomic<bool> helloPending{false};
std::atomic<bool> ackPending{false};
std::atomic<uint16_t> ackId{0};
std::atomic<uint8_t> ackStatus{0};
std::atomic<int> frontResult{-1};     // last screen delivery: -1 none, 0 failed, 1 delivered
std::atomic<uint32_t> frontResultAt{0};
std::atomic<int> cameraState{-1};     // last photo: -1 none yet, 0 failed, 1 OK
std::atomic<bool> acceptingPhotos{false};
QueueHandle_t photoQueue;

// ---- used only from loop() ----
struct ClipScan {
  uint32_t at;  // millis() when processed
  uint8_t flags;
  uint16_t speed10;
  uint16_t count;
  uint8_t points[clipMaxPoints * 5];
};
ClipScan ring[clipSlots];
size_t ringNext;
size_t ringUsed;
bool anyScan;
uint32_t lastScanAt;
uint16_t lastApproach10 = wire::unknown16;
uint32_t lastApproachAt;

struct Package {
  bool active = false;       // a close pass is being collected or sent
  bool photosDone = false;
  bool clipDone = false;
  bool ready = false;        // header, segments, length and CRC built
  uint16_t eventId = 0;
  uint32_t eventAt = 0;
  uint16_t minDistanceMm = wire::unknown16;
  uint16_t speed10 = wire::unknown16;
  Photo photos[burstShots];
  unsigned photoCount = 0;
  uint8_t *clip = nullptr;
  size_t clipSize = 0;
  uint16_t transferId = 0;
  uint8_t header[16] = {};
  uint8_t photoHeaders[burstShots][8] = {};
  uint8_t emptyClip[4] = {0, 0, 0, 0};
  wire::Segment segments[2 + 2 * burstShots] = {};
  size_t segmentCount = 0;
  size_t total = 0;
  uint32_t crc = 0;

  void release() {
    for (unsigned i = 0; i < photoCount; ++i) free(photos[i].bytes);
    free(clip);
    *this = Package();
  }
};
Package collecting;  // the close pass being gathered (photos + clip)
Package outgoing;    // the finished package being sent to the phone

enum class Send : uint8_t { idle, start, data, end, waitAck };
Send send = Send::idle;
size_t sendOffset;
uint32_t ackDeadline;
unsigned resends;
uint32_t sendConnection;
uint16_t transferCounter;
uint16_t eventCounter;
uint8_t pendingAlert[7];
bool alertWaiting;
bool helloSeen;
uint32_t helloConnection;
uint32_t lastStatusAt;
bool wasConnected;

void log(const char *format, ...) {
  if (!serialOutput || xSemaphoreTake(serialOutput, pdMS_TO_TICKS(50)) != pdTRUE) return;
  char line[160];
  va_list args;
  va_start(args, format);
  vsnprintf(line, sizeof(line), format, args);
  va_end(args);
  Serial.println(line);
  xSemaphoreGive(serialOutput);
}

bool notify(const uint8_t *data, size_t length) {
  return bluetoothReady && connected.load() && toPhone->notify(data, length);
}

// ---- Bluetooth callbacks (Bluetooth task) ----
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *server, NimBLEConnInfo &info) override {
    mtu = info.getMTU();
    ++connection;
    connected = true;
    // 7.5-15 ms interval, 4 s supervision timeout: faster package transfers.
    server->updateConnParams(info.getConnHandle(), 6, 12, 0, 400);
  }
  void onDisconnect(NimBLEServer *, NimBLEConnInfo &, int) override {
    connected = false;  // advertising restarts by itself (advertiseOnDisconnect)
  }
  void onMTUChange(uint16_t value, NimBLEConnInfo &) override { mtu = value; }
};

class PhoneWrites : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *characteristic, NimBLEConnInfo &) override {
    const NimBLEAttValue value = characteristic->getValue();
    const uint8_t *data = value.data();
    const size_t length = value.size();
    if (!length) return;
    switch (data[0]) {
      case wire::msgDisplayChunk:
        // Forward unchanged; the Front Module decodes it (ESP-NOW max 250 bytes).
        if (radioReady && length >= 5 && length <= 250) esp_now_send(frontMac, data, length);
        break;
      case wire::msgHello:
        helloPending = true;
        break;
      case wire::msgTransferAck:
        if (length >= 4) {
          ackId = wire::get16(data + 1);
          ackStatus = data[3];
          ackPending = true;
        }
        break;
    }
  }
};

ServerCallbacks serverCallbacks;
PhoneWrites phoneWrites;

// ESP-NOW delivery reports (Wi-Fi task). The send callback's signature
// changed in ESP-IDF 5.5 (ESP32 Arduino core 3.3).
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
void espNowSent(const esp_now_send_info_t *info, esp_now_send_status_t result) {
  const uint8_t *mac = info ? info->des_addr : nullptr;
#else
void espNowSent(const uint8_t *mac, esp_now_send_status_t result) {
#endif
  if (!mac || memcmp(mac, frontMac, 6)) return;
  frontResult = result == ESP_NOW_SEND_SUCCESS ? 1 : 0;
  frontResultAt = millis();
}

// ---- called from the firmware above ----
void begin() {
  photoQueue = xQueueCreate(burstShots + 1, sizeof(Photo));
  if (radioReady) {
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, frontMac, 6);
    peer.channel = channel;
    peer.ifidx = WIFI_IF_STA;
    esp_now_add_peer(&peer);
    esp_now_register_send_cb(espNowSent);
  }
  if (!NimBLEDevice::init(deviceName)) return;
  NimBLEDevice::setMTU(517);
  NimBLEServer *server = NimBLEDevice::createServer();
  server->setCallbacks(&serverCallbacks);
  server->advertiseOnDisconnect(true);
  NimBLEService *service = server->createService(serviceUuid);
  NimBLECharacteristic *fromPhone =
      service->createCharacteristic(phoneToMainUuid, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  fromPhone->setCallbacks(&phoneWrites);
  toPhone = service->createCharacteristic(mainToPhoneUuid, NIMBLE_PROPERTY::NOTIFY);
  server->start();
  NimBLEAdvertising *advertising = NimBLEDevice::getAdvertising();
  advertising->addServiceUUID(serviceUuid);  // the app finds the module by this
  NimBLEAdvertisementData response;
  response.setName(deviceName);              // name goes in the scan response
  advertising->setScanResponseData(response);
  advertising->enableScanResponse(true);
  bluetoothReady = advertising->start();
}

bool bluetoothStarted() { return bluetoothReady; }

void cameraResult(bool ok) { cameraState = ok ? 1 : 0; }

// sendPhotos task: hand a burst photo to the package being collected.
// Returns true if the phone link took it (and will free the bytes).
bool keepPhoto(const Photo &photo) {
  return acceptingPhotos.load() && photoQueue && xQueueSend(photoQueue, &photo, 0) == pdTRUE;
}

// Every processed scan goes into the ring buffer for clips.
void onScan(uint32_t now) {
  anyScan = true;
  lastScanAt = now;
  if (scanResult.incoming && scanResult.speed > 0) {
    lastApproach10 = wire::kmh10(scanResult.speed);
    lastApproachAt = now;
  }
  static uint8_t classes[wire::maxScanPoints];
  static uint16_t keep[clipMaxPoints];
  const size_t n = std::min(scanPoints.size(), wire::maxScanPoints);
  memset(classes, 0, n);
  for (uint16_t i : scanResult.closePoints) if (i < n) classes[i] = 1;
  for (uint16_t i : scanResult.approachPoints) if (i < n) classes[i] = 2;  // as in LIVE

  ClipScan &scan = ring[ringNext];
  ringNext = (ringNext + 1) % clipSlots;
  if (ringUsed < clipSlots) ++ringUsed;
  scan.at = now;
  scan.flags = (scanResult.close ? 1 : 0) | (scanResult.incoming ? 2 : 0);
  scan.speed10 = scanResult.incoming ? wire::kmh10(scanResult.speed) : wire::unknown16;
  scan.count = static_cast<uint16_t>(wire::choosePoints(classes, n, clipMaxPoints, keep));
  for (size_t k = 0; k < scan.count; ++k) {
    const lidar::Point &p = scanPoints[keep[k]];
    wire::point(scan.points + 5 * k, wire::coordinate(p.x), wire::coordinate(p.y), classes[keep[k]]);
  }
}

// A close pass: alert the phone now, start collecting its package.
void onClosePass(uint32_t now, bool burstStarted, bool simulated) {
  const uint16_t eventId = ++eventCounter;
  uint16_t minDistanceMm = wire::unknown16;
  if (!simulated) {
    float best = 1e9f;
    for (uint16_t i : scanResult.closePoints)
      if (i < scanPoints.size()) best = std::min(best, std::hypot(scanPoints[i].x, scanPoints[i].y));
    if (best < 1e8f) minDistanceMm = wire::millimetres(best);
  }
  const uint16_t speed10 =
      lastApproach10 != wire::unknown16 && now - lastApproachAt <= approachMemoryMs ? lastApproach10 : wire::unknown16;

  if (connected.load()) {
    wire::alert(pendingAlert, eventId, minDistanceMm, speed10);
    alertWaiting = !notify(pendingAlert, sizeof(pendingAlert));  // retried from service()
  }
  if (collecting.active) {
    log("INFO Close pass %u: alert only, close pass %u is still being collected", eventId, collecting.eventId);
    return;
  }
  collecting = Package();
  collecting.active = true;
  collecting.eventId = eventId;
  collecting.eventAt = now;
  collecting.minDistanceMm = minDistanceMm;
  collecting.speed10 = speed10;
  collecting.photosDone = !burstStarted;  // no new burst: send the clip alone
  acceptingPhotos = burstStarted;
}

// Copies the ring's scans from clipBeforeMs before to clipAfterMs after the
// event into one buffer, in the package's clip layout.
void buildClip(Package &p) {
  const size_t first = (ringNext + clipSlots - ringUsed) % clipSlots;
  auto inClip = [&](const ClipScan &s) {
    const int32_t offset = static_cast<int32_t>(s.at - p.eventAt);
    return offset >= -static_cast<int32_t>(clipBeforeMs) && offset <= static_cast<int32_t>(clipAfterMs);
  };
  size_t scans = 0, bytes = 4;
  for (size_t j = 0; j < ringUsed; ++j) {
    const ClipScan &s = ring[(first + j) % clipSlots];
    if (inClip(s)) {
      ++scans;
      bytes += 9 + s.count * 5;
    }
  }
  p.clipDone = true;
  if (!scans) return;  // no LiDAR: the package carries an empty clip
  p.clip = static_cast<uint8_t *>(malloc(bytes));
  if (!p.clip) {
    log("ERR Close pass %u: no memory for the LiDAR clip (%u bytes)", p.eventId, unsigned(bytes));
    return;
  }
  uint8_t *out = p.clip + wire::clipHeader(p.clip, static_cast<uint16_t>(scans));
  for (size_t j = 0; j < ringUsed; ++j) {
    const ClipScan &s = ring[(first + j) % clipSlots];
    if (!inClip(s)) continue;
    out += wire::scanHeader(out, static_cast<int32_t>(s.at - p.eventAt), s.flags, s.speed10, s.count);
    memcpy(out, s.points, s.count * 5);
    out += s.count * 5;
  }
  p.clipSize = bytes;
}

// Builds the header, photo headers and segment list once the package sits
// in `outgoing` (the segments point into the struct, so it must not move).
void finalize(Package &p) {
  wire::packageHeader(p.header, p.eventId, p.eventAt, p.minDistanceMm, p.speed10,
                      static_cast<uint8_t>(p.photoCount));
  size_t n = 0;
  p.segments[n++] = {p.header, sizeof(p.header)};
  for (unsigned i = 0; i < p.photoCount; ++i) {
    wire::photoHeader(p.photoHeaders[i], static_cast<int32_t>(p.photos[i].takenAt - p.eventAt), p.photos[i].size);
    p.segments[n++] = {p.photoHeaders[i], 8};
    p.segments[n++] = {p.photos[i].bytes, p.photos[i].size};
  }
  p.segments[n++] = p.clip ? wire::Segment{p.clip, p.clipSize} : wire::Segment{p.emptyClip, sizeof(p.emptyClip)};
  p.segmentCount = n;
  p.total = wire::totalLength(p.segments, n);
  p.crc = wire::crcOf(p.segments, n);
  p.transferId = ++transferCounter;
  p.ready = true;
}

void collect(uint32_t now) {
  Photo photo;
  while (photoQueue && xQueueReceive(photoQueue, &photo, 0) == pdTRUE) {
    if (!collecting.active || collecting.photosDone) {
      free(photo.bytes);
      continue;
    }
    if (photo.bytes && collecting.photoCount < burstShots) collecting.photos[collecting.photoCount++] = photo;
    else free(photo.bytes);
    if (photo.shot >= burstShots) {
      collecting.photosDone = true;
      acceptingPhotos = false;
    }
  }
  if (!collecting.active) return;
  if (!collecting.clipDone && now - collecting.eventAt >= clipAfterMs) buildClip(collecting);
  if (!collecting.photosDone && now - collecting.eventAt >= photoWaitMs) {
    log("ERR Close pass %u: only %u of %u photos arrived", collecting.eventId, collecting.photoCount, burstShots);
    collecting.photosDone = true;
    acceptingPhotos = false;
  }
  if (!collecting.photosDone || !collecting.clipDone) return;

  // Finished. Only one package is kept: if the phone isn't connected, the
  // newest close pass replaces an older one that was never delivered.
  if (outgoing.active) {
    if (connected.load()) return;  // still sending the older one; try again later
    log("INFO Close pass %u was never delivered; keeping close pass %u instead", outgoing.eventId,
        collecting.eventId);
    outgoing.release();
    send = Send::idle;
  }
  outgoing = collecting;
  collecting = Package();  // ownership of the photos and clip moved to outgoing
  finalize(outgoing);
  resends = 0;
  send = Send::idle;
  log("INFO Close pass %u ready for the phone: %u photos, %u bytes", outgoing.eventId, outgoing.photoCount,
      unsigned(outgoing.total));
}

void giveUpOrResend(const char *why) {
  if (resends < maxResends) {
    ++resends;
    send = Send::idle;  // send the whole package again
    log("INFO Close pass %u: %s, sending again", outgoing.eventId, why);
  } else {
    log("ERR Close pass %u dropped: %s after %u tries", outgoing.eventId, why, maxResends + 1);
    outgoing.release();
    send = Send::idle;
  }
}

void transfer(uint32_t now) {
  if (ackPending.exchange(false) && outgoing.ready && ackId.load() == outgoing.transferId) {
    if (ackStatus.load() == wire::ackOk) {
      log("INFO Close pass %u delivered to the phone", outgoing.eventId);
      outgoing.release();
      send = Send::idle;
      return;
    }
    giveUpOrResend("phone asked again");
  }
  if (!outgoing.ready) return;
  if (!connected.load() || !helloSeen) {
    send = Send::idle;  // start again from the beginning after reconnecting
    return;
  }
  if (sendConnection != connection.load()) {
    sendConnection = connection.load();
    send = Send::idle;
  }
  uint8_t message[maxNotify];
  switch (send) {
    case Send::idle:
      sendOffset = 0;
      send = Send::start;
      [[fallthrough]];
    case Send::start:
      if (!notify(message, wire::transferStart(message, outgoing.transferId, wire::transferClosePass,
                                               static_cast<uint32_t>(outgoing.total)))) return;
      send = Send::data;
      [[fallthrough]];
    case Send::data: {
      const size_t largest = std::min<size_t>(mtu.load() - 3, maxNotify);
      if (largest <= wire::dataHeaderBytes) return;
      for (unsigned part = 0; part < partsPerLoop && sendOffset < outgoing.total; ++part) {
        const size_t header =
            wire::transferDataHeader(message, outgoing.transferId, static_cast<uint32_t>(sendOffset));
        const size_t n = wire::copyOut(outgoing.segments, outgoing.segmentCount, sendOffset, message + header,
                                       largest - header);
        if (!notify(message, header + n)) return;  // Bluetooth queue full: carry on next pass
        sendOffset += n;
      }
      if (sendOffset < outgoing.total) return;
      send = Send::end;
    }
      [[fallthrough]];
    case Send::end:
      if (!notify(message, wire::transferEnd(message, outgoing.transferId, outgoing.crc))) return;
      send = Send::waitAck;
      ackDeadline = now + ackTimeoutMs;
      return;
    case Send::waitAck:
      if (static_cast<int32_t>(now - ackDeadline) >= 0) giveUpOrResend("no answer from the phone");
      return;
  }
}

void sendStatus(uint32_t now) {
  uint8_t message[5];
  const bool lidarOk = anyScan && now - lastScanAt <= lidarFreshMs;
  const bool cameraOk = radioReady && cameraState.load() != 0;  // OK until a photo fails
  const bool frontOk = frontResult.load() == 1 && now - frontResultAt.load() <= frontFreshMs;
  wire::status(message, lidarOk, cameraOk, frontOk, 255, firmwareMajor, firmwareMinor);
  if (notify(message, sizeof(message))) lastStatusAt = now;
}

void service() {
  const uint32_t now = millis();
  const bool isConnected = connected.load();
  if (isConnected != wasConnected) {
    wasConnected = isConnected;
    log(isConnected ? "INFO Phone connected" : "INFO Phone disconnected");
  }
  if (helloConnection != connection.load()) {
    helloConnection = connection.load();
    helloSeen = false;
    alertWaiting = false;
  }
  if (isConnected && helloPending.exchange(false)) {
    helloSeen = true;
    sendStatus(now);
  }
  if (alertWaiting) alertWaiting = isConnected && !notify(pendingAlert, sizeof(pendingAlert));
  if (isConnected && helloSeen && now - lastStatusAt >= statusEveryMs) sendStatus(now);
  collect(now);
  transfer(now);
}

}  // namespace phone
