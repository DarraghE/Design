#include "Radio.h"
#include "ScanInput.h"
#include "Stl19p.h"
#include <esp_system.h>
#include <atomic>
#include <Adafruit_NeoPixel.h>

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
