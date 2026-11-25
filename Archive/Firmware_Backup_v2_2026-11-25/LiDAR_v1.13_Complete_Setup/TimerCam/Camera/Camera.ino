#include <Arduino.h>
#include <WiFi.h>
#include <esp_camera.h>
#include <esp_now.h>
#include <esp_wifi.h>

constexpr uint8_t radioChannel = 6;
constexpr uint32_t idleTimeoutMs = 2000;
constexpr size_t chunkSize = 224;
constexpr size_t maxJpegSize = 128 * 1024;

struct Request {
  uint32_t token;
  uint32_t offset;
};

struct Reply {
  Request request;
  uint32_t total;
  uint8_t bytes[chunkSize];
};

struct Incoming {
  Request request;
  uint8_t sender[6];
};

static_assert(sizeof(Request) == 8 && sizeof(Reply) == 236,
              "Packet layout must match the existing relay");

QueueHandle_t requests;
bool cameraReady;
bool radioReady;
camera_fb_t *photo;
uint8_t requester[6];
uint32_t activeToken;
uint32_t lastRequest;
bool transferFinished;

void receiveRequest(const esp_now_recv_info_t *info, const uint8_t *data, int size) {
  if (size != sizeof(Request)) return;
  Incoming incoming;
  memcpy(&incoming.request, data, sizeof(Request));
  memcpy(incoming.sender, info->src_addr, sizeof(incoming.sender));
  if (incoming.request.token && incoming.request.offset < maxJpegSize &&
      incoming.request.offset % chunkSize == 0) {
    xQueueSend(requests, &incoming, 0);
  }
}

void releaseTransfer() {
  if (photo) esp_camera_fb_return(photo);
  if (activeToken) esp_now_del_peer(requester);
  photo = nullptr;
  activeToken = 0;
  transferFinished = false;
}

void serveRequest(const Incoming &incoming);

void serveRequest(const Incoming &incoming) {
  const Request &request = incoming.request;
  const bool sameTransfer = request.token == activeToken &&
      memcmp(requester, incoming.sender, sizeof(requester)) == 0;

  if (!sameTransfer) {
    if (request.offset || (activeToken && !transferFinished)) return;
    releaseTransfer();

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, incoming.sender, sizeof(requester));
    peer.channel = radioChannel;
    peer.ifidx = WIFI_IF_STA;
    if (esp_now_add_peer(&peer) != ESP_OK) return;

    memcpy(requester, incoming.sender, sizeof(requester));
    activeToken = request.token;
    if (cameraReady) {
      // Discard the buffered frame so the reply shows a fresh capture.
      camera_fb_t *old = esp_camera_fb_get();
      if (old) esp_camera_fb_return(old);
      photo = esp_camera_fb_get();
      if (photo && (!photo->len || photo->len > maxJpegSize)) {
        esp_camera_fb_return(photo);
        photo = nullptr;
      }
    }
  }

  if (request.offset && (!photo || request.offset >= photo->len)) return;
  lastRequest = millis();

  Reply reply = {};
  reply.request = request;
  reply.total = photo ? photo->len : UINT32_MAX;
  if (photo) {
    const size_t length = min<size_t>(chunkSize, photo->len - request.offset);
    memcpy(reply.bytes, photo->buf + request.offset, length);
    if (request.offset + length == photo->len) transferFinished = true;
  } else {
    transferFinished = true;
  }

  // Retain the frame briefly so a repeated chunk request returns the same bytes.
  esp_now_send(requester, reinterpret_cast<const uint8_t *>(&reply), sizeof(reply));
}

void setup() {
  pinMode(33, OUTPUT);  // Timer Camera X battery power hold.
  digitalWrite(33, HIGH);
  Serial.begin(115200);
  camera_config_t camera = {};
  // OV3660 pin mapping for the M5Stack Timer Camera X.
  camera.pin_d0 = 32; camera.pin_d1 = 35;
  camera.pin_d2 = 34; camera.pin_d3 = 5;
  camera.pin_d4 = 39; camera.pin_d5 = 18;
  camera.pin_d6 = 36; camera.pin_d7 = 19;
  camera.pin_xclk = 27; camera.pin_pclk = 21;
  camera.pin_vsync = 22; camera.pin_href = 26;
  camera.pin_sccb_sda = 25; camera.pin_sccb_scl = 23;
  camera.pin_pwdn = -1; camera.pin_reset = 15;
  camera.ledc_timer = LEDC_TIMER_0;
  camera.ledc_channel = LEDC_CHANNEL_0;
  camera.xclk_freq_hz = 20000000;
  camera.pixel_format = PIXFORMAT_JPEG;
  camera.frame_size = FRAMESIZE_QVGA;  // 320 × 240 keeps USB transfer quick.
  camera.jpeg_quality = 12;
  camera.fb_count = 1;
  camera.fb_location = CAMERA_FB_IN_DRAM;
  camera.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  cameraReady = esp_camera_init(&camera) == ESP_OK;
  if (!cameraReady) Serial.println("Camera initialization failed");

  requests = xQueueCreate(8, sizeof(Incoming));
  if (requests && WiFi.mode(WIFI_STA)) {
    WiFi.setSleep(false);
    radioReady = esp_wifi_set_channel(radioChannel, WIFI_SECOND_CHAN_NONE) == ESP_OK &&
                 esp_now_init() == ESP_OK &&
                 esp_now_register_recv_cb(receiveRequest) == ESP_OK;
  }
  Serial.println(radioReady ? "ESP-NOW camera ready on channel 6" : "ESP-NOW initialization failed");
}

void loop() {
  if (radioReady) {
    if (activeToken && millis() - lastRequest >= idleTimeoutMs) releaseTransfer();
    Incoming incoming;
    if (xQueueReceive(requests, &incoming, pdMS_TO_TICKS(10)) == pdTRUE) {
      serveRequest(incoming);
    }
  } else {
    delay(100);
  }
}
