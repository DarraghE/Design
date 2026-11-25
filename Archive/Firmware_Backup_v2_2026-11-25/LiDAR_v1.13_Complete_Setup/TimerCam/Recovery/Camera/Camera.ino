#include "esp_camera.h"
#include "Radio.h"

extern bool cameraReady;
QueueHandle_t requests;
bool radioReady;
uint32_t frameToken;
uint32_t frameSize;
uint32_t lastRequest;
uint8_t *frameBytes;

void receiveRequest(const esp_now_recv_info_t *info, const uint8_t *data, int size) {
  if (size != sizeof(Request) || memcmp(info->src_addr, relayMac, 6)) return;
  Request request;
  memcpy(&request, data, sizeof(request));
  xQueueSend(requests, &request, 0);
}

void releasePhoto() {
  free(frameBytes);
  frameBytes = nullptr;
  frameToken = 0;
  frameSize = 0;
}

void sendChunk(const Request &request) {
  if (request.offset == 0 && request.token != frameToken) {
    releasePhoto();
    frameToken = request.token;
    frameSize = invalidSize;
    if (cameraReady) {
      camera_fb_t *old = esp_camera_fb_get();
      if (old) esp_camera_fb_return(old);
      camera_fb_t *photo = esp_camera_fb_get();
      if (photo) {
        if (photo->len && photo->len <= maxJpeg) {
          frameBytes = static_cast<uint8_t *>(malloc(photo->len));
          if (frameBytes) {
            memcpy(frameBytes, photo->buf, photo->len);
            frameSize = photo->len;
          }
        }
        esp_camera_fb_return(photo);
      }
    }
  }

  if (request.token != frameToken) return;
  lastRequest = millis();
  Reply reply = {};
  reply.request = request;
  reply.total = frameSize;
  if (frameBytes && request.offset < frameSize) {
    const size_t length = min(chunkSize, size_t(frameSize - request.offset));
    memcpy(reply.bytes, frameBytes + request.offset, length);
  }
  esp_now_send(relayMac, reinterpret_cast<const uint8_t *>(&reply), sizeof(reply));
}

bool cameraReady;

// The letter P is the Mac's USB capture command.
void takePhoto() {
  if (!cameraReady) {
    Serial.println("ERR Camera failed to initialize");
    return;
  }
  // Release any frame captured before the button was pressed.
  camera_fb_t *old = esp_camera_fb_get();
  if (old) esp_camera_fb_return(old);
  camera_fb_t *photo = esp_camera_fb_get();
  if (!photo) {
    Serial.println("ERR Capture failed");
    return;
  }
  // Send the byte count, followed by the raw JPEG over USB serial.
  Serial.printf("JPEG %u\n", (unsigned)photo->len);
  Serial.write(photo->buf, photo->len);
  esp_camera_fb_return(photo);
}

void setup() {
  Serial.begin(115200);
  pinMode(33, OUTPUT);  // Timer Camera X power hold.
  digitalWrite(33, HIGH);
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
  requests = xQueueCreate(8, sizeof(Request));
  radioReady = requests && startRadio(relayMac, receiveRequest);

}

void loop() {
  if (Serial.available() && Serial.read() == 'P') takePhoto();
  if (radioReady) {
    Request request;
    if (xQueueReceive(requests, &request, 0) == pdTRUE) sendChunk(request);
    if (frameToken && millis() - lastRequest > 8000) releasePhoto();
  }
  delay(1);
}
