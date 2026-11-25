#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

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
