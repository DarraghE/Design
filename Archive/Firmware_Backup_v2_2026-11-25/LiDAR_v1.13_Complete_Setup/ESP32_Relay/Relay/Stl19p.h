#pragma once
#include "Detection.h"
#include <cstring>

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
