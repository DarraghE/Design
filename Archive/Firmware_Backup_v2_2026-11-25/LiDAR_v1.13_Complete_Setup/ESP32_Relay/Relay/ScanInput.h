#pragma once
#include "Detection.h"
#include <cstddef>

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
