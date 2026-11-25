// Host test for the Main Module's phone-protocol code.
//
// run_test.py copies the "BEGIN PHONE WIRE" ... "END PHONE WIRE" block out of
// Main Module/main_module.ino into phone_wire.inc, compiles this file on the
// computer and runs it. It checks the block on its own (CRC, point picking,
// segment copying), then writes every notification the Main Module would
// send for one close pass, at several Bluetooth MTU sizes, to
// firmware_notifications.txt. The phone app's test
// (Phone App/app/test/firmware_vectors_test.dart) decodes that file with the
// app's real decoder.
//
// The package is built here the same way main_module.ino builds it
// (phone::onScan, buildClip, finalize, transfer). If you change those
// functions, change this file to match.
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "phone_wire.inc"

static int failures = 0;
#define CHECK(cond)                                                   \
  do {                                                                \
    if (!(cond)) {                                                    \
      std::fprintf(stderr, "FAILED line %d: %s\n", __LINE__, #cond); \
      ++failures;                                                     \
    }                                                                 \
  } while (0)

static void writeHex(FILE *out, const char *group, const uint8_t *data, size_t length) {
  std::fprintf(out, "%s ", group);
  for (size_t i = 0; i < length; ++i) std::fprintf(out, "%02x", data[i]);
  std::fprintf(out, "\n");
}

// ---- checks of the block on its own -------------------------------------------
static void checkCrc() {
  const uint8_t text[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  CHECK(wire::crc32Update(0, text, 9) == 0xCBF43926u);
  CHECK(wire::crc32Update(wire::crc32Update(0, text, 4), text + 4, 5) == 0xCBF43926u);
  CHECK(wire::crc32Update(0, text, 0) == 0);
}

static void checkChoosePoints() {
  std::vector<uint8_t> classes(450, 0);
  for (int i = 100; i < 130; ++i) classes[i] = 1;
  for (int i = 300; i < 310; ++i) classes[i] = 2;
  uint16_t keep[200];
  const size_t kept = wire::choosePoints(classes.data(), classes.size(), 200, keep);
  CHECK(kept == 200);
  size_t special = 0;
  for (size_t k = 0; k < kept; ++k) {
    special += classes[keep[k]] != 0;
    if (k) CHECK(keep[k] > keep[k - 1]);  // original order, no repeats
  }
  CHECK(special == 40);  // every close/approach point kept
  CHECK(keep[0] == 0);   // normal points spread from the start...
  CHECK(keep[kept - 1] >= 440);  // ...to the end

  std::vector<uint8_t> few(100, 0);
  CHECK(wire::choosePoints(few.data(), few.size(), 200, keep) == 100);  // all kept

  std::vector<uint8_t> manySpecial(400, 1);
  CHECK(wire::choosePoints(manySpecial.data(), manySpecial.size(), 200, keep) == 200);

  CHECK(wire::choosePoints(few.data(), 0, 200, keep) == 0);
}

static void checkHelpers() {
  CHECK(wire::millimetres(0.85f) == 850);
  CHECK(wire::millimetres(-1) == 0);
  CHECK(wire::millimetres(1000) == 0xFFFE);
  CHECK(wire::kmh10(38.0f / 3.6f) == 380);
  CHECK(wire::coordinate(-1.2345f) == -1235 || wire::coordinate(-1.2345f) == -1234);
  CHECK(wire::coordinate(40) == 32767);
  CHECK(wire::coordinate(-40) == -32768);
}

static void checkCopyOut() {
  const uint8_t a[] = {1, 2, 3}, b[] = {4}, c[] = {5, 6, 7, 8};
  const wire::Segment s[] = {{a, 3}, {b, 1}, {c, 4}};
  uint8_t out[8];
  CHECK(wire::totalLength(s, 3) == 8);
  CHECK(wire::copyOut(s, 3, 2, out, 4) == 4 && out[0] == 3 && out[1] == 4 && out[2] == 5 && out[3] == 6);
  CHECK(wire::copyOut(s, 3, 6, out, 8) == 2 && out[0] == 7 && out[1] == 8);
  CHECK(wire::copyOut(s, 3, 8, out, 8) == 0);
  const uint8_t all[] = {1, 2, 3, 4, 5, 6, 7, 8};
  CHECK(wire::crcOf(s, 3) == wire::crc32Update(0, all, 8));
}

// ---- one close pass, as the firmware would send it -------------------------------
// Must match firmware_vectors_test.dart.
constexpr uint16_t eventId = 7;
constexpr uint32_t eventAt = 123456;
constexpr uint16_t transferId = 3;
constexpr size_t clipMaxPoints = 200;  // phone::clipMaxPoints

struct Scan {
  int32_t offsetMs;
  uint8_t flags;
  uint16_t speed10;
  std::vector<uint8_t> bytes;  // points, 5 bytes each
  uint16_t count;
};

// Like phone::onScan: 300 points round the right side, a car's side close by
// (class 1) on scan 2, an approaching car (class 2) on scans 0-1.
static Scan makeScan(int index, int32_t offsetMs) {
  const size_t n = 300;
  std::vector<float> xs(n), ys(n);
  std::vector<uint8_t> classes(n, 0);
  for (size_t i = 0; i < n; ++i) {
    const float angle = (55 + 160.0f * i / (n - 1)) * 0.01745329252f;
    const float r = 3.0f + 0.01f * i;
    xs[i] = r * std::sin(angle);
    ys[i] = r * std::cos(angle);
  }
  if (index == 2)
    for (size_t i = 120; i < 140; ++i) {
      xs[i] = 0.85f;
      ys[i] = -0.5f + 0.05f * (i - 120);
      classes[i] = 1;
    }
  if (index <= 1)
    for (size_t i = 250; i < 256; ++i) classes[i] = 2;
  uint16_t keep[clipMaxPoints];
  Scan scan;
  scan.offsetMs = offsetMs;
  scan.flags = index == 2 ? 1 : (index <= 1 ? 2 : 0);
  scan.speed10 = index <= 1 ? wire::kmh10(38.0f / 3.6f) : wire::unknown16;
  scan.count = static_cast<uint16_t>(wire::choosePoints(classes.data(), n, clipMaxPoints, keep));
  scan.bytes.resize(scan.count * 5);
  for (size_t k = 0; k < scan.count; ++k)
    wire::point(scan.bytes.data() + 5 * k, wire::coordinate(xs[keep[k]]), wire::coordinate(ys[keep[k]]),
                classes[keep[k]]);
  return scan;
}

int main(int argc, char **argv) {
  checkCrc();
  checkChoosePoints();
  checkHelpers();
  checkCopyOut();

  FILE *out = std::fopen(argc > 1 ? argv[1] : "firmware_notifications.txt", "w");
  if (!out) {
    std::perror("output");
    return 2;
  }
  std::fprintf(out, "# Written by Tools/Phone Link Test/host_test.cpp from the PHONE WIRE block of\n");
  std::fprintf(out, "# Main Module/main_module.ino. One notification per line: <group> <hex>.\n");

  uint8_t message[512];
  writeHex(out, "alert", message, wire::alert(message, eventId, wire::millimetres(0.85f), wire::kmh10(38.0f / 3.6f)));
  writeHex(out, "status", message, wire::status(message, true, false, true, 255, 1, 25));

  // Photos: three fake JPEGs (FF D8 ... FF D9) of different sizes.
  std::vector<std::vector<uint8_t>> photos;
  const uint32_t sizes[] = {1000, 2500, 7};
  const uint32_t takenAt[] = {eventAt + 300, eventAt + 800, eventAt + 1300};
  for (int p = 0; p < 3; ++p) {
    std::vector<uint8_t> jpeg(sizes[p]);
    for (size_t i = 0; i < jpeg.size(); ++i) jpeg[i] = static_cast<uint8_t>(i * 7 + p);
    jpeg[0] = 0xFF;
    jpeg[1] = 0xD8;
    jpeg[jpeg.size() - 2] = 0xFF;
    jpeg[jpeg.size() - 1] = 0xD9;
    photos.push_back(jpeg);
  }

  // LiDAR clip, like phone::buildClip.
  const int32_t offsets[] = {-2000, -1000, 0, 700, 1500};
  std::vector<Scan> scans;
  for (int s = 0; s < 5; ++s) scans.push_back(makeScan(s, offsets[s]));
  size_t clipBytes = 4;
  for (const Scan &s : scans) clipBytes += 9 + s.bytes.size();
  std::vector<uint8_t> clip(clipBytes);
  uint8_t *p = clip.data() + wire::clipHeader(clip.data(), static_cast<uint16_t>(scans.size()));
  for (const Scan &s : scans) {
    p += wire::scanHeader(p, s.offsetMs, s.flags, s.speed10, s.count);
    std::memcpy(p, s.bytes.data(), s.bytes.size());
    p += s.bytes.size();
  }
  CHECK(static_cast<size_t>(p - clip.data()) == clipBytes);

  // Header and segments, like phone::finalize.
  uint8_t header[16], photoHeaders[3][8];
  wire::packageHeader(header, eventId, eventAt, wire::millimetres(0.85f), wire::kmh10(38.0f / 3.6f), 3);
  std::vector<wire::Segment> segments = {{header, 16}};
  for (int i = 0; i < 3; ++i) {
    wire::photoHeader(photoHeaders[i], static_cast<int32_t>(takenAt[i] - eventAt), sizes[i]);
    segments.push_back({photoHeaders[i], 8});
    segments.push_back({photos[i].data(), photos[i].size()});
  }
  segments.push_back({clip.data(), clip.size()});
  const size_t total = wire::totalLength(segments.data(), segments.size());
  const uint32_t crc = wire::crcOf(segments.data(), segments.size());

  // Notifications, like phone::transfer, at several MTUs.
  for (unsigned mtu : {23u, 185u, 247u, 517u}) {
    char group[32];
    std::snprintf(group, sizeof(group), "transfer-mtu%u", mtu);
    writeHex(out, group, message, wire::transferStart(message, transferId, wire::transferClosePass, total));
    const size_t largest = std::min<size_t>(mtu - 3, sizeof(message));
    size_t offset = 0;
    while (offset < total) {
      const size_t h = wire::transferDataHeader(message, transferId, static_cast<uint32_t>(offset));
      const size_t n = wire::copyOut(segments.data(), segments.size(), offset, message + h, largest - h);
      CHECK(n > 0 && h + n <= mtu - 3);
      writeHex(out, group, message, h + n);
      offset += n;
    }
    writeHex(out, group, message, wire::transferEnd(message, transferId, crc));
  }
  std::fclose(out);

  if (failures) {
    std::fprintf(stderr, "%d check(s) failed\n", failures);
    return 1;
  }
  std::printf("All host checks passed. Package: %zu bytes, CRC-32 %08x.\n", total, crc);
  return 0;
}
