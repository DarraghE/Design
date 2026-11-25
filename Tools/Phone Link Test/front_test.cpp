// Host test for the Front Module's phone-screen decoder.
//
// run_test.py copies the "BEGIN SCREEN DECODER" ... "END SCREEN DECODER"
// block out of Front Module/front_module.ino into screen_decoder.inc,
// compiles this file and runs it on the phone app's test screens:
//   front_test <packets.txt> <dart.txt>
// Both files come from `dart run tool/export_screens.dart <dir>` in
// Phone App/navigation: packets.txt has the exact 0x10 packets the phone
// sends for each of 415 screens; dart.txt shows what each must look like.
//
// Every screen is decoded four ways and compared pixel by pixel:
//   1. the packets as the phone sends them (chunks of <= 180 bytes),
//   2. re-cut into 15-byte chunks (Bluetooth MTU 23) in reverse order,
//   3. with every packet sent twice,
//   4. after a broken half-frame with another frame number.
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "screen_decoder.inc"

using Bytes = std::vector<uint8_t>;
static int failures = 0;

static Bytes fromHex(const std::string &hex) {
  Bytes out;
  for (size_t i = 0; i + 1 < hex.size(); i += 2) out.push_back(static_cast<uint8_t>(std::stoi(hex.substr(i, 2), nullptr, 16)));
  return out;
}

// '#'/'.' art (64 lines of 128) -> 1024-byte drawBitmap layout.
static Bytes fromArt(const std::vector<std::string> &rows) {
  Bytes out(1024, 0);
  for (int y = 0; y < 64; ++y)
    for (int x = 0; x < 128; ++x)
      if (rows[y][x] == '#') out[y * 16 + x / 8] |= 0x80 >> (x % 8);
  return out;
}

// Feeds packets; returns the last completed frame (empty if none).
static Bytes decode(screen::Assembler &a, const std::vector<Bytes> &packets, bool *flash = nullptr) {
  Bytes frame(1024), last;
  bool f = false;
  for (const Bytes &p : packets)
    if (a.add(p.data(), p.size(), frame.data(), f)) {
      last = frame;
      if (flash) *flash = f;
    }
  return last;
}

// Re-cuts a frame's packets into `size`-byte chunks with frame number `number`.
static std::vector<Bytes> rechunk(const std::vector<Bytes> &packets, size_t size, uint8_t number) {
  Bytes data;
  for (const Bytes &p : packets) data.insert(data.end(), p.begin() + 5, p.end());
  const uint8_t flags = packets[0][4];
  const size_t count = (data.size() + size - 1) / size;
  std::vector<Bytes> out;
  for (size_t i = 0; i < count; ++i) {
    Bytes p = {0x10, number, static_cast<uint8_t>(i), static_cast<uint8_t>(count), flags};
    p.insert(p.end(), data.begin() + i * size, data.begin() + std::min(data.size(), (i + 1) * size));
    out.push_back(p);
  }
  return out;
}

int main(int argc, char **argv) {
  if (argc < 3) {
    std::fprintf(stderr, "usage: front_test packets.txt dart.txt\n");
    return 2;
  }
  std::map<std::string, std::vector<Bytes>> packets;
  std::vector<std::string> order;
  {
    std::ifstream in(argv[1]);
    std::string line;
    while (std::getline(in, line)) {
      std::istringstream words(line);
      std::string name, hex;
      words >> name;
      if (name.empty()) continue;
      order.push_back(name);
      while (words >> hex) packets[name].push_back(fromHex(hex));
    }
  }
  std::map<std::string, std::pair<Bytes, bool>> expected;
  {
    std::ifstream in(argv[2]);
    std::string line;
    while (std::getline(in, line)) {
      if (line.rfind("== ", 0) != 0) continue;
      std::string name = line.substr(3);
      const bool flash = name.size() > 8 && name.substr(name.size() - 8) == " (flash)";
      if (flash) name = name.substr(0, name.size() - 8);
      std::vector<std::string> rows(64);
      for (auto &r : rows) std::getline(in, r);
      expected[name] = {fromArt(rows), flash};
    }
  }
  if (order.size() < 400 || expected.size() != order.size()) {
    std::fprintf(stderr, "test vectors missing or mismatched (%zu packets, %zu pictures)\n", order.size(), expected.size());
    return 2;
  }

  // Basic checks of the block on its own.
  {
    uint8_t out[8];
    const uint8_t literal[] = {2, 1, 2, 3, 0xFE, 9};  // copy 3, repeat 9 three times
    if (!screen::unpackBits(literal, sizeof(literal), out, 6) || out[2] != 3 || out[5] != 9) ++failures;
    if (screen::unpackBits(literal, sizeof(literal), out, 5)) ++failures;  // wrong size
    const uint8_t truncated[] = {5, 1, 2};
    if (screen::unpackBits(truncated, sizeof(truncated), out, 6)) ++failures;
    screen::Assembler a;
    Bytes frame(1024);
    bool f;
    const uint8_t bad[] = {0x10, 0, 3, 2, 0, 1};  // index >= count
    if (a.add(bad, sizeof(bad), frame.data(), f)) ++failures;
    const uint8_t notScreen[] = {0x11, 0, 0, 1, 0, 1};
    if (a.add(notScreen, sizeof(notScreen), frame.data(), f)) ++failures;
    if (failures) std::fprintf(stderr, "basic decoder checks failed\n");
  }

  size_t checked = 0, flashes = 0;
  uint8_t number = 0;
  for (const std::string &name : order) {
    const auto &want = expected[name];
    const auto &sent = packets[name];
    auto check = [&](const char *how, const Bytes &got, bool flash) {
      if (got != want.first || flash != want.second) {
        if (failures < 10) std::fprintf(stderr, "FAILED %s (%s)\n", name.c_str(), how);
        ++failures;
      }
    };
    screen::Assembler a;
    bool flash = false;
    check("as sent", decode(a, sent, &flash), flash);

    auto small = rechunk(sent, 15, ++number);
    std::reverse(small.begin(), small.end());
    flash = false;
    check("15-byte chunks, reversed", decode(a, small, &flash), flash);

    std::vector<Bytes> twice;
    for (const Bytes &p : rechunk(sent, 180, ++number)) { twice.push_back(p); twice.push_back(p); }
    flash = false;
    check("every packet twice", decode(a, twice, &flash), flash);

    auto broken = rechunk(sent, 15, ++number);
    std::vector<Bytes> mixed(broken.begin(), broken.begin() + broken.size() / 2);  // half a frame...
    for (const Bytes &p : rechunk(sent, 60, ++number)) mixed.push_back(p);       // ...then a full one
    flash = false;
    check("after a broken frame", decode(a, mixed, &flash), flash);

    ++checked;
    flashes += want.second;
  }
  if (failures) {
    std::fprintf(stderr, "%d check(s) failed\n", failures);
    return 1;
  }
  std::printf("Front decoder: all %zu phone screens decode pixel-exact 4 ways (%zu flashing).\n", checked, flashes);
  return 0;
}
