// =============================================================================
// Front Module firmware v1.22: handlebar OLED. Shows the phone's navigation
// screens (forwarded by the Main Module) and, when there is no phone, the
// camera JPEGs the Main Module broadcasts.
// =============================================================================
//
// Board:      ESP32 DevKit V1. In the Arduino IDE choose "ESP32 Dev Module"
//             (FQBN esp32:esp32:esp32), upload speed 115200.
// Core:       esp32 by Espressif, 3.x (compile-checked with 3.3.8).
// Libraries:  Adafruit SSD1306 (2.5.17), Adafruit GFX Library (2.0.0),
//             Adafruit BusIO (1.17.4), TJpg_Decoder (1.1.0, by Bodmer).
//
// What it does (details in SPECIFICATION.md in this folder):
//   - Listens to the Main Module only (relayMac D4:8C:49:FA:75:48) over
//     ESP-NOW, channel 6.
//   - v1.21: phone screens. Packets starting with 0x10 are parts of one
//     compressed 128x64 screen: [0x10][frame#][index][count][flags][data].
//     When every part of a frame is in: join, PackBits-decode to exactly
//     1024 bytes, undo the row XOR, draw. flags bit 0 = flash (invert every
//     125 ms). Format: Phone App/SPECIFICATION.md section 3.5.
//   - v1.21: no screen for 10 s after the phone was seen -> "NO PHONE".
//   - JPEGs (236-byte packets {token, offset, total, 224 bytes}): decoded at
//     1/4 scale and drawn, then acked {0x44495350, token, size, status}. While
//     phone screens are arriving (within 10 s) photos are acked with status 6
//     and not drawn, so a close-pass photo burst can't cover the directions.
//   - Still accepts the USB test picture ("OLED" + 64 + 1024 bytes, 115200
//     baud) from Tools/OLED Image Test/.
//
// v1.22: everything is drawn rotated 180 degrees (screenRotation = 2),
// because the OLED is mounted upside down on the bike.
//
// Settings at a glance: OLED SSD1306 128x64 at I2C 0x3C, SDA GPIO15,
// SCL GPIO22; ESP-NOW channel 6; JPEG max 128 KB; a part-received JPEG is
// dropped after 1.8 s; phoneTimeoutMs = 10000; flashMs = 125.
//
// Provenance: the installed "BikeNav OLED v1.19" receiver (team archive
// BikeNav_Installed_Firmware_2026-11-25, Front/Receiver/Receiver.ino). Every
// v1.21 change is marked "v1.21:"; the screen decoder is new code between
// the BEGIN/END SCREEN DECODER lines.
// =============================================================================
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TJpg_Decoder.h>
#include <cstddef>
#include <cstdint>
#include <cstring>

// ==========================================================================
// v1.21: phone screen decoder. Plain C++ (no Arduino calls) so
// Tools/Phone Link Test/ can compile it on a computer and check it against
// every screen the phone app draws. Keep it that way, and keep the
// BEGIN/END marker lines.
// ==========================================================================
// BEGIN SCREEN DECODER
namespace screen {

constexpr uint8_t packetType = 0x10;    // [0x10][frame#][index][count][flags][data...]
constexpr size_t headerBytes = 5;
constexpr size_t maxPacket = 250;       // ESP-NOW limit; the phone sends <= 185
constexpr size_t maxChunks = 96;        // MTU 23 gives 15-byte chunks: <= 69 per screen
constexpr size_t frameBytes = 1024;     // 128 x 64, row by row, MSB = leftmost pixel
constexpr size_t rowBytes = 16;
constexpr size_t maxPacked = 2048;      // PackBits of 1024 bytes is <= 1032

// PackBits: h 0-127 copy h+1 bytes, h 129-255 repeat the next byte 257-h
// times, 128 skip. Returns false unless the output is exactly `want` bytes.
inline bool unpackBits(const uint8_t *in, size_t n, uint8_t *out, size_t want) {
  size_t i = 0, o = 0;
  while (i < n) {
    const uint8_t h = in[i++];
    if (h < 128) {
      const size_t count = size_t(h) + 1;
      if (i + count > n || o + count > want) return false;
      std::memcpy(out + o, in + i, count);
      i += count;
      o += count;
    } else if (h > 128) {
      const size_t count = 257 - size_t(h);
      if (i >= n || o + count > want) return false;
      std::memset(out + o, in[i++], count);
      o += count;
    }
  }
  return o == want;
}

// Undo the phone's row XOR (each row was XORed with the row above).
inline void unxorRows(uint8_t *frame) {
  for (size_t i = rowBytes; i < frameBytes; ++i) frame[i] ^= frame[i - rowBytes];
}

// Collects the parts of one screen, in any order. Same rules as the app's
// DisplayAssembler: a different frame number or part count starts over.
class Assembler {
 public:
  // Returns true when `packet` completed a screen; it is then in `frame`
  // (1024 bytes, ready for drawBitmap) and `flash` says whether to flash.
  bool add(const uint8_t *packet, size_t length, uint8_t *frame, bool &flash) {
    if (length < headerBytes || length > maxPacket || packet[0] != packetType) return false;
    const uint8_t number = packet[1], index = packet[2], count = packet[3];
    if (count == 0 || count > maxChunks || index >= count) return false;
    if (!active_ || number != number_ || count != count_) {
      active_ = true;
      number_ = number;
      count_ = count;
      received_ = 0;
      std::memset(have_, 0, sizeof(have_));
    }
    flash_ = packet[4] & 1;
    if (!have_[index]) {
      have_[index] = true;
      ++received_;
    }
    length_[index] = static_cast<uint8_t>(length - headerBytes);
    std::memcpy(data_[index], packet + headerBytes, length - headerBytes);
    if (received_ < count_) return false;

    active_ = false;  // finished (or broken): the next packet starts over
    size_t packed = 0;
    for (size_t c = 0; c < count_; ++c) {
      if (packed + length_[c] > maxPacked) return false;
      std::memcpy(joined_ + packed, data_[c], length_[c]);
      packed += length_[c];
    }
    if (!unpackBits(joined_, packed, frame, frameBytes)) return false;
    unxorRows(frame);
    flash = flash_;
    return true;
  }

 private:
  bool active_ = false;
  bool flash_ = false;
  uint8_t number_ = 0, count_ = 0;
  size_t received_ = 0;
  bool have_[maxChunks] = {};
  uint8_t length_[maxChunks] = {};
  uint8_t data_[maxChunks][maxPacket - headerBytes];
  uint8_t joined_[maxPacked];
};

}  // namespace screen
// END SCREEN DECODER

Adafruit_SSD1306 display(128,64,&Wire,-1);
// v1.22: 2 = rotate 180 degrees (the OLED is mounted upside down). 0 = as v1.21.
constexpr uint8_t screenRotation=2;
constexpr uint32_t maxJpeg=128*1024, chunkSize=224;
const uint8_t relayMac[]={0xd4,0x8c,0x49,0xfa,0x75,0x48};
struct Packet {uint32_t token,offset,total; uint8_t bytes[chunkSize];};
struct Ack {uint32_t magic,token,size,status;};
QueueHandle_t packets;
uint8_t* jpeg=nullptr;
uint8_t seen[(maxJpeg+chunkSize-1)/chunkSize];
uint32_t token=0,total=0,received=0,lastPacket=0;
bool displayReady=false, radioReady=false;
uint8_t bitmap[1024]; int n=0;

// v1.21: phone screens.
struct ScreenPacket {uint8_t length; uint8_t bytes[screen::maxPacket];};
QueueHandle_t screenPackets;
screen::Assembler assembler;
uint8_t phoneFrame[screen::frameBytes];
constexpr uint32_t phoneTimeoutMs=10000, flashMs=125;
bool phoneSeen=false, noPhoneShown=false, flashing=false, inverted=false;
uint32_t lastScreenAt=0, lastFlipAt=0;
bool phoneActive(){return phoneSeen && millis()-lastScreenAt<phoneTimeoutMs;}

void ack(uint32_t t,uint32_t size,uint32_t status){
  Ack a={0x44495350,t,size,status};
  esp_err_t result=esp_now_send(relayMac,(uint8_t*)&a,sizeof(a));
  Serial.printf("ACK %lu %lu %lu queue=%d\n",(unsigned long)t,(unsigned long)size,(unsigned long)status,(int)result);
}
// v1.21: also accepts the phone's 0x10 screen packets (any size except a
// JPEG packet's 236 bytes). Was: only 236-byte packets.
void onPacket(const esp_now_recv_info_t* info,const uint8_t* data,int len){
  if(memcmp(info->src_addr,relayMac,6))return;
  if(len==(int)sizeof(Packet)){Packet p; memcpy(&p,data,sizeof(p)); xQueueSend(packets,&p,0); return;}
  if(screenPackets&&len>=(int)screen::headerBytes&&len<=(int)screen::maxPacket&&data[0]==screen::packetType){
    ScreenPacket s; s.length=(uint8_t)len; memcpy(s.bytes,data,len); xQueueSend(screenPackets,&s,0);
  }
}
bool drawBlock(int16_t x,int16_t y,uint16_t w,uint16_t h,uint16_t* pixels){
  for(unsigned j=0;j<h;j++)for(unsigned i=0;i<w;i++){
    int xx=x+i+24, yy=y+j+2;
    if(xx<0||xx>=128||yy<0||yy>=64)continue;
    uint16_t c=pixels[j*w+i]; unsigned r=(c>>11)&31,g=(c>>5)&63,b=c&31;
    if( 299*r*2+587*g+114*b*2 > 31500)display.drawPixel(xx,yy,SSD1306_WHITE);
  } return true;
}
void resetFrame(){free(jpeg);jpeg=nullptr;token=total=received=0;}
void process(const Packet& p){
  if(!p.token||!p.total||p.total>maxJpeg||p.offset>=p.total||p.offset%chunkSize)return;
  if(!jpeg||token!=p.token){
    if(jpeg){ack(token,total,2);resetFrame();}
    token=p.token;total=p.total;received=0;lastPacket=millis();memset(seen,0,sizeof(seen));
    jpeg=(uint8_t*)malloc(total); if(!jpeg){ack(token,total,3);resetFrame();return;}
    Serial.printf("BEGIN %lu %lu\n",(unsigned long)token,(unsigned long)total);
  }
  if(total!=p.total)return;
  lastPacket=millis(); uint32_t index=p.offset/chunkSize;
  if(!seen[index]){uint32_t count=min(chunkSize,total-p.offset);memcpy(jpeg+p.offset,p.bytes,count);seen[index]=1;received+=count;}
  if(received==total){
    bool valid=total>=4&&jpeg[0]==255&&jpeg[1]==216&&jpeg[total-2]==255&&jpeg[total-1]==217;
    uint32_t status=4;
    // v1.21: phone screens have priority; a photo is not drawn (status 6).
    if(phoneActive())status=6;
    else if(valid&&displayReady){display.invertDisplay(false);inverted=false;display.clearDisplay(); if(TJpgDec.drawJpg(0,0,jpeg,total)==0){display.display();status=1;}}
    if(!displayReady)status=5;
    ack(token,total,status);resetFrame();
  }
}
// v1.21: draw one phone screen.
void showPhoneFrame(bool flash){
  lastScreenAt=millis(); phoneSeen=true; noPhoneShown=false;
  if(!displayReady)return;
  display.clearDisplay(); display.drawBitmap(0,0,phoneFrame,128,64,SSD1306_WHITE); display.display();
  flashing=flash;
  if(!flash&&inverted){display.invertDisplay(false);inverted=false;}
  if(flash&&!inverted){display.invertDisplay(true);inverted=true;lastFlipAt=millis();}
}
// v1.21: flashing, and "NO PHONE" when the phone's screens stop.
void servicePhoneScreen(){
  if(!displayReady||!phoneSeen)return;
  if(flashing&&phoneActive()&&millis()-lastFlipAt>=flashMs){inverted=!inverted;display.invertDisplay(inverted);lastFlipAt=millis();}
  if(!phoneActive()&&!noPhoneShown){
    noPhoneShown=true; flashing=false;
    if(inverted){display.invertDisplay(false);inverted=false;}
    display.clearDisplay(); display.setTextColor(SSD1306_WHITE);
    display.setTextSize(2); display.setCursor(16,16); display.println("NO PHONE");
    display.setTextSize(1); display.setCursor(1,44); display.println("Open the Bike Nav app");
    display.display();
    Serial.println("INFO NO PHONE");
  }
}
void setup(){
  Serial.begin(115200);Serial.setTimeout(3000);Wire.begin(15,22);
  displayReady=display.begin(SSD1306_SWITCHCAPVCC,0x3C,false,false);
  display.setRotation(screenRotation);  // v1.22
  if(displayReady){display.clearDisplay();display.setTextColor(SSD1306_WHITE);display.setCursor(0,0);display.println("Wireless receiver");display.display();}
  TJpgDec.setJpgScale(4);TJpgDec.setSwapBytes(false);TJpgDec.setCallback(drawBlock);
  packets=xQueueCreate(32,sizeof(Packet));
  screenPackets=xQueueCreate(32,sizeof(ScreenPacket));  // v1.21
  WiFi.mode(WIFI_STA);WiFi.setSleep(false);
  if(packets&&esp_wifi_set_channel(6,WIFI_SECOND_CHAN_NONE)==ESP_OK&&esp_now_init()==ESP_OK){
    esp_now_peer_info_t peer={};memcpy(peer.peer_addr,relayMac,6);peer.channel=6;peer.ifidx=WIFI_IF_STA;
    radioReady=esp_now_add_peer(&peer)==ESP_OK&&esp_now_register_recv_cb(onPacket)==ESP_OK;
  }
  Serial.printf("READY display=%d radio=%d MAC=%s\n",displayReady,radioReady,WiFi.macAddress().c_str());
  Serial.println("INFO PHONE SCREENS 1.22");  // v1.22: was 1.21
}
void loop(){
  Packet p;for(unsigned i=0;i<32&&packets&&xQueueReceive(packets,&p,0)==pdTRUE;i++)process(p);
  if(jpeg&&millis()-lastPacket>1800){ack(token,total,2);resetFrame();}
  // v1.21: phone screens.
  ScreenPacket s; bool flash;
  for(unsigned i=0;i<32&&screenPackets&&xQueueReceive(screenPackets,&s,0)==pdTRUE;i++)
    if(assembler.add(s.bytes,s.length,phoneFrame,flash))showPhoneFrame(flash);
  servicePhoneScreen();
  const char *h="OLED";
  while(Serial.available()) {char c=Serial.read();n=(c==h[n])?n+1:(c=='O'?1:0);if(n==4){n=0;uint8_t height;if(Serial.readBytes(&height,1)!=1||height!=64){Serial.println("HEIGHT_ERROR");continue;}if(Serial.readBytes(bitmap,sizeof(bitmap))!=sizeof(bitmap)){Serial.println("TRANSFER_ERROR");continue;}if(displayReady){display.clearDisplay();display.drawBitmap(0,0,bitmap,128,64,SSD1306_WHITE);display.display();Serial.println("DISPLAYED");}else Serial.println("DISPLAY_ERROR");}}
  delay(1);
}
