// =============================================================================
// Front Module firmware: wireless JPEG receiver for the 128x64 OLED
// ("BikeNav OLED v1.19", the source of the installed front board, 25 Nov 2026)
// =============================================================================
//
// Board:      ESP32 DevKit V1. In the Arduino IDE choose "ESP32 Dev Module"
//             (FQBN esp32:esp32:esp32), upload speed 115200.
// Core:       esp32 by Espressif, 3.x (compile-checked with 3.3.8).
// Libraries:  Adafruit SSD1306 (2.5.17), Adafruit GFX Library (2.0.0),
//             Adafruit BusIO (1.17.4), TJpg_Decoder (1.1.0, by Bodmer).
//
// What it does (details in SPECIFICATION.md in this folder):
//   - Receives JPEGs from the Main Module over ESP-NOW (channel 6, only from
//     relayMac D4:8C:49:FA:75:48): 236-byte packets {token, offset, total,
//     224 bytes}, the same layout as the camera's replies.
//   - When a JPEG is complete, decodes it at 1/4 scale, thresholds it to
//     black and white, draws it on the OLED (offset x+24, y+2) and sends an
//     ack {0x44495350 "DISP", token, size, status} back to the Main Module.
//   - Still accepts the USB test picture ("OLED" + 64 + 1024 bytes, 115200
//     baud) from Tools/OLED Image Test/.
//   - Does NOT yet show the phone's navigation screens (0x10 packets from
//     the Main Module are ignored: wrong size). See SPECIFICATION.md section 8.
//
// Settings at a glance: OLED SSD1306 128x64 at I2C 0x3C, SDA GPIO15,
// SCL GPIO22; ESP-NOW channel 6; JPEG max 128 KB; a part-received JPEG is
// dropped after 1.8 s.
//
// Provenance: Front/Receiver/Receiver.ino from the team's
// "BikeNav_Installed_Firmware_2026-11-25" archive, unchanged apart from this
// header comment. On 25 Nov 2026 the front board (MAC 8C:94:DF:6C:BC:80) was
// flashed and its flash hash verified; the archive notes the installed build
// may have been labelled v1.20.
// =============================================================================
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TJpg_Decoder.h>
Adafruit_SSD1306 display(128,64,&Wire,-1);
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
void ack(uint32_t t,uint32_t size,uint32_t status){
  Ack a={0x44495350,t,size,status};
  esp_err_t result=esp_now_send(relayMac,(uint8_t*)&a,sizeof(a));
  Serial.printf("ACK %lu %lu %lu queue=%d\n",(unsigned long)t,(unsigned long)size,(unsigned long)status,(int)result);
}
void onPacket(const esp_now_recv_info_t* info,const uint8_t* data,int len){
  if(len!=sizeof(Packet)||memcmp(info->src_addr,relayMac,6))return;
  Packet p; memcpy(&p,data,sizeof(p)); xQueueSend(packets,&p,0);
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
    if(valid&&displayReady){display.clearDisplay(); if(TJpgDec.drawJpg(0,0,jpeg,total)==0){display.display();status=1;}}
    if(!displayReady)status=5;
    ack(token,total,status);resetFrame();
  }
}
void setup(){
  Serial.begin(115200);Serial.setTimeout(3000);Wire.begin(15,22);
  displayReady=display.begin(SSD1306_SWITCHCAPVCC,0x3C,false,false);
  if(displayReady){display.clearDisplay();display.setTextColor(SSD1306_WHITE);display.setCursor(0,0);display.println("Wireless receiver");display.display();}
  TJpgDec.setJpgScale(4);TJpgDec.setSwapBytes(false);TJpgDec.setCallback(drawBlock);
  packets=xQueueCreate(32,sizeof(Packet));
  WiFi.mode(WIFI_STA);WiFi.setSleep(false);
  if(packets&&esp_wifi_set_channel(6,WIFI_SECOND_CHAN_NONE)==ESP_OK&&esp_now_init()==ESP_OK){
    esp_now_peer_info_t peer={};memcpy(peer.peer_addr,relayMac,6);peer.channel=6;peer.ifidx=WIFI_IF_STA;
    radioReady=esp_now_add_peer(&peer)==ESP_OK&&esp_now_register_recv_cb(onPacket)==ESP_OK;
  }
  Serial.printf("READY display=%d radio=%d MAC=%s\n",displayReady,radioReady,WiFi.macAddress().c_str());
}
void loop(){
  Packet p;for(unsigned i=0;i<32&&packets&&xQueueReceive(packets,&p,0)==pdTRUE;i++)process(p);
  if(jpeg&&millis()-lastPacket>1800){ack(token,total,2);resetFrame();}
  const char *h="OLED";
  while(Serial.available()) {char c=Serial.read();n=(c==h[n])?n+1:(c=='O'?1:0);if(n==4){n=0;uint8_t height;if(Serial.readBytes(&height,1)!=1||height!=64){Serial.println("HEIGHT_ERROR");continue;}if(Serial.readBytes(bitmap,sizeof(bitmap))!=sizeof(bitmap)){Serial.println("TRANSFER_ERROR");continue;}if(displayReady){display.clearDisplay();display.drawBitmap(0,0,bitmap,128,64,SSD1306_WHITE);display.display();Serial.println("DISPLAYED");}else Serial.println("DISPLAY_ERROR");}}
  delay(1);
}
