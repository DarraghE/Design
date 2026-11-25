// =============================================================================
// Front Module firmware: OLED image receiver (test version)
// =============================================================================
//
// Board:      ESP32 DevKit V1. In the Arduino IDE choose "ESP32 Dev Module"
//             (FQBN esp32:esp32:esp32), upload speed 115200.
// Display:    0.96" 128x64 SSD1306 OLED over I2C, address 0x3C, no reset pin.
// Wiring:     OLED SDA -> GPIO15, OLED SCL (may be labelled SCK) -> GPIO22,
//             VDD -> 3V3, GND -> GND. To use the usual DevKit pins instead,
//             change Wire.begin(15,22) to Wire.begin(21,22).
// Core:       esp32 by Espressif (built with 3.2.x).
// Libraries:  Adafruit GFX Library (seen: 1.13.0), Adafruit SSD1306 (2.5.17),
//             Adafruit BusIO (1.17.4).
//
// What it does (details in SPECIFICATION.md in this folder):
//   Waits on USB serial (115200 baud) for one 1029-byte packet:
//     "OLED" (4 bytes) + height 64 (1 byte) + 1024 bytes of pixels
//     (128x64, row by row, 8 pixels per byte, leftmost pixel in the top bit,
//     1 = lit). It then shows that picture and replies "DISPLAYED".
//   Other replies: READY (started), DISPLAY_ERROR (screen not found),
//   HEIGHT_ERROR, TRANSFER_ERROR (fewer than 1024 bytes within 3 s).
//   Use the tools in Tools/OLED Image Test/ to send a picture.
//
// This is a TEST of the screen, not the finished handlebar display: it does
// not use Bluetooth and does not talk to any other module yet.
//
// Provenance: code below is OLED_GPIO15_22/Receiver/Receiver.ino from the
// firmware backup of 25 Nov 2026, unchanged (only this comment was added).
// This GPIO15 version has NOT been confirmed flashed. The GPIO21 version
// (upload confirmed, screen not confirmed) is in
// Archive/Firmware_Backup_v2_2026-11-25/OLED_Test_v1/.
// =============================================================================

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
Adafruit_SSD1306 display(128,64,&Wire,-1);
uint8_t pixels[1024];
int n = 0;
void setup();
void loop();
void setup(){ Serial.begin(115200); Serial.setTimeout(3000); Wire.begin(15,22); if(!display.begin(SSD1306_SWITCHCAPVCC,0x3C)){Serial.println("DISPLAY_ERROR");while(true)delay(1000);} display.clearDisplay();display.setTextColor(SSD1306_WHITE);display.setCursor(0,0);display.println("Receiver ready");display.display();Serial.println("READY"); }
void loop() {
  const char *h="OLED"; while(Serial.available()){char c=Serial.read(); n=(c==h[n])?n+1:(c=='O'?1:0);if(n==4){n=0;uint8_t height;if(Serial.readBytes(&height,1)!=1||height!=64){Serial.println("HEIGHT_ERROR");continue;}if(Serial.readBytes(pixels,sizeof(pixels))!=sizeof(pixels)){Serial.println("TRANSFER_ERROR");continue;}display.clearDisplay();display.drawBitmap(0,0,pixels,128,64,SSD1306_WHITE);display.display();Serial.println("DISPLAYED");}}}
