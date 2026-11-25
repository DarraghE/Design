#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
Adafruit_SSD1306 display(128,64,&Wire,-1);
uint8_t pixels[1024];
int n = 0;
void setup();
void loop();
void setup(){ Serial.begin(115200); Serial.setTimeout(3000); Wire.begin(21,22); if(!display.begin(SSD1306_SWITCHCAPVCC,0x3C)){Serial.println("DISPLAY_ERROR");while(true)delay(1000);} display.clearDisplay();display.setTextColor(SSD1306_WHITE);display.setCursor(0,0);display.println("Receiver ready");display.display();Serial.println("READY"); }
void loop() {
  const char *h="OLED"; while(Serial.available()){char c=Serial.read(); n=(c==h[n])?n+1:(c=='O'?1:0);if(n==4){n=0;uint8_t height;if(Serial.readBytes(&height,1)!=1||height!=64){Serial.println("HEIGHT_ERROR");continue;}if(Serial.readBytes(pixels,sizeof(pixels))!=sizeof(pixels)){Serial.println("TRANSFER_ERROR");continue;}display.clearDisplay();display.drawBitmap(0,0,pixels,128,64,SSD1306_WHITE);display.display();Serial.println("DISPLAYED");}}}
