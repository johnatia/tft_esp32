#include <Arduino.h>
#include "D:\engazat\EmbeddedSystem\WorkSpaces\VsCode_Arduino\TouchScreen\tft_esp32\include\tft.h"

// #include <SoftwareSerial.h>

//#define LCD_CS A3
//#define LCD_CD A2
//#define LCD_WR A1
//#define LCD_RD A0
//// optional
//#define LCD_RESET A4

// Assign human-readable names to some common 16-bit color values:
#define  BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF


 unsigned long test_FillScreen(void)
{
  unsigned long start = micros();
  tft_FillScreen(RED);
  tft_FillScreen(GREEN);
  tft_FillScreen(BLUE);
  tft_FillScreen(RED);
  tft_FillScreen(GREEN);
  tft_FillScreen(RED);
  tft_FillScreen(GREEN);
  tft_FillScreen(BLUE);
  tft_FillScreen(RED);
  tft_FillScreen(GREEN);

  tft_FillScreen(RED);
  tft_FillScreen(GREEN);
  tft_FillScreen(BLUE);
  tft_FillScreen(RED);
  tft_FillScreen(GREEN);
  tft_FillScreen(RED);
  tft_FillScreen(GREEN);
  tft_FillScreen(BLUE);
  tft_FillScreen(RED);
  tft_FillScreen(GREEN);

  tft_FillScreen(RED);
  tft_FillScreen(GREEN);
  tft_FillScreen(BLUE);
  tft_FillScreen(RED);
  tft_FillScreen(GREEN);
  tft_FillScreen(RED);
  tft_FillScreen(GREEN);
  tft_FillScreen(BLUE);
  tft_FillScreen(RED);
  tft_FillScreen(GREEN);

//  tft_FillScreen(RED);
//  tft_FillScreen(GREEN);
//  tft_FillScreen(BLUE);
//  tft_FillScreen(RED);
//  tft_FillScreen(GREEN);
//  tft_FillScreen(RED);
//  tft_FillScreen(GREEN);
//  tft_FillScreen(BLUE);
//  tft_FillScreen(RED);
//  tft_FillScreen(GREEN);

//  tft_FillScreen(RED);
//  tft_FillScreen(GREEN);
//  tft_FillScreen(BLUE);
//  tft_FillScreen(RED);
//  tft_FillScreen(GREEN);
//  tft_FillScreen(RED);
//  tft_FillScreen(GREEN);
//  tft_FillScreen(BLUE);
//  tft_FillScreen(RED);
//  tft_FillScreen(GREEN);
  return (micros()-start);  
}

void setup() {

//  pinMode(A0,OUTPUT);
//  pinMode(A1,OUTPUT);
//  pinMode(A2,OUTPUT);
//  pinMode(A3,OUTPUT);
//  pinMode(A4,OUTPUT);
  //Serial.begin(9600);
  delay(1000);
  tft_lcd(LCD_CS , LCD_CD ,LCD_WR ,LCD_RD ,LCD_RESET);
  tft_init();
  tft_reset();
  tft_begin();
  Serial.begin(9600);
  delay(2000);
  tft_FillScreen(BLUE);
}

void loop() {
  // tft_FillScreen(BLUE);
  // tft_FillScreen(BLACK);
  // tft_FillScreen(YELLOW);
  // tft_FillScreen(RED);
  // tft_FillScreen(GREEN);
  //for Arduino DUE test 50Frame take 7.91Sec and 30Frames take 4.95Sec
  //for ESP-32 test 50Frame take 2.84Sec and 30Frames take 1.7Sec
Serial.println(test_FillScreen()/1000000.0);
}