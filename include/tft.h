#include <Arduino.h>

#ifndef _TFT_H_
#define _TFT_H_
  
//#include "C:\Users\LENOVO\esp\esp-idf\components\hal\esp32\include\hal\gpio_ll.h"
//#include "C:\Users\LENOVO\esp\esp-idf\components\hal\include\hal\gpio_hal.h"
#include "C:\Users\LENOVO\.platformio\packages\framework-arduinoespressif32\tools\sdk\include\soc\soc\gpio_struct.h"
#include "C:\Users\LENOVO\.platformio\packages\framework-arduinoespressif32\cores\esp32\esp32-hal-gpio.h"


  #define setWriteDirInline() { DDRD |=  B11111100; DDRB |=  B00000011; }
  #define write8inline(d) {                          \
    PORTD = (PORTD & B00000011) | ((d) & B11111100); \
    PORTB = (PORTB & B11111100) | ((d) & B00000011); \
    WR_STROBE; }

 
    
/* Common Macros Begin */
#define SET_BIT(VAR,BIT)                      VAR |=  (1 << (BIT) )
#define CLR_BIT(VAR,BIT)                      VAR &= ~(1 << (BIT) )
#define TOG_BIT(VAR,BIT)                      VAR ^=  (1 << (BIT) ) 
#define GET_BIT(VAR,BIT)                      ((VAR >> BIT) & 1 )
/* Common Macros End  */

/* Configuration begin  */
#define LCD_CS 1 
#define LCD_CD 2 //LCD_RS
#define LCD_WR 3
#define LCD_RD 4
// optional
#define LCD_RESET 5


/* ESP-32 */
#define RD_MASK       (1<<4)             //PA1 
#define WR_MASK       (1<<3)             //PA2
#define CD_MASK       (1<<2)             //PA3
#define CS_MASK       (1<<1)             //PA4

/* Configuration End    */

/* Macros Begin */
/* Arduino UNO  */
// #define RD_ACTIVE  *rdPort &=  rdPinUnset
// #define RD_IDLE    *rdPort |=  rdPinSet
// #define WR_ACTIVE  *wrPort &=  wrPinUnset
// #define WR_IDLE    *wrPort |=  wrPinSet
// #define CD_COMMAND *cdPort &=  cdPinUnset
// #define CD_DATA    *cdPort |=  cdPinSet
// #define CS_ACTIVE  *csPort &=  csPinUnset
// #define CS_IDLE    *csPort |=  csPinSet

 /* Arduino DUE */
//#define RD_PORT    PIOA
//#define WR_PORT    PIOA
//#define CD_PORT    PIOA
//#define CS_PORT    PIOA
//#define RD_MASK 0x00010000             //PA16 
//#define WR_MASK 0x01000000             //PA24
//#define CD_MASK 0x00800000             //PA23
//#define CS_MASK 0x00400000             //PA22
//#define RD_ACTIVE  RD_PORT->PIO_CODR |= RD_MASK
//#define RD_IDLE    RD_PORT->PIO_SODR |= RD_MASK
//#define WR_ACTIVE  WR_PORT->PIO_CODR |= WR_MASK
//#define WR_IDLE    WR_PORT->PIO_SODR |= WR_MASK
//#define CD_COMMAND CD_PORT->PIO_CODR |= CD_MASK
//#define CD_DATA    CD_PORT->PIO_SODR |= CD_MASK
//#define CS_ACTIVE  CS_PORT->PIO_CODR |= CS_MASK
//#define CS_IDLE    CS_PORT->PIO_SODR |= CS_MASK
//


/* ESP-32 */
// #define RD_MASK 0x02             //PA1 
// #define WR_MASK 0x04             //PA2
// #define CD_MASK 0x08             //PA3
// #define CS_MASK 0x10             //PA4

#define RD_ACTIVE  GPIO.out_w1tc |= RD_MASK
#define RD_IDLE    GPIO.out_w1ts |= RD_MASK
#define WR_ACTIVE  GPIO.out_w1tc |= WR_MASK
#define WR_IDLE    GPIO.out_w1ts |= WR_MASK
#define CD_COMMAND GPIO.out_w1tc |= CD_MASK
#define CD_DATA    GPIO.out_w1ts |= CD_MASK
#define CS_ACTIVE  GPIO.out_w1tc |= CS_MASK
#define CS_IDLE    GPIO.out_w1ts |= CS_MASK     





#define WR_STROBE { WR_ACTIVE; WR_IDLE; }
/* Macros End  */


/* TFT Specs Begin  */
#define TFTWIDTH   240
#define TFTHEIGHT  320
/* TFT Specs End  */

/* TFT Routines Begin  */
#define ILI9341_SOFTRESET          0x01
#define ILI9341_SLEEPIN            0x10
#define ILI9341_SLEEPOUT           0x11
#define ILI9341_NORMALDISP         0x13
#define ILI9341_INVERTOFF          0x20
#define ILI9341_INVERTON           0x21
#define ILI9341_GAMMASET           0x26
#define ILI9341_DISPLAYOFF         0x28
#define ILI9341_DISPLAYON          0x29
#define ILI9341_COLADDRSET         0x2A
#define ILI9341_PAGEADDRSET        0x2B
#define ILI9341_MEMORYWRITE        0x2C
#define ILI9341_PIXELFORMAT        0x3A
#define ILI9341_FRAMECONTROL       0xB1
#define ILI9341_DISPLAYFUNC        0xB6
#define ILI9341_ENTRYMODE          0xB7
#define ILI9341_POWERCONTROL1      0xC0
#define ILI9341_POWERCONTROL2      0xC1
#define ILI9341_VCOMCONTROL1       0xC5
#define ILI9341_VCOMCONTROL2       0xC7
#define ILI9341_MEMCONTROL         0x36
#define ILI9341_MADCTL             0x36

#define ILI9341_MADCTL_MY  0x80
#define ILI9341_MADCTL_MX  0x40
#define ILI9341_MADCTL_MV  0x20
#define ILI9341_MADCTL_ML  0x10
#define ILI9341_MADCTL_RGB 0x00
#define ILI9341_MADCTL_BGR 0x08
#define ILI9341_MADCTL_MH  0x04
/* TFT Routines End */

void write8(uint8_t value);
void writeRegister8(uint8_t a, uint8_t d);
void writeRegister16(uint16_t a, uint16_t d);
void writeRegister32(uint8_t r, uint32_t d);

void tft_lcd( uint8_t cs, uint8_t cd, uint8_t wr, uint8_t rd, uint8_t reset) ;

void tft_init(void);
void tft_reset(void);
void setAddrWindow(int x1, int y1, int x2, int y2);
void tft_begin(void);
void flood(uint16_t color, uint32_t len);
void tft_FillScreen(uint16_t color);


#endif