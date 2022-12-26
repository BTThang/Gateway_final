#ifndef HD44780_H_
#define HD44780_H_
#ifdef __cplusplus
extern "C" {
#endif

#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>

#define LCD_ADDR 0x27
// LCD module defines
#define LCD_LINEONE 0x00   // start of line 1
#define LCD_LINETWO 0x40   // start of line 2
#define LCD_LINETHREE 0x14 // start of line 3
#define LCD_LINEFOUR 0x54  // start of line 4

#define LCD_BACKLIGHT 0x08
#define LCD_DISPLAY_CONTROL 0x08
#define LCD_ENABLE 0x04
#define LCD_COMMAND 0x00
#define LCD_WRITE 0x01

#define LCD_SET_DDRAM_ADDR 0x80
#define LCD_READ_BF 0x40

// commands
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
// #define LCD_SETCGRAMADDR 0x40
// #define LCD_SETDDRAMADDR 0x80

// flags for display on/off control
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

#define LCD_BLINK_OFF 0x00
#define LCD_CURSOR_OFF 0x00 
#define LCD_DISPLAY 0x04 

// LCD instructions
#define LCD_CLEAR 0x01             // replace all characters with ASCII 'space'
#define LCD_HOME 0x02              // return cursor to first position on first line
#define LCD_ENTRY_MODE 0x06        // shift cursor from left to right on read/write
#define LCD_DISPLAY_OFF 0x08       // turn display off
#define LCD_DISPLAY_ON 0x0C        // display on, cursor off, don't blink character
#define LCD_FUNCTION_RESET 0x30    // reset the LCD
#define LCD_FUNCTION_SET_4BIT 0x28 // 4-bit data, 2-line display, 5 x 7 font
#define LCD_SET_CURSOR 0x80        // set cursor position


// LCD Delay times
#define LCD_PRE_PULSE_DELAY_US 1000 /*!< Not sure what this corresponds to in datasheet, but it is necessary */
#define LCD_STD_EXEC_TIME_US 40 /*!< The standard execution time for most instructions */
#define LCD_HOME_EXEC_TIME_US 15200 /*!< Execution time for Return home instruction */
// typedef struct {
//     i2c_port_t port;
//     uint8_t    addr;
// } i2c_dev_t;

esp_err_t lcd_init(i2c_port_t i2c_port);
void LCD_setCursor(uint8_t col, uint8_t row);
void LCD_writeChar(char c);
void LCD_writeStr(char *str);
void LCD_home(void);
void LCD_clearScreen(void);


void cursor();
void noCursor();
void noBlink();
void blink();
#ifdef __cplusplus
}
#endif
#endif /* HD44780 */