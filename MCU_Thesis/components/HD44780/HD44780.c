#include "HD44780.h"

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include "driver/i2c.h"
#include "esp_err.h"



uint8_t _displaycontrol = LCD_DISPLAY | LCD_CURSOR_OFF | LCD_BLINK_OFF;

static void LCD_writeNibble(uint8_t nibble, uint8_t mode);
static void LCD_writeByte(uint8_t data, uint8_t mode);
static void LCD_pulseEnable(uint8_t nibble);

static char tag[] = "LCD Driver";
static i2c_port_t g_i2c_port = -1;

esp_err_t lcd_init(i2c_port_t i2c_port)
{
    g_i2c_port = i2c_port;

    vTaskDelay(100 / portTICK_RATE_MS);
    //  Reset LCD
    LCD_writeNibble(LCD_FUNCTION_RESET, LCD_COMMAND);
    vTaskDelay(10 / portTICK_RATE_MS);
    LCD_writeNibble(LCD_FUNCTION_RESET, LCD_COMMAND);
    ets_delay_us(200);
    LCD_writeNibble(LCD_FUNCTION_RESET, LCD_COMMAND);
    LCD_writeNibble(LCD_FUNCTION_SET_4BIT, LCD_COMMAND);
    ets_delay_us(80);

    // --Busy flag now available--
    // Function set instruction
    LCD_writeByte(LCD_FUNCTION_SET_4BIT, LCD_COMMAND);
    ets_delay_us(80);

    // Clear display intruction
    LCD_writeByte(LCD_ENTRY_MODE, LCD_COMMAND);
    ets_delay_us(80);
    LCD_writeByte(LCD_DISPLAY_ON, LCD_COMMAND);
    return ESP_OK;
}

void LCD_setCursor(uint8_t col, uint8_t row)
{
    if (row > 20 - 1)
    {
        ESP_LOGE(
            tag,
            "Cannot write to row %d. Please select a row in the range (0, %d)",
            row, 20 - 1);
        row = 20 - 1;
    }
    uint8_t row_offsets[] = {LCD_LINEONE, LCD_LINETWO, LCD_LINETHREE,
                             LCD_LINEFOUR};
    LCD_writeByte(LCD_SET_DDRAM_ADDR | (col + row_offsets[row]), LCD_COMMAND);
}

void LCD_writeChar(char c)
{
    LCD_writeByte(c, LCD_WRITE); // Write data to DDRAM
}

void LCD_writeStr(char *str)
{
    while (*str)
    {
        LCD_writeChar(*str++);
    }
}

void LCD_home(void)
{
    LCD_writeByte(LCD_HOME, LCD_COMMAND);
    vTaskDelay(2 / portTICK_RATE_MS); // This command takes a while to complete
}

void LCD_clearScreen(void)
{
    LCD_writeByte(LCD_CLEAR, LCD_COMMAND);
    vTaskDelay(2 / portTICK_RATE_MS); // This command takes a while to complete
}

static void LCD_writeNibble(uint8_t nibble, uint8_t mode)
{
    uint8_t data = (nibble & 0xF0) | mode | LCD_BACKLIGHT;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(
        i2c_master_write_byte(cmd, (LCD_ADDR << 1) | I2C_MASTER_WRITE, 1));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, data, 1));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(
        i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);

    LCD_pulseEnable(data);
}

static void LCD_writeByte(uint8_t data, uint8_t mode)
{
    LCD_writeNibble(data & 0xF0, mode);
    LCD_writeNibble((data << 4) & 0xF0, mode);
}

void command(uint8_t data)
{
    LCD_writeByte(data, 0);
}

static void LCD_pulseEnable(uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(
        i2c_master_write_byte(cmd, (LCD_ADDR << 1) | I2C_MASTER_WRITE, 1));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, data | LCD_ENABLE, 1));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(
        i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);
    ets_delay_us(1);

    cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(
        i2c_master_write_byte(cmd, (LCD_ADDR << 1) | I2C_MASTER_WRITE, 1));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (data & ~LCD_ENABLE), 1));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(
        i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);
    ets_delay_us(500);
}

void noCursor()
{
    _displaycontrol &= ~LCD_CURSORON;
    command(LCD_BACKLIGHT | _displaycontrol);
}

void cursor()
{
    _displaycontrol |= LCD_CURSORON;
    command(LCD_BACKLIGHT | _displaycontrol);
}

void noBlink()
{
    _displaycontrol &= ~LCD_BLINKON;
    command(LCD_BACKLIGHT | _displaycontrol);
}

void blink()
{
    _displaycontrol |= LCD_BLINKON;
    command(LCD_BACKLIGHT | _displaycontrol);
}