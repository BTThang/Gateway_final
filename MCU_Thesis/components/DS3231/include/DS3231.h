#ifndef DS3231_H_
#define DS3231_H_
#ifdef __cplusplus
extern "C" {
#endif
#include <stdbool.h>
#include <time.h>

#include "driver/i2c.h"

#define DS3231_ADDR               0x68
#define DS3231_STAT_OSCILLATOR    0x80
#define DS3231_STAT_32KHZ         0x08
#define DS3231_STAT_BUSY          0x04
#define DS3231_STAT_ALARM_2       0x02
#define DS3231_STAT_ALARM_1       0x01

#define DS3231_CTRL_OSCILLATOR    0x80
#define DS3231_CTRL_SQUAREWAVE_BB 0x40
#define DS3231_CTRL_TEMPCONV      0x20
#define DS3231_CTRL_ALARM_INTS    0x04
#define DS3231_CTRL_ALARM2_INT    0x02
#define DS3231_CTRL_ALARM1_INT    0x01

#define DS3231_ALARM_WDAY         0x40
#define DS3231_ALARM_NOTSET       0x80

#define DS3231_ADDR_TIME          0x00
#define DS3231_ADDR_ALARM1        0x07
#define DS3231_ADDR_ALARM2        0x0b
#define DS3231_ADDR_CONTROL       0x0e
#define DS3231_ADDR_STATUS        0x0f
#define DS3231_ADDR_AGING         0x10
#define DS3231_ADDR_TEMP          0x11

#define DS3231_12HOUR_FLAG        0x40
#define DS3231_12HOUR_MASK        0x1f
#define DS3231_PM_FLAG            0x20
#define DS3231_MONTH_MASK         0x1f

uint8_t bcd2dec(uint8_t val);
uint8_t dec2bcd(uint8_t val);

esp_err_t i2c_ds3231_read(const void *outdata, size_t out_size, void *in_data,
                          size_t in_size);
esp_err_t i2c_ds3231_write(const void *out_reg, size_t out_reg_size,
                           const void *out_data, size_t out_size);

esp_err_t ds3231_get_time(struct tm *time);
esp_err_t ds3231_get_temp_float(float *temp);
esp_err_t ds3231_get_temp_interger(int8_t *temp);
esp_err_t ds3231_get_raw_temp(int16_t *temp);
esp_err_t ds3231_set_time(struct tm *time);
esp_err_t ds3231_init(i2c_port_t i2c_port);
esp_err_t i2c_ds3231_read_reg(uint8_t reg, void *in_data, size_t in_size);
esp_err_t i2c_ds3231_write_reg(uint8_t reg, const void *out_data,
                               size_t out_size);

#ifdef __cplusplus
}
#endif
#endif /* DS3231 */
