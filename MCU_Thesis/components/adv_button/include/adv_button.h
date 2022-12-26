#ifndef _ADV_BUTTON_H_
#define _ADV_BUTTON_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/gpio.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

    typedef enum
    {
        BUTTON_ACTIVE_LOW,
        BUTTON_ACTIVE_HIGH,
    } button_active_level_t;

    typedef enum
    {
        ADV_BUTTON_EVENT_SINGLE_PRESS,
        ADV_BUTTON_EVENT_DOUBLE_PRESS,
        ADV_BUTTON_EVENT_TRIPPLE_PRESS,
        ADV_BUTTON_EVENT_HOLD_PRESS,
    } adv_button_event_id_t;

    typedef void (*adv_button_callback_t)(adv_button_event_id_t event_id, void *params);
    typedef void (*toggle_callback_fn)(bool high, void *params);

    typedef struct
    {
        button_active_level_t active_level;

        //  time in miliseconds
        uint16_t hold_press_time_ms;
        uint16_t repeat_press_timeout_ms;
        uint16_t max_repeat_press;
        uint32_t task_stack_size;
        UBaseType_t task_prio;
    } adv_button_config_t;

#define ADVANCED_BUTTON_CONFIG_DEFAULT()                                       \
    {                                                                          \
        .active_level = BUTTON_ACTIVE_LOW, .hold_press_time_ms = 1000,         \
        .repeat_press_timeout_ms = 300,                                        \
        .max_repeat_press = 3, .task_stack_size = 1024*4, \
        .task_prio = 10  ,                                                      \
    }
    int adv_button_init(void);
    int adv_button_create(gpio_num_t io_num, adv_button_config_t *config, adv_button_callback_t callback, void *params);

    int adv_button_destroy(const uint8_t gpio_num);
    void read_button(gpio_num_t button);

#ifdef __cplusplus
}
#endif
#endif