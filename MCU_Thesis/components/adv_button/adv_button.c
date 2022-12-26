#include <stdio.h>
#include "adv_button.h"
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

#define MAX_TOGGLE_VALUE 4
#define MIN(a, b) (((b) < (a)) ? (b) : (a))
#define MAX(a, b) (((b) < (a)) ? (a) : (b))

typedef struct _button
{
    gpio_num_t io_num;
    adv_button_config_t config;
    QueueHandle_t queue;
    TaskHandle_t task;
    void *params;
    adv_button_callback_t callback;

    uint8_t press_count;
    int8_t value;
    uint32_t debounce_cnt;
    TimerHandle_t hold_press_timer;
    TimerHandle_t repeat_press_timeout_timer;
    bool last_high;
    struct _button *next;
} adv_button_t;

static TimerHandle_t toggle_timer = NULL;
static bool toggles_initialized = false;
static SemaphoreHandle_t button_lock = NULL;
static adv_button_t *g_adv_button = NULL;

static void button_toggle_callback(bool high, void *params);
static void button_free(adv_button_t *button);

static void button_fire_event(adv_button_t *button)
{
    adv_button_event_id_t event = ADV_BUTTON_EVENT_SINGLE_PRESS;
    switch (button->press_count)
    {
    case 1:
        event = ADV_BUTTON_EVENT_SINGLE_PRESS;
        break;
    case 2:
        event = ADV_BUTTON_EVENT_DOUBLE_PRESS;
        break;
    case 3:
        event = ADV_BUTTON_EVENT_TRIPPLE_PRESS;
        break;
    case 4:
        event = ADV_BUTTON_EVENT_HOLD_PRESS;
        break;
    }
    button->callback(event, button->params);
    button->press_count = 0;
}

static void button_long_press_timer_callback(TimerHandle_t timer)
{
    if (xSemaphoreTake(button_lock, 0) != pdTRUE)
        return;

    adv_button_t *button = (adv_button_t *)pvTimerGetTimerID(timer);
    button->callback(ADV_BUTTON_EVENT_HOLD_PRESS, button->params);
    button->press_count = 0;
    xSemaphoreGive(button_lock);
}

static void button_repeat_press_timeout_timer_callback(TimerHandle_t timer)
{
    if (xSemaphoreTake(button_lock, 0) != pdTRUE)
        return;

    adv_button_t *button = (adv_button_t *)pvTimerGetTimerID(timer);
    button_fire_event(button);
    xSemaphoreGive(button_lock);
}

static void toggle_timer_callback(TimerHandle_t timer)
{
    if (xSemaphoreTake(button_lock, 0) != pdTRUE)
        return;

    adv_button_t *button = g_adv_button;

    while (button)
    {
        if (gpio_get_level(button->io_num) == 1)
        {
            button->value = MIN(button->value + 1, MAX_TOGGLE_VALUE);
            if (button->value == MAX_TOGGLE_VALUE && !button->last_high)
            {
                button->last_high = true;
                button_toggle_callback(true, button);
            }
        }
        else
        {
            button->value = MAX(button->value - 1, 0);
            if (button->value == 0 && button->last_high)
            {
                button->last_high = false;
                button_toggle_callback(false, button);
            }
        }

        button = button->next;
    }

    xSemaphoreGive(button_lock);
}

static void button_toggle_callback(bool high, void *params)
{
    if (!params)
        return;

    adv_button_t *button = (adv_button_t *)params;
    if (high == (button->config.active_level == BUTTON_ACTIVE_HIGH))
    {
        // pressed
        button->press_count++;
        if (button->config.hold_press_time_ms && button->press_count == 1)
            xTimerStart(button->hold_press_timer, 1);
    }
    else
    {
        // released
        if (!button->press_count)
            return;

        if (button->hold_press_timer && xTimerIsTimerActive(button->hold_press_timer))
        {
            xTimerStop(button->hold_press_timer, 1);
        }

        if (button->press_count >= button->config.max_repeat_press || !button->config.repeat_press_timeout_ms)
        {
            if (button->repeat_press_timeout_timer && xTimerIsTimerActive(button->repeat_press_timeout_timer))
            {
                xTimerStop(button->repeat_press_timeout_timer, 1);
            }

            button_fire_event(button);
        }
        else
        {
            xTimerStart(button->repeat_press_timeout_timer, 1);
        }
    }
}

void button_free(adv_button_t *button)
{
    if (button->hold_press_timer)
    {
        xTimerStop(button->hold_press_timer, 1);
        xTimerDelete(button->hold_press_timer, 1);
    }
    if (button->repeat_press_timeout_timer)
    {
        xTimerStop(button->repeat_press_timeout_timer, 1);
        xTimerDelete(button->repeat_press_timeout_timer, 1);
    }
    gpio_reset_pin(button->io_num);
    free(button);
}

void toggle_delete(const uint8_t io_num)
{
    if (!toggles_initialized)
        adv_button_init();

    xSemaphoreTake(button_lock, portMAX_DELAY);

    if (!g_adv_button)
    {
        xSemaphoreGive(button_lock);
        return;
    }

    adv_button_t *button = NULL;
    if (g_adv_button->io_num == io_num)
    {
        button = g_adv_button;
        g_adv_button = g_adv_button->next;
    }
    else
    {
        adv_button_t *temp = g_adv_button;
        while (temp->next)
        {
            if (temp->next->io_num == io_num)
            {
                button = temp->next;
                temp->next = temp->next->next;
                break;
            }
        }
    }

    if (!g_adv_button)
    {
        xTimerStop(toggle_timer, 1);
    }

    xSemaphoreGive(button_lock);
    if (!button)
        return;
    free(button);
}

int adv_button_init(void)
{
    if (!button_lock)
    {
        button_lock = xSemaphoreCreateBinary();
        xSemaphoreGive(button_lock);
        toggle_timer = xTimerCreate("Toggle Timer", pdMS_TO_TICKS(10), pdTRUE, NULL, toggle_timer_callback);
        toggles_initialized = true;
    }

    return 0;
}

int adv_button_create(gpio_num_t io_num, adv_button_config_t *config, adv_button_callback_t callback, void *params)
{
    if (!toggles_initialized)
        adv_button_init();
    if (!button_lock)
    {
        adv_button_init();
    }
    xSemaphoreTake(button_lock, portMAX_DELAY);
    adv_button_t *button = g_adv_button;
    while (button && button->io_num != io_num)
    {
        button = button->next;
    }
    bool exists = button != NULL;
    xSemaphoreGive(button_lock);

    if (exists)
        return -1;

    button = malloc(sizeof(adv_button_t));
    bzero(button, sizeof(adv_button_t));
    button->io_num = io_num;
    button->config = *config;
    button->callback = callback;
    button->params = params;
    button->last_high = gpio_get_level(button->io_num) == 1;

    if (config->hold_press_time_ms)
    {
        button->hold_press_timer = xTimerCreate("Button Long Press Timer", pdMS_TO_TICKS(config->hold_press_time_ms), pdFALSE, button, button_long_press_timer_callback);
        if (!button->hold_press_timer)
        {
            button_free(button);
            return -1;
        }
    }

    if (config->max_repeat_press > 1)
    {
        // button->config.max_repeat_press = MIN(button->config.max_repeat_press, ADV_BUTTON_EVENT_MAX - 1);
        button->repeat_press_timeout_timer = xTimerCreate("Button Repeat Press Timeout Timer", pdMS_TO_TICKS(config->repeat_press_timeout_ms), pdFALSE, button, button_repeat_press_timeout_timer_callback);
        if (!button->repeat_press_timeout_timer)
        {
            button_free(button);
            return -2;
        }
    }

    gpio_set_direction(button->io_num, GPIO_MODE_INPUT);
    gpio_set_pull_mode(button->io_num, (button->config.active_level == BUTTON_ACTIVE_LOW) ? GPIO_PULLUP_ONLY : GPIO_PULLDOWN_ONLY);

    if (!xTimerIsTimerActive(toggle_timer))
    {
        xTimerStart(toggle_timer, 1);
    }

    xSemaphoreTake(button_lock, portMAX_DELAY);
    button->next = g_adv_button;
    g_adv_button = button;
    xSemaphoreGive(button_lock);

    return 0;

    // _button_fail:
    //     button_free(button);
    //     return 0;
}

int adv_button_destroy(const uint8_t io_num)
{

    if (!button_lock)
    {
        adv_button_init();
    }
    xSemaphoreTake(button_lock, portMAX_DELAY);

    if (!g_adv_button)
    {
        xSemaphoreGive(button_lock);
        return 0;
    }
    adv_button_t *prev = NULL;
    if (g_adv_button->io_num == io_num)
    {
        prev = g_adv_button;
        g_adv_button = g_adv_button->next;
    }
    else
    {
        adv_button_t *temp = g_adv_button;
        while (temp->next)
        {
            if (temp->next->io_num == io_num)
            {
                prev = temp->next;
                temp->next = temp->next->next;
                break;
            }
            temp = temp->next;
        }
    }

    if (prev)
    {
        if (!g_adv_button)
        {
            printf("Stop toggle timer\n");
            xTimerStop(toggle_timer, 1);
        }

        button_free(prev);
    }
    xSemaphoreGive(button_lock);
    return 0;
}

void read_button(gpio_num_t button)
{
    gpio_set_direction(button, GPIO_MODE_INPUT);
    gpio_set_pull_mode(button, GPIO_PULLUP_ONLY);
}