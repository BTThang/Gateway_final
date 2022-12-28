#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/FreeRTOSConfig.h>
#include <freertos/task.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#include <cstdlib>
#include <iostream>

#include "DS3231.h"
#include "HD44780.h"
#include "adv_button.h"
#include "esp_attr.h"
#include "esp_event.h"
#include "esp_firebase/app.h"
#include "esp_firebase/rtdb.h"
#include "esp_netif.h"
#include "esp_sleep.h"
#include "esp_sntp.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "firebase_config.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "jsoncpp/json.h"
#include "jsoncpp/value.h"
#include "lora.h"
#include "lwip/dns.h"
#include "lwip/err.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "nvs_flash.h"
#include "wifi_utils.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SDA_PIN         GPIO_NUM_21
#define SCL_PIN         GPIO_NUM_22
#define BUTTON_MENU     GPIO_NUM_34
#define BUTTON_MODE     GPIO_NUM_35
#define BUTTON_UP       GPIO_NUM_33
#define BUTTON_DOWN     GPIO_NUM_32
#define BUTTON_RELAY1   GPIO_NUM_25
#define BUTTON_RELAY2   GPIO_NUM_26
#define BUTTON_RELAY3   GPIO_NUM_27
#define CONFIG_TIMEZONE 7

#define LCD_MENU_BIT    BIT7
#define MODE_SYSTEMS    BIT2
#define DOWN_BIT        BIT3
#define RELAY1_BIT      BIT4
#define RELAY2_BIT      BIT5
#define RELAY3_BIT      BIT6

#define NTP_SERVER      CONFIG_NTP_SERVER
#define WEB_SERVER      "api.thingspeak.com"
#define WEB_PORT        "80"
#define WEB_PATH        "/"

char REQUEST[512];

using namespace ESPFirebase;
FirebaseApp       app   = FirebaseApp(API_KEY);
RTDB              db    = RTDB(&app, DATABASE_URL);
static const char TAG[] = "DS3231";
uint32_t          txBuf[128];
uint8_t           buf[128];
char              timeBuf[128];
uint8_t           bufferRx[10];
uint8_t           receive[20];
uint8_t           send_lora[40];
uint8_t           send_mode[20];

int Manu, Aut, Ala;
int temperature, hum, soil, lights;

time_t ba;
time_t set_time;
time_t t3, t5, t7, t9;
// struct tm *t2, *t4, *t6, *t8;

time_t     t1 = time(NULL);
struct tm* t2 = localtime(&t1);
struct tm* t4 = localtime(&t1);
struct tm* t6 = localtime(&t1);
struct tm* t8 = localtime(&t1);

RTC_DATA_ATTR static int boot_count = 0;
EventGroupHandle_t       xLCDButtonHandle;
EventGroupHandle_t       xUpdateFirebase;
static SemaphoreHandle_t xLoraData;

TaskHandle_t xHandle1 = NULL;
TaskHandle_t xHandle2 = NULL;
TaskHandle_t xHandle3 = NULL;
TaskHandle_t xHandle4 = NULL;
TaskHandle_t xHandle5 = NULL;
TaskHandle_t xHandle6 = NULL;
TaskHandle_t xHandle7 = NULL;

adv_button_event_id_t event;
adv_button_event_id_t event1;
adv_button_config_t   button_cfg = ADVANCED_BUTTON_CONFIG_DEFAULT();

const struct addrinfo hints = {
    .ai_family   = AF_INET,
    .ai_socktype = SOCK_STREAM,
};
struct addrinfo* res;
struct in_addr*  addr;
int              s;

typedef enum {
    MENU_LCD_1 = 0,
    MENU_LCD_2,
    MENU_LCD_3,
    MENU_LCD_4,
    MENU_LCD_5,
    MENU_LCD_MAX,
} menu_lcd_t;
menu_lcd_t menu_lcd = MENU_LCD_1;

typedef struct {
    uint8_t max_val;
    uint8_t min_val;
} sensor_param_data_t;

typedef struct {
    int hour;
    int minutes;
    int statu;
    int activity;
} alarm_t;

struct {
    alarm_t lamp1;
    alarm_t lamp2;
    alarm_t fan1;
    alarm_t fan2;
    alarm_t pump1;
    alarm_t pump2;
    alarm_t humidifier1;
    alarm_t humidifier2;
    bool    sta;
} firebase_data;

typedef enum {
    TEMP = 0,
    HUMID,
    LUX,
    GROUND,
    CURSOR_MAX,
} cursor_enum;

struct {
    sensor_param_data_t temp;
    sensor_param_data_t humid;
    sensor_param_data_t lux;
    sensor_param_data_t ground;
    cursor_enum         cursor;
    bool                select;
    bool                edit_param;
} menu4_data;

struct {
    int relay1;
    int relay2;
    int relay3;
    int relay4;
} status_relay;

int Rle1, Rle2, Rle3, Rle4;

typedef enum {
    MODE_MIN = 0,
    MODE_AUTO,
    MODE_MANUAL,
    MODE_ALARM,
    MODE_MAX,
} mode_sys_t;
mode_sys_t mode_sys = MODE_MIN;

Json::Value data;
Json::Value data1;
Json::Value data2;
Json::Value data3;
Json::Value data4;
Json::Value data5;
Json::Value data10;
Json::Value data9;
Json::Value data8;
Json::Value data7;

Json::Value  data11;
Json::Value  data12;
Json::Value  data13;
Json::Reader reader;
// declare function

void time_sync_notification_cb(struct timeval* tv);

static esp_err_t i2c_master_init(void);
static void      initialize_sntp(void);
void             getClock(void* pvParameters);
void             getLoRa(void* pvParameters);

void menubutton_callback(adv_button_event_id_t event, void* params);
void modebutton_callback(adv_button_event_id_t event, void* params);
void upbutton_callback(adv_button_event_id_t event, void* params);
void downbutton_callback(adv_button_event_id_t event, void* params);

void button_relay1(adv_button_event_id_t event, void* params);
void button_relay2(adv_button_event_id_t event, void* params);
void button_relay3(adv_button_event_id_t event, void* params);

void task_relay1(void* pvParameters);
void task_relay2(void* pvParameters);
void task_relay3(void* pvParameters);

void button_mode_sys(void* pvParameters);
// void update_firebase_auto(void* pvParameters);
// void update_firebase_man(void* pvParameters);
// void update_firebase_alarm(void* pvParameters);

void        handle_cursor();
void        handle_lcd_menu();
void        mode_hold();
void        handledown_edit();
void        handleup_edit();
void        lcd_menu2(void);
void        menu_lcd3(void* pvParameters);
void        menu_lcd4(void* pvParameters);
void        menu_lcd5(void* pvParameters);
static void http_get_task(void* pvParameters);

void lcd_display(void);

/*-------------------------------------Button_RELAY1_Callback------------------------------------*/
void button_relay1(adv_button_event_id_t event, void* params) {
    switch (event) {
        case ADV_BUTTON_EVENT_SINGLE_PRESS:
            if (Rle1 == 1)
                Rle1 = 2;
            else
                Rle1 = 1;
            break;
        default:
            break;
    }
}

/*-------------------------------------Button_RELAY2_Callback------------------------------------*/
void button_relay2(adv_button_event_id_t event, void* params) {
    switch (event) {
        case ADV_BUTTON_EVENT_SINGLE_PRESS:
            if (Rle2 == 1)
                Rle2 = 2;
            else
                Rle2 = 1;
            // status_relay.relay2 ^= 1;
            // xEventGroupSetBits(xLCDButtonHandle, RELAY2_BIT);
            break;
        default:
            break;
    }
}

/*-------------------------------------Button_RELAY3_Callback------------------------------------*/
void button_relay3(adv_button_event_id_t event, void* params) {
    switch (event) {
        case ADV_BUTTON_EVENT_SINGLE_PRESS:
            if (Rle3 == 1)
                Rle3 = 2;
            else
                Rle3 = 1;
            // xEventGroupSetBits(xLCDButtonHandle, RELAY3_BIT);
            break;

        case ADV_BUTTON_EVENT_HOLD_PRESS:
            if (Rle4 == 1)
                Rle4 = 2;
            else
                Rle4 = 1;
            break;
        default:
            break;
    }
}

/*-------------------------------------Button_MODE_Callback------------------------------------*/
void modebutton_callback(adv_button_event_id_t event, void* params) {
    switch (event) {
        case ADV_BUTTON_EVENT_SINGLE_PRESS:
            if (menu_lcd == MENU_LCD_4 && menu4_data.select == true) {
                menu4_data.edit_param ^= 1;
            }
            break;
        case ADV_BUTTON_EVENT_HOLD_PRESS:
            if (menu_lcd == MENU_LCD_4) {
                menu4_data.select ^= 1;
                if (menu4_data.select) {
                    menu4_data.cursor = TEMP;
                    LCD_setCursor(6, 0);
                    blink();
                } else
                    noBlink();
            }
            break;
        default:
            break;
    }
}

/*--------------------------------CALLBACK______________BUTTON_______________DOWN---------------------------*/
void downbutton_callback(adv_button_event_id_t event, void* params) {
    switch (event) {
        case ADV_BUTTON_EVENT_SINGLE_PRESS:
            if (menu4_data.cursor <= TEMP) menu4_data.cursor = TEMP;
            if (menu_lcd != MENU_LCD_4) {
                return;
            }
            if (menu4_data.select == false) {
                return;
            }
            if (menu4_data.edit_param == true && menu4_data.select == true) {
                handledown_edit();
            } else if (menu4_data.select == true) {
                menu4_data.cursor = (cursor_enum)((int)menu4_data.cursor - 1);
                handle_cursor();
            }
            break;

        default:
            break;
    }
}

/*--------------------------------Callback_UpButton---------------------------*/
void upbutton_callback(adv_button_event_id_t event, void* params) {
    switch (event) {
        case ADV_BUTTON_EVENT_SINGLE_PRESS:
            if (menu4_data.cursor >= CURSOR_MAX - 1) menu4_data.cursor = GROUND;
            if (menu_lcd != MENU_LCD_4) {
                return;
            }
            if (menu4_data.select == false) {
                return;
            }
            if (menu4_data.edit_param && menu4_data.select == true) {
                handleup_edit();
            } else if (menu4_data.select == true) {
                menu4_data.cursor = (cursor_enum)((int)menu4_data.cursor + 1);
                handle_cursor();
            }
            break;
        default:
            break;
    }
}

/*--------------------------------Handle_Cursor---------------------------*/
void handle_cursor(void) {
    switch (menu4_data.cursor) {
        case TEMP:
            LCD_setCursor(6, 0);
            blink();
            break;
        case HUMID:
            LCD_setCursor(6, 1);
            blink();
            break;
        case LUX:
            LCD_setCursor(6, 2);
            blink();
            break;
        case GROUND:
            LCD_setCursor(6, 3);
            blink();
            break;
        default:
            break;
    }
}
/*-------------------------------------Button
 * MenuCallback------------------------------------*/

void menubutton_callback(adv_button_event_id_t event, void* params) {
    switch (event) {
        case ADV_BUTTON_EVENT_SINGLE_PRESS:
            menu_lcd = (menu_lcd_t)(int)(menu_lcd + 1);
            if (menu_lcd >= MENU_LCD_MAX) menu_lcd = MENU_LCD_2;
            xEventGroupSetBits(xLCDButtonHandle, LCD_MENU_BIT);
            break;
        case ADV_BUTTON_EVENT_HOLD_PRESS:
            ESP_LOGI(TAG, "Xin chao----------------- TMT\n");
            mode_sys = (mode_sys_t)(int)(mode_sys + 1);
            if (mode_sys == MODE_MAX) mode_sys = MODE_AUTO;
            xEventGroupSetBits(xLCDButtonHandle, MODE_SYSTEMS);
            break;
        default:
            break;
    }
}

void button_mode_sys(void* pvParameters) {
    while (1) {
        if (xEventGroupWaitBits(xLCDButtonHandle, MODE_SYSTEMS, true, false,
                                portMAX_DELAY)) {
            xSemaphoreTake(xLoraData, portMAX_DELAY);
            if (mode_sys == MODE_AUTO) {
                data11["S1_Auto"] = 1;
                db.patchData("TGarden", data11);
                data12["S1_Manual"] = 0;
                db.patchData("TGarden", data12);
                data13["Status"] = 0;
                db.patchData("TGarden/Alarm", data13);
            }
            if (mode_sys == MODE_MANUAL) {
                data12["S1_Manual"] = 1;
                db.patchData("TGarden", data12);
                data13["Status"] = 0;
                db.patchData("TGarden/Alarm", data13);
                data11["S1_Auto"] = 0;
                db.patchData("TGarden", data11);
            }
            if (mode_sys == MODE_ALARM) {
                data13["Status"] = 1;
                db.patchData("TGarden/Alarm", data13);
                data11["S1_Auto"] = 0;
                db.patchData("TGarden", data11);
                data12["S1_Manual"] = 0;
                db.patchData("TGarden", data12);
            }
            xSemaphoreGive(xLoraData);
        }
    }
}
struct tm thoigian;

void mode_systems(void* pvParameters) {
    while (1) {
        switch (mode_sys) {
            case MODE_AUTO:
                xSemaphoreTake(xLoraData, portMAX_DELAY);
                // ==========================Temp_max================================
                if (menu4_data.temp.max_val != temperature) {
                    menu4_data.temp.max_val = temperature;
                    data1["Auto/Temp_high"] = menu4_data.temp.max_val;
                    db.patchData("TGarden", data1);
                    send_mode[1] = menu4_data.temp.max_val;
                    lora_send_packet((uint8_t*)send_mode, 5);
                } else {
                    std::string Temp_max =
                        db.getData("TGarden/Auto/Temp_high").asString();
                    menu4_data.temp.max_val = atoi(Temp_max.c_str());
                    temperature             = atoi(Temp_max.c_str());
                    send_mode[1]            = menu4_data.temp.max_val;
                    lora_send_packet((uint8_t*)send_mode, 5);
                }
                // =============================Hum_ON=========================================
                if (menu4_data.humid.min_val != hum) {
                    menu4_data.humid.min_val = hum;
                    data1["Auto/Hum"]        = menu4_data.humid.min_val;
                    db.patchData("TGarden", data1);
                    send_mode[2] = menu4_data.humid.min_val;
                    lora_send_packet((uint8_t*)send_mode, 5);
                } else {
                    std::string Hum_min =
                        db.getData("TGarden/Auto/Hum").asString();
                    menu4_data.humid.min_val = atoi(Hum_min.c_str());
                    hum                      = atoi(Hum_min.c_str());
                    send_mode[2]             = menu4_data.humid.min_val;
                    lora_send_packet((uint8_t*)send_mode, 5);
                }
                lora_send_packet((uint8_t*)send_mode, 5);
                // Lights
                if (menu4_data.lux.min_val != lights) {
                    menu4_data.lux.min_val = lights;
                    data1["Auto/Lights"]   = menu4_data.lux.min_val;
                    db.patchData("TGarden", data1);
                    send_mode[3] = menu4_data.lux.min_val;
                    lora_send_packet((uint8_t*)send_mode, 5);
                } else {
                    std::string Lights_min =
                        db.getData("TGarden/Auto/Lights").asString();
                    menu4_data.lux.min_val = atoi(Lights_min.c_str());
                    lights                 = atoi(Lights_min.c_str());
                    send_mode[3]           = menu4_data.lux.min_val;
                    lora_send_packet((uint8_t*)send_mode, 5);
                }
                lora_send_packet((uint8_t*)send_mode, 5);
                // ========================================Ground_ON============================================
                if (menu4_data.ground.min_val != soil) {
                    menu4_data.ground.min_val = soil;
                    data1["Auto/Soil"]        = menu4_data.ground.min_val;
                    db.patchData("TGarden", data1);
                    send_mode[4] = menu4_data.ground.min_val;
                    lora_send_packet((uint8_t*)send_mode, 5);
                } else {
                    std::string Soil_min =
                        db.getData("TGarden/Auto/Soil").asString();
                    menu4_data.ground.min_val = atoi(Soil_min.c_str());
                    soil                      = atoi(Soil_min.c_str());
                    send_mode[4]              = menu4_data.ground.min_val;
                    lora_send_packet((uint8_t*)send_mode, 5);
                }
                lora_send_packet((uint8_t*)send_mode, 5);
                xSemaphoreGive(xLoraData);
                break;
            case MODE_MANUAL:
                xSemaphoreTake(xLoraData, portMAX_DELAY);
                // ========================================Relay
                // 1=================================
                if (status_relay.relay1 != Rle1) {
                    status_relay.relay1 = Rle1;
                    data10["S1_Lamp"]   = status_relay.relay1;
                    db.patchData("RELAY1", data10);
                    send_mode[1] = status_relay.relay1;
                    // send_mode[1] = 'status_relay.relay1';
                    lora_send_packet((uint8_t*)send_mode, 5);
                } else {
                    std::string Lamp = db.getData("RELAY1/S1_Lamp").asString();
                    status_relay.relay1 = atoi(Lamp.c_str());
                    Rle1                = atoi(Lamp.c_str());
                    send_mode[1]        = (uint8_t)status_relay.relay1;
                    // send_mode[1]        = 'status_relay.relay1';
                    lora_send_packet((uint8_t*)send_mode, 5);
                }

                lora_send_packet((uint8_t*)send_mode, 5);
                // =====================================Relay
                // 2=====================================
                if (status_relay.relay2 != Rle2) {
                    status_relay.relay2 = Rle2;
                    data9["S2_Fan"]     = status_relay.relay2;
                    db.patchData("RELAY2", data9);
                    send_mode[2] = status_relay.relay2;
                    // send_mode[2] = 'status_relay.relay2';
                    lora_send_packet((uint8_t*)send_mode, 5);
                } else {
                    std::string Fan = db.getData("RELAY2/S2_Fan").asString();
                    status_relay.relay2 = atoi(Fan.c_str());
                    Rle2                = atoi(Fan.c_str());
                    send_mode[2]        = status_relay.relay2;
                    // send_mode[2]        = 'status_relay.relay2';
                    lora_send_packet((uint8_t*)send_mode, 5);
                }
                // printf("Relay 2222222:::::::::%d\n", send_mode[2]);
                lora_send_packet((uint8_t*)send_mode, 5);

                // ========================================Relay
                // 3==================================
                if (status_relay.relay3 != Rle3) {
                    status_relay.relay3 = Rle3;
                    data8["S3_Pump"]    = status_relay.relay3;
                    db.patchData("RELAY3", data8);
                    send_mode[3] = status_relay.relay3;
                    // send_mode[3] = 'status_relay.relay3';
                    lora_send_packet((uint8_t*)send_mode, 5);
                } else {
                    std::string Pump = db.getData("RELAY3/S3_Pump").asString();
                    status_relay.relay3 = atoi(Pump.c_str());
                    Rle3                = atoi(Pump.c_str());
                    send_mode[3]        = (uint8_t)status_relay.relay3;
                    // send_mode[3]        = 'status_relay.relay3';

                    lora_send_packet((uint8_t*)send_mode, 5);
                }
                lora_send_packet((uint8_t*)send_mode, 5);
                // ========================================Relay
                // 4==================================
                if (status_relay.relay4 != Rle4) {
                    status_relay.relay4 = Rle4;
                    data7["S4_Hum"]     = status_relay.relay4;
                    db.patchData("RELAY4", data7);
                    send_mode[4] = status_relay.relay4;
                    // send_mode[4] = 'status_relay.relay4';
                    lora_send_packet((uint8_t*)send_mode, 5);
                } else {
                    std::string Humidity =
                        db.getData("RELAY4/S4_Hum").asString();
                    status_relay.relay4 = atoi(Humidity.c_str());
                    Rle4                = atoi(Humidity.c_str());
                    send_mode[4]        = status_relay.relay4;
                    // send_mode[4]        = 'status_relay.relay4';
                    lora_send_packet((uint8_t*)send_mode, 5);
                }
                lora_send_packet((uint8_t*)send_mode, 5);
                xSemaphoreGive(xLoraData);
                break;
            case MODE_ALARM:
                xSemaphoreTake(xLoraData, portMAX_DELAY);
                {
                    // ds3231_get_time(&thoigian);
                    time(&ba);
                    ba = ba + 45;
                    // ESP_LOGI(pcTaskGetTaskName(0),
                    //          "hahahahahahahahahahaha::::::%ld\n", t3);

                    // ESP_LOGI(pcTaskGetTaskName(0),
                    //          "hhuhuhuhuhaaaaaaaaaaa:::%ld\n", ba);
                    // Fan1---------------------
                    std::string Alarm_Fan1_Hour =
                        db.getData("TGarden/Alarm/Fan/AmFan1/Hour").asString();
                    firebase_data.fan1.hour = atoi(Alarm_Fan1_Hour.c_str());

                    std::string Alarm_Fan1_Minu =
                        db.getData("TGarden/Alarm/Fan/AmFan1/Minutes")
                            .asString();
                    firebase_data.fan1.minutes = atoi(Alarm_Fan1_Minu.c_str());

                    std::string Alarm_Fan1_Ac =
                        db.getData("TGarden/Alarm/Fan/AmFan1/Activity")
                            .asString();
                    firebase_data.fan1.activity = atoi(Alarm_Fan1_Ac.c_str());

                    // Compare time1111111111
                    t2->tm_hour = firebase_data.fan1.hour;
                    t2->tm_min  = firebase_data.fan1.minutes;
                    t3          = mktime(t2);
                    // printf("Thoi gian thuc te: %ld", ba);

                    // printf("Thoi gian cai dat: %ld", t3);
                    if ((t3) < ba &&
                        ba < (t3 + (firebase_data.fan1.activity * 60))) {
                        send_mode[2] = '5';
                        lora_send_packet((uint8_t*)send_mode, 5);

                        // printf(
                        //     "========Thoi Gian bang nhau===== Bat thiet "
                        //     "bi11111111111111111\n");
                    } else {
                        send_mode[2] = '6';
                        lora_send_packet((uint8_t*)send_mode, 5);
                    };
                    lora_send_packet((uint8_t*)send_mode, 5);

                    // std::string Alarm_Fan1_Sta =
                    //     db.getData("TGarden/Alarm/Fan/AmFan1/Status")
                    //         .asString();
                    // firebase_data.fan1.statu = atoi(Alarm_Fan1_Sta.c_str());
                    // Fan2-----------------------
                    // std::string Alarm_Fan2_Hour =
                    //     db.getData("TGarden/Alarm/Fan/AmFan2/Hour").asString();
                    // firebase_data.fan2.hour = atoi(Alarm_Fan2_Hour.c_str());

                    // std::string Alarm_Fan2_Minu =
                    //     db.getData("TGarden/Alarm/Fan/AmFan2/Minutes")
                    //         .asString();
                    // firebase_data.fan2.minutes =
                    // atoi(Alarm_Fan2_Minu.c_str());

                    // std::string Alarm_Fan2_Ac =
                    //     db.getData("TGarden/Alarm/Fan/AmFan2/Activity")
                    //         .asString();
                    // firebase_data.fan2.activity =
                    // atoi(Alarm_Fan2_Ac.c_str());

                    // if (firebase_data.fan2.hour == thoigian.tm_hour &&
                    //     firebase_data.fan2.minutes == thoigian.tm_min) {
                    //     printf(
                    //         "========Thoi Gian bang nhau===== Bat thiet "
                    //         "bi1111111111111111\n");
                    // }

                    // std::string Alarm_Fan2_Sta =
                    //     db.getData("TGarden/Alarm/Fan/AmFan2/Status")
                    //         .asString();
                    // firebase_data.fan2.statu = atoi(Alarm_Fan2_Sta.c_str());

                    // Doc gia tri hen gio cua den

                    // Lamp1----------------
                    std::string Alarm_Lamp1_Hour =
                        db.getData("TGarden/Alarm/Lamp/AmLamp1/Hour")
                            .asString();
                    firebase_data.lamp1.hour = atoi(Alarm_Lamp1_Hour.c_str());

                    std::string Alarm_Lamp1_Minu =
                        db.getData("TGarden/Alarm/Lamp/AmLamp1/Minutes")
                            .asString();
                    firebase_data.lamp1.minutes =
                        atoi(Alarm_Lamp1_Minu.c_str());

                    std::string Alarm_Lamp1_Ac =
                        db.getData("TGarden/Alarm/Lamp/AmLamp1/Activity")
                            .asString();
                    firebase_data.lamp1.activity = atoi(Alarm_Lamp1_Ac.c_str());

                    // Compare time222222222
                    t4->tm_hour = firebase_data.lamp1.hour;
                    t4->tm_min  = firebase_data.lamp1.minutes;
                    t5          = mktime(t4);
                    if (t5 < ba &&
                        ba < (t5 + (firebase_data.lamp1.activity * 60))) {
                        // printf(
                        //     "========Thoi Gian bang nhau===== Bat thiet "
                        //     "bi2222\n");
                        send_mode[1] = '5';

                        lora_send_packet((uint8_t*)send_mode, 5);
                    } else {
                        send_mode[1] = '6';
                        lora_send_packet((uint8_t*)send_mode, 5);
                    };
                    lora_send_packet((uint8_t*)send_mode, 5);
                    // std::string Alarm_Lamp1_Sta =
                    //     db.getData("TGarden/Alarm/Lamp/AmLamp1/Status")
                    //         .asString();
                    // firebase_data.lamp1.statu =
                    // atoi(Alarm_Lamp1_Sta.c_str());
                    // Lamp2-------------------------------------------
                    // std::string Alarm_Lamp2_Hour =
                    //     db.getData("TGarden/Alarm/Lamp/AmLamp2/Hour")
                    //         .asString();
                    // firebase_data.lamp2.hour =
                    // atoi(Alarm_Lamp2_Hour.c_str());

                    // std::string Alarm_Lamp2_Minu =
                    //     db.getData("TGarden/Alarm/Lamp/AmLamp2/Minutes")
                    //         .asString();
                    // firebase_data.lamp2.minutes =
                    //     atoi(Alarm_Lamp2_Minu.c_str());

                    // std::string Alarm_Lamp2_Ac =
                    //     db.getData("TGarden/Alarm/Lamp/AmLamp2/Activity")
                    //         .asString();
                    // firebase_data.lamp2.activity =
                    // atoi(Alarm_Lamp2_Ac.c_str());

                    // if (firebase_data.lamp2.hour == thoigian.tm_hour &&
                    //     firebase_data.lamp2.minutes == thoigian.tm_min) {
                    //     printf(
                    //         "========Thoi Gian bang nhau===== Bat thiet "
                    //         "bi222222222222222222\n");
                    // }
                    // Pump1-----------------------------------
                    // Doc gia tri hen gio cua bom nuoc
                    std::string Alarm_Pump1_Hour =
                        db.getData("TGarden/Alarm/Pump/AmPump1/Hour")
                            .asString();
                    firebase_data.pump1.hour = atoi(Alarm_Pump1_Hour.c_str());

                    std::string Alarm_Pump1_Minu =
                        db.getData("TGarden/Alarm/Pump/AmPump1/Minutes")
                            .asString();
                    firebase_data.pump1.minutes =
                        atoi(Alarm_Pump1_Minu.c_str());

                    std::string Alarm_Pump1_Ac =
                        db.getData("TGarden/Alarm/Pump/AmPump1/Activity")
                            .asString();
                    firebase_data.pump1.activity = atoi(Alarm_Pump1_Ac.c_str());

                    // Compare time3333333
                    t6->tm_hour = firebase_data.pump1.hour;
                    t6->tm_min  = firebase_data.pump1.minutes;
                    t7          = mktime(t4);
                    if (t7 < ba &&
                        ba < (t7 + (firebase_data.pump1.activity * 60))) {
                        // printf(
                        //     "========Thoi Gian bang nhau===== Bat thiet "
                        //     "bi3333\n");
                        send_mode[3] = '5';
                        lora_send_packet((uint8_t*)send_mode, 5);

                    } else {
                        send_mode[3] = '6';
                        lora_send_packet((uint8_t*)send_mode, 5);
                    }
                    lora_send_packet((uint8_t*)send_mode, 5);
                    // Pump2---------------------------
                    // std::string Alarm_Pump2_Hour =
                    //     db.getData("TGarden/Alarm/Pump/AmPump2/Hour")
                    //         .asString();
                    // firebase_data.pump2.hour =
                    // atoi(Alarm_Pump2_Hour.c_str());

                    // std::string Alarm_Pump2_Minu =
                    //     db.getData("TGarden/Alarm/Pump/AmPump2/Minutes")
                    //         .asString();
                    // firebase_data.pump2.minutes =
                    //     atoi(Alarm_Pump2_Minu.c_str());

                    // std::string Alarm_Pump2_Ac =
                    //     db.getData("TGarden/Alarm/Pump/AmPump2/Activity")
                    //         .asString();
                    // firebase_data.pump2.activity =
                    // atoi(Alarm_Pump2_Ac.c_str());

                    // if (firebase_data.pump2.hour == thoigian.tm_hour &&
                    //     firebase_data.pump2.minutes == thoigian.tm_min) {
                    //     printf(
                    //         "========Thoi Gian bang nhau===== Bat thiet "
                    //         "bi3333333333333\n");
                    // }

                    // Hum1------------------------
                    // Doc gia tri hen gio cua may phun suong
                    std::string Alarm_Hum1_Hour =
                        db.getData("TGarden/Alarm/Humidifier/AmHum1/Hour")
                            .asString();
                    firebase_data.humidifier1.hour =
                        atoi(Alarm_Hum1_Hour.c_str());

                    std::string Alarm_Hum1_Minu =
                        db.getData("TGarden/Alarm/Humidifier/AmHum1/Minutes")
                            .asString();
                    firebase_data.humidifier1.minutes =
                        atoi(Alarm_Hum1_Minu.c_str());

                    std::string Alarm_Hum1_Ac =
                        db.getData("TGarden/Alarm/Humidifier/AmHum1/Activity")
                            .asString();
                    firebase_data.humidifier1.activity =
                        atoi(Alarm_Hum1_Ac.c_str());

                    // Compare time444444
                    t8->tm_hour = firebase_data.humidifier1.hour;
                    t8->tm_min  = firebase_data.humidifier1.minutes;
                    t9          = mktime(t4);
                    if (t9 < ba &&
                        ba < (t9 + (firebase_data.humidifier1.activity * 60))) {
                        // printf(
                        //     "========Thoi Gian bang nhau===== Bat thiet "
                        //     "bi44444\n");
                        send_mode[4] = '5';

                        lora_send_packet((uint8_t*)send_mode, 5);
                    } else {
                        send_mode[4] = '6';
                        lora_send_packet((uint8_t*)send_mode, 5);
                    }
                    lora_send_packet((uint8_t*)send_mode, 5);
                    // Hum2-----------------------------------
                    // std::string Alarm_Hum2_Hour =
                    //     db.getData("TGarden/Alarm/Humidifier/AmHum2/Hour")
                    //         .asString();
                    // firebase_data.humidifier2.hour =
                    //     atoi(Alarm_Hum2_Hour.c_str());

                    // std::string Alarm_Hum2_Minu =
                    //     db.getData("TGarden/Alarm/Humidifier/AmHum2/Minutes")
                    //         .asString();
                    // firebase_data.humidifier2.minutes =
                    //     atoi(Alarm_Hum2_Minu.c_str());

                    // std::string Alarm_Hum2_Ac =
                    //     db.getData("TGarden/Alarm/Humidifier/AmHum2/Activity")
                    //         .asString();
                    // firebase_data.humidifier2.activity =
                    //     atoi(Alarm_Hum2_Ac.c_str());

                    // if (firebase_data.humidifier2.hour == thoigian.tm_hour &&
                    //     firebase_data.humidifier2.hour == thoigian.tm_min) {
                    //     printf(
                    //         "========Thoi Gian bang nhau===== Bat thiet "
                    //         "bi4444444444444444\n");
                    // }
                }
                xSemaphoreGive(xLoraData);
                break;
            default:
                break;
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

/*-----------------------------------Task_LCD1-----------------------------*/
void lcd_task1(void* pvParameters) {
    lcd_display();
    while (1) {
        if (xEventGroupWaitBits(xLCDButtonHandle, LCD_MENU_BIT, true, false,
                                portMAX_DELAY)) {
            switch (menu_lcd) {
                case MENU_LCD_2:
                    if (xHandle4 != NULL) {
                        vTaskDelete(xHandle4);
                    }
                    xTaskCreate(&getClock, "getClock", 1024 * 4, NULL, 6,
                                &xHandle1);
                    break;
                case MENU_LCD_3:
                    if (xHandle1 != NULL) {
                        vTaskDelete(xHandle1);
                    }
                    xTaskCreate(&menu_lcd3, "MenuLCD 3", 1024 * 4, NULL, 6,
                                &xHandle2);

                    break;
                case MENU_LCD_4:
                    if (xHandle2 != NULL) {
                        vTaskDelete(xHandle2);
                    }
                    xTaskCreate(&menu_lcd4, "MENU_LCD4", 1024 * 4, NULL, 6,
                                &xHandle3);
                    break;
                case MENU_LCD_5:
                    if (xHandle3 != NULL) {
                        vTaskDelete(xHandle3);
                    }
                    xTaskCreate(&menu_lcd5, "MENU_LCD5", 1024 * 4, NULL, 6,
                                &xHandle4);
                    break;
                default:
                    break;
            }
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/*----------------------------GET_LORA-----------------------------*/

void menu_lcd3(void* pvParameters) {
    while (1) {
        LCD_clearScreen();
        // Display on temperature
        LCD_setCursor(0, 0);
        sprintf(timeBuf, "Temperature: %02doC", bufferRx[0]);
        LCD_writeStr(timeBuf);

        // Display on humidity
        LCD_setCursor(0, 1);
        sprintf(timeBuf, "Humidity: %02d", bufferRx[1]);
        LCD_writeStr(timeBuf);

        // Display on temperature
        LCD_setCursor(0, 2);
        sprintf(timeBuf, "Lights: %02dlux", bufferRx[2]);
        LCD_writeStr(timeBuf);

        // Display on soil m
        LCD_setCursor(0, 3);
        sprintf(timeBuf, "Soil: %02d--", bufferRx[4]);
        LCD_writeStr(timeBuf);

        if (bufferRx[3] == 0) {
            LCD_setCursor(12, 3);
            sprintf(timeBuf, "Rain");
            LCD_writeStr(timeBuf);
        } else if (bufferRx[3] == 1) {
            LCD_setCursor(12, 3);
            sprintf(timeBuf, "Sunny");
            LCD_writeStr(timeBuf);
        }

        vTaskDelay(10000 / portTICK_RATE_MS);
    }
}

void menu_lcd4(void* pvParameters) {
    // Get RTC date and time
    while (1) {
        // LCD_clearScreen();
        // Display on temperature
        LCD_clearScreen();
        LCD_setCursor(0, 0);
        sprintf(timeBuf, "Temp: %d *C", temperature);
        LCD_writeStr(timeBuf);

        // Display on humidity
        LCD_setCursor(0, 1);
        sprintf(timeBuf, "Humi: %d ", hum);
        LCD_writeStr(timeBuf);

        // // Display on soid sensor
        LCD_setCursor(0, 2);
        sprintf(timeBuf, "Ligh: %d", lights);
        LCD_writeStr(timeBuf);

        // Display on light intensiy
        LCD_setCursor(0, 3);
        sprintf(timeBuf, "Soil: %d lux", soil);
        LCD_writeStr(timeBuf);

        vTaskDelay(2000 / portTICK_RATE_MS);
    }
}

// MenuLCD5=============
void menu_lcd5(void* pvParameters) {
    while (1) {
        LCD_clearScreen();
        // Display on temperature
        LCD_setCursor(0, 0);
        if (Manu == 1) {
            sprintf(timeBuf, "----MODE: Manual----");
            LCD_writeStr(timeBuf);
        } else if (Aut == 1) {
            sprintf(timeBuf, "--MODE: Automation--");
            LCD_writeStr(timeBuf);
        } else if (Ala == 1) {
            sprintf(timeBuf, "---MODE: Alarm---");
            LCD_writeStr(timeBuf);
        }

        // Display on humidity
        LCD_setCursor(0, 1);
        sprintf(timeBuf, "TRANG THAI THIET BI");
        LCD_writeStr(timeBuf);

        // Display on temperature
        if (mode_sys == MODE_MANUAL) {
            LCD_setCursor(0, 2);

            if (status_relay.relay1 == 1 && status_relay.relay2 == 1) {
                sprintf(timeBuf, "  DEN:ON--QUAT:ON");
                LCD_writeStr(timeBuf);
            } else if (status_relay.relay1 == 1 && status_relay.relay2 == 2) {
                sprintf(timeBuf, "  DEN:ON--QUAT:OFF");
                LCD_writeStr(timeBuf);
            } else if (status_relay.relay1 == 2 && status_relay.relay2 == 1) {
                sprintf(timeBuf, "  DEN:OFF--QUAT:ON");
                LCD_writeStr(timeBuf);
            } else if (status_relay.relay1 == 2 && status_relay.relay2 == 2) {
                sprintf(timeBuf, "  DEN:OFF--QUAT:OFF");
                LCD_writeStr(timeBuf);
            }
            LCD_setCursor(0, 3);

            if (status_relay.relay3 == 1 && status_relay.relay4 == 1) {
                sprintf(timeBuf, "  BOM:ON--SUONG:ON");
                LCD_writeStr(timeBuf);
            } else if (status_relay.relay3 == 1 && status_relay.relay4 == 2) {
                sprintf(timeBuf, "  BOM:ON--SUONG:OFF");
                LCD_writeStr(timeBuf);
            } else if (status_relay.relay3 == 2 && status_relay.relay4 == 1) {
                sprintf(timeBuf, "  BOM:OFF--SUONG:ON");
                LCD_writeStr(timeBuf);
            } else if (status_relay.relay3 == 2 && status_relay.relay4 == 2) {
                sprintf(timeBuf, "  BOM:OFF--SUONG:OFF");
                LCD_writeStr(timeBuf);
            }
        } else {
            LCD_setCursor(0, 2);
            if (receive[15] == 1 && receive[16] == 1) {
                sprintf(timeBuf, "  DEN:ON--QUAT:ON");
                LCD_writeStr(timeBuf);
            } else if (receive[15] == 1 && receive[16] == 2) {
                sprintf(timeBuf, "  DEN:ON--QUAT:OFF");
                LCD_writeStr(timeBuf);
            } else if (receive[15] == 2 && receive[16] == 1) {
                sprintf(timeBuf, "  DEN:OFF--QUAT:ON");
                LCD_writeStr(timeBuf);
            } else if (receive[15] == 2 && receive[16] == 2) {
                sprintf(timeBuf, "  DEN:OFF--QUAT:OFF");
                LCD_writeStr(timeBuf);
            }
            LCD_setCursor(0, 3);

            if (receive[17] == 1 && receive[18] == 1) {
                sprintf(timeBuf, "  BOM:ON--SUONG:ON");
                LCD_writeStr(timeBuf);
            } else if (receive[17] == 1 && receive[18] == 2) {
                sprintf(timeBuf, "  BOM:ON--SUONG:OFF");
                LCD_writeStr(timeBuf);
            } else if (receive[17] == 2 && receive[18] == 1) {
                sprintf(timeBuf, "  BOM:OFF--SUONG:ON");
                LCD_writeStr(timeBuf);
            } else if (receive[17] == 2 && receive[18] == 2) {
                sprintf(timeBuf, "  BOM:OFF--SUONG:OFF");
                LCD_writeStr(timeBuf);
            }
        }

        vTaskDelay(2000 / portTICK_RATE_MS);
    }
}
/*Handle Edit*/
void handleup_edit() {
    if (menu4_data.cursor == 0) {
        temperature++;
    }
    if (menu4_data.cursor == 1) {
        hum++;
    }
    if (menu4_data.cursor == 2) {
        lights++;
    }
    if (menu4_data.cursor == 3) {
        soil++;
    }
}

void handledown_edit() {
    if (menu4_data.cursor == 0) {
        temperature--;
    }
    if (menu4_data.cursor == 1) {
        hum--;
    }
    if (menu4_data.cursor == 2) {
        lights--;
    }
    if (menu4_data.cursor == 3) {
        soil--;
    }
}

/*------------------------------------------------------------Time_Sync_Notify_callback--------------------------------------------------------------------------------------------*/
void time_sync_notification_cb(struct timeval* tv) {
    ESP_LOGI(TAG, "Notification of a time synchronization event");
}

/*------------------------------------------------------------Init_SNTP--------------------------------------------------------------------------------------------*/
static void initialize_sntp(void) {
    // ESP_LOGI(TAG, "Initializing SNTP");
    // sntp_setoperatingmode(SNTP_OPMODE_POLL);
    // // sntp_setservername(0, "pool.ntp.org");
    // ESP_LOGI(TAG, "Your NTP Server is %s", NTP_SERVER);
    // sntp_setservername(0, NTP_SERVER);
    // sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    // sntp_init();
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_servermode_dhcp(1);
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "vn.pool.ntp.org");
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH);
    sntp_init();

    /* Set timezone GMT+7 */
    setenv("TZ", "CST-7", 1);
    tzset();
}

/*------------------------------------------------------------Init_I2c_Master
 * --------------------------------------------------------------------------------------------*/
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode          = I2C_MODE_MASTER,
        .sda_io_num    = SDA_PIN,
        .scl_io_num    = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
    };
    conf.master.clk_speed = 100 * 1000;
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    return ESP_OK;
}

// Set clock
void setClock(void* pvParameters) {
    // obtain time over NTP
    ESP_LOGI(pcTaskGetTaskName(0),
             "Connecting to WiFi and getting time over NTP.");

    // update 'now' variable with current time
    time_t    now;
    struct tm timeinfo;
    char      strftime_buf[64];
    time(&now);

    ESP_LOGI(pcTaskGetTaskName(0), "%ld", now);
    // now = now + (CONFIG_TIMEZONE * 60 * 60);
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(pcTaskGetTaskName(0), "The current date/time is: %s",
             strftime_buf);

    ESP_LOGD(pcTaskGetTaskName(0), "timeinfo.tm_sec=%d", timeinfo.tm_sec);
    ESP_LOGD(pcTaskGetTaskName(0), "timeinfo.tm_min=%d", timeinfo.tm_min);
    ESP_LOGD(pcTaskGetTaskName(0), "timeinfo.tm_hour=%d", timeinfo.tm_hour);
    ESP_LOGD(pcTaskGetTaskName(0), "timeinfo.tm_wday=%d", timeinfo.tm_wday);
    ESP_LOGD(pcTaskGetTaskName(0), "timeinfo.tm_mday=%d", timeinfo.tm_mday);
    ESP_LOGD(pcTaskGetTaskName(0), "timeinfo.tm_mon=%d", timeinfo.tm_mon);
    ESP_LOGD(pcTaskGetTaskName(0), "timeinfo.tm_year=%d", timeinfo.tm_year);

    struct tm time = {
        .tm_sec  = timeinfo.tm_sec,
        .tm_min  = timeinfo.tm_min,
        .tm_hour = timeinfo.tm_hour,
        .tm_mday = timeinfo.tm_mday,
        .tm_mon  = timeinfo.tm_mon,
        .tm_year = (timeinfo.tm_year + 1900),
    };

    if (ds3231_set_time(&time) != ESP_OK) {
        ESP_LOGE(pcTaskGetTaskName(0), "Could not set time.");
        while (1) {
            vTaskDelay(1);
        }
    }
    ESP_LOGI(pcTaskGetTaskName(0), "Set initial date time done");

    // goto deep sleep
    const int deep_sleep_sec = 2;
    ESP_LOGI(pcTaskGetTaskName(0), "Entering deep sleep for %d seconds",
             deep_sleep_sec);
    esp_deep_sleep(1000000LL * deep_sleep_sec);
}

/*------------------------------------------------------------Get_Clock--------------------------------------------------------------------------------------------*/

void getClock(void* pvParameters) {
    char txtBuf[128];
    if (ds3231_init(I2C_NUM_0) != ESP_OK) {
        ESP_LOGE(pcTaskGetTaskName(0), "Could not init device decriptior. ");
        while (1) vTaskDelay(10);
    }

    // Get RTC date and time
    while (1) {
        // printf("Gia tri Posix ==========%d", time(&ba));

        float       temp;
        struct tm   rtcinfo;
        EventBits_t bits = xEventGroupGetBits(s_wifi_event_group);
        LCD_home();
        LCD_clearScreen();

        if (ds3231_get_temp_float(&temp) != ESP_OK) {
            ESP_LOGE(pcTaskGetTaskName(0), "Could not get temperature.");
            while (1) {
                vTaskDelay(1);
            }
        }
        if (ds3231_get_time(&rtcinfo) != ESP_OK) {
            ESP_LOGE(pcTaskGetTaskName(0), "Could not get time. ");
            while (1) {
                vTaskDelay(1);
            }
        }
        ESP_LOGI(pcTaskGetTaskName(0),
                 "%04d-%02d-%02d %02d:%02d:%02d, %.2f deg Cel", rtcinfo.tm_year,
                 rtcinfo.tm_mon + 1, rtcinfo.tm_mday, rtcinfo.tm_hour,
                 rtcinfo.tm_min, rtcinfo.tm_sec, temp);

        // set_time = mktime(&rtcinfo);

        // Display on time
        LCD_setCursor(0, 0);
        sprintf(txtBuf, "Time: %02d:%02d:%02d", rtcinfo.tm_hour, rtcinfo.tm_min,
                rtcinfo.tm_sec);
        LCD_writeStr(txtBuf);

        // Display on date
        LCD_setCursor(0, 1);
        sprintf(txtBuf, "Date: %02d-%02d-%04d", rtcinfo.tm_mday,
                rtcinfo.tm_mon + 1, rtcinfo.tm_year);
        LCD_writeStr(txtBuf);

        // Display staus wifi
        if (bits & WIFI_CONNECTED_BIT) {
            LCD_setCursor(0, 2);
            sprintf(txtBuf, "WIFI is connected");
            LCD_writeStr(txtBuf);
        } else if (bits & WIFI_FAIL_BIT) {
            LCD_setCursor(0, 2);
            sprintf(txtBuf, "Fail connected to WIFI");
            LCD_writeStr(txtBuf);
        }

        // Display status relay

        // if (status_relay.relay1 == true) {
        // }
        // LCD_setCursor(0, 3);
        // sprintf(txtBuf, "Temperature: %.2f C", temp);
        // LCD_writeStr(txtBuf);

        vTaskDelay(5000 / portTICK_RATE_MS);
        // vTaskDelayUntil(&xLastWakeTime, 100);
    }
}

/*--------------------------------------------------------------Update_Time_now--------------------------------------------------------------*/

void receive_data_LoRa(void* pvParameters) {
    while (1) {
        xSemaphoreTake(xLoraData, portMAX_DELAY);
        printf("Xin chao TMT\n");
        lora_receive();
        if (lora_received()) {
            lora_receive_packet(receive, 20);
            bufferRx[0] = receive[10];
            bufferRx[1] = receive[11];
            bufferRx[2] = receive[12];
            bufferRx[3] = receive[13];
            bufferRx[4] = receive[14];

            // bufferRx[5] = receive[15];
            // bufferRx[6] = receive[16];
            // bufferRx[7] = receive[17];
            // bufferRx[8] = receive[18];
            for (int i = 10; i < 18; i++) {
                printf("Received: %d\n", receive[i]);
            }
            printf("Kich thuoc cua size of: %d\n", sizeof(receive));
            lora_receive();
            data5["sensor/dht22/temp"] = receive[10];
            db.patchData("TGarden", data5);
            data5["sensor/dht22/hum"] = receive[11];
            db.patchData("TGarden", data5);
            data5["sensor/lights"] = receive[12];
            db.patchData("TGarden", data5);
            data5["sensor/rain"] = receive[13];
            db.patchData("TGarden", data5);
            data5["sensor/soil"] = receive[14];
            db.patchData("TGarden", data5);

            if (mode_sys == MODE_AUTO || mode_sys == MODE_ALARM) {
                data2["lamp"] = receive[15];
                db.patchData("Auto", data2);
                data2["fan"] = receive[16];
                db.patchData("Auto", data2);
                data2["pump"] = receive[17];
                db.patchData("Auto", data2);
                data2["hum"] = receive[18];
                db.patchData("Auto", data2);
            }
        }
        xSemaphoreGive(xLoraData);
        vTaskDelay(10000 / portTICK_RATE_MS);
    }
}

static void http_get_task(void* pvParameters) {
    while (1) {
        // if (xLoraData != NULL) {
        //     if (xSemaphoreTake(xLoraData, portMAX_DELAY) == pdTRUE) {
        int err = getaddrinfo(WEB_SERVER, WEB_PORT, &hints, &res);
        s       = socket(res->ai_family, res->ai_socktype, 0);
        if (s < 0) {
            ESP_LOGE(TAG, "... Failed to allocate socket.");
            freeaddrinfo(res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... allocated socket");

        if (connect(s, res->ai_addr, res->ai_addrlen) != 0) {
            ESP_LOGE(TAG, "... socket connect failed errno=%d", errno);
            close(s);
            freeaddrinfo(res);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }

        freeaddrinfo(res);
        sprintf(REQUEST,
                "GET "
                "http://api.thingspeak.com/"
                "update.json?api_key=9C4L9E6D45DL1BGD&field1=%d&field2="
                "%d&field3=%d&field4=%d\r\n",
                bufferRx[0], bufferRx[1], bufferRx[4], bufferRx[2]);
        if (write(s, REQUEST, strlen(REQUEST)) < 0) {
            ESP_LOGE(TAG, "... socket send failed");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... socket send success");

        close(s);
        // xSemaphoreGive(xLoraData);
        //     }
        // }
        vTaskDelay(60000 / portTICK_PERIOD_MS);
    }
}

void read_mode_firebase(void* pvParameters) {
    while (1) {
        xSemaphoreTake(xLoraData, portMAX_DELAY);
        std::string Manual = db.getData("TGarden/S1_Manual").asString();
        Manu               = atoi(Manual.c_str());
        std::string Auto   = db.getData("TGarden/S1_Auto").asString();
        Aut                = atoi(Auto.c_str());
        std::string Al     = db.getData("TGarden/Alarm/Status").asString();
        Ala                = atoi(Al.c_str());

        if (Aut == 1) {
            mode_sys     = MODE_AUTO;
            send_mode[0] = 'A';
            lora_send_packet((uint8_t*)send_mode, 1);
            printf("Auto------------\n");
        }
        if (Manu == 1) {
            mode_sys     = MODE_MANUAL;
            send_mode[0] = 'M';
            lora_send_packet((uint8_t*)send_mode, 1);
            printf("Manual------------\n");
        }
        if (Ala == 1) {
            mode_sys     = MODE_ALARM;
            send_mode[0] = 'L';
            lora_send_packet((uint8_t*)send_mode, 1);
            printf("Alarm------------\n");
        }
        // xEventGroupSetBits(xLCDButtonHandle, MODE_SYSTEMS);
        xSemaphoreGive(xLoraData);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

void lcd_display(void) {
    LCD_clearScreen();
    LCD_home();
    LCD_setCursor(0, 0);
    sprintf(timeBuf, "LUAN VAN TOT NGHIEP");
    LCD_writeStr(timeBuf);

    LCD_setCursor(0, 1);
    sprintf(timeBuf, "-------****--------");
    LCD_writeStr(timeBuf);
    // Display on humidity
    LCD_setCursor(1, 2);
    sprintf(timeBuf, "CHAM SOC CAY TRONG");
    LCD_writeStr(timeBuf);

    // Display on temperature
    LCD_setCursor(7, 3);
    sprintf(timeBuf, "TU DONG");
    LCD_writeStr(timeBuf);
}

void app_main(void) {
    ++boot_count;
    xLCDButtonHandle = xEventGroupCreate();
    xLoraData        = xSemaphoreCreateMutex();

    adv_button_config_t button_cfg = ADVANCED_BUTTON_CONFIG_DEFAULT();

    lora_init();
    lora_set_frequency(433e6);
    lora_enable_crc();
    lora_set_coding_rate(CONFIG_CODING_RATE);
    lora_set_bandwidth(CONFIG_BANDWIDTH);
    lora_set_spreading_factor(CONFIG_SF_RATE);

    //     /*Init Firebase*/
    wifiInit(SSID, PASSWORD);
    user_account_t account = {USER_EMAIL, USER_PASSWORD};
    // blocking until it connects
    initialize_sntp();
    // Config and Authentication
    app.loginUserAccount(account);
    i2c_master_init();
    lcd_init(I2C_NUM_0);
    ds3231_init(I2C_NUM_0);
    // db.semss();
#if CONFIG_SET_CLOCK
    if (boot_count == 1) {
        xTaskCreate(setClock, "setClock", 1024 * 4, NULL, 2, NULL);
    }
#endif

    adv_button_create(BUTTON_MENU, &button_cfg, menubutton_callback,
                      (void*)BUTTON_MENU);
    adv_button_create(BUTTON_UP, &button_cfg, upbutton_callback,
                      (void*)BUTTON_UP);
    adv_button_create(BUTTON_DOWN, &button_cfg, downbutton_callback,
                      (void*)BUTTON_DOWN);
    adv_button_create(BUTTON_MODE, &button_cfg, modebutton_callback,
                      (void*)BUTTON_MODE);

    adv_button_create(BUTTON_RELAY1, &button_cfg, button_relay1,
                      (void*)BUTTON_RELAY1);

    adv_button_create(BUTTON_RELAY2, &button_cfg, button_relay2,
                      (void*)BUTTON_RELAY2);

    adv_button_create(BUTTON_RELAY3, &button_cfg, button_relay3,
                      (void*)BUTTON_RELAY3);

    xTaskCreate(&lcd_task1, "LCD TASK", 1024 * 4, NULL, 6, NULL);
    // xTaskCreate(&task_relay1, "RELAY1 BUTTON", 1024 * 4, NULL, 8, NULL);
    // xTaskCreate(&task_relay2, "RELAY2 BUTTON", 1024 * 4, NULL, 8, NULL);
    // xTaskCreate(&task_relay3, "RELAY3 BUTTON", 1024 * 4, NULL, 8, NULL);

    xTaskCreate(&read_mode_firebase, "Read Mode", 1024 * 8, NULL, 9, NULL);

    xTaskCreate(&mode_systems, "MODE SYSTEMS", 1024 * 10, NULL, 9, NULL);
    xTaskCreate(&receive_data_LoRa, "Data LoRa Received", 1024 * 4, NULL, 10,
                NULL);
    xTaskCreate(&button_mode_sys, "Mode Button", 1024 * 4, NULL, 10, NULL);
    xTaskCreate(&http_get_task, "http_get_task", 4096, NULL, 6, NULL);

    // lcd_display();
}

#ifdef __cplusplus
}
#endif