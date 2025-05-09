/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/i2c_master.h"
#include "i2c_ht16k33.h"
#include "esp_task_wdt.h"
#include "esp_check.h"
#include "esp_random.h"

#include <inttypes.h>
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#define SDA_IO_PIN                  22
#define SCL_IO_PIN                  23

#define EXAMPLE_ESP_WIFI_SSID       "generic-ssid"
#define EXAMPLE_ESP_WIFI_PASS       "generic-password"
#define EXAMPLE_ESP_MAXIMUM_RETRY   5

// #define ESP_WIFI_SAE_MODE           WPA3_SAE_PWE_HUNT_AND_PECK
// #define EXAMPLE_H2E_IDENTIFIER      ""
// #define ESP_WIFI_SAE_MODE           WPA3_SAE_PWE_HASH_TO_ELEMENT
// #define EXAMPLE_H2E_IDENTIFIER      CONFIG_ESP_WIFI_PW_ID
#define ESP_WIFI_SAE_MODE           WPA3_SAE_PWE_BOTH
#define EXAMPLE_H2E_IDENTIFIER      ""

// #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
// #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
// #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
// #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
// #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
// #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
// #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

#define TAG "MAIN"
#include "esp_log.h"

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

static int s_retry_num = 0;

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void i2c_init(i2c_ht16k33_handle_t *ht16k33_handle){
    printf("I2C_INIT\n");
    // I2C declare new master bus
    i2c_master_bus_handle_t bus_handle;
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_RC_FAST,
        .i2c_port   = I2C_NUM_0,
        .sda_io_num = SDA_IO_PIN,
        .scl_io_num = SCL_IO_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    // Init ht16k33 device
    ESP_ERROR_CHECK(i2c_ht16k33_init(bus_handle, 0x70, ht16k33_handle));

    // Config ht16k33: Turn on System oscillator
    ESP_ERROR_CHECK(i2c_ht16k33_turn_on_oscillator(*ht16k33_handle));

    // Config ht16k33: Set dimming
    ESP_ERROR_CHECK(i2c_ht16k33_set_brightness(*ht16k33_handle,0));

    // Config ht16k33: Blinking default + display on
    ESP_ERROR_CHECK(i2c_ht16k33_set_blinking(*ht16k33_handle,BLINK_OFF));
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (password len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
             * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
            .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

void app_mainxxx(void)
{
    // Init var
    i2c_ht16k33_handle_t ht16k33_handle;

    // Watchdog cfg
    // esp_task_wdt_deinit();

    // I2C init
    i2c_init(&ht16k33_handle);

    uint16_t display_char[36] = { \
        DISPLAY_16_SEGMENT_VAL_A, \
        DISPLAY_16_SEGMENT_VAL_B, \
        DISPLAY_16_SEGMENT_VAL_C, \
        DISPLAY_16_SEGMENT_VAL_D, \
        DISPLAY_16_SEGMENT_VAL_E, \
        DISPLAY_16_SEGMENT_VAL_F, \
        DISPLAY_16_SEGMENT_VAL_G, \
        DISPLAY_16_SEGMENT_VAL_H, \
        DISPLAY_16_SEGMENT_VAL_I, \
        DISPLAY_16_SEGMENT_VAL_J, \
        DISPLAY_16_SEGMENT_VAL_K, \
        DISPLAY_16_SEGMENT_VAL_L, \
        DISPLAY_16_SEGMENT_VAL_M, \
        DISPLAY_16_SEGMENT_VAL_N, \
        DISPLAY_16_SEGMENT_VAL_O, \
        DISPLAY_16_SEGMENT_VAL_P, \
        DISPLAY_16_SEGMENT_VAL_Q, \
        DISPLAY_16_SEGMENT_VAL_R, \
        DISPLAY_16_SEGMENT_VAL_S, \
        DISPLAY_16_SEGMENT_VAL_T, \
        DISPLAY_16_SEGMENT_VAL_U, \
        DISPLAY_16_SEGMENT_VAL_V, \
        DISPLAY_16_SEGMENT_VAL_W, \
        DISPLAY_16_SEGMENT_VAL_X, \
        DISPLAY_16_SEGMENT_VAL_Y, \
        DISPLAY_16_SEGMENT_VAL_Z, \
        DISPLAY_16_SEGMENT_VAL_0, \
        DISPLAY_16_SEGMENT_VAL_1, \
        DISPLAY_16_SEGMENT_VAL_2, \
        DISPLAY_16_SEGMENT_VAL_3, \
        DISPLAY_16_SEGMENT_VAL_4, \
        DISPLAY_16_SEGMENT_VAL_5, \
        DISPLAY_16_SEGMENT_VAL_6, \
        DISPLAY_16_SEGMENT_VAL_7, \
        DISPLAY_16_SEGMENT_VAL_8, \
        DISPLAY_16_SEGMENT_VAL_9  \
    };

    uint8_t  wdata[16];
    for (int i = 0; i < 8; ++i){
        i2c_ht16k33_buffer_write(wdata,DISPLAY_16_SEGMENT_VAL_NULL,i);
    }

    // Forever loop
    uint8_t time_idx = 0;
    uint8_t row0_idx = 0;
    uint8_t row1_idx = 1;
    while (1) {
        // Display something
        i2c_ht16k33_buffer_write(wdata,display_char[row0_idx],0);
        // i2c_ht16k33_buffer_write(wdata,esp_random() & 0xFFFF,0);
        i2c_ht16k33_buffer_write(wdata,display_char[row1_idx],1);
        ESP_ERROR_CHECK(i2c_ht16k33_update_ram(ht16k33_handle,wdata));
        // delay
        vTaskDelay(100/portTICK_PERIOD_MS); 
        time_idx++;
        printf("time_idx %d\n",time_idx);

        // Update row 0 index
        row0_idx = esp_random() & 0x3F;
        if (row0_idx > 35){
            row0_idx -= 35;
        }

        // Update row 1 index (slower)
        if (time_idx < 5){
            continue;
        } else {
            time_idx = 0;
        }
        if (row1_idx < 35){
            row1_idx++;
        } else {
            row1_idx = 0;
        }
    }
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}

void app_main(void)
{
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();
}
