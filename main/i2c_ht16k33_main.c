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
#include "driver/i2c_master.h"
#include "i2c_ht16k33.h"
#include "esp_task_wdt.h"
#include "esp_check.h"
#include "esp_random.h"

#include <inttypes.h>
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"

#define SCL_IO_PIN CONFIG_I2C_MASTER_SCL
#define SDA_IO_PIN CONFIG_I2C_MASTER_SDA
#define MASTER_FREQUENCY CONFIG_I2C_MASTER_FREQUENCY

#define TAG "MAIN"
#include "esp_log.h"

static void i2c_init(i2c_ht16k33_handle_t *ht16k33_handle){
    printf("I2C_INIT\n");
    // I2C declare new master bus
    i2c_master_bus_handle_t bus_handle;
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_RC_FAST,
        .i2c_port   = I2C_NUM_0,
        .sda_io_num = 22, // SDA_IO_PIN
        .scl_io_num = 23, // SCL_IO_PIN
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

void app_main(void)
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
