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
    uint32_t address = 0x21;
    uint8_t  data    = 0x0;
    ESP_ERROR_CHECK(i2c_ht16k33_write(*ht16k33_handle, address, &data, 0));

    // Config ht16k33: ROW/INT default
    // Config ht16k33: Dimming default
    address = 0xE0; // | (uint8_t) brigthness
    ESP_ERROR_CHECK(i2c_ht16k33_write(*ht16k33_handle, address, &data, 0));

    // Config ht16k33: Blinking default + display on
    address = 0x81;
    ESP_ERROR_CHECK(i2c_ht16k33_write(*ht16k33_handle, address, &data, 0));

    // Display something
    address = 0x00;
    uint8_t wdata[16];
    wdata[0]  = 0x3F;

    // Exterieur
    // lsb -> A0 - A1
    //        A1 - A2
    //        A2 - B
    //        A3 - C
    //        A4 - D1
    //        A5 - D2
    //        A6 - E
    // msb -> A7 - F

    wdata[1]  = 0x4A; // Interieur

    // lsb -> A8  - G1
    //        A9  - G2
    //        A10 - H
    //        A11 - J
    //        A12 - K
    //        A13 - L
    //        A14 - M
    // msb -> A15 - N

    wdata[2]  = 0xC3;
    wdata[3]  = 0x01;


    wdata[4]  = 0;
    wdata[5]  = 0;
    wdata[6]  = 0;
    wdata[7]  = 0;
    wdata[8]  = 0;
    wdata[9]  = 0;
    wdata[10] = 0;
    wdata[11] = 0;
    wdata[12] = 0;
    wdata[13] = 0;
    wdata[14] = 0;
    wdata[15] = 0;
    ESP_ERROR_CHECK(i2c_ht16k33_write(*ht16k33_handle, address, wdata, 16));

    // address = 0x20;
    // ESP_ERROR_CHECK(i2c_ht16k33_read(*ht16k33_handle, address, &data, 1));
    // printf("Read 0x%02lx = 0x%02x \n", address, data);
}

void app_main(void)
{
    // Init var
    i2c_ht16k33_handle_t ht16k33_handle;
    // uint8_t  read_buf[1]  = {0};
    // uint8_t  write_buf[1] = {0};
    // uint32_t block_addr = 0x00;

    // Watchdog cfg
    // esp_task_wdt_deinit();

    // I2C init
    i2c_init(&ht16k33_handle);

    printf("Hello world, human!\n");
    // ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));

    // Forever loop
    // while (1) {
        // Needs wait for eeprom hardware done, referring from datasheet
        // i2c_ht16k33_wait_idle(ht16k33_handle);

        // vTaskDelay(pdMS_TO_TICKS(10));

        // i2c_master_receive(&bus_handle, read_buf, 1, 100);

        // read_buf[0] = 0;
        // ESP_ERROR_CHECK(i2c_ht16k33_read(ht16k33_handle, block_addr, read_buf, 1));
        // printf("Read1 0x%02x\n ", read_buf[0]);
        // i2c_ht16k33_wait_idle(ht16k33_handle);

        // write_buf[0] = 3;
        // ESP_ERROR_CHECK(i2c_ht16k33_write(ht16k33_handle,block_addr, write_buf, 1));

        // // i2c_ht16k33_wait_idle(ht16k33_handle);

        // printf("Read1 0x%02x\n ", read_buf[0]);
        // ESP_ERROR_CHECK(i2c_ht16k33_read(ht16k33_handle, block_addr, read_buf, 1));
        // printf("Read2 0x%02x\n ", read_buf[0]);

    //     vTaskDelay(50);
    // }



    // /* Print chip information */
    // esp_chip_info_t chip_info;
    // uint32_t flash_size;
    // esp_chip_info(&chip_info);
    // printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
    //        CONFIG_IDF_TARGET,
    //        chip_info.cores,
    //        (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
    //        (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
    //        (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
    //        (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    // unsigned major_rev = chip_info.revision / 100;
    // unsigned minor_rev = chip_info.revision % 100;
    // printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    // if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
    //     printf("Get flash size failed");
    //     return;
    // }

    // printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
    //        (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    // printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    // for (int i = 10; i >= 0; i--) {
    //     printf("Restarting in %d seconds...\n", i);
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }
    // printf("Restarting now.\n");
    // fflush(stdout);
    // esp_restart();

}
