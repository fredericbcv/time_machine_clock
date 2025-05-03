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
#define PORT_NUMBER -1
#define LENGTH 48

#define TAG "MAIN"
#include "esp_log.h"

// static void i2c_init(i2c_ht16k33_handle_t *ht16k33_handle){

//     i2c_master_bus_handle_t bus_handle;

//     i2c_master_bus_config_t i2c_bus_config = {
//         .clk_source = I2C_CLK_SRC_DEFAULT,
//         .i2c_port = PORT_NUMBER,
//         .scl_io_num = SCL_IO_PIN,
//         .sda_io_num = SDA_IO_PIN,
//         .glitch_ignore_cnt = 7,
//     };

//     ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));

//     i2c_ht16k33_config_t ht16k33_config = {
//         .ht16k33_device.scl_speed_hz = MASTER_FREQUENCY,
//         .ht16k33_device.device_address = 0xE0,
//         .addr_wordlen = 1,
//         .write_time_ms = 10,
//     };

//     ESP_ERROR_CHECK(i2c_ht16k33_init(bus_handle, &ht16k33_config, ht16k33_handle));

// }

void app_main(void)
{
    // Init var
    // i2c_ht16k33_handle_t ht16k33_handle;
    // uint8_t  read_buf[1];
    // uint32_t block_addr = 0x40;

    printf("SCL_IO_PIN: %d \n",SCL_IO_PIN);
    printf("SDA_IO_PIN: %d \n",SDA_IO_PIN);
    printf("MASTER_FREQUENCY: %d \n",MASTER_FREQUENCY);

    // Watchdog cfg
    // esp_task_wdt_deinit();

    // I2C init
    // i2c_init(&ht16k33_handle);

    i2c_master_bus_handle_t bus_handle;
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_RC_FAST,
        .i2c_port   = I2C_NUM_0,
        .sda_io_num = 22,
        .scl_io_num = 23,
        .glitch_ignore_cnt = 7,
        // .intr_priority = 3,
        // .trans_queue_depth = 0,
        // .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0xE0,
        .scl_speed_hz = 100000,
    };
    i2c_master_dev_handle_t dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));


    printf("toto1\n");

    // while(true){
    //     i2c_master_probe(bus_handle, 0xE0, 100);
    // }
    // printf("toto2\n");


    //     i2c_master_probe(bus_handle, 0xE0, 100);
    // }

    if (i2c_master_probe(bus_handle, 0x70, 100) != ESP_ERR_NOT_FOUND)
    {
        ESP_LOGI(TAG, "Found I2C-Device @ 0x%02X", 0xE0);
    }
    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
    printf("Hello world, human!\n");

    // for (uint8_t i = 1; i < 128; i++)
    // {
    //     if (i2c_master_probe(bus_handle, i, 100) != ESP_ERR_NOT_FOUND)
    //     {
    //         ESP_LOGI(TAG, "Found I2C-Device @ 0x%02X", i);
    //     }
    // }


    // Forever loop
    // while (1) {
        // Needs wait for eeprom hardware done, referring from datasheet
        // i2c_ht16k33_wait_idle(ht16k33_handle);

        // vTaskDelay(pdMS_TO_TICKS(10));

        // i2c_master_receive(&bus_handle, read_buf, 1, 100);

        // printf("0\n");
        // ESP_ERROR_CHECK(i2c_ht16k33_read(ht16k33_handle, block_addr, read_buf, 1));
        // printf("1");
    //     printf("%02x\n ", read_buf[0]);
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
