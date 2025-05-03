/*
 * SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_types.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/i2c_master.h"
#include "i2c_ht16k33.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_HT16K33_MAX_TRANS_UNIT (48+1) // +1 for address
// Different HT16K33 device might share one I2C bus

static const char TAG[] = "i2c-ht16k33";

esp_err_t i2c_ht16k33_init(i2c_master_bus_handle_t bus_handle, uint16_t device_address, i2c_ht16k33_handle_t *ht16k33_handle)
{
    esp_err_t ret = ESP_OK;
    i2c_ht16k33_handle_t out_handle;
    out_handle = (i2c_ht16k33_handle_t)calloc(1, sizeof(*out_handle));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c ht16k33 device");

    i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = device_address,
        .scl_speed_hz = 100000,
    };

    if (out_handle->i2c_dev == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(bus_handle, &i2c_dev_conf, &out_handle->i2c_dev), err, TAG, "i2c new bus failed");
    }

    out_handle->buffer = (uint8_t*)calloc(1, I2C_HT16K33_MAX_TRANS_UNIT);
    ESP_GOTO_ON_FALSE(out_handle->buffer, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c ht16k33 device buffer");

    out_handle->addr_wordlen  = 1;
    out_handle->write_time_ms = 10;
    *ht16k33_handle = out_handle;

    return ret;

    err:
        if (out_handle && out_handle->i2c_dev) {
            i2c_master_bus_rm_device(out_handle->i2c_dev);
        }
        free(out_handle);
        return ret;
}

esp_err_t i2c_ht16k33_write(i2c_ht16k33_handle_t ht16k33_handle, uint32_t address, const uint8_t *data, uint32_t size)
{
    ESP_RETURN_ON_FALSE(ht16k33_handle, ESP_ERR_NO_MEM, TAG, "no mem for buffer");
    for (int i = 0; i < ht16k33_handle->addr_wordlen; i++) {
        ht16k33_handle->buffer[i] = (address & (0xff << ((ht16k33_handle->addr_wordlen - 1 - i) * 8))) >> ((ht16k33_handle->addr_wordlen - 1 - i) * 8);
    }
    memcpy(ht16k33_handle->buffer + ht16k33_handle->addr_wordlen, data, size);

    return i2c_master_transmit(ht16k33_handle->i2c_dev, ht16k33_handle->buffer, ht16k33_handle->addr_wordlen + size, -1);
}

esp_err_t i2c_ht16k33_read(i2c_ht16k33_handle_t ht16k33_handle, uint32_t address, uint8_t *data, uint32_t size)
{
    ESP_RETURN_ON_FALSE(ht16k33_handle, ESP_ERR_NO_MEM, TAG, "no mem for buffer");
    for (int i = 0; i < ht16k33_handle->addr_wordlen; i++) {
        ht16k33_handle->buffer[i] = (address & (0xff << ((ht16k33_handle->addr_wordlen - 1 - i) * 8))) >> ((ht16k33_handle->addr_wordlen - 1 - i) * 8);
    }

    return i2c_master_transmit_receive(ht16k33_handle->i2c_dev, ht16k33_handle->buffer, ht16k33_handle->addr_wordlen, data, size, -1);
}

void i2c_ht16k33_wait_idle(i2c_ht16k33_handle_t ht16k33_handle)
{
    // This is time for HT16K33 Self-Timed Write Cycle
    vTaskDelay(pdMS_TO_TICKS(ht16k33_handle->write_time_ms));
}
