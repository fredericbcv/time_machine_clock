/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <stdint.h>
#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

struct i2c_ht16k33_t {
    i2c_master_dev_handle_t i2c_dev;      /*!< I2C device handle */
    uint8_t addr_wordlen;                 /*!< block address wordlen */
    uint8_t *buffer;                      /*!< I2C transaction buffer */
    uint8_t write_time_ms;                /*!< I2C eeprom write time(ms)*/
};
typedef struct i2c_ht16k33_t i2c_ht16k33_t;

/* handle of EEPROM device */
typedef struct i2c_ht16k33_t *i2c_ht16k33_handle_t;


/**
 * @brief Init an ht16k33 device.
 *
 * @param[in] bus_handle I2C master bus handle
 * @param[in] eeprom_config Configuration of EEPROM
 * @param[out] eeprom_handle Handle of EEPROM
 * @return ESP_OK: Init success. ESP_FAIL: Not success.
 */
esp_err_t i2c_ht16k33_init(i2c_master_bus_handle_t bus_handle, uint16_t device_address, i2c_ht16k33_handle_t *ht16k33_handle);

/**
 * @brief Write data to ht16k33
 *
 * @param[in] eeprom_handle ht16k33 handle
 * @param[in] address Block address inside ht16k33
 * @param[in] data Data to write
 * @param[in] size Data write size
 * @return ESP_OK: Write success. Otherwise failed, please check I2C function fail reason.
 */
esp_err_t i2c_ht16k33_write(i2c_ht16k33_handle_t ht16k33_handle, uint32_t address, const uint8_t *data, uint32_t size);

/**
 * @brief Read data from ht16k33
 *
 * @param eeprom_handle ht16k33 handle
 * @param address Block address inside ht16k33
 * @param data Data read from ht16k33
 * @param size Data read size
 * @return ESP_OK: Read success. Otherwise failed, please check I2C function fail reason.
 */
esp_err_t i2c_ht16k33_read(i2c_ht16k33_handle_t ht16k33_handle, uint32_t address, uint8_t *data, uint32_t size);

/**
 * @brief Wait eeprom finish. Typically 5ms
 *
 * @param eeprom_handle ht16k33 handle
 */
void i2c_ht16k33_wait_idle(i2c_ht16k33_handle_t ht16k33_handle);

#ifdef __cplusplus
}
#endif
