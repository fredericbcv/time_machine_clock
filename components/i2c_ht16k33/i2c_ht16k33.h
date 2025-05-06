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

#ifndef I2C_HT16K33_H
#define I2C_HT16K33_H

#define DISPLAY_16_SEGMENT_LED_A1       0x0001
#define DISPLAY_16_SEGMENT_LED_A2       0x0002
#define DISPLAY_16_SEGMENT_LED_B        0x0004
#define DISPLAY_16_SEGMENT_LED_C        0x0008
#define DISPLAY_16_SEGMENT_LED_D1       0x0010
#define DISPLAY_16_SEGMENT_LED_D2       0x0020
#define DISPLAY_16_SEGMENT_LED_E        0x0040
#define DISPLAY_16_SEGMENT_LED_F        0x0080
#define DISPLAY_16_SEGMENT_LED_G1       0x0100
#define DISPLAY_16_SEGMENT_LED_G2       0x0200
#define DISPLAY_16_SEGMENT_LED_H        0x0400
#define DISPLAY_16_SEGMENT_LED_J        0x0800
#define DISPLAY_16_SEGMENT_LED_K        0x1000
#define DISPLAY_16_SEGMENT_LED_L        0x2000
#define DISPLAY_16_SEGMENT_LED_M        0x4000
#define DISPLAY_16_SEGMENT_LED_N        0x8000

#define DISPLAY_16_SEGMENT_VAL_NULL     0    

#define DISPLAY_16_SEGMENT_VAL_A       (DISPLAY_16_SEGMENT_LED_A1 | \
                                        DISPLAY_16_SEGMENT_LED_A2 | \
                                        DISPLAY_16_SEGMENT_LED_B  | \
                                        DISPLAY_16_SEGMENT_LED_C  | \
                                        DISPLAY_16_SEGMENT_LED_E  | \
                                        DISPLAY_16_SEGMENT_LED_F  | \
                                        DISPLAY_16_SEGMENT_LED_G1 | \
                                        DISPLAY_16_SEGMENT_LED_G2   )

#define DISPLAY_16_SEGMENT_VAL_B       (DISPLAY_16_SEGMENT_LED_A1 | \
                                        DISPLAY_16_SEGMENT_LED_A2 | \
                                        DISPLAY_16_SEGMENT_LED_B  | \
                                        DISPLAY_16_SEGMENT_LED_C  | \
                                        DISPLAY_16_SEGMENT_LED_D1 | \
                                        DISPLAY_16_SEGMENT_LED_D2 | \
                                        DISPLAY_16_SEGMENT_LED_G2 | \
                                        DISPLAY_16_SEGMENT_LED_J  | \
                                        DISPLAY_16_SEGMENT_LED_M    )

#define DISPLAY_16_SEGMENT_VAL_C       (DISPLAY_16_SEGMENT_LED_A1 | \
                                        DISPLAY_16_SEGMENT_LED_A2 | \
                                        DISPLAY_16_SEGMENT_LED_D1 | \
                                        DISPLAY_16_SEGMENT_LED_D2 | \
                                        DISPLAY_16_SEGMENT_LED_E  | \
                                        DISPLAY_16_SEGMENT_LED_F    )

#define DISPLAY_16_SEGMENT_VAL_D       (DISPLAY_16_SEGMENT_LED_A1 | \
                                        DISPLAY_16_SEGMENT_LED_A2 | \
                                        DISPLAY_16_SEGMENT_LED_B  | \
                                        DISPLAY_16_SEGMENT_LED_C  | \
                                        DISPLAY_16_SEGMENT_LED_D1 | \
                                        DISPLAY_16_SEGMENT_LED_D2 | \
                                        DISPLAY_16_SEGMENT_LED_J  | \
                                        DISPLAY_16_SEGMENT_LED_M    )

#define DISPLAY_16_SEGMENT_VAL_E       (DISPLAY_16_SEGMENT_LED_A1 | \
                                        DISPLAY_16_SEGMENT_LED_A2 | \
                                        DISPLAY_16_SEGMENT_LED_D1 | \
                                        DISPLAY_16_SEGMENT_LED_D2 | \
                                        DISPLAY_16_SEGMENT_LED_E  | \
                                        DISPLAY_16_SEGMENT_LED_F  | \
                                        DISPLAY_16_SEGMENT_LED_G1   )

#define DISPLAY_16_SEGMENT_VAL_F       (DISPLAY_16_SEGMENT_LED_A1 | \
                                        DISPLAY_16_SEGMENT_LED_A2 | \
                                        DISPLAY_16_SEGMENT_LED_E  | \
                                        DISPLAY_16_SEGMENT_LED_F  | \
                                        DISPLAY_16_SEGMENT_LED_G1   )

#define DISPLAY_16_SEGMENT_VAL_G       (DISPLAY_16_SEGMENT_LED_A1 | \
                                        DISPLAY_16_SEGMENT_LED_A2 | \
                                        DISPLAY_16_SEGMENT_LED_C  | \
                                        DISPLAY_16_SEGMENT_LED_D1 | \
                                        DISPLAY_16_SEGMENT_LED_D2 | \
                                        DISPLAY_16_SEGMENT_LED_E  | \
                                        DISPLAY_16_SEGMENT_LED_F  | \
                                        DISPLAY_16_SEGMENT_LED_G2   )

#define DISPLAY_16_SEGMENT_VAL_H       (DISPLAY_16_SEGMENT_LED_B  | \
                                        DISPLAY_16_SEGMENT_LED_C  | \
                                        DISPLAY_16_SEGMENT_LED_E  | \
                                        DISPLAY_16_SEGMENT_LED_F  | \
                                        DISPLAY_16_SEGMENT_LED_G1 | \
                                        DISPLAY_16_SEGMENT_LED_G2   )

#define DISPLAY_16_SEGMENT_VAL_I       (DISPLAY_16_SEGMENT_LED_A1 | \
                                        DISPLAY_16_SEGMENT_LED_A2 | \
                                        DISPLAY_16_SEGMENT_LED_D1 | \
                                        DISPLAY_16_SEGMENT_LED_D2 | \
                                        DISPLAY_16_SEGMENT_LED_J  | \
                                        DISPLAY_16_SEGMENT_LED_M    )

#define DISPLAY_16_SEGMENT_VAL_J       (DISPLAY_16_SEGMENT_LED_B  | \
                                        DISPLAY_16_SEGMENT_LED_C  | \
                                        DISPLAY_16_SEGMENT_LED_D1 | \
                                        DISPLAY_16_SEGMENT_LED_D2 | \
                                        DISPLAY_16_SEGMENT_LED_E    )

#define DISPLAY_16_SEGMENT_VAL_K       (DISPLAY_16_SEGMENT_LED_E  | \
                                        DISPLAY_16_SEGMENT_LED_F  | \
                                        DISPLAY_16_SEGMENT_LED_G1 | \
                                        DISPLAY_16_SEGMENT_LED_K  | \
                                        DISPLAY_16_SEGMENT_LED_L    )

#define DISPLAY_16_SEGMENT_VAL_L       (DISPLAY_16_SEGMENT_LED_D1 | \
                                        DISPLAY_16_SEGMENT_LED_D2 | \
                                        DISPLAY_16_SEGMENT_LED_E  | \
                                        DISPLAY_16_SEGMENT_LED_F    )

#define DISPLAY_16_SEGMENT_VAL_M       (DISPLAY_16_SEGMENT_LED_B  | \
                                        DISPLAY_16_SEGMENT_LED_C  | \
                                        DISPLAY_16_SEGMENT_LED_E  | \
                                        DISPLAY_16_SEGMENT_LED_F  | \
                                        DISPLAY_16_SEGMENT_LED_H  | \
                                        DISPLAY_16_SEGMENT_LED_K    )

#define DISPLAY_16_SEGMENT_VAL_N       (DISPLAY_16_SEGMENT_LED_B  | \
                                        DISPLAY_16_SEGMENT_LED_C  | \
                                        DISPLAY_16_SEGMENT_LED_E  | \
                                        DISPLAY_16_SEGMENT_LED_F  | \
                                        DISPLAY_16_SEGMENT_LED_H  | \
                                        DISPLAY_16_SEGMENT_LED_L    )

#define DISPLAY_16_SEGMENT_VAL_O       (DISPLAY_16_SEGMENT_LED_A1 | \
                                        DISPLAY_16_SEGMENT_LED_A2 | \
                                        DISPLAY_16_SEGMENT_LED_B  | \
                                        DISPLAY_16_SEGMENT_LED_C  | \
                                        DISPLAY_16_SEGMENT_LED_D1 | \
                                        DISPLAY_16_SEGMENT_LED_D2 | \
                                        DISPLAY_16_SEGMENT_LED_E  | \
                                        DISPLAY_16_SEGMENT_LED_F    )

#define DISPLAY_16_SEGMENT_VAL_P       (DISPLAY_16_SEGMENT_LED_A1 | \
                                        DISPLAY_16_SEGMENT_LED_A2 | \
                                        DISPLAY_16_SEGMENT_LED_B  | \
                                        DISPLAY_16_SEGMENT_LED_E  | \
                                        DISPLAY_16_SEGMENT_LED_F  | \
                                        DISPLAY_16_SEGMENT_LED_G1 | \
                                        DISPLAY_16_SEGMENT_LED_G2   )

#define DISPLAY_16_SEGMENT_VAL_Q       (DISPLAY_16_SEGMENT_LED_A1 | \
                                        DISPLAY_16_SEGMENT_LED_A2 | \
                                        DISPLAY_16_SEGMENT_LED_B  | \
                                        DISPLAY_16_SEGMENT_LED_C  | \
                                        DISPLAY_16_SEGMENT_LED_D1 | \
                                        DISPLAY_16_SEGMENT_LED_D2 | \
                                        DISPLAY_16_SEGMENT_LED_E  | \
                                        DISPLAY_16_SEGMENT_LED_F  | \
                                        DISPLAY_16_SEGMENT_LED_L    )

#define DISPLAY_16_SEGMENT_VAL_R       (DISPLAY_16_SEGMENT_LED_A1 | \
                                        DISPLAY_16_SEGMENT_LED_A2 | \
                                        DISPLAY_16_SEGMENT_LED_B  | \
                                        DISPLAY_16_SEGMENT_LED_E  | \
                                        DISPLAY_16_SEGMENT_LED_F  | \
                                        DISPLAY_16_SEGMENT_LED_G1 | \
                                        DISPLAY_16_SEGMENT_LED_G2 | \
                                        DISPLAY_16_SEGMENT_LED_L    )

#define DISPLAY_16_SEGMENT_VAL_S       (DISPLAY_16_SEGMENT_LED_A1 | \
                                        DISPLAY_16_SEGMENT_LED_A2 | \
                                        DISPLAY_16_SEGMENT_LED_C  | \
                                        DISPLAY_16_SEGMENT_LED_D1 | \
                                        DISPLAY_16_SEGMENT_LED_D2 | \
                                        DISPLAY_16_SEGMENT_LED_F  | \
                                        DISPLAY_16_SEGMENT_LED_G1 | \
                                        DISPLAY_16_SEGMENT_LED_G2   )

#define DISPLAY_16_SEGMENT_VAL_T       (DISPLAY_16_SEGMENT_LED_A1 | \
                                        DISPLAY_16_SEGMENT_LED_A2 | \
                                        DISPLAY_16_SEGMENT_LED_J  | \
                                        DISPLAY_16_SEGMENT_LED_M    )

#define DISPLAY_16_SEGMENT_VAL_U       (DISPLAY_16_SEGMENT_LED_B  | \
                                        DISPLAY_16_SEGMENT_LED_C  | \
                                        DISPLAY_16_SEGMENT_LED_D1 | \
                                        DISPLAY_16_SEGMENT_LED_D2 | \
                                        DISPLAY_16_SEGMENT_LED_E  | \
                                        DISPLAY_16_SEGMENT_LED_F    )

#define DISPLAY_16_SEGMENT_VAL_V       (DISPLAY_16_SEGMENT_LED_E  | \
                                        DISPLAY_16_SEGMENT_LED_F  | \
                                        DISPLAY_16_SEGMENT_LED_K  | \
                                        DISPLAY_16_SEGMENT_LED_N    )

#define DISPLAY_16_SEGMENT_VAL_W       (DISPLAY_16_SEGMENT_LED_B  | \
                                        DISPLAY_16_SEGMENT_LED_C  | \
                                        DISPLAY_16_SEGMENT_LED_E  | \
                                        DISPLAY_16_SEGMENT_LED_F  | \
                                        DISPLAY_16_SEGMENT_LED_L  | \
                                        DISPLAY_16_SEGMENT_LED_N    )

#define DISPLAY_16_SEGMENT_VAL_X       (DISPLAY_16_SEGMENT_LED_H  | \
                                        DISPLAY_16_SEGMENT_LED_K  | \
                                        DISPLAY_16_SEGMENT_LED_L  | \
                                        DISPLAY_16_SEGMENT_LED_N    )

#define DISPLAY_16_SEGMENT_VAL_Y       (DISPLAY_16_SEGMENT_LED_H  | \
                                        DISPLAY_16_SEGMENT_LED_K  | \
                                        DISPLAY_16_SEGMENT_LED_M    )

#define DISPLAY_16_SEGMENT_VAL_Z       (DISPLAY_16_SEGMENT_LED_A1 | \
                                        DISPLAY_16_SEGMENT_LED_A2 | \
                                        DISPLAY_16_SEGMENT_LED_D1 | \
                                        DISPLAY_16_SEGMENT_LED_D2 | \
                                        DISPLAY_16_SEGMENT_LED_K  | \
                                        DISPLAY_16_SEGMENT_LED_N    )

#define DISPLAY_16_SEGMENT_VAL_0       (DISPLAY_16_SEGMENT_LED_A1 | \
                                        DISPLAY_16_SEGMENT_LED_A2 | \
                                        DISPLAY_16_SEGMENT_LED_B  | \
                                        DISPLAY_16_SEGMENT_LED_C  | \
                                        DISPLAY_16_SEGMENT_LED_D1 | \
                                        DISPLAY_16_SEGMENT_LED_D2 | \
                                        DISPLAY_16_SEGMENT_LED_E  | \
                                        DISPLAY_16_SEGMENT_LED_F    )

#define DISPLAY_16_SEGMENT_VAL_1       (DISPLAY_16_SEGMENT_LED_B  | \
                                        DISPLAY_16_SEGMENT_LED_C    )

#define DISPLAY_16_SEGMENT_VAL_2       (DISPLAY_16_SEGMENT_LED_A1 | \
                                        DISPLAY_16_SEGMENT_LED_A2 | \
                                        DISPLAY_16_SEGMENT_LED_B  | \
                                        DISPLAY_16_SEGMENT_LED_D1 | \
                                        DISPLAY_16_SEGMENT_LED_D2 | \
                                        DISPLAY_16_SEGMENT_LED_E  | \
                                        DISPLAY_16_SEGMENT_LED_G1 | \
                                        DISPLAY_16_SEGMENT_LED_G2   )

#define DISPLAY_16_SEGMENT_VAL_3       (DISPLAY_16_SEGMENT_LED_A1 | \
                                        DISPLAY_16_SEGMENT_LED_A2 | \
                                        DISPLAY_16_SEGMENT_LED_B  | \
                                        DISPLAY_16_SEGMENT_LED_C  | \
                                        DISPLAY_16_SEGMENT_LED_D1 | \
                                        DISPLAY_16_SEGMENT_LED_D2 | \
                                        DISPLAY_16_SEGMENT_LED_G1 | \
                                        DISPLAY_16_SEGMENT_LED_G2   )

#define DISPLAY_16_SEGMENT_VAL_4       (DISPLAY_16_SEGMENT_LED_B  | \
                                        DISPLAY_16_SEGMENT_LED_C  | \
                                        DISPLAY_16_SEGMENT_LED_F  | \
                                        DISPLAY_16_SEGMENT_LED_G1 | \
                                        DISPLAY_16_SEGMENT_LED_G2   )

#define DISPLAY_16_SEGMENT_VAL_5       (DISPLAY_16_SEGMENT_LED_A1 | \
                                        DISPLAY_16_SEGMENT_LED_A2 | \
                                        DISPLAY_16_SEGMENT_LED_C  | \
                                        DISPLAY_16_SEGMENT_LED_D1 | \
                                        DISPLAY_16_SEGMENT_LED_D2 | \
                                        DISPLAY_16_SEGMENT_LED_F  | \
                                        DISPLAY_16_SEGMENT_LED_G1 | \
                                        DISPLAY_16_SEGMENT_LED_G2   )

#define DISPLAY_16_SEGMENT_VAL_6       (DISPLAY_16_SEGMENT_LED_A1 | \
                                        DISPLAY_16_SEGMENT_LED_A2 | \
                                        DISPLAY_16_SEGMENT_LED_C  | \
                                        DISPLAY_16_SEGMENT_LED_D1 | \
                                        DISPLAY_16_SEGMENT_LED_D2 | \
                                        DISPLAY_16_SEGMENT_LED_E  | \
                                        DISPLAY_16_SEGMENT_LED_F  | \
                                        DISPLAY_16_SEGMENT_LED_G1 | \
                                        DISPLAY_16_SEGMENT_LED_G2   )

#define DISPLAY_16_SEGMENT_VAL_7       (DISPLAY_16_SEGMENT_LED_A1 | \
                                        DISPLAY_16_SEGMENT_LED_A2 | \
                                        DISPLAY_16_SEGMENT_LED_B  | \
                                        DISPLAY_16_SEGMENT_LED_C    )

#define DISPLAY_16_SEGMENT_VAL_8       (DISPLAY_16_SEGMENT_LED_A1 | \
                                        DISPLAY_16_SEGMENT_LED_A2 | \
                                        DISPLAY_16_SEGMENT_LED_B  | \
                                        DISPLAY_16_SEGMENT_LED_C  | \
                                        DISPLAY_16_SEGMENT_LED_D1 | \
                                        DISPLAY_16_SEGMENT_LED_D2 | \
                                        DISPLAY_16_SEGMENT_LED_E  | \
                                        DISPLAY_16_SEGMENT_LED_F  | \
                                        DISPLAY_16_SEGMENT_LED_G1 | \
                                        DISPLAY_16_SEGMENT_LED_G2   )

#define DISPLAY_16_SEGMENT_VAL_9       (DISPLAY_16_SEGMENT_LED_A1 | \
                                        DISPLAY_16_SEGMENT_LED_A2 | \
                                        DISPLAY_16_SEGMENT_LED_B  | \
                                        DISPLAY_16_SEGMENT_LED_C  | \
                                        DISPLAY_16_SEGMENT_LED_D1 | \
                                        DISPLAY_16_SEGMENT_LED_D2 | \
                                        DISPLAY_16_SEGMENT_LED_F  | \
                                        DISPLAY_16_SEGMENT_LED_G1 | \
                                        DISPLAY_16_SEGMENT_LED_G2   )

typedef enum {
    BLINK_OFF = 0,
    BLINK_2_0HZ,
    BLINK_1_0HZ,
    BLINK_0_5HZ,
} blink_type_t;

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
 * @param[out] device_address Device address
 * @return ESP_OK: Init success. ESP_FAIL: Not success.
 */
esp_err_t i2c_ht16k33_init(i2c_master_bus_handle_t bus_handle, uint16_t device_address, i2c_ht16k33_handle_t *ht16k33_handle);

/**
 * @brief Write data to ht16k33
 *
 * @param[in] i2c_ht16k33_handle_t ht16k33 handle
 * @param[in] address Block address inside ht16k33
 * @param[in] data Data to write
 * @param[in] size Data write size
 * @return ESP_OK: Write success. Otherwise failed, please check I2C function fail reason.
 */
esp_err_t i2c_ht16k33_write(i2c_ht16k33_handle_t ht16k33_handle, uint32_t address, const uint8_t *data, uint32_t size);

/**
 * @brief Read data from ht16k33
 *
 * @param[in]  i2c_ht16k33_handle_t ht16k33 handle
 * @param[in]  address Block address inside ht16k33
 * @param[out] data Data read from ht16k33
 * @param[in] size Data read size
 * @return ESP_OK: Read success. Otherwise failed, please check I2C function fail reason.
 */
esp_err_t i2c_ht16k33_read(i2c_ht16k33_handle_t ht16k33_handle, uint32_t address, uint8_t *data, uint32_t size);

/**
 * @brief Wait eeprom finish. Typically 5ms
 *
 * @param[in] i2c_ht16k33_handle_t ht16k33 handle
 */
void i2c_ht16k33_wait_idle(i2c_ht16k33_handle_t ht16k33_handle);

/**
 * @brief Set brightness
 *
 * @param[in] i2c_ht16k33_handle_t ht16k33 handle
 * @param[in] uint8_t Brightness value on 4 bits
 * @return ESP_OK: Read success. Otherwise failed, please check I2C function fail reason.
 */
esp_err_t i2c_ht16k33_set_brightness(i2c_ht16k33_handle_t ht16k33_handle, uint8_t brigthness);

/**
 * @brief Set blinking
 *
 * @param[in] i2c_ht16k33_handle_t ht16k33 handle
 * @param[in] blink_type_t blink speed
 * @return ESP_OK: Read success. Otherwise failed, please check I2C function fail reason.
 */
esp_err_t i2c_ht16k33_set_blinking(i2c_ht16k33_handle_t ht16k33_handle, blink_type_t blink_type);

/**
 * @brief Turn on oscillator
 *
 * @param[in] i2c_ht16k33_handle_t ht16k33 handle
 * @return ESP_OK: Read success. Otherwise failed, please check I2C function fail reason.
 */
esp_err_t i2c_ht16k33_turn_on_oscillator(i2c_ht16k33_handle_t ht16k33_handle);

/**
 * @brief Update buffer following row character
 *
 * @param[out] uint8_t* buffer
 * @param[in]  uint16_t character value
 * @param[in]  uint8_t row position
 */
void i2c_ht16k33_buffer_write(uint8_t *buffer, uint16_t character_value, uint8_t row_position);

/**
 * @brief Turn on oscillator
 *
 * @param[in] i2c_ht16k33_handle_t ht16k33 handle
 * @param[in] uint8_t* buffer
 * @return ESP_OK: Read success. Otherwise failed, please check I2C function fail reason.
 */
esp_err_t i2c_ht16k33_update_ram(i2c_ht16k33_handle_t ht16k33_handle, uint8_t* buffer);

#endif

#ifdef __cplusplus
}
#endif
