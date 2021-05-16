/*
 * Copyright (c) YYYY YOUR NAME HERE <user@your.dom.ain>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/**
 * @file example.h
 * @defgroup example example
 * @{
 *
 * An example component.
 *
 */

#ifndef __msa301__H__
#define __msa301__H__

#include <stdbool.h>
#include <i2cdev.h>
#include <esp_err.h>

/*=========================================================================
I2C ADDRESS/BITS
-----------------------------------------------------------------------*/
#define MSA301_I2C_ADDRESS (0x26) ///< Fixed I2C address
/*=========================================================================*/

#define MSA301_ERR_BASE 0xa000

#define MSA301_ERR_BOOT_MODE      (MSA301_ERR_BASE + 1) //!< firmware is in boot mode
#define MSA301_ERR_NO_APP         (MSA301_ERR_BASE + 2) //!< no application firmware loaded
#define MSA301_ERR_NO_NEW_DATA    (MSA301_ERR_BASE + 3) //!< no new data samples are ready
#define MSA301_ERR_NO_IAQ_DATA    (MSA301_ERR_BASE + 4) //!< no new data samples are ready
#define MSA301_ERR_HW_ID          (MSA301_ERR_BASE + 5) //!< wrong hardware ID
#define MSA301_ERR_INV_SENS       (MSA301_ERR_BASE + 6) //!< invalid sensor ID
#define MSA301_ERR_WR_REG_INV     (MSA301_ERR_BASE + 7) //!< invalid register addr on write
#define MSA301_ERR_RD_REG_INV     (MSA301_ERR_BASE + 8) //!< invalid register addr on read
#define MSA301_ERR_MM_INV         (MSA301_ERR_BASE + 9) //!< invalid measurement mode
#define MSA301_ERR_MAX_RESIST     (MSA301_ERR_BASE + 10) //!< max sensor resistance reached
#define MSA301_ERR_HEAT_FAULT     (MSA301_ERR_BASE + 11) //!< heater current not in range
#define MSA301_ERR_HEAT_SUPPLY    (MSA301_ERR_BASE + 12) //!< heater voltage not correct
#define MSA301_ERR_WRONG_MODE     (MSA301_ERR_BASE + 13) //!< wrong measurement mode
#define MSA301_ERR_RD_STAT_FAILED (MSA301_ERR_BASE + 14) //!< read status register failed
#define MSA301_ERR_RD_DATA_FAILED (MSA301_ERR_BASE + 15) //!< read sensor data failed
#define MSA301_ERR_APP_START_FAIL (MSA301_ERR_BASE + 16) //!< sensor app start failure
#define MSA301_ERR_WRONG_PARAMS   (MSA301_ERR_BASE + 17) //!< wrong parameters used

#define MSA301_REG_PARTID 0x01    ///< Register that contains the part ID
#define MSA301_REG_OUT_X_L 0x02   ///< Register address for X axis lower byte
#define MSA301_REG_OUT_X_H 0x03   ///< Register address for X axis higher byte
#define MSA301_REG_OUT_Y_L 0x04   ///< Register address for Y axis lower byte
#define MSA301_REG_OUT_Y_H 0x05   ///< Register address for Y axis higher byte
#define MSA301_REG_OUT_Z_L 0x06   ///< Register address for Z axis lower byte
#define MSA301_REG_OUT_Z_H 0x07   ///< Register address for Z axis higher byte
#define MSA301_REG_MOTIONINT 0x09 ///< Register address for motion interrupt
#define MSA301_REG_DATAINT 0x0A   ///< Register address for data interrupt
#define MSA301_REG_CLICKSTATUS                                                 \
  0x0B ///< Register address for click/doubleclick status
#define MSA301_REG_RESRANGE 0x0F  ///< Register address for resolution range
#define MSA301_REG_ODR 0x10       ///< Register address for data rate setting
#define MSA301_REG_POWERMODE 0x11 ///< Register address for power mode setting
#define MSA301_REG_INTSET0 0x16   ///< Register address for interrupt setting #0
#define MSA301_REG_INTSET1 0x17   ///< Register address for interrupt setting #1
#define MSA301_REG_INTMAP0 0x19   ///< Register address for interrupt map #0
#define MSA301_REG_INTMAP1 0x1A   ///< Register address for interrupt map #1
#define MSA301_REG_TAPDUR 0x2A    ///< Register address for tap duration
#define MSA301_REG_TAPTH 0x2B     ///< Register address for tap threshold

#ifdef __cplusplus

/* in most cases, break before an opening brace, but do not in case of `extern
 * "C"`. otherwise, all the code would have been indented.
 */
extern "C" {
#endif

/** The accelerometer ranges */
typedef enum {
  MSA301_RANGE_2_G = 0b00,  ///< +/- 2g (default value)
  MSA301_RANGE_4_G = 0b01,  ///< +/- 4g
  MSA301_RANGE_8_G = 0b10,  ///< +/- 8g
  MSA301_RANGE_16_G = 0b11, ///< +/- 16g
} msa301_range_t;

/** The accelerometer axes */
typedef enum {
  MSA301_AXIS_X = 0x0, ///< X axis bit
  MSA301_AXIS_Y = 0x1, ///< Y axis bit
  MSA301_AXIS_Z = 0x2, ///< Z axis bit
} msa301_axis_t;

/** The accelerometer data rate */
typedef enum {
  MSA301_DATARATE_1_HZ = 0b0000,     ///<  1 Hz
  MSA301_DATARATE_1_95_HZ = 0b0001,  ///<  1.95 Hz
  MSA301_DATARATE_3_9_HZ = 0b0010,   ///<  3.9 Hz
  MSA301_DATARATE_7_81_HZ = 0b0011,  ///<  7.81 Hz
  MSA301_DATARATE_15_63_HZ = 0b0100, ///<  15.63 Hz
  MSA301_DATARATE_31_25_HZ = 0b0101, ///<  31.25 Hz
  MSA301_DATARATE_62_5_HZ = 0b0110,  ///<  62.5 Hz
  MSA301_DATARATE_125_HZ = 0b0111,   ///<  125 Hz
  MSA301_DATARATE_250_HZ = 0b1000,   ///<  250 Hz
  MSA301_DATARATE_500_HZ = 0b1001,   ///<  500 Hz
  MSA301_DATARATE_1000_HZ = 0b1010,  ///<  1000 Hz
} msa301_dataRate_t;

/** The accelerometer bandwidth */
typedef enum {
  MSA301_BANDWIDTH_1_95_HZ = 0b0000,  ///<  1.95 Hz
  MSA301_BANDWIDTH_3_9_HZ = 0b0011,   ///<  3.9 Hz
  MSA301_BANDWIDTH_7_81_HZ = 0b0100,  ///<  7.81 Hz
  MSA301_BANDWIDTH_15_63_HZ = 0b0101, ///<  15.63 Hz
  MSA301_BANDWIDTH_31_25_HZ = 0b0110, ///<  31.25 Hz
  MSA301_BANDWIDTH_62_5_HZ = 0b0111,  ///<  62.5 Hz
  MSA301_BANDWIDTH_125_HZ = 0b1000,   ///<  125 Hz
  MSA301_BANDWIDTH_250_HZ = 0b1001,   ///<  250 Hz
  MSA301_BANDWIDTH_500_HZ = 0b1010,   ///<  500 Hz
} msa301_bandwidth_t;

/** The accelerometer power mode */
typedef enum {
  MSA301_NORMALMODE = 0b00,   ///< Normal (high speed) mode
  MSA301_LOWPOWERMODE = 0b01, ///< Low power (slow speed) mode
  MSA301_SUSPENDMODE = 0b010, ///< Suspend (sleep) mode
} msa301_powermode_t;

/** The accelerometer read resolution */
typedef enum {
  MSA301_RESOLUTION_14 = 0b00, ///< 14-bit resolution
  MSA301_RESOLUTION_12 = 0b01, ///< 12-bit resolution
  MSA301_RESOLUTION_10 = 0b10, ///< 10-bit resolution
  MSA301_RESOLUTION_8 = 0b11,  ///< 8-bit resolution
} msa301_resolution_t;

/** Tap duration parameter */
typedef enum {
  MSA301_TAPDUR_50_MS = 0b000,  ///< 50 millis
  MSA301_TAPDUR_100_MS = 0b001, ///< 100 millis
  MSA301_TAPDUR_150_MS = 0b010, ///< 150 millis
  MSA301_TAPDUR_200_MS = 0b011, ///< 200 millis
  MSA301_TAPDUR_250_MS = 0b100, ///< 250 millis
  MSA301_TAPDUR_375_MS = 0b101, ///< 375 millis
  MSA301_TAPDUR_500_MS = 0b110, ///< 500 millis
  MSA301_TAPDUR_700_MS = 0b111, ///< 50 millis700 millis
} msa301_tapduration_t;

/** Interrupts available */
typedef enum {
  MSA301_INT_ORIENT = 0b100000, ///< Orientation change interrupt
  MSA301_INT_SINGLETAP,         ///< Single tap interrupt
  MSA301_INT_DOUBLETAP,         ///< Double tap interrupt
  MSA301_INT_ACTIVE,            ///< Active motion interrupt
  MSA301_INT_NEWDATA,           ///< New data interrupt
} msa301_interrupt_t;

/**
 * @brief CCS811 sensor device data structure
 */
typedef struct
{
    i2c_dev_t i2c_dev;  //!< I2C device handle
    // ccs811_mode_t mode; //!< operation mode
} msa301_dev_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev Pointer to the sensor device data structure
 * @param addr Sensor address
 * @param port I2C port number
 * @param sda_gpio GPIO pin number for SDA
 * @param scl_gpio GPIO pin number for SCL
 * @returns ESP_OK on success
 */
esp_err_t msa301_init_desc(msa301_dev_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev Pointer to the sensor device data structure
 * @returns ESP_OK on success
 */
esp_err_t msa301_free_desc(msa301_dev_t *dev);

/**
 * @brief Initialize a MSA301 sensor
 *
 * The function initializes the MSA301 sensor and checks its availability.
 *
 * @param dev Pointer to the sensor device data structure
 *
 * @returns ESP_OK on success
 */
esp_err_t msa301_init(msa301_dev_t *dev);

esp_err_t msa301_set_data_rate(msa301_dev_t *dev, msa301_dataRate_t dataRate);
esp_err_t msa301_get_data_rate(msa301_dev_t *dev, msa301_dataRate_t *data_rate);
esp_err_t msa301_enable_axes(msa301_dev_t *dev, bool x, bool y, bool z);

esp_err_t msa301_set_power_mode(msa301_dev_t *dev, msa301_powermode_t mode);
esp_err_t msa301_get_power_mode(msa301_dev_t *dev, msa301_powermode_t *mode);
esp_err_t msa301_set_bandwidth(msa301_dev_t *dev, msa301_bandwidth_t bandwidth);
esp_err_t msa301_get_bandwidth(msa301_dev_t *dev, msa301_bandwidth_t *bandwith);
esp_err_t msa301_set_range(msa301_dev_t *dev, msa301_range_t range);
esp_err_t msa301_get_range(msa301_dev_t *dev, msa301_range_t *range);
esp_err_t msa301_set_resolution(msa301_dev_t *dev, msa301_resolution_t resolution);
esp_err_t msa301_get_resolution(msa301_dev_t *dev, msa301_resolution_t *resolution);

esp_err_t msa301_get_results(msa301_dev_t *dev, uint16_t *x, uint16_t *y, uint16_t *z, float *x_g, float *y_g, float *z_g);

// esp_err_t msa301_enable_interrupts(msa301_dev_t *dev, bool single_tap = false, bool double_tap = false,
//                       bool active_x = false, bool active_y = false,
//                       bool active_z = false, bool new_data = false,
//                       bool free_fall = false, bool orient = false);
// esp_err_t msa301_map_interrupt_pin(msa301_dev_t *dev, bool single_tap = false, bool double_tap = false,
//                      bool activity = false, bool new_data = false,
//                      bool free_fall = false, bool orient = false);
esp_err_t msa301_enable_interrupts(msa301_dev_t *dev, bool single_tap, bool double_tap,
                                   bool active_x, bool active_y,
                                   bool active_z, bool new_data,
                                   bool free_fall, bool orient);
esp_err_t msa301_map_interrupt_pin(msa301_dev_t *dev, bool single_tap, bool double_tap,
                                   bool activity, bool new_data,
                                   bool free_fall, bool orient);

esp_err_t msa301_get_click(msa301_dev_t *dev, uint8_t *click_status);
esp_err_t msa301_get_motion_interrupt_status(msa301_dev_t *dev, uint8_t *motion_interrupt_status);
esp_err_t msa301_get_data_interrupt_status(msa301_dev_t *dev, uint8_t *data_interrupt_status);

esp_err_t msa301_setClick(msa301_dev_t *dev, bool tap_quiet, bool tap_shock,
                msa301_tapduration_t tap_duration, uint8_t tap_thresh);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif // __msa301__H__
