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

/*
 * this file is supposed to conform the code style.
 */

#include <string.h>
#include <esp_log.h>
#include <esp_idf_lib_helpers.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_err.h>
#include <esp_log.h>

#include "msa301.h"

#define I2C_FREQ_HZ 400000 // 400kHz max

#define USE_UPPER_CASE_FOR_MACRO (1)

static char *TAG = "msa301";

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define CHECK_LOGE(x, msg, ...) do { \
        esp_err_t __; \
        if ((__ = x) != ESP_OK) { \
            ESP_LOGE(TAG, msg, ## __VA_ARGS__); \
            return __; \
        } \
    } while (0)

static esp_err_t read_reg_nolock(msa301_dev_t *dev, uint8_t reg, uint8_t *data, uint32_t len)
{
    ESP_LOGD(TAG, "Read %d byte from i2c slave starting at reg addr %02x.", len, reg);

    esp_err_t res = i2c_dev_read_reg(&dev->i2c_dev, reg, data, len);
    if (res != ESP_OK)
    {
        ESP_LOGE(TAG, "Error %d on read %d byte from I2C slave reg addr %02x.", res, len, reg);
        return res;
    }

    return ESP_OK;
}

static esp_err_t write_reg_nolock(msa301_dev_t *dev, uint8_t reg, uint8_t *data, uint32_t len)
{
    ESP_LOGD(TAG, "Write %d bytes to i2c slave starting at reg addr %02x", len, reg);

    esp_err_t res = i2c_dev_write_reg(&dev->i2c_dev, reg, data, len);
    if (res != ESP_OK)
    {
        ESP_LOGE(TAG, "Error %d on write %d byte to i2c slave register %02x.", res, len, reg);
        return res;
    }

    return ESP_OK;
}

static esp_err_t msa301_is_available(msa301_dev_t *dev)
{
    CHECK_ARG(dev);

    uint8_t reg_data[1];

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_reg_nolock(dev, MSA301_REG_PARTID, reg_data, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    if (reg_data[0] != 0x13)
    {
        ESP_LOGE(TAG, "Wrong hardware ID %02x, should be 0x81", reg_data[0]);
        return MSA301_ERR_HW_ID;
    }

    // ESP_LOGD(TAG, "hardware version:      %02x", reg_data[1]);
    // ESP_LOGD(TAG, "firmware boot version: %02x", reg_data[3]);
    // ESP_LOGD(TAG, "firmware app version:  %02x", reg_data[4]);

    return ESP_OK;
}

esp_err_t msa301_init_desc(msa301_dev_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);
    if (addr != MSA301_I2C_ADDRESS)
    {
        ESP_LOGE(TAG, "Invalid device address: 0x%02x", addr);
        return ESP_ERR_INVALID_ARG;
    }

    dev->i2c_dev.port = port;
    dev->i2c_dev.addr = addr;
    dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif
    dev->i2c_dev.timeout_ticks = I2CDEV_MAX_STRETCH_TIME;

    return i2c_dev_create_mutex(&dev->i2c_dev);
}

esp_err_t msa301_free_desc(msa301_dev_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t msa301_init(msa301_dev_t *dev)
{
    CHECK_ARG(dev);

    // check whether sensor is available including the check of the hardware
    // id and the error state
    CHECK_LOGE(msa301_is_available(dev), "Sensor is not available.");

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    // enable all axes
    msa301_enable_axes(dev, true, true, true);
    // normal mode
    msa301_set_power_mode(dev, MSA301_NORMALMODE);
    // 500Hz rate
    msa301_set_data_rate(dev, MSA301_DATARATE_500_HZ);
    // 250Hz bw
    msa301_set_bandwidth(dev, MSA301_BANDWIDTH_250_HZ);
    msa301_set_range(dev, MSA301_RANGE_4_G);
    msa301_set_resolution(dev, MSA301_RESOLUTION_14);

    return ESP_OK;
}

esp_err_t msa301_set_data_rate(msa301_dev_t *dev, msa301_dataRate_t data_rate)
{
    CHECK_ARG(dev);

    uint8_t reg_data[1] = { 0 };

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_reg_nolock(dev, MSA301_REG_ODR, reg_data, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    reg_data[0] &= ~((1 << 4) - 1);
    reg_data[0] |= data_rate;

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, write_reg_nolock(dev, MSA301_REG_ODR, reg_data, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t msa301_get_data_rate(msa301_dev_t *dev, msa301_dataRate_t *data_rate)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_reg_nolock(dev, MSA301_REG_ODR, (uint8_t*) data_rate, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    *data_rate = (uint8_t) *data_rate & ((1 << 4) - 1);
    
    return ESP_OK;
}

esp_err_t msa301_enable_axes(msa301_dev_t *dev, bool x, bool y, bool z)
{
    CHECK_ARG(dev);

    uint8_t reg_data[1] = { 0 };
    uint8_t temp;
    uint8_t mask = 0;
    // x 7, y 6, z 5 disable bites=inverse boolean

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_reg_nolock(dev, MSA301_REG_ODR, reg_data, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    // x
    mask = (1 << 1) - 1;
    temp = (uint8_t) !x;
    // temp &= mask;
    mask <<= 7;
    reg_data[0] &= ~mask;
    reg_data[0] |= temp << 7;

    // y
    mask = 0;
    mask = (1 << 1) - 1;
    temp = (uint8_t) !y;
    // temp &= mask;
    mask <<= 6;
    reg_data[0] &= ~mask;
    reg_data[0] |= temp << 6;

    // z
    mask = 0;
    mask = (1 << 1) - 1;
    temp = (uint8_t) !z;
    // temp &= mask;
    mask <<= 5;
    reg_data[0] &= ~mask;
    reg_data[0] |= temp << 5;

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, write_reg_nolock(dev, MSA301_REG_ODR, reg_data, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t msa301_set_power_mode(msa301_dev_t *dev, msa301_powermode_t mode)
{
    CHECK_ARG(dev);

    uint8_t reg_data[1] = { 0 };
    uint8_t temp;
    uint8_t mask = 0;

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_reg_nolock(dev, MSA301_REG_POWERMODE, reg_data, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    mask = (1 << 2) - 1;
    temp = (uint8_t) mode;
    // temp &= mask;
    mask <<= 6;
    reg_data[0] &= ~mask;
    reg_data[0] |= temp << 6;

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, write_reg_nolock(dev, MSA301_REG_POWERMODE, reg_data, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t msa301_get_power_mode(msa301_dev_t *dev, msa301_powermode_t *mode)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_reg_nolock(dev, MSA301_REG_POWERMODE, (uint8_t*) mode, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    *mode = ((uint8_t) *mode) >> 6;
    *mode = ((uint8_t) *mode) & ((1 << 2) - 1);
    
    return ESP_OK;
}

esp_err_t msa301_set_bandwidth(msa301_dev_t *dev, msa301_bandwidth_t bandwidth)
{
    CHECK_ARG(dev);

    uint8_t reg_data[1] = { 0 };
    uint8_t temp;
    uint8_t mask = 0;

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_reg_nolock(dev, MSA301_REG_POWERMODE, reg_data, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    mask = (1 << 4) - 1;
    temp = (uint8_t) bandwidth;
    // temp &= mask;
    mask <<= 1;
    reg_data[0] &= ~mask;
    reg_data[0] |= temp << 1;

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, write_reg_nolock(dev, MSA301_REG_POWERMODE, reg_data, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t msa301_get_bandwidth(msa301_dev_t *dev, msa301_bandwidth_t *bandwith)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_reg_nolock(dev, MSA301_REG_POWERMODE, (uint8_t*) bandwith, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    *bandwith = ((uint8_t) *bandwith) >> 1;
    *bandwith = ((uint8_t) *bandwith) & ((1 << 4) - 1);
    
    return ESP_OK;
}

esp_err_t msa301_set_range(msa301_dev_t *dev, msa301_range_t range)
{
    CHECK_ARG(dev);

    uint8_t reg_data[1] = { 0 };
    uint8_t temp;
    uint8_t mask = 0;

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_reg_nolock(dev, MSA301_REG_RESRANGE, reg_data, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    mask = (1 << 2) - 1;
    temp = (uint8_t) range;
    // temp &= mask;
    // mask <<= 1;
    reg_data[0] &= ~mask;
    reg_data[0] |= temp;

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, write_reg_nolock(dev, MSA301_REG_RESRANGE, reg_data, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t msa301_get_range(msa301_dev_t *dev, msa301_range_t *range)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_reg_nolock(dev, MSA301_REG_RESRANGE, (uint8_t*) range, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    // *range = ((uint8_t) *range) >> 1;
    *range = ((uint8_t) *range) & ((1 << 2) - 1);
    
    return ESP_OK;
}

esp_err_t msa301_set_resolution(msa301_dev_t *dev, msa301_resolution_t resolution)
{
    CHECK_ARG(dev);

    uint8_t reg_data[1] = { 0 };
    uint8_t temp;
    uint8_t mask = 0;

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_reg_nolock(dev, MSA301_REG_RESRANGE, reg_data, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    mask = (1 << 2) - 1;
    temp = (uint8_t) resolution;
    // temp &= mask;
    mask <<= 2;
    reg_data[0] &= ~mask;
    reg_data[0] |= temp;

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, write_reg_nolock(dev, MSA301_REG_RESRANGE, reg_data, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t msa301_get_resolution(msa301_dev_t *dev, msa301_resolution_t *resolution)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_reg_nolock(dev, MSA301_REG_RESRANGE, (uint8_t*) resolution, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    *resolution = ((uint8_t) *resolution) >> 2;
    *resolution = ((uint8_t) *resolution) & ((1 << 2) - 1);
    
    return ESP_OK;
}

esp_err_t msa301_get_results(msa301_dev_t *dev, uint16_t *x, uint16_t *y, uint16_t *z, float *x_g, float *y_g, float *z_g)
{
    CHECK_ARG(dev);

    uint8_t reg_data[6] = { 0 };

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_reg_nolock(dev, MSA301_REG_OUT_X_L, reg_data, 6));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    *x = reg_data[0];
    *x |= reg_data[1] << 8;
    *y = reg_data[2];
    *y |= reg_data[3] << 8;
    *z = reg_data[4];
    *z |= reg_data[5] << 8;

    // 14 bits of data in 16 bit range
    *x >>= 2;
    *y >>= 2;
    *z >>= 2;

    msa301_range_t range;
    msa301_get_range(dev, &range);
    float scale = 1;
    if (range == MSA301_RANGE_16_G)
        scale = 512;
    if (range == MSA301_RANGE_8_G)
        scale = 1024;
    if (range == MSA301_RANGE_4_G)
        scale = 2048;
    if (range == MSA301_RANGE_2_G)
        scale = 4096;

    *x_g = (float) *x / scale;
    *y_g = (float) *y / scale;
    *z_g = (float) *z / scale;
    
    return ESP_OK;
}

esp_err_t msa301_enable_interrupts(msa301_dev_t *dev, bool single_tap, bool double_tap, bool active_x, bool active_y, bool active_z, bool new_data, bool free_fall, bool orient)
{
    CHECK_ARG(dev);

    uint8_t reg_data[1] = { 0 };

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_reg_nolock(dev, MSA301_REG_INTSET0, reg_data, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    reg_data[0] |= orient << 6;
    reg_data[0] |= single_tap << 5;
    reg_data[0] |= double_tap << 4;
    reg_data[0] |= active_x << 0;
    reg_data[0] |= active_y << 1;
    reg_data[0] |= active_z << 2;
    
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, write_reg_nolock(dev, MSA301_REG_INTSET0, reg_data, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_reg_nolock(dev, MSA301_REG_INTSET1, reg_data, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    reg_data[0] |= new_data << 4;
    reg_data[0] |= free_fall << 3;
    
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, write_reg_nolock(dev, MSA301_REG_INTSET1, reg_data, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t msa301_map_interrupt_pin(msa301_dev_t *dev, bool single_tap, bool double_tap, bool activity, bool new_data, bool free_fall, bool orient)
{
    CHECK_ARG(dev);

    uint8_t reg_data[1] = { 0 };

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_reg_nolock(dev, MSA301_REG_INTMAP0, reg_data, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    reg_data[0] |= orient << 6;
    reg_data[0] |= single_tap << 5;
    reg_data[0] |= double_tap << 4;
    reg_data[0] |= activity << 2;
    reg_data[0] |= free_fall << 0;
    
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, write_reg_nolock(dev, MSA301_REG_INTMAP0, reg_data, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_reg_nolock(dev, MSA301_REG_INTMAP1, reg_data, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    reg_data[0] = 0;
    reg_data[0] |= new_data << 0;
    
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, write_reg_nolock(dev, MSA301_REG_INTMAP1, reg_data, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t msa301_get_click(msa301_dev_t *dev, uint8_t *click_status)
{
    CHECK_ARG(dev);

    uint8_t reg_data[1] = { 0 };

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_reg_nolock(dev, MSA301_REG_CLICKSTATUS, reg_data, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    *click_status = reg_data[0];

    return ESP_OK;
}
esp_err_t msa301_get_motion_interrupt_status(msa301_dev_t *dev, uint8_t *motion_interrupt_status)
{
    CHECK_ARG(dev);

    uint8_t reg_data[1] = { 0 };

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_reg_nolock(dev, MSA301_REG_MOTIONINT, reg_data, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    *motion_interrupt_status = reg_data[0];

    return ESP_OK;
}

esp_err_t msa301_get_data_interrupt_status(msa301_dev_t *dev, uint8_t *data_interrupt_status)
{
    CHECK_ARG(dev);

    uint8_t reg_data[1] = { 0 };

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_reg_nolock(dev, MSA301_REG_DATAINT, reg_data, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    *data_interrupt_status = reg_data[0];

    return ESP_OK;
}

esp_err_t msa301_set_click(msa301_dev_t *dev, bool tap_quiet, bool tap_shock, msa301_tapduration_t tap_duration, uint8_t tap_thresh)
{
    CHECK_ARG(dev);

    uint8_t reg_data[1] = { 0 };
    uint8_t temp;
    uint8_t mask = 0;
    // x 7, y 6, z 5 disable bites=inverse boolean

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_reg_nolock(dev, MSA301_REG_TAPDUR, reg_data, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    // tap_quiet
    mask = (1 << 1) - 1;
    temp = (uint8_t) tap_quiet;
    // temp &= mask;
    mask <<= 7;
    reg_data[0] &= ~mask;
    reg_data[0] |= temp << 7;

    // tap_shock
    mask = 0;
    mask = (1 << 1) - 1;
    temp = (uint8_t) tap_shock;
    // temp &= mask;
    mask <<= 6;
    reg_data[0] &= ~mask;
    reg_data[0] |= temp << 6;

    // tapduration
    mask = 0;
    mask = (1 << 3) - 1;
    temp = (uint8_t) tap_duration;
    // temp &= mask;
    // mask <<= 5;
    reg_data[0] &= ~mask;
    // reg_data[0] |= temp << 5;
    reg_data[0] |= temp;

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, write_reg_nolock(dev, MSA301_REG_TAPDUR, reg_data, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, read_reg_nolock(dev, MSA301_REG_TAPTH, reg_data, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    // tapthresh
    mask = 0;
    mask = (1 << 5) - 1;
    temp = tap_thresh;
    // temp &= mask;
    // mask <<= 5;
    reg_data[0] &= ~mask;
    // reg_data[0] |= temp << 5;
    reg_data[0] |= temp;

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, write_reg_nolock(dev, MSA301_REG_TAPTH, reg_data, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}
