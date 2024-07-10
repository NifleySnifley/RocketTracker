#ifndef ADXL375_H
#define ADXL375_H

#define ADXL_WHOAMI_VALUE 0b11100101

#define ADXL_REG_DEVID 0x00
#define ADXL_REG_0x1C 0x01
#define ADXL_REG_THRESH_SHOCK 0x1D
#define ADXL_REG_OFSX 0x1E
#define ADXL_REG_OFSY 0x1F
#define ADXL_REG_OFSZ 0x20
#define ADXL_REG_DUR 0x21
#define ADXL_REG_Latent 0x22
#define ADXL_REG_Window 0x23
#define ADXL_REG_THRESH_ACT 0x24
#define ADXL_REG_THRESH_INACT 0x25
#define ADXL_REG_TIME_INACT 0x26
#define ADXL_REG_ACT_INACT_CTL 0x27
#define ADXL_REG_SHOCK_AXES 0x2A
#define ADXL_REG_ACT_SHOCK_STATUS 0x2B
#define ADXL_REG_BW_RATE 0x2C
#define ADXL_REG_POWER_CTL 0x2D
#define ADXL_REG_INT_ENABLE 0x2E
#define ADXL_REG_INT_MAP 0x2F
#define ADXL_REG_INT_SOURCE 0x30
#define ADXL_REG_DATA_FORMAT 0x31
#define ADXL_REG_DATAX0 0x32
#define ADXL_REG_DATAX1 0x33
#define ADXL_REG_DATAY0 0x34
#define ADXL_REG_DATAY1 0x35
#define ADXL_REG_DATAZ0 0x36
#define ADXL_REG_DATAZ1 0x37
#define ADXL_REG_FIFO_CTL 0x38
#define ADXL_REG_FIFO_STATUS 0x39

#define ADXL_SHOCK_SOURCE_X (1<<2)
#define ADXL_SHOCK_SOURCE_Y (1<<1)
#define ADXL_SHOCK_SOURCE_Z (1<<0)
#define ADXL_ACT_SOURCE_X (1<<4)
#define ADXL_ACT_SOURCE_Y (1<<5)
#define ADXL_ACT_SOURCE_Z (1<<6)

#define ADXL_RATE_3200 0b1111
#define ADXL_RATE_1600 0b1110
#define ADXL_RATE_800  0b1101
#define ADXL_RATE_400  0b1100
#define ADXL_RATE_200  0b1011
#define ADXL_RATE_100  0b1010
#define ADXL_RATE_50   0b1001
#define ADXL_RATE_25   0b1000
#define ADXL_RATE_12_5 0b0111
#define ADXL_RATE_6_25 0b0110
#define ADXL_RATE_3_13 0b0101
#define ADXL_RATE_1_56 0b0100
#define ADXL_RATE_0_78 0b0011
#define ADXL_RATE_0_39 0b0010
#define ADXL_RATE_0_20 0b0001
#define ADXL_RATE_0_10 0b0000

#define ADXL_POWER_SLEEP 0b0100
#define ADXL_POWER_STANDBY 0b0000
#define ADXL_POWER_MEASURE 0b1000

#define ADXL_SLEEP_8HZ 0b00
#define ADXL_SLEEP_4HZ 0b01
#define ADXL_SLEEP_2HZ 0b10
#define ADXL_SLEEP_1HZ 0b11

#define ADXL_INT_DRDY (1<<7)
#define ADXL_INT_SHOCK_SINGLE (1<<6)
#define ADXL_INT_SHOCK_DOUBLE (1<<5)
#define ADXL_INT_ACT (1<<4)
#define ADXL_INT_INACT (1<<3)
#define ADXL_INT_WATERMARK (1<<1)
#define ADXL_INT_OVERRUN (1<<0)

#include "driver/spi_master.h"
#include "driver/gpio.h"

typedef struct adxl375_handle_t {
    spi_device_handle_t* device;
    gpio_num_t cs;
} adxl375_handle_t;

esp_err_t adxl375_init(adxl375_handle_t* adxl, spi_device_handle_t* devhandle, gpio_num_t cs_pin);

esp_err_t adxl375_set_thresh_act(adxl375_handle_t* adxl, int thresh_mg);
esp_err_t adxl375_get_shock_act_status(adxl375_handle_t* adxl, uint8_t* status);
esp_err_t adxl375_act_clear(adxl375_handle_t* adxl);
esp_err_t adxl375_set_act_mode(adxl375_handle_t* adxl, bool ac, bool x_en, bool y_en, bool z_en);

esp_err_t adxl375_set_thresh_inact(adxl375_handle_t* adxl, int thresh_mg);
esp_err_t adxl375_set_time_inact(adxl375_handle_t* adxl, int time_seconds);
esp_err_t adxl375_set_inact_mode(adxl375_handle_t* adxl, bool ac, bool x_en, bool y_en, bool z_en);

esp_err_t adxl375_set_thresh_shock(adxl375_handle_t* adxl, int thresh_mg);
esp_err_t adxl375_set_shock_duration(adxl375_handle_t* adxl, int dur_us);
esp_err_t adxl375_set_shock_latent(adxl375_handle_t* adxl, float lat_ms);
esp_err_t adxl375_set_shock_window(adxl375_handle_t* adxl, float window_ms);
esp_err_t adxl375_set_shock_mode(adxl375_handle_t* adxl, bool suppress, bool x_en, bool y_en, bool z_en);
esp_err_t adxl375_shock_clear(adxl375_handle_t* adxl);

esp_err_t adxl375_get_asleep(adxl375_handle_t* adxl, bool* asleep);
esp_err_t adxl375_set_bw_rate(adxl375_handle_t* adxl, uint8_t rate, bool low_power);
esp_err_t adxl375_set_link(adxl375_handle_t* adxl, bool link);
esp_err_t adxl375_set_autosleep(adxl375_handle_t* adxl, bool autosleep);
esp_err_t adxl375_set_mode(adxl375_handle_t* adxl, uint8_t mode);
esp_err_t adxl375_set_sleep_sample_freq(adxl375_handle_t* adxl, uint8_t freq);

esp_err_t adxl375_enable_interrupts(adxl375_handle_t* adxl, uint8_t interrupts);
// Pass 0 to interrupts for all on pin 1
esp_err_t adxl375_set_interrupts_pin2(adxl375_handle_t* adxl, uint8_t interrupts);
esp_err_t adxl375_get_int_source(adxl375_handle_t* adxl, uint8_t* source);
esp_err_t adxl375_set_standard_format(adxl375_handle_t* adxl);
esp_err_t adxl375_get_acceleration_raw(adxl375_handle_t* adxl, int16_t* measurement);
esp_err_t adxl375_reg_write(adxl375_handle_t* adxl, uint8_t addr, uint8_t* data, int len);
esp_err_t adxl375_reg_read(adxl375_handle_t* adxl, uint8_t addr, uint8_t* buffer, int len);

#endif