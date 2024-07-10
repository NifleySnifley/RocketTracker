#include "adxl375.h"
#include "esp_log.h"

esp_err_t adxl375_init(adxl375_handle_t* adxl, spi_device_handle_t* devhandle, gpio_num_t cs_pin) {
    adxl->device = devhandle;

    adxl->cs = cs_pin;

    ESP_LOGI("ADXL", "Trying to init...");
    // Check the WHOAMI!
    uint8_t whoami;
    adxl375_reg_read(adxl, ADXL_REG_DEVID, &whoami, 1);
    if (whoami != ADXL_WHOAMI_VALUE) {
        ESP_LOGE("ADXL", "Invalid WHOAMI. Expected %d, got %d", ADXL_WHOAMI_VALUE, whoami);
        return ESP_ERR_INVALID_RESPONSE;
    }

    adxl375_set_standard_format(adxl);

    return ESP_OK;
}

esp_err_t adxl375_set_thresh_shock(adxl375_handle_t* adxl, int thresh_mg) {
    int reg_value = thresh_mg / 780;
    if (reg_value < 0 || reg_value > 255) {
        return ESP_ERR_INVALID_ARG;
    } else {
        uint8_t val = reg_value;
        return adxl375_reg_write(adxl, ADXL_REG_THRESH_SHOCK, &val, 1);
    }
}

// NOT IMPLEMENTED: setting offsets with registers on the sensor

esp_err_t adxl375_set_shock_duration(adxl375_handle_t* adxl, int dur_us) {
    int reg_value = dur_us / 625;
    if (reg_value < 0 || reg_value > 255) {
        return ESP_ERR_INVALID_ARG;
    } else {
        uint8_t val = reg_value;
        return adxl375_reg_write(adxl, ADXL_REG_DUR, &val, 1);
    }
}

esp_err_t adxl375_set_shock_latent(adxl375_handle_t* adxl, float lat_ms) {
    int reg_value = (lat_ms / 1.25f);
    if (reg_value < 0 || reg_value > 255) {
        return ESP_ERR_INVALID_ARG;
    } else {
        uint8_t val = reg_value;
        return adxl375_reg_write(adxl, ADXL_REG_Latent, &val, 1);
    }
}

esp_err_t adxl375_set_shock_window(adxl375_handle_t* adxl, float window_ms) {
    int reg_value = (window_ms / 1.25f);
    if (reg_value < 0 || reg_value > 255) {
        return ESP_ERR_INVALID_ARG;
    } else {
        uint8_t val = reg_value;
        return adxl375_reg_write(adxl, ADXL_REG_Window, &val, 1);
    }
}

esp_err_t adxl375_set_thresh_act(adxl375_handle_t* adxl, int thresh_mg) {
    int reg_value = (thresh_mg / 780);
    if (reg_value < 0 || reg_value > 255) {
        return ESP_ERR_INVALID_ARG;
    } else {
        uint8_t val = reg_value;
        return adxl375_reg_write(adxl, ADXL_REG_THRESH_ACT, &val, 1);
    }
}

esp_err_t adxl375_set_thresh_inact(adxl375_handle_t* adxl, int thresh_mg) {
    int reg_value = (thresh_mg / 780);
    if (reg_value < 0 || reg_value > 255) {
        return ESP_ERR_INVALID_ARG;
    } else {
        uint8_t val = reg_value;
        return adxl375_reg_write(adxl, ADXL_REG_THRESH_ACT, &val, 1);
    }
}

esp_err_t adxl375_set_time_inact(adxl375_handle_t* adxl, int time_seconds) {
    if (time_seconds < 0 || time_seconds > 255) {
        return ESP_ERR_INVALID_ARG;
    } else {
        uint8_t val = time_seconds;
        return adxl375_reg_write(adxl, ADXL_REG_THRESH_ACT, &val, 1);
    }
}

esp_err_t adxl375_set_act_mode(adxl375_handle_t* adxl, bool ac, bool x_en, bool y_en, bool z_en) {
    uint8_t reg_value;
    esp_err_t e = adxl375_reg_read(adxl, ADXL_REG_ACT_INACT_CTL, &reg_value, 1);
    if (e != ESP_OK) return e;
    reg_value = reg_value & 0x0F; // Mask out all ACT configuration
    reg_value |= (ac << 7) | (x_en << 6) | (y_en << 5) | (z_en << 4);
    return adxl375_reg_write(adxl, ADXL_REG_ACT_INACT_CTL, &reg_value, 1);
}

esp_err_t adxl375_set_inact_mode(adxl375_handle_t* adxl, bool ac, bool x_en, bool y_en, bool z_en) {
    uint8_t reg_value;
    esp_err_t e = adxl375_reg_read(adxl, ADXL_REG_ACT_INACT_CTL, &reg_value, 1);
    if (e != ESP_OK) return e;
    reg_value = reg_value & 0xF0; // Mask out all ACT configuration
    reg_value |= (ac << 3) | (x_en << 2) | (y_en << 1) | (z_en << 0);
    return adxl375_reg_write(adxl, ADXL_REG_ACT_INACT_CTL, &reg_value, 1);
}

esp_err_t adxl375_set_shock_mode(adxl375_handle_t* adxl, bool suppress, bool x_en, bool y_en, bool z_en) {
    uint8_t  reg_value = (suppress << 3) | (x_en << 2) | (y_en << 1) | (z_en << 0);
    return adxl375_reg_write(adxl, ADXL_REG_SHOCK_AXES, &reg_value, 1);
}

esp_err_t adxl375_get_shock_act_status(adxl375_handle_t* adxl, uint8_t* status) {
    return adxl375_reg_read(adxl, ADXL_REG_ACT_SHOCK_STATUS, status, 1);
}

esp_err_t adxl375_act_clear(adxl375_handle_t* adxl) {
    uint8_t reg_value;
    esp_err_t e = adxl375_reg_read(adxl, ADXL_REG_ACT_SHOCK_STATUS, &reg_value, 1);
    if (e != ESP_OK) return e;
    reg_value &= 0x0F;
    return adxl375_reg_write(adxl, ADXL_REG_ACT_SHOCK_STATUS, &reg_value, 1);
}

esp_err_t adxl375_shock_clear(adxl375_handle_t* adxl) {
    uint8_t reg_value;
    esp_err_t e = adxl375_reg_read(adxl, ADXL_REG_ACT_SHOCK_STATUS, &reg_value, 1);
    if (e != ESP_OK) return e;
    reg_value &= 0xF8;// Don't clear asleep!
    return adxl375_reg_write(adxl, ADXL_REG_ACT_SHOCK_STATUS, &reg_value, 1);
}

esp_err_t adxl375_get_asleep(adxl375_handle_t* adxl, bool* asleep) {
    uint8_t reg_value;
    esp_err_t e = adxl375_reg_read(adxl, ADXL_REG_ACT_SHOCK_STATUS, &reg_value, 1);
    if (e != ESP_OK) return e;
    *asleep = (reg_value & (1 << 3));
    return e;
}


esp_err_t adxl375_set_bw_rate(adxl375_handle_t* adxl, uint8_t rate, bool low_power) {
    rate &= 0x0F;
    rate |= low_power << 4;
    return adxl375_reg_write(adxl, ADXL_REG_BW_RATE, &rate, 1);
}

/*
"The link bit serially links the activity and inactivity functions. If
both the activity and inactivity functions are enabled, a setting of
1 in the link bit delays the start of the activity detection function
until inactivity is detected. After activity is detected, inactivity
detection begins, preventing the detection of activity. When this
bit is set to 0, the inactivity and activity functions are concurrent.
For more information about the link feature, see the Link Mode
section." (ADXL375 datasheet)
"*/
esp_err_t adxl375_set_link(adxl375_handle_t* adxl, bool link) {
    uint8_t powerctl_value;
    esp_err_t e = adxl375_reg_read(adxl, ADXL_REG_POWER_CTL, &powerctl_value, 1);
    if (e != ESP_OK) return e;
    powerctl_value &= ~(1 << 5);
    powerctl_value |= (link << 5);
    return adxl375_reg_write(adxl, ADXL_REG_POWER_CTL, &powerctl_value, 1);
}

esp_err_t adxl375_set_autosleep(adxl375_handle_t* adxl, bool autosleep) {
    uint8_t powerctl_value;
    esp_err_t e = adxl375_reg_read(adxl, ADXL_REG_POWER_CTL, &powerctl_value, 1);
    if (e != ESP_OK) return e;
    powerctl_value &= ~(1 << 4);
    powerctl_value |= (autosleep << 4);
    return adxl375_reg_write(adxl, ADXL_REG_POWER_CTL, &powerctl_value, 1);
}

esp_err_t adxl375_set_mode(adxl375_handle_t* adxl, uint8_t mode) {
    uint8_t powerctl_value;
    esp_err_t e = adxl375_reg_read(adxl, ADXL_REG_POWER_CTL, &powerctl_value, 1);
    if (e != ESP_OK) return e;
    powerctl_value &= ~(0b1100);
    powerctl_value |= mode;
    return adxl375_reg_write(adxl, ADXL_REG_POWER_CTL, &powerctl_value, 1);
}

esp_err_t adxl375_set_sleep_sample_freq(adxl375_handle_t* adxl, uint8_t freq) {
    uint8_t powerctl_value;
    esp_err_t e = adxl375_reg_read(adxl, ADXL_REG_POWER_CTL, &powerctl_value, 1);
    if (e != ESP_OK) return e;
    powerctl_value &= ~(0b11);
    powerctl_value |= freq;
    return adxl375_reg_write(adxl, ADXL_REG_POWER_CTL, &powerctl_value, 1);
}

esp_err_t adxl375_enable_interrupts(adxl375_handle_t* adxl, uint8_t interrupts) {
    return adxl375_reg_write(adxl, ADXL_REG_INT_ENABLE, &interrupts, 1);
}

// Pass 0 to interrupts for all on pin 1
esp_err_t adxl375_set_interrupts_pin2(adxl375_handle_t* adxl, uint8_t interrupts) {
    return adxl375_reg_write(adxl, ADXL_REG_INT_MAP, &interrupts, 1);
}

esp_err_t adxl375_get_int_source(adxl375_handle_t* adxl, uint8_t* source) {
    return adxl375_reg_read(adxl, ADXL_REG_INT_SOURCE, source, 1);
}

// Little endian, active-high interrupts, 4-wire SPI, no self test
esp_err_t adxl375_set_standard_format(adxl375_handle_t* adxl) {
    uint8_t value = 0b00001011;
    return adxl375_reg_write(adxl, ADXL_REG_DATA_FORMAT, &value, 1);
}

esp_err_t adxl375_get_acceleration_raw(adxl375_handle_t* adxl, int16_t* measurement) {
    return adxl375_reg_read(adxl, ADXL_REG_DATAX0, measurement, 6);
}

esp_err_t adxl375_reg_write(adxl375_handle_t* adxl, uint8_t addr, uint8_t* data, int len) {
    // gpio_set_level(adxl->cs, 0);
    uint8_t a = addr;
    a &= ~(0 << 7); // Write
    if (len > 1)
        a |= (1 << 6); // Multiple bit (MB)
    else
        a &= ~(1 << 6);

    spi_transaction_t t = {
            .length = 8 * len,
            .addr = a,
            .tx_buffer = data,
            .rx_buffer = NULL
    };

    esp_err_t e = spi_device_transmit(*adxl->device, &t);
    // gpio_set_level(adxl->cs, 1);
    return e;
}

/*
Unless bus traffic can be ade-
quately controlled to ensure that such a condition never occurs,
it is recommended that a logic gate be added in front of Pin 13
(SDA/SDI/SDIO), as shown in Figure 24. This OR gate holds the
SDA line high when CS is high to prevent SPI bus traffic at the
ADXL375 from appearing as an I 2 C start command.
*/

esp_err_t adxl375_reg_read(adxl375_handle_t* adxl, uint8_t addr, uint8_t* buffer, int len) {
    // gpio_set_level(adxl->cs, 0);
    uint8_t a = addr;
    a |= (1 << 7); // Read (high)
    if (len > 1)
        a |= (1 << 6); // Multiple bit (MB)
    else
        a &= ~(1 << 6);

    spi_transaction_t t = {
        .length = 8 * len,
        .addr = a,
        .tx_buffer = NULL,
        .rx_buffer = buffer,
    };

    esp_err_t e = spi_device_transmit(*adxl->device, &t);
    // gpio_set_level(adxl->cs, 1);
    return e;
}