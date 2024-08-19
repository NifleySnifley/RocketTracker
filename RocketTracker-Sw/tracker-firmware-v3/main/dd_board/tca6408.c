#include "tca6408.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

esp_err_t tca6408_init(TCA6408_t* exp, i2c_master_bus_handle_t bus) {
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TCA6408_ADDR,
        .scl_speed_hz = 400000
    };

    esp_err_t e = i2c_master_bus_add_device(bus, &dev_config, &exp->device);
    if (e != ESP_OK) {
        return e;
    }

    return ESP_OK;
}

void tca6408_write(TCA6408_t* exp, uint8_t addr, uint8_t value) {
    uint8_t buffer[2] = {
        addr,
        value
    };
    esp_err_t e = i2c_master_transmit(exp->device, &buffer, sizeof(buffer), 10);
    if (e != ESP_OK) {
        ESP_LOGE("TCA6408", "Error writing I2C: %s", esp_err_to_name(e));
    }
}

uint8_t tca6408_read(TCA6408_t* exp, uint8_t addr) {
    uint8_t recvbuffer[1] = { 0 };
    esp_err_t e = i2c_master_transmit_receive(exp->device, &addr, 1, recvbuffer, 1, 10);
    if (e != ESP_OK) {
        ESP_LOGE("TCA6408", "Error reading I2C: %s", esp_err_to_name(e));
    }
    return recvbuffer[0];
}

void tca6408_set_pindir(TCA6408_t* exp, int pin, bool is_output) {
    if (pin < 0 || pin > 7) return;
    if (is_output) {
        exp->output_mask |= 1 << pin;
    } else {
        exp->output_mask &= ~(1 << pin);
    }

    tca6408_write(exp, TCA6408_REG_CONFIGURATION, ~exp->output_mask);
}

void tca6408_set_level(TCA6408_t* exp, int pin, bool level) {
    if (pin < 0 || pin > 7) return;
    if (level) {
        exp->output_state |= (1 << pin);
    } else {
        exp->output_state &= ~(1 << pin);
    }
    exp->output_state &= exp->output_mask;

    tca6408_write(exp, TCA6408_REG_OUTPUT, exp->output_state);
}

bool tca6408_get_level(TCA6408_t* exp, int pin) {
    if (pin < 0 || pin > 7) return false;
    uint8_t inputs = tca6408_read(exp, TCA6408_REG_INPUT);
    return inputs & (1 << pin);
}