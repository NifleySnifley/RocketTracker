#include "pca9633.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "stdbool.h"
#include "esp_attr.h"

// 190Hz / 4 = ~50Hz (round-robin time division between the three drivers)
// Can create PWM values from 0ms to 5.2ms using blink feature
// Of those values, ~50 are in the servo PWM range (3.6deg steps, not great but OK enough?)

static bool i2c_null_callback(i2c_master_dev_handle_t i2c_dev, const i2c_master_event_data_t* evt_data, void* arg) {
    return false;
}

static i2c_master_event_callbacks_t cbs;
esp_err_t pca9633_init(PCA9633_t* exp, i2c_master_bus_handle_t bus) {
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = PCA9633_ADDR,
        .scl_speed_hz = 1000000 // Supports FS+
    };

    esp_err_t e = i2c_master_bus_add_device(bus, &dev_config, &exp->device);
    if (e != ESP_OK) {
        return e;
    }

    // PCA9633_MODE2_t mode2 = {
    //     .dmblnk = 0,
    //     .invrt = 1,
    //     .outdrv = 1,
    //     .och = 0,
    // };

    exp->ldr.reg = 0;
    pca9633_write(exp, PCA9633_REG_MODE1, 0);
    pca9633_write(exp, PCA9633_REG_MODE2, 1 << 2);
    pca9633_write(exp, PCA9633_REG_LEDOUT, exp->ldr.reg);

    // cbs.on_trans_done = i2c_null_callback;
    // e = i2c_master_register_event_callbacks(exp->device, &cbs, NULL);
    // if (e != ESP_OK) {
    //     ESP_LOGE("PCA9633", "Error registering i2c callback: %s", esp_err_to_name(e));
    // }

    return ESP_OK;
}

void pca9633_write(PCA9633_t* exp, uint8_t addr, uint8_t value) {
    uint8_t buffer[2] = {
        addr,
        value
    };
    esp_err_t e = i2c_master_transmit(exp->device, &buffer, sizeof(buffer), 10);
    if (e != ESP_OK) {
        ESP_LOGE("PCA9633", "Error writing I2C: %s", esp_err_to_name(e));
    }
}

// NOT NEEDED!
// uint8_t pca9633_read(PCA9633_t* exp, uint8_t addr) {
//     uint8_t recvbuffer[1] = { 0 };
//     esp_err_t e = i2c_master_transmit_receive(exp->device, &addr, 1, recvbuffer, 1, 10);
//     if (e != ESP_OK) {
//         ESP_LOGE("PCA9633", "Error reading I2C: %s", esp_err_to_name(e));
//     }
//     return recvbuffer[0];
// }

void pca9633_set_channel_level(PCA9633_t* exp, int channel, bool level) {
    if (level) {
        exp->ldr.reg |= (1 << (channel << 1));
    } else {
        exp->ldr.reg &= ~(3 << (channel << 1));
    }
    pca9633_write(exp, PCA9633_REG_LEDOUT, exp->ldr.reg); // Let's see what happens!
}
