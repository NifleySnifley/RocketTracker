#include "dd_board.h"
#include "esp_intr_alloc.h"

// TODO: Somehow be able to access configuration from here.... Make it a component? Need brightness for LEDs.....
void dd_task(void* args) {
    dd_board_t* board = (dd_board_t*)args;
    while (1) {
        bool pyro1_continuity = tca6408_get_level(&board->pyro_expander, TCA_PIN_PYRO_1_STAT);
        bool pyro2_continuity = tca6408_get_level(&board->pyro_expander, TCA_PIN_PYRO_2_STAT);
        bool pyro3_continuity = tca6408_get_level(&board->pyro_expander, TCA_PIN_PYRO_3_STAT);
        bool pyro4_continuity = tca6408_get_level(&board->pyro_expander, TCA_PIN_PYRO_4_STAT);

        dd_board_set_indicator(board, 0, pyro1_continuity ? 0 : 64, pyro1_continuity ? 64 : 0, 0);
        dd_board_set_indicator(board, 1, pyro2_continuity ? 0 : 64, pyro2_continuity ? 64 : 0, 0);
        dd_board_set_indicator(board, 2, pyro3_continuity ? 0 : 64, pyro3_continuity ? 64 : 0, 0);
        dd_board_set_indicator(board, 3, pyro4_continuity ? 0 : 64, pyro4_continuity ? 64 : 0, 0);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void dd_board_set_indicator(dd_board_t* board, int indicator, uint8_t red, uint8_t green, uint8_t blue) {
    if (indicator >= 0 && indicator < 4) {
        led_strip_set_pixel(board->leds, 3 - indicator, green, red, blue);
    }
    led_strip_refresh(board->leds);
}

esp_err_t dd_init_adc() {
    return ESP_OK;
}

esp_err_t dd_init_pwms() {
    return ESP_OK;
}

esp_err_t dd_board_init(dd_board_t* board, gpio_num_t pin_leds, gpio_num_t pin_serial_rx, gpio_num_t pin_sda, gpio_num_t pin_scl) {
    // INDICATORS, TODO: USE DMA!!!
    led_strip_config_t strip_config = {
        .strip_gpio_num = pin_leds, // The GPIO that connected to the LED strip's data line
        .max_leds = 4, // The number of LEDs in the strip,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB, // Pixel format of your LED strip
        .led_model = LED_MODEL_WS2812, // LED strip model
        .flags.invert_out = false, // whether to invert the output signal (useful when your hardware has a level inverter)
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // different clock source can lead to different power consumption
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = true, // whether to enable the DMA feature
    };

    // esp_intr_dump(NULL);

    esp_err_t e = led_strip_new_rmt_device(&strip_config, &rmt_config, &board->leds);
    if (e != ESP_OK) {
        ESP_LOGE("DD", "Error initializing LEDs: %s", esp_err_to_name(e));
        return e;
    }

    led_strip_clear(board->leds);

    // dd_board_set_indicator(board, 0, 64, 0, 0);
    // dd_board_set_indicator(board, 1, 0, 64, 0);
    // dd_board_set_indicator(board, 2, 64, 0, 0);
    // dd_board_set_indicator(board, 3, 0, 64, 0);


    // I2C

    i2c_master_bus_config_t i2c_mst_config = {
        .i2c_port = I2C_NUM_1,
        .sda_io_num = pin_sda,
        .scl_io_num = pin_scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
    };
    i2c_mst_config.flags.enable_internal_pullup = false;

    e = i2c_new_master_bus(&i2c_mst_config, &board->i2c);
    if (e != ESP_OK) {
        ESP_LOGE("DD", "Error initializing I2C bus: %s", esp_err_to_name(e));
        return e;
    }

    // Check for all devices!
    esp_err_t eprobe = i2c_master_probe(board->i2c, TCA6408_ADDR, 10);
    if (eprobe != ESP_OK) {
        // No DD board
        return ESP_ERR_NOT_FOUND;
    }
    // TODO: All devices

    // Pyro Expander


    xTaskCreate(dd_task, "dd_task", 1024 * 4, board, 10, &board->task_handle);

    e = tca6408_init(&board->pyro_expander, board->i2c);

    tca6408_set_pindir(&board->pyro_expander, TCA_PIN_PYRO_1_STAT, false);
    tca6408_set_pindir(&board->pyro_expander, TCA_PIN_PYRO_2_STAT, false);
    tca6408_set_pindir(&board->pyro_expander, TCA_PIN_PYRO_3_STAT, false);
    tca6408_set_pindir(&board->pyro_expander, TCA_PIN_PYRO_4_STAT, false);
    tca6408_set_pindir(&board->pyro_expander, TCA_PIN_PYRO_1_CTL, true);
    tca6408_set_pindir(&board->pyro_expander, TCA_PIN_PYRO_2_CTL, true);
    tca6408_set_pindir(&board->pyro_expander, TCA_PIN_PYRO_3_CTL, true);
    tca6408_set_pindir(&board->pyro_expander, TCA_PIN_PYRO_4_CTL, true);

    // tca6408_set_level(&board->pyro_expander, TCA_PIN_PYRO_1_CTL, true);
    tca6408_write(&board->pyro_expander, TCA6408_REG_OUTPUT, 0);

    e = dd_init_adc();
    if (e != ESP_OK) {
        ESP_LOGE("DD", "Error initializing ADC");
        return e;
    }

    e = dd_init_pwms();
    if (e != ESP_OK) {
        ESP_LOGE("DD", "Error initializing PWM expander");
        return e;
    }

    return ESP_OK;
}