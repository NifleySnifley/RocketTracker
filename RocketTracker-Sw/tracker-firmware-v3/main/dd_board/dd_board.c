#include "dd_board.h"
#include "esp_intr_alloc.h"
#include "esp_err.h"
#include "esp_check.h"
#include "configutils.h"

static void IRAM_ATTR dd_pwm_generation_timer_cb(void* arg) {
    dd_pwm_gen_ctx_t* ctx = (dd_pwm_gen_ctx_t*)arg;
    BaseType_t hpt_woken;
    xTaskNotifyFromISR(((dd_board_t*)ctx->board)->pwm_gen_task_handle, ctx->channel + 1, eSetValueWithOverwrite, &hpt_woken);
    if (hpt_woken) esp_timer_isr_dispatch_need_yield();

    // Version 2
}

static void dd_pwm_generation_task(void* arg) {
    // Need IRAM??
    dd_board_t* board = (dd_board_t*)arg;
    uint8_t ldr_state = 0b01010101;

    while (1) {
        BaseType_t notval;
        // Get notified by 50Hz timer
        xTaskNotifyWait(0, ULONG_MAX, &notval, portMAX_DELAY);


        if (notval == 0) {
            // Notification = 0 -> Levels high
            ldr_state = 0;//0b01010101 & ~((uint8_t)((uint16_t)(1 << (channel * 2)) & 0xFF));

            if (xSemaphoreTakeRecursive(board->i2c_lock, pdMS_TO_TICKS(20)) == pdPASS) {
                // START TIMERS!

                for (int i = 0; i < 4; ++i) {
                    if (board->pwm_channel_setting_us[i] == 0) {
                        ldr_state |= ((uint8_t)((uint16_t)(1 << (i * 2)) & 0xFF));
                    }
                }

                pca9633_write(&board->pwm_expander, PCA9633_REG_LEDOUT, ldr_state);

                for (int i = 0; i < 4; ++i) {
                    if (board->pwm_channel_setting_us[i] != 0) {
                        // if (esp_timer_is_active(board->pwm_timers[i])) esp_timer_stop(board->pwm_timers[i]);
                        esp_err_t e = esp_timer_start_once(board->pwm_timers[i], board->pwm_channel_setting_us[i]);
                    }
                }

            } else {
                ESP_LOGW("DD-PWM", "Failed to get i2c - missed send");
            }
        } else {
            // Notification = channel to go low + 1
            int channel = notval - 1;
            ldr_state |= ((uint8_t)((uint16_t)(1 << (channel * 2)) & 0xFF));
            pca9633_write(&board->pwm_expander, PCA9633_REG_LEDOUT, ldr_state);

            if (ldr_state == 0b01010101) {
                // ESP_LOGI("DD-PWM", "Finished Send");
                xSemaphoreGive(board->i2c_lock);
            }
        }
    }

    // while (1) {
    //     BaseType_t notval;
    //     // Get notified by 50Hz timer
    //     xTaskNotifyWait(0, ULONG_MAX, &notval, portMAX_DELAY);

    //     int channel = notval;
    //     int channel = notval - 1;
    //     ldr_state |= ((uint8_t)((uint16_t)(1 << (channel * 2)) & 0xFF));
    //     pca9633_write(&board->pwm_expander, PCA9633_REG_LEDOUT, ldr_state);

    //     if (ldr_state == 0b01010101) {
    //         // ESP_LOGI("DD-PWM", "Finished Send");
    //         // xSemaphoreGive(board->i2c_lock);
    //     }
    // }
}

// TODO: Somehow be able to access configuration from here.... Make it a component? Need brightness for LEDs.....
void dd_pyro_task(void* args) {
    dd_board_t* board = (dd_board_t*)args;

    uint8_t val = 0;

    while (1) {
        // board->pyro_continuity[0] = tca6408_get_level(&board->pyro_expander, TCA_PIN_PYRO_1_STAT);
        // board->pyro_continuity[1] = tca6408_get_level(&board->pyro_expander, TCA_PIN_PYRO_2_STAT);
        // board->pyro_continuity[2] = tca6408_get_level(&board->pyro_expander, TCA_PIN_PYRO_3_STAT);
        // board->pyro_continuity[3] = tca6408_get_level(&board->pyro_expander, TCA_PIN_PYRO_4_STAT);

        // dd_board_set_indicator_color(board, 0, board->pyro_enabled[0] ? (board->pyro_continuity[0] ? LED_COLOR_GREEN : LED_COLOR_RED) : LED_COLOR_NONE);
        // dd_board_set_indicator_color(board, 1, board->pyro_enabled[1] ? (board->pyro_continuity[1] ? LED_COLOR_GREEN : LED_COLOR_RED) : LED_COLOR_NONE);
        // dd_board_set_indicator_color(board, 2, board->pyro_enabled[2] ? (board->pyro_continuity[2] ? LED_COLOR_GREEN : LED_COLOR_RED) : LED_COLOR_NONE);
        // dd_board_set_indicator_color(board, 3, board->pyro_enabled[3] ? (board->pyro_continuity[3] ? LED_COLOR_GREEN : LED_COLOR_RED) : LED_COLOR_NONE);

        ESP_LOGI("DD-PWM", "50Hz timer running: %d", esp_timer_is_active(board->pwm_50Hz_timer));
        vTaskDelay(pdMS_TO_TICKS(1000 / board->pyro_hz));
    }
}

void dd_board_set_indicator_rgb(dd_board_t* board, int indicator, uint8_t red, uint8_t green, uint8_t blue) {
    red = (red * board->led_brightness) / 255;
    green = (green * board->led_brightness) / 255;
    blue = (blue * board->led_brightness) / 255;

    if (indicator >= 0 && indicator < 4) {
        led_strip_set_pixel(board->leds, 3 - indicator, green, red, blue);
    }
    led_strip_refresh(board->leds);
}

void dd_board_set_indicator_color(dd_board_t* board, int indicator, led_color_t color) {
    dd_board_set_indicator_rgb(board, indicator, color.r, color.g, color.b);
}

static void adc_reader_task(void* arg) {
    dd_board_t* board = (dd_board_t*)arg;
    // static int current_channel = 0;
    int current_channel = 0;
    int next_channel = 0;

    while (!board->adc_channel_enabled[next_channel]) {
        next_channel = (next_channel + 1) % 4;
    }

    while (1) {
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
        // Do the read sequence (start converting on the next channel, then quickly read the current channel)

        if (xSemaphoreTakeRecursive(board->i2c_lock, pdMS_TO_TICKS(10)) == pdPASS) {
            current_channel = next_channel;
            board->adc_channel_reading[current_channel] = tla2024_read_active_channel_raw(&board->adc);
            xEventGroupSetBits(board->adc_reading_fresh, 1 << current_channel);

            // Seek next
            do {
                next_channel = (next_channel + 1) % 4;
            } while (!board->adc_channel_enabled[next_channel]);

            // Start reading next channel
            tla2024_set_active_channel(&board->adc, next_channel);
            xSemaphoreGive(board->i2c_lock);
        }

    }
}

static void dd_adc_read_timer_cb(void* arg) {
    dd_board_t* board = (dd_board_t*)arg;
    BaseType_t hpt_wake = pdFALSE;
    xTaskNotify(board->adc_reader_task, 0, eNoAction);
}

esp_err_t dd_init_adc(dd_board_t* board) {
    if (xSemaphoreTakeRecursive(board->i2c_lock, pdMS_TO_TICKS(50)) == pdPASS) {
        esp_err_t e = tla2024_init(&board->adc, board->i2c);
        ESP_RETURN_ON_ERROR(e, "TLA2024", "Error initializing TLA2024");

        board->adc_num_channels_enabled = 0;
        memset(board->adc_channel_enabled, 0, sizeof(board->adc_channel_enabled));
        board->adc_hz = 0.0f;

        if (config_get_bool_inline(CONFIG_DDIO_ADC_ADC0_EN_KEY)) {
            board->adc_channel_enabled[0] = true;
            board->adc_num_channels_enabled++;
        }
        if (config_get_bool_inline(CONFIG_DDIO_ADC_ADC1_EN_KEY)) {
            board->adc_channel_enabled[1] = true;
            board->adc_num_channels_enabled++;
        }
        if (config_get_bool_inline(CONFIG_DDIO_ADC_ADC2_EN_KEY)) {
            board->adc_channel_enabled[2] = true;
            board->adc_num_channels_enabled++;
        }
        if (config_get_bool_inline(CONFIG_DDIO_ADC_ADC3_EN_KEY)) {
            board->adc_channel_enabled[3] = true;
            board->adc_num_channels_enabled++;
        }

        // Don't use the ADC if there aren't any channels enabled
        if (board->adc_num_channels_enabled == 0) {
            ESP_LOGI("DD", "No ADC channels enabled, disabling ADC");
            return ESP_OK;
        }

        board->adc_reading_fresh = xEventGroupCreate();

        board->adc_hz = config_get_float_inline(CONFIG_DDIO_ADC_RATE_KEY);
        uint32_t adc_period_us = (1000000 / (board->adc_hz * board->adc_num_channels_enabled));

        // Start the reader task
        xTaskCreatePinnedToCore(adc_reader_task, "adc_reader", 1024 * 4, board, 15, &board->adc_reader_task, 1);

        // Fix jitter:
        // - GPTimer
        // - Async starting I2C transaction
        const esp_timer_create_args_t timer_args = {
            .callback = &dd_adc_read_timer_cb,
            .name = "adc_read",
            .arg = board,
        };

        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &board->adc_timer));
        ESP_ERROR_CHECK(esp_timer_start_periodic(board->adc_timer, adc_period_us));

        xSemaphoreGive(board->i2c_lock);
        return ESP_OK;
    } else {
        return ESP_ERR_TIMEOUT;
    }
}

esp_err_t dd_init_pwms(dd_board_t* board) {
    ESP_RETURN_ON_ERROR(pca9633_init(&board->pwm_expander, board->i2c), "PCA9633", "Error initializing PCA9633");

    // 21? really? I guess it's time critical...
    xTaskCreatePinnedToCore(dd_pwm_generation_task, "pwm_gen_task", 1024 * 8, board, 19, &board->pwm_gen_task_handle, 1);

    board->pwm_channel_setting_us[0] = 0;
    board->pwm_channel_setting_us[1] = 1000;
    board->pwm_channel_setting_us[2] = 1500;
    board->pwm_channel_setting_us[3] = 2000;

    // const esp_timer_create_args_t timer_args = {
    //     .callback = &dd_pwm_generation_timer_cb,
    //     .name = "dd_pwm_gen",
    //     .arg = board,
    // };

    // ESP_RETURN_ON_ERROR(esp_timer_create(&timer_args, &board->pwm_gen_timer), "DD-PWMGEN", "Error timer_create");
    // ESP_RETURN_ON_ERROR(esp_timer_start_periodic(board->pwm_gen_timer, 1000), "DD-PWMGEN", "Error timer_start");

    for (int i = 0; i < 4;++i) {
        board->pwm_timer_ctx[i] = (dd_pwm_gen_ctx_t){
            .board = board,
            .channel = i
        };

        const esp_timer_create_args_t timer_args = {
            .callback = &dd_pwm_generation_timer_cb,
            .name = "dd_pwm_gen", // TODO: Add index
            .arg = &board->pwm_timer_ctx[i],
            .dispatch_method = ESP_TIMER_ISR
        };

        ESP_RETURN_ON_ERROR(esp_timer_create(&timer_args, &board->pwm_timers[i]), "DD-PWM", "Error initializing timer %d", i);
    }

    board->pwm_timer_ctx[4] = (dd_pwm_gen_ctx_t){
        .board = board,
        .channel = -1
    };

    const esp_timer_create_args_t timer_args = {
        .callback = &dd_pwm_generation_timer_cb,
        .name = "dd_pwm_seq", // TODO: Add index
        .arg = &board->pwm_timer_ctx[4],
        .dispatch_method = ESP_TIMER_ISR
    };

    ESP_RETURN_ON_ERROR(esp_timer_create(&timer_args, &board->pwm_50Hz_timer), "DD-PWM", "Error initializing 50Hz timer");
    ESP_RETURN_ON_ERROR(esp_timer_start_periodic(board->pwm_50Hz_timer, 1000000 / 50), "DD-PWM", "Error initializing 50Hz timer");

    return ESP_OK;
}

esp_err_t dd_init_pyros(dd_board_t* board) {
    esp_err_t e = tca6408_init(&board->pyro_expander, board->i2c);
    ESP_RETURN_ON_ERROR(e, "TCA6408", "Error initializing TCA6408");

    if (xSemaphoreTakeRecursive(board->i2c_lock, pdMS_TO_TICKS(50)) == pdPASS) {
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

        xSemaphoreGive(board->i2c_lock);
    } else {
        return ESP_ERR_TIMEOUT;
    }

    board->pyro_enabled[0] = config_get_bool_inline(CONFIG_DDIO_PYRO_PYRO0_EN_KEY);
    board->pyro_enabled[1] = config_get_bool_inline(CONFIG_DDIO_PYRO_PYRO1_EN_KEY);
    board->pyro_enabled[2] = config_get_bool_inline(CONFIG_DDIO_PYRO_PYRO2_EN_KEY);
    board->pyro_enabled[3] = config_get_bool_inline(CONFIG_DDIO_PYRO_PYRO3_EN_KEY);

    board->pyro_hz = config_get_float_inline(CONFIG_DDIO_PYRO_CONT_RATE_KEY);

    return ESP_OK;
}

esp_err_t dd_board_init(dd_board_t* board, gpio_num_t pin_leds, gpio_num_t pin_serial_rx, gpio_num_t pin_sda, gpio_num_t pin_scl, logger_t* logger) {
    board->logger = logger;
    board->i2c_lock = xSemaphoreCreateRecursiveMutex();

    // esp_timer_init();

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
    ESP_RETURN_ON_ERROR(e, "DD", "Error initializing LEDs");

    led_strip_clear(board->leds);
    board->led_brightness = config_get_int_inline(CONFIG_DDIO_BRIGHTNESS_KEY);

    // I2C

    i2c_master_bus_config_t i2c_mst_config = {
        .i2c_port = I2C_NUM_1,
        .sda_io_num = pin_sda,
        .scl_io_num = pin_scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .trans_queue_depth = 16
    };
    i2c_mst_config.flags.enable_internal_pullup = false;

    e = i2c_new_master_bus(&i2c_mst_config, &board->i2c);
    ESP_RETURN_ON_ERROR(e, "DD", "Error initializing I2C bus");

    // Check for all devices!
    esp_err_t eprobe = i2c_master_probe(board->i2c, TCA6408_ADDR, 10);
    eprobe |= i2c_master_probe(board->i2c, TLA2024_ADDR, 10);
    if (eprobe != ESP_OK) {
        // No DD board
        return ESP_ERR_NOT_FOUND;
    }

    // Pyro Expander



    e = dd_init_pwms(board);
    ESP_RETURN_ON_ERROR(e, "DD", "Error initializing PWM expander");

    // e = dd_init_pyros(board);
    // ESP_RETURN_ON_ERROR(e, "DD", "Error initializing pyro IO expander");

    // e = dd_init_adc(board);
    // ESP_RETURN_ON_ERROR(e, "DD", "Error initializing ADC");


    xTaskCreatePinnedToCore(dd_pyro_task, "dd_task", 1024 * 4, board, 10, &board->task_handle, 1);

    return ESP_OK;
}

void dd_board_log_now(dd_board_t* board) {
    for (int i = 0; i < 4; ++i) {
        EventBits_t bit_fresh = 1 << i;
        EventBits_t bits = xEventGroupClearBits(board->adc_reading_fresh, bit_fresh);
        if (bits & bit_fresh) {
            // log_data_adc_t* adc_log = calloc(1, sizeof(log_data_adc_t));
            // adc_log->channel = i;
            // adc_log->value = board->adc_channel_reading[i];
            // logger_queue_log_data_now(board->logger, LOG_DTYPE_ADC, (uint8_t*)adc_log, sizeof(log_data_adc_t));
            log_data_adc_t adc_log = {
                .channel = i,
                .value = board->adc_channel_reading[i]
            };
            logger_log_data_now(board->logger, LOG_DTYPE_ADC, (uint8_t*)&adc_log, sizeof(log_data_adc_t));
        }
    }
}

DDSensorData dd_board_get_data_report(dd_board_t* board) {
    DDSensorData data = { 0 };

    if (board->adc_channel_enabled[0]) {
        data.adc.has_ch0 = true;
        data.adc.ch0 = tla2024_lsb2mv(&board->adc, board->adc_channel_reading[0]);
    }

    if (board->adc_channel_enabled[1]) {
        data.adc.has_ch1 = true;
        data.adc.ch1 = tla2024_lsb2mv(&board->adc, board->adc_channel_reading[1]);
    }

    if (board->adc_channel_enabled[2]) {
        data.adc.has_ch2 = true;
        data.adc.ch2 = tla2024_lsb2mv(&board->adc, board->adc_channel_reading[2]);
    }

    if (board->adc_channel_enabled[3]) {
        data.adc.has_ch3 = true;
        data.adc.ch3 = tla2024_lsb2mv(&board->adc, board->adc_channel_reading[3]);
    }

    data.pyros.has_ch0_cont = board->pyro_enabled[0]; // TODO: Pyro channel enabling
    data.pyros.ch0_cont = board->pyro_continuity[0];

    data.pyros.has_ch1_cont = board->pyro_enabled[1]; // TODO: Pyro channel enabling
    data.pyros.ch1_cont = board->pyro_continuity[1];

    data.pyros.has_ch2_cont = board->pyro_enabled[2]; // TODO: Pyro channel enabling
    data.pyros.ch2_cont = board->pyro_continuity[2];

    data.pyros.has_ch3_cont = board->pyro_enabled[3]; // TODO: Pyro channel enabling
    data.pyros.ch3_cont = board->pyro_continuity[3];

    return data;
}