#include "dd_board.h"
#include "esp_intr_alloc.h"
#include "esp_err.h"
#include "esp_check.h"
#include "configutils.h"

// TODO: Somehow be able to access configuration from here.... Make it a component? Need brightness for LEDs.....
void dd_task(void* args) {
    dd_board_t* board = (dd_board_t*)args;
    while (1) {
        bool pyro1_continuity = tca6408_get_level(&board->pyro_expander, TCA_PIN_PYRO_1_STAT);
        bool pyro2_continuity = tca6408_get_level(&board->pyro_expander, TCA_PIN_PYRO_2_STAT);
        bool pyro3_continuity = tca6408_get_level(&board->pyro_expander, TCA_PIN_PYRO_3_STAT);
        bool pyro4_continuity = tca6408_get_level(&board->pyro_expander, TCA_PIN_PYRO_4_STAT);

        dd_board_set_indicator_color(board, 0, pyro1_continuity ? LED_COLOR_GREEN : LED_COLOR_RED);
        dd_board_set_indicator_color(board, 1, pyro2_continuity ? LED_COLOR_GREEN : LED_COLOR_RED);
        dd_board_set_indicator_color(board, 2, pyro3_continuity ? LED_COLOR_GREEN : LED_COLOR_RED);
        dd_board_set_indicator_color(board, 3, pyro4_continuity ? LED_COLOR_GREEN : LED_COLOR_RED);

        // ESP_LOGI("DD", "ADC Channels: %.0fmV, %.0fmV, %.0fmV, %.0fmV",
        //     tla2024_lsb2mv(&board->adc, board->adc_channel_reading[0]),
        //     tla2024_lsb2mv(&board->adc, board->adc_channel_reading[1]),
        //     tla2024_lsb2mv(&board->adc, board->adc_channel_reading[2]),
        //     tla2024_lsb2mv(&board->adc, board->adc_channel_reading[3])
        // );
        vTaskDelay(pdMS_TO_TICKS(100));
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

        current_channel = next_channel;
        board->adc_channel_reading[current_channel] = tla2024_read_active_channel_raw(&board->adc);
        xEventGroupSetBits(board->adc_reading_fresh, 1 << current_channel);

        // Seek next
        do {
            next_channel = (next_channel + 1) % 4;
        } while (!board->adc_channel_enabled[next_channel]);

        // Start reading next channel
        tla2024_set_active_channel(&board->adc, next_channel);
    }
}

static void dd_adc_read_timer_cb(void* arg) {
    dd_board_t* board = (dd_board_t*)arg;
    BaseType_t hpt_wake = pdFALSE;
    xTaskNotify(board->adc_reader_task, 0, eNoAction);
}

esp_err_t dd_init_adc(dd_board_t* board) {
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

    // board->gptimer = NULL;
    // gptimer_config_t timer_config = {
    //     .clk_src = GPTIMER_CLK_SRC_DEFAULT,
    //     .direction = GPTIMER_COUNT_UP,
    //     .resolution_hz = 1000000, // 1MHz, 1 tick=1us
    // };
    // ESP_RETURN_ON_ERROR(gptimer_new_timer(&timer_config, &board->gptimer), "DD", "Error starting GPTimer for ADC reading");

    // gptimer_event_callbacks_t cbs = {
    //     .on_alarm = dd_adc_read_timer_cb,
    // };
    // ESP_RETURN_ON_ERROR(gptimer_register_event_callbacks(board->gptimer, &cbs, board), "DD", "Error registering GPTimer callback for ADC reading");

    // gptimer_enable(board->gptimer);

    // gptimer_alarm_config_t alarm_config = {
    //     .alarm_count = adc_period_us,
    //     .flags.auto_reload_on_alarm = true // Periodic timer
    // };
    // gptimer_set_alarm_action(board->gptimer, &alarm_config);
    // gptimer_start(board->gptimer);

    const esp_timer_create_args_t timer_args = {
        .callback = &dd_adc_read_timer_cb,
        .name = "adc_read",
        .arg = board
    };

    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &board->adc_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(board->adc_timer, adc_period_us));

    return ESP_OK;
}

esp_err_t dd_init_pwms(dd_board_t* board) {
    return ESP_OK;
}

esp_err_t dd_init_pyros(dd_board_t* board) {
    esp_err_t e = tca6408_init(&board->pyro_expander, board->i2c);
    ESP_RETURN_ON_ERROR(e, "TCA6408", "Error initializing TCA6408");

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

    return ESP_OK;
}

esp_err_t dd_board_init(dd_board_t* board, gpio_num_t pin_leds, gpio_num_t pin_serial_rx, gpio_num_t pin_sda, gpio_num_t pin_scl, logger_t* logger) {
    board->logger = logger;

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

    e = dd_init_pyros(board);
    ESP_RETURN_ON_ERROR(e, "DD", "Error initializing pyro IO expander");

    e = dd_init_adc(board);
    ESP_RETURN_ON_ERROR(e, "DD", "Error initializing ADC");

    e = dd_init_pwms(board);
    ESP_RETURN_ON_ERROR(e, "DD", "Error initializing PWM expander");

    xTaskCreate(dd_task, "dd_task", 1024 * 4, board, 10, &board->task_handle);

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