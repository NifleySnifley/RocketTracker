#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "rtc_wdt.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_types.h"
#include "esp_event.h"
#include "esp_err.h"
#include "driver/uart.h"
#include "driver/adc.h"
#include "pindefs.h"
#include <pb_encode.h>
#include "lwgps.h"
#include "radio.h"
#include "protocol.pb.h"
#include "frame_manager.h"

#define GSM_CFG_DOUBLE 1
// #include "lwgps_opts.h"

#define DBG_TAG "DBG"

#define GPS_UART_BUFSIZE 512

// Idle LED blinking...
// void led_task(void* args) {
//     while (1) {
//         gpio_set_level(TRACKER_LED_RED, 1);
//         gpio_set_level(TRACKER_LED_GRN, 1);
//         vTaskDelay(500);
//         gpio_set_level(TRACKER_LED_RED, 0);
//         gpio_set_level(TRACKER_LED_GRN, 1);
//         vTaskDelay(500);
//         gpio_set_level(TRACKER_LED_RED, 0);
//         gpio_set_level(TRACKER_LED_GRN, 0);
//         vTaskDelay(500);
//         gpio_set_level(TRACKER_LED_RED, 1);
//         gpio_set_level(TRACKER_LED_GRN, 0);
//         vTaskDelay(500);

//         ESP_LOGI(DBG_TAG, "Cycle!");
//     }
// }
RFM97_LoRa radio(SPI2_HOST, TRACKER_RFM_CS, TRACKER_DIO0, TRACKER_RFM_RST, TRACKER_COPI, TRACKER_CIPO, TRACKER_SCK, false);
FrameManager fmg;

lwgps_t gps;
QueueHandle_t gps_uart_q;

void gps_uart_task(void* args) {
    static uint8_t* readbuf = (uint8_t*)malloc(GPS_UART_BUFSIZE + 1);
    uart_event_t event;

    while (1) {
        if (xQueueReceive(gps_uart_q, &event, portMAX_DELAY)) {
            if (event.type == UART_PATTERN_DET) {
                int pos = uart_pattern_pop_pos(TRACKER_GPS_UART);

                while (pos > 0) {
                    int len = uart_read_bytes(TRACKER_GPS_UART, readbuf, pos, 1000 / portTICK_PERIOD_MS);
                    if (len > 0) {
                        lwgps_process(&gps, &readbuf[1], len - 1);
                        readbuf[len] = '\0';

                        // printf("L %s\r\n", &readbuf[1]);
                        // GPS Fresh
                        if (strncmp((char*)&readbuf[1], "$GPGGA", strlen("$GPGGA")) == 0) {
                            // printf("%f,%f,%f\n", gps.latitude, gps.longitude, gps.);
                        }
                    }

                    pos = uart_pattern_pop_pos(TRACKER_GPS_UART);
                }

            }
        }
    }

    free(readbuf);
}

void radio_task(void* args) {
    while (1) {
        vTaskDelay(1000);

        GPS_Info gi;
        gi.lat = gps.latitude;
        gi.lon = gps.longitude;
        gi.has_fix_status = true;
        gi.fix_status = gps.fix;
        gi.has_sats_used = true;
        gi.sats_used = gps.sats_in_use;
        gi.utc_time = gps.seconds * 1000 + gps.minutes * 100000 + gps.hours * 10000000;
        gi.alt = gps.altitude;

        // THIS IS FAKE!!!
        Battery_Info bi;
        bi.battery_voltage = 5.0f;

        fmg.reset();

        fmg.encode_datum(MessageTypeID_TLM_Battery_Info, Battery_Info_fields, &bi);

        if (lwgps_is_valid(&gps) && gps.altitude > 0)
            fmg.encode_datum(MessageTypeID_TLM_GPS_Info, GPS_Info_fields, &gi);

        int size;
        uint8_t* frame = fmg.get_frame(&size);

        radio.transmit(frame, size, true);

        // TODO: Better tx LED?
        gpio_set_level(TRACKER_LED_RED, 1);
        vTaskDelay(100);
        gpio_set_level(TRACKER_LED_RED, 0);
    }
}

// TODO: NEED dma or at least use a queue for receive in an event loop because reading the FIFO in ISR is REALLY BAD
void IRAM_ATTR radio_isr_a(void* arg) {
    static int data;
    xQueueSendFromISR(radio.intq, &data, NULL);
}

extern "C" void app_main() {
    // initArduino();
    esp_log_level_set("*", ESP_LOG_VERBOSE);

    gpio_reset_pin(TRACKER_LED_RED);
    gpio_reset_pin(TRACKER_LED_GRN);
    gpio_set_direction(TRACKER_LED_RED, GPIO_MODE_OUTPUT);
    gpio_set_direction(TRACKER_LED_GRN, GPIO_MODE_OUTPUT);

    uart_config_t uart_config = {
        .baud_rate = 9600 ,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    if (uart_driver_install(TRACKER_GPS_UART, GPS_UART_BUFSIZE * 2, 0, 16, &gps_uart_q, 0) != ESP_OK) {
        ESP_LOGE(DBG_TAG, "Driver installation failed");
    }
    if (uart_param_config(TRACKER_GPS_UART, &uart_config) != ESP_OK) {
        ESP_LOGE(DBG_TAG, "Uart config failed");
    };
    uart_set_pin(TRACKER_GPS_UART, TRACKER_GPS_TX, TRACKER_GPS_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_enable_pattern_det_baud_intr(TRACKER_GPS_UART, '\n', 1, 9, 0, 0);
    uart_pattern_queue_reset(TRACKER_GPS_UART, 16);
    uart_flush(TRACKER_GPS_UART);

    lwgps_init(&gps);

    if (!radio.init())
        ESP_LOGE(DBG_TAG, "Radio is broke :(");

    radio.setPower(20, true);
    radio.setFreq(914.0);
    radio.setISRA(radio_isr_a);
    radio.startReceiving();

    // xTaskCreate(led_task, "led_blinky_task", 1024 * 2, NULL, 1, NULL);
    xTaskCreate(gps_uart_task, "gps_uart_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(radio_task, "radio_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);
}