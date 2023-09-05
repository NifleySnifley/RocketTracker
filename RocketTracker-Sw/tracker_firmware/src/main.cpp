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

                        printf("# %s\r\n", &readbuf[1]);
                        // GPS Fresh
                        // if (strncmp((char*)&readbuf[1], "$GPGGA", strlen("$GPGGA")) == 0) {
                        printf("G %f,%f,%f,%d,%d\n", gps.latitude, gps.longitude, gps.altitude, gps.fix, gps.is_valid);
                        // }
                    }

                    pos = uart_pattern_pop_pos(TRACKER_GPS_UART);
                }

            }
        }
    }

    free(readbuf);
}

// uint8_t test_message[200] = {
//     0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9, 0xa, 0xb, 0xc, 0xd, 0xe, 0xf, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f, 0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf, 0xb0, 0xb1, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xbb, 0xbc, 0xbd, 0xbe, 0xbf, 0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7
// };

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

    xTaskCreate(gps_uart_task, "gps_uart_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(radio_task, "radio_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);
}