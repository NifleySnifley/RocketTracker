#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "defs.h"
#include "tracker_config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_flash.h"
#include "esp_flash_spi_init.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "memory.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "driver/usb_serial_jtag.h"
#include "pb_decode.h"
#include "esp_task_wdt.h"
#include "adxl375.h"
#include "driver/ledc.h"
#include "esp_app_desc.h"

#include "ubxlib.h"

#include "u_debug_utils.h"

#include "max17048.h"
#include "sensors.h"

#include "nmea.h"
#include "gpgll.h"
#include "gpgga.h"
#include "gprmc.h"
#include "gpgsa.h"
#include "gpvtg.h"
#include "gptxt.h"
#include "gpgsv.h"

#include <sx127x.h>
#include "sx127x_spi.h"

#include "fmgr.h"
#include "logging.h"
#include "prelog.h"

#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include <string.h>

#include "encoded_reader.h"

#include "configuration.h"
config_provider_t GLOBAL_CONFIG; // Define this here

////////////////// GLOBALS //////////////////
// Peripherals
i2c_master_bus_handle_t i2c_main_bus_handle;

// ICs
MAX17048_t battery_monitor;

// Radio
SemaphoreHandle_t radio_mutex; // Used to "hold" the radio during transmit/receive gaps
sx127x* radio = NULL;
spi_device_handle_t radio_spi_device;

////////////////// GLOBAL HANDLES //////////////////
fmgr_t lora_outgoing_fmgr;
fmgr_t lora_incoming_fmgr;
fmgr_t tcp_outgoing_fmgr;
fmgr_t tcp_incoming_fmgr;
SemaphoreHandle_t lora_txdone_sem;
SemaphoreHandle_t fmgr_mutex;

char WIFI_SSID[32] = { 0 };
char WIFI_PASS[32] = { 0 };
int32_t LINK_TCP_PORT = CONFIG_WIFI_TCP_PORT_DEFAULT;
#define WIFI_CHANNEL 1
bool WIFI_ENABLED = CONFIG_WIFI_ENABLED_DEFAULT;
int TCP_ACTIVE_SOCK = -1;

fmgr_t usb_outgoing_fmgr;
fmgr_t usb_incoming_fmgr;

// Logging
esp_flash_t* ext_flash;
logger_t logger;

logging_state_t log_state = LOGSTATE_LOGGING_STOPPED;
int log_state_hz = 0;
esp_timer_handle_t logging_timer;
#define LOGDL_NOTIFY_START 0x01
#define LOGDL_NOTIFY_ACK 0x02 // TODO: Use this to receive a acknowledgement for each segment in the future
static TaskHandle_t log_download_task_handle;
prelog_t prelogger;
esp_timer_handle_t prelog_recording_timer;

esp_timer_handle_t flight_timer;
esp_timer_handle_t log_led_timer;

static SemaphoreHandle_t tl_spi_hold_sem;

nvs_handle_t nvs_config_handle;

static TaskHandle_t telemetry_task;
static bool telemetry_enable = true;

// Sensor output over USB
static int sensor_output_hz = 0;
static TaskHandle_t sensor_output_task_handle;
static bool sensor_output_raw = false;

//////////////// PREDEFS
void link_send_datum(DatumTypeID type, const pb_msgdesc_t* fields, const void* src_struct);
static void radio_tx(uint8_t* data, int len);
static esp_err_t usb_tx(uint8_t* data, int len);
static esp_err_t tcp_tx(uint8_t* data, int len);
void set_logstate(logging_state_t state, int manual_hz);
static void log_download_task(void* arg);
void link_flush();
static void flight_timer_callback(void* arg);
void start_autolog();
void stop_autolog();
static void sensor_output_task(void* arg);
static bool tcp_flush();
// static void radio_tx_blocking(uint8_t* data, int len);

// USB
static bool usb_active = false;
static bool tcp_active = false;
#define USB_AVAILABLE (usb_serial_jtag_is_connected() && usb_active)
#define TCP_AVAILABLE (tcp_active && WIFI_ENABLED)

#define LED_G_CHANNEL LEDC_CHANNEL_0
#define LED_G_TIMER LEDC_TIMER_0

bool log_led_state = false;
void log_led_toggle() {
    gpio_set_level(PIN_LED_G, log_led_state);
    log_led_state = !log_led_state;
}

////////////////// INIT FUNCTIONS //////////////////
static void init_leds(void) {
    gpio_reset_pin(PIN_LED_G);
    gpio_reset_pin(PIN_LED_R);

    /* Set the GPIO as a push/pull output */
    gpio_set_direction(PIN_LED_G, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_LED_R, GPIO_MODE_OUTPUT);

    const esp_timer_create_args_t args = {
        .callback = &log_led_toggle,
        .name = "log_led"
    };

    ESP_ERROR_CHECK(esp_timer_create(&args, &log_led_timer));
}

void get_logstatus(LogStatus* stat) {
    stat->cur_logging_hz = log_state_hz;
    stat->is_armed = log_state == LOGSTATE_LOGGING_AUTO_ARMED;
    stat->is_auto = log_state == LOGSTATE_LOGGING_AUTO_ARMED || log_state == LOGSTATE_LOGGING_AUTO_FLIGHT || log_state == LOGSTATE_LOGGING_AUTO_LANDED || log_state == LOGSTATE_LOGGING_AUTO_LIFTOFF;
    stat->has_cur_logging_hz = true;
    stat->log_maxsize = LOG_MEMORY_SIZE_B;
    stat->log_size = logger_get_current_log_size(&logger);
}

static void init_i2c() {
    i2c_master_bus_config_t i2c_mst_config = {
        .i2c_port = I2C_MAIN_PORT,
        .sda_io_num = PIN_SDA,
        .scl_io_num = PIN_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
    };
    i2c_mst_config.flags.enable_internal_pullup = false;

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &i2c_main_bus_handle));
}

static void init_battmon() {
    MAX17048_Init(&battery_monitor, i2c_main_bus_handle);
}

// GLOBAL DATUM DECODING HERE
void link_decode_datum(int i, DatumTypeID id, int len, uint8_t* data, LinkID link) {
    pb_istream_t istream = pb_istream_from_buffer(data, len);
    if (id == DatumTypeID_INFO_Raw) {
        ESP_LOGI("LINK", "INFO_Raw datum from %s: (%d bytes)", ((link == LinkID_USBSerial) ? "USB" : (link == LinkID_TCP ? "TCP" : "LoRa")), len);
        ESP_LOG_BUFFER_HEX("LINK", data, len);
        ESP_LOGI("LINK", "Writing RAW datum to log!");
        logger_log_data_now(&logger, LOG_DTYPE_DATA_RAW, data, len);
        // uint8_t buf[256];
        // ESP_ERROR_CHECK(logger_read_bytes_raw(&logger, 0, sizeof(buf), buf));
        // ESP_LOG_BUFFER_HEX("FLASH CONTENTS", buf, sizeof(buf));
    } else if (id == DatumTypeID_CMD_EraseLog) {
        Command_EraseLog cmd;
        Resp_BasicError resp;
        resp.has_error = false;

        if (pb_decode(&istream, Command_EraseLog_fields, &cmd)) {
            if (xSemaphoreTake(tl_spi_hold_sem, pdMS_TO_TICKS(1000)) == pdTRUE) {
                telemetry_enable = false;
                set_logstate(LOGSTATE_LOGGING_STOPPED, 0);
                if (cmd.type == EraseType_Erase_Log) {
                    ESP_LOGI("LINK", "Erasing log as commanded.");
                    esp_err_t e = logger_erase_log(&logger);
                    e |= logger_flush(&logger);
                    if (e != ESP_OK) {
                        ESP_LOGE("LINK", "Error %s erasing log!", esp_err_to_name(e));
                        resp.has_error = true;
                    }
                    ESP_LOGI("LINK", "Logger erase done");
                    resp.error = e;
                    link_send_datum(DatumTypeID_RESP_EraseLog, Resp_BasicError_fields, &resp);
                }

                if (cmd.type == EraseType_Erase_Clean) {
                    ESP_LOGI("LINK", "Cleaning log as commanded.");
                    esp_err_t e = logger_clean_log(&logger);
                    e |= logger_flush(&logger);
                    if (e != ESP_OK) {
                        ESP_LOGE("LINK", "Error %s cleaning log!", esp_err_to_name(e));
                        resp.has_error = true;
                    }
                    ESP_LOGI("LINK", "Logger clean done");
                    resp.error = e;
                    link_send_datum(DatumTypeID_RESP_EraseLog, Resp_BasicError_fields, &resp);
                }
                xSemaphoreGive(tl_spi_hold_sem);
                link_flush();
                telemetry_enable = true;
            } else {
                ESP_LOGE("LINK", "Error stopping TX for erase/clean");
            }
        } else {
            ESP_LOGW("LINK", "Failed to parse Command_EraseLog");
        }
    } else if (id == DatumTypeID_CMD_Ping) {
        Command_Ping cmd;
        Resp_Ping resp;
        if (pb_decode(&istream, Command_Ping_fields, &cmd)) {
            ESP_LOGI("LINK", "Pong!");
            // Link info is unneccesary
            resp.link = link;
            link_send_datum(DatumTypeID_RESP_Ping, Resp_Ping_fields, &resp);
            link_flush();
        }
    } else if (id == DatumTypeID_CMD_ConfigureLogging) {
        Command_ConfigureLogging cmd;
        Resp_BasicError resp = {
            .error = 0,
            .has_error = false
        };

        if (pb_decode(&istream, Command_ConfigureLogging_fields, &cmd)) {
            ESP_LOGI("LINK", "Configuring logging");
            if (cmd.setting == LoggingMode_Stopped) {
                set_logstate(LOGSTATE_LOGGING_STOPPED, 0);
            } else if (cmd.setting == LoggingMode_ManualHz) {
                if (cmd.has_parameter) {
                    set_logstate(LOGSTATE_LOGGING_MANUAL_HZ, cmd.parameter);
                } else {
                    resp.has_error = true;
                    resp.error = 1;
                }
            } else if (cmd.setting == LoggingMode_Armed) {
                start_autolog();
            } else {
                resp.has_error = true;
                resp.error = 2;
            }

            logger_flush(&logger);

            // Link info is unneccesary
            link_send_datum(DatumTypeID_RESP_ConfigureLogging, Resp_BasicError_fields, &resp);
            link_flush();
        }
        link_flush();
    } else if (id == DatumTypeID_CMD_LogStatus) {
        ESP_LOGI("LINK", "Sending log status");
        LogStatus resp;
        get_logstatus(&resp);

        link_send_datum(DatumTypeID_INFO_LogStatus, LogStatus_fields, &resp);
        link_flush();
    } else if (id == DatumTypeID_CMD_DownloadLog) {
        Resp_BasicError resp = {
            .error = 0,
            .has_error = false
        };

        // Must not be logging now
        bool stopped = (log_state_hz == 0) && (log_state == LOGSTATE_LOGGING_STOPPED);
        if (!stopped) {
            resp.error = 1;
            resp.has_error = true;
        }
        // Must have a log to download
        bool has_log = logger_get_current_log_size(&logger) > 0;
        if (!has_log) {
            resp.error = 2;
            resp.has_error = true;
        }
        // Must be connected over USB
        if (!USB_AVAILABLE) {
            resp.error = 3;
            resp.has_error = true;
        }

        link_send_datum(DatumTypeID_RESP_DownloadLog, Resp_BasicError_fields, &resp);
        if (USB_AVAILABLE) link_flush(); // FLush OK here because it must be ok USB

        if (!resp.has_error) {
            xTaskNotify(log_download_task_handle, LOGDL_NOTIFY_START, eSetValueWithOverwrite);
        }
    } else if (id == DatumTypeID_CMD_ConfigSensorOutput) {
        Command_ConfigSensorOutput cmd;
        if (pb_decode(&istream, Command_ConfigSensorOutput_fields, &cmd)) {
            sensor_output_raw = cmd.raw;
            ESP_LOGW("LINK", "Setting sensor date output rate to %dHz", (int)cmd.rate_hz);
            if ((sensor_output_hz == 0) && (cmd.rate_hz > 0)) {
                // Start
                sensor_output_hz = cmd.rate_hz;
                xTaskCreate(sensor_output_task, "sensor_output", 1024 * 4, NULL, 10, &sensor_output_task_handle);
            } else if ((sensor_output_hz > 0) && (cmd.rate_hz <= 0)) {
                // Stop
                sensor_output_hz = 0;
                vTaskDelete(sensor_output_task_handle);
            } else if ((sensor_output_hz > 0) && (cmd.rate_hz > 0)) {
                // Reconfigure timing
                // For now, do nothing!
                sensor_output_hz = cmd.rate_hz;
            }

        } else {
            ESP_LOGW("LINK", "Error decoding Command_ConfigSensorOutput");
        }
    } else if (id == DatumTypeID_CMD_Config) {
        Config cmd;
        if (pb_decode(&istream, Config_fields, &cmd)) {
            DatumTypeID typeout;
            // All the actual handling of the message is dealt with by config
            Config response = config_handle_request(&cmd, &typeout);
            link_send_datum(typeout, Config_fields, &response);
        } else {
            ESP_LOGW("LINK", "Error decoding Config");
        }
    } else if (id == DatumTypeID_CMD_ConfigEnumerate) {
        // TODO: USB ONLY???
        telemetry_enable = false;

        int num = 0;
        for (config_iterator_t it = config_iterator_first(); !config_iterator_done(it); it = config_iterator_next(it)) {
            config_entry_t entry = config_iterator_get_value(it);
            Config message = config_value_as_message(entry);
            link_send_datum(DatumTypeID_RESP_ConfigValue, Config_fields, &message);
            num += 1;
        }

        Resp_ConfigEnumerateDone finish = {
            .has_error = false,
            .num_values = num
        };
        link_send_datum(DatumTypeID_RESP_ConfigEnumerateDone, Resp_ConfigEnumerateDone_fields, &finish);
        link_flush();
        telemetry_enable = true;
    } else if (id == DatumTypeID_CMD_Reboot) {
        // Welp, it reboots!
        esp_restart();
    } else if (id == DatumTypeID_CMD_RequestDeviceInfo) {
        DeviceInfo info = {
            .device_type = TalkerID_Tracker_V3,
            .has_name = false
        };

        const char* appvers = esp_app_get_description()->version;
        memcpy(info.fw_version, (char*)appvers, 32);

        if (config_is_set(CONFIG_DEVICE_NAME_KEY)) {
            info.has_name = true;
            config_get_string(CONFIG_DEVICE_NAME_KEY, (char*)info.name);
        }

        link_send_datum(DatumTypeID_RESP_DeviceInfo, DeviceInfo_fields, &info);
        link_flush();
    }
}

void link_lora_decode_datum(int i, DatumTypeID id, int len, uint8_t* data) {
    link_decode_datum(i, id, len, data, LinkID_LoRa);
}

void link_usb_decode_datum(int i, DatumTypeID id, int len, uint8_t* data) {
    link_decode_datum(i, id, len, data, LinkID_USBSerial);
}

void link_tcp_decode_datum(int i, DatumTypeID id, int len, uint8_t* data) {
    link_decode_datum(i, id, len, data, LinkID_TCP);
}


void lora_link_flush() {
    int size, ndatum;
    uint8_t* tx_buffer = (uint8_t*)calloc(1, 256);
    xSemaphoreTakeRecursive(fmgr_mutex, portMAX_DELAY);
    ndatum = fmgr_get_n_datum_encoded(&lora_outgoing_fmgr);

    fmgr_set_frame_id(&lora_outgoing_fmgr, TalkerID_Tracker_V3);
    uint8_t* data = fmgr_get_frame(&lora_outgoing_fmgr, &size);
    memcpy(tx_buffer, data, size);
    fmgr_reset(&lora_outgoing_fmgr);
    xSemaphoreGiveRecursive(fmgr_mutex);
    radio_tx(tx_buffer, size);
    free(tx_buffer);
}

static void usb_receive_task(void* args) {
    static int usb_recv_frameidx = 0;
    // 0 read len MSB, 4 read len LSB, 1 reading data, 2 esc, 3 done UNUSED
    static int usb_recv_state = 0;
    static int usb_recv_state_ret = 0;
    static uint16_t frame_size;
    static uint8_t val_unesc;

    int buffer_size;
    uint8_t* buffer = fmgr_get_buffer(&usb_incoming_fmgr, &buffer_size);

    while (1) {
        // Blocking read 1 byte
        uint8_t byte;
        int nread = usb_serial_jtag_read_bytes(&byte, 1, pdMS_TO_TICKS(USB_TIMEOUT_MS));
        if (nread != 1) {
            usb_active = false;
            continue;
        } else {
            usb_active = true;
        }

        // Reset FSM on zero byte!
        if (byte == 0) {
            usb_recv_frameidx = 0;
            usb_recv_state = 0;
            continue;
        }

        // On esc, save state and go to escape state
        if (byte == USB_SER_ESC) {
            usb_recv_state_ret = usb_recv_state;
            usb_recv_state = 2;
            // printf("ESC\n");
            continue;
            // If in esc state, set val unesc to the unescaped!
        } else if (usb_recv_state == 2) {
            val_unesc = (byte == USB_SER_ESC_ESC) ? USB_SER_ESC : 0;
            usb_recv_state = usb_recv_state_ret; // Pop state
            // printf("UNESC: %d\n", val_unesc);
        } else {
            // Otherwise the value is just the byte
            val_unesc = byte;
        }

        switch (usb_recv_state) {
            // Receive MSB of frame length
            case 0:
                frame_size = val_unesc;

                usb_recv_state = 4;
                break;
            case 4:
                frame_size |= (uint16_t)val_unesc << 8;

                usb_recv_state = 1;
                usb_recv_frameidx = 0;
                // ESP_LOGI("LINK_USB", "Got frame size: %d", frame_size);
                break;

            case 1:
                // ESP_LOGI("LINK_USB", "buffer[%d] = %d", usb_recv_frameidx, val_unesc);
                buffer[usb_recv_frameidx++] = val_unesc;
                if (usb_recv_frameidx >= frame_size) {
                    usb_recv_state = 3;

                    ESP_LOGI("LINK_USB", "Frame received over usb, %d bytes.", frame_size);
                    fmgr_load_frame(&usb_incoming_fmgr, buffer, frame_size);
                    bool ok = fmgr_decode_frame(&usb_incoming_fmgr, link_usb_decode_datum);
                    if (!ok)
                        ESP_LOGW("LINK_USB", "Error decoding frame!");
                    fmgr_reset(&usb_incoming_fmgr);


                    // For command/response, will need a set of semaphores to signal that the response has been received so that task can do this
                    // link_send_datum(cmd)
                    // <<wait for semaphore to signal response>>
                    // handle_response(global_variable) (or even better use a queue/semaphore to send a pointer to the data that is "hot" on the buffer)

                    usb_recv_state = 0;
                    usb_recv_frameidx = 0;
                }
                break;
        }
    }
}

static void init_usb() {
    usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
        .rx_buffer_size = USB_SERIAL_BUF_SIZE,
        .tx_buffer_size = USB_SERIAL_BUF_SIZE,
    };

    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_serial_jtag_config));
    ESP_LOGI("USB", "USB_SERIAL_JTAG init done");

    xTaskCreate(usb_receive_task, "usb_receive_task", 1024 * 4, NULL, 10, NULL);
    ESP_LOGI("USB", "Created USB receiving task");
}

void link_flush() {
    // This dumb thing was blocking... make sure this never happens!!!
     // printf("Flush eeeeEeEEE!\n");
     // const char* msg = "Flush eeeeEeEEE!\n";
     // usb_serial_jtag_write_bytes(msg, strlen(msg), portMAX_DELAY);
    xSemaphoreTakeRecursive(fmgr_mutex, portMAX_DELAY);
    int n_lora_datum = fmgr_get_n_datum_encoded(&lora_outgoing_fmgr);
    int n_usb_datum = fmgr_get_n_datum_encoded(&usb_outgoing_fmgr);
    int n_tcp_datum = fmgr_get_n_datum_encoded(&tcp_outgoing_fmgr);
    xSemaphoreGiveRecursive(fmgr_mutex);

    int size;

    // TODO: One FMGR (high capacity) for USB/TCP?
    if (TCP_AVAILABLE) {
        if (n_lora_datum > 0) {
            fmgr_set_frame_id(&lora_outgoing_fmgr, TalkerID_Tracker_V3);
            uint8_t* data = fmgr_get_frame(&lora_outgoing_fmgr, &size);
            if (tcp_tx(data, size) != ESP_OK) {
                ESP_LOGW("LINK_TCP", "Error transmitting frame");
            }
            fmgr_reset(&lora_outgoing_fmgr);
        }
        if (n_tcp_datum > 0) {
            fmgr_set_frame_id(&tcp_outgoing_fmgr, TalkerID_Tracker_V3);
            uint8_t* data = fmgr_get_frame(&tcp_outgoing_fmgr, &size);
            if (tcp_tx(data, size) != ESP_OK) {
                ESP_LOGW("LINK_TCP", "Error transmitting frame");
            }
            fmgr_reset(&tcp_outgoing_fmgr);
        }
        if (n_usb_datum > 0) {
            fmgr_set_frame_id(&usb_outgoing_fmgr, TalkerID_Tracker_V3);
            uint8_t* data = fmgr_get_frame(&usb_outgoing_fmgr, &size);

            if (tcp_tx(data, size) != ESP_OK) {
                ESP_LOGW("LINK_TCP", "Error transmitting frame");
            }
            fmgr_reset(&usb_outgoing_fmgr);
        }
    } else if (USB_AVAILABLE) {
        // ESP_LOGI("LINK_USB", "Sending datum (%d USB, %d LoRa)", n_usb_datum, n_lora_datum);
        xSemaphoreTakeRecursive(fmgr_mutex, portMAX_DELAY);

        if (n_lora_datum > 0) {
            fmgr_set_frame_id(&lora_outgoing_fmgr, TalkerID_Tracker_V3);
            uint8_t* data = fmgr_get_frame(&lora_outgoing_fmgr, &size);
            if (usb_tx(data, size) != ESP_OK) {
                ESP_LOGW("LINK_USB", "Error transmitting frame");
            }
            fmgr_reset(&lora_outgoing_fmgr);

        }
        if (n_tcp_datum > 0) {
            fmgr_set_frame_id(&tcp_outgoing_fmgr, TalkerID_Tracker_V3);
            uint8_t* data = fmgr_get_frame(&tcp_outgoing_fmgr, &size);
            if (usb_tx(data, size) != ESP_OK) {
                ESP_LOGW("LINK_USB", "Error transmitting frame");
            }
            fmgr_reset(&tcp_outgoing_fmgr);
        }
        if (n_usb_datum > 0) {
            fmgr_set_frame_id(&usb_outgoing_fmgr, TalkerID_Tracker_V3);
            uint8_t* data = fmgr_get_frame(&usb_outgoing_fmgr, &size);
            // ESP_LOGI("LINK_USB", "Flushing %d bytes", size);

            if (usb_tx(data, size) != ESP_OK) {
                ESP_LOGW("LINK_USB", "Error transmitting frame");
            }
            fmgr_reset(&usb_outgoing_fmgr);
        }


        xSemaphoreGiveRecursive(fmgr_mutex);
    } else {
        if (n_lora_datum > 0) {
            lora_link_flush();
        }
    }
}

void link_send_datum(DatumTypeID type, const pb_msgdesc_t* fields, const void* src_struct) {
    xSemaphoreTakeRecursive(fmgr_mutex, portMAX_DELAY);
    // DONE: Switch between USB and LoRa accordingly
    // ESP_LOGI("LINK", "Sending datum %d", (int)type);

    if (TCP_AVAILABLE) {
        if (!fmgr_encode_datum(&tcp_outgoing_fmgr, type, fields, src_struct)) {
            ESP_LOGW("LINK_TCP", "Failed to encode datum");

            // TX should be "instant" (blocking buffer-copy)
            link_flush();

            if (!fmgr_encode_datum(&tcp_outgoing_fmgr, type, fields, src_struct)) {
                while (1) {
                    ESP_LOGE("LINK_TCP", "Critical link error: clogged");
                    fmgr_reset(&tcp_outgoing_fmgr);
                    // vTaskDelay(pdMS_TO_TICKS(500));
                }
            }
        }
        // This should also be "sorta" instant...
        link_flush();
    } else if (USB_AVAILABLE) {
        // Send with USB link
        // Keep trying!!!
        if (!fmgr_encode_datum(&usb_outgoing_fmgr, type, fields, src_struct)) {
            ESP_LOGW("LINK_USB", "Failed to encode datum");

            // TX should be "instant" (blocking buffer-copy)
            link_flush();

            if (!fmgr_encode_datum(&usb_outgoing_fmgr, type, fields, src_struct)) {
                while (1) {
                    ESP_LOGE("LINK_USB", "Critical link error: clogged");
                    fmgr_reset(&usb_outgoing_fmgr);
                    // vTaskDelay(pdMS_TO_TICKS(500));
                }
            }
        }

        // This is USB, make it fast!
        link_flush();

    } else {
        // Send with LoRa link

        // Keep trying!!!
        if (!fmgr_encode_datum(&lora_outgoing_fmgr, type, fields, src_struct)) {
            // TODO: Encode this data into a backlog, then schedule to send asynchronously rather than block

            ESP_LOGW("LINK_LoRa", "Automatically Flushing");
            if (USB_AVAILABLE || TCP_AVAILABLE) {
                link_flush();
            } else {
                lora_link_flush();
            }

            // Wait for TX done!
            // xSemaphoreTake(lora_txdone_sem, portMAX_DELAY);
            // xSemaphoreTake(lora_txdone_sem, portMAX_DELAY);

            if (!fmgr_encode_datum(&lora_outgoing_fmgr, type, fields, src_struct)) {
                while (1) {
                    ESP_LOGE("LINK_LoRa", "Critical link error: clogged");
                    fmgr_reset(&lora_outgoing_fmgr);
                    // vTaskDelay(pdMS_TO_TICKS(500));
                }
            }
        }
    }

    xSemaphoreGiveRecursive(fmgr_mutex);
}

static bool usb_write_byte(uint8_t b, TickType_t wait) {
    static uint8_t tmpbuf[2] = { 0 };
    if (b == 0) {
        tmpbuf[0] = USB_SER_ESC;
        tmpbuf[1] = USB_SER_ESC_NULL;
        return usb_serial_jtag_write_bytes(tmpbuf, 2, wait) == 2;
    } else if (b == USB_SER_ESC) {
        tmpbuf[0] = USB_SER_ESC;
        tmpbuf[1] = USB_SER_ESC_ESC;
        return usb_serial_jtag_write_bytes(tmpbuf, 2, wait) == 2;
    } else {
        return usb_serial_jtag_write_bytes(&b, 1, wait) == 1;
    }
}

static esp_err_t usb_tx(uint8_t* data, int len) {
    if (!USB_AVAILABLE) {
        return ESP_ERR_INVALID_STATE;
    } else {
        uint16_t len_b = len;
        // LSB first
        bool sent = usb_write_byte((len_b) & 0xFF, pdMS_TO_TICKS(10));
        sent &= usb_write_byte((len_b >> 8) & 0xFF, pdMS_TO_TICKS(10));

        for (int i = 0; i < len; ++i)
            sent &= usb_write_byte(data[i], pdMS_TO_TICKS(10));

        uint8_t zero = 0x00;
        for (int i = 0; i < 10; ++i)
            sent &= usb_serial_jtag_write_bytes(&zero, 1, pdMS_TO_TICKS(10));

        if (!sent) {
            ESP_LOGE("LINK_USB", "Error sending over USB (likely the buffer is full)");
            return ESP_ERR_NOT_FINISHED;
        } else
            return ESP_OK;
    }
}

static void init_tl_spi() {
    ESP_LOGI("SYS", "Initializing Telemetry/Logging SPI...");
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_TL_MISO,
        .mosi_io_num = PIN_TL_MOSI,
        .sclk_io_num = PIN_TL_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
    };
    //Initialize the SPI bus
    esp_err_t e = spi_bus_initialize(TL_SPI, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(e);

    gpio_reset_pin(PIN_FLASH_CS);
    gpio_reset_pin(PIN_RFM_CS);
}

static void log_timer_task() {
    esp_err_t e = logger_log_data_now(&logger, LOG_DTYPE_DATA_DEFAULT, (uint8_t*)&log_data, sizeof(log_data));
    log_data.flags = 0; // Clear flags
    if (e != ESP_OK) {
        ESP_LOGE("LOGGER", "Error logging data in timer task!");
        esp_timer_restart(log_led_timer, 1000000 / 16);

        // set_logstate(LOGSTATE_LOGGING_STOPPED, 0);
    }
    prelog_flush_some(&prelogger);
}

void set_logstate(logging_state_t state, int manual_hz) {
    if (esp_timer_is_active(log_led_timer)) esp_timer_stop(log_led_timer);
    log_state = state;
    switch (state) {
        case LOGSTATE_LOGGING_STOPPED:
            log_state_hz = 0;
            if (state == LOGSTATE_LOGGING_AUTO_ARMED || state == LOGSTATE_LOGGING_AUTO_FLIGHT || state == LOGSTATE_LOGGING_AUTO_LANDED || state == LOGSTATE_LOGGING_AUTO_LIFTOFF)
                stop_autolog();

            gpio_set_level(PIN_LED_G, 0);
            break;
        case LOGSTATE_LOGGING_MANUAL_HZ:
            log_state_hz = (manual_hz < 0) ? 0 : manual_hz;

            gpio_set_level(PIN_LED_G, 1);
            break;
        case LOGSTATE_LOGGING_AUTO_ARMED:
            // Blink at 3Hz
            ESP_ERROR_CHECK(esp_timer_start_periodic(log_led_timer, 1000000 / 6));
            prelog_start_recording(&prelogger);

            log_state_hz = LOG_HZ_AUTO_ARMED;
            break;
        case LOGSTATE_LOGGING_AUTO_FLIGHT:
            log_state_hz = LOG_HZ_AUTO_FLIGHT;

            gpio_set_level(PIN_LED_G, 1);
            break;
        case LOGSTATE_LOGGING_AUTO_LANDED:
            log_state_hz = LOG_HZ_AUTO_LANDED;

            gpio_set_level(PIN_LED_G, 1);
            break;
        case LOGSTATE_LOGGING_AUTO_LIFTOFF:
            log_state_hz = LOG_HZ_AUTO_LIFTOFF;
            prelog_start_flushing(&prelogger);

            gpio_set_level(PIN_LED_G, 1);
            break;
    }

    log_data_event_t* logevent = (log_data_event_t*)calloc(1, sizeof(log_data_event_t));
    logevent->event = LOGDATA_EVENT_STATE;
    logevent->argument = (log_state << 10) | (log_state_hz & 0b1111111111);
    logger_queue_log_data_now(&logger, LOG_DTYPE_DATA_EVENT, (uint8_t*)logevent, sizeof(*logevent));

    ESP_LOGI("LOGGER", "Hz: %d", log_state_hz);

    if (log_state_hz == 0) {
        if (esp_timer_is_active(logging_timer))
            ESP_ERROR_CHECK(esp_timer_stop(logging_timer));
    } else {
        if (esp_timer_is_active(logging_timer)) {
            ESP_ERROR_CHECK(esp_timer_restart(logging_timer, 1000000 / log_state_hz));
        } else {
            ESP_ERROR_CHECK(esp_timer_start_periodic(logging_timer, 1000000 / log_state_hz));
        }
    }
}

static void prelog_recording_timer_task() {
    prelog_give_data(&prelogger, log_data);
}

static void init_logging() {
    const esp_flash_spi_device_config_t device_config = {
        .host_id = TL_SPI,
        .cs_id = 0,
        .cs_io_num = PIN_FLASH_CS,
        .io_mode = SPI_FLASH_DIO,
        .freq_mhz = 20, // 20MHz is a safe speed below the GPIO matrix limit
    };
    // esp_err_t e = spi_bus_add_flash_device()
    ESP_ERROR_CHECK(spi_bus_add_flash_device(&ext_flash, &device_config));

    // Probe the Flash chip and initialize it
    esp_err_t err = esp_flash_init(ext_flash);
    if (err != ESP_OK) {
        ESP_LOGE("FLASH", "Failed to initialize external Flash: %s (0x%x)", esp_err_to_name(err), err);
    }

    uint32_t id;
    ESP_ERROR_CHECK(esp_flash_read_id(ext_flash, &id));
    ESP_LOGI("FLASH", "Initialized external Flash, size=%" PRIu32 " KB, ID=0x%" PRIx32, ext_flash->size / 1024, id);

    err = logger_init(&logger, ext_flash);
    if (err != ESP_OK) {
        ESP_LOGE("FLASH", "Failed to initialize logger.");
    }

    logger_flush(&logger);

    // uint8_t buf[256];
    // ESP_ERROR_CHECK(logger_read_bytes_raw(&logger, 0, sizeof(buf), buf));
    // ESP_LOG_BUFFER_HEX("FLASH CONTENTS", buf, sizeof(buf));

    const esp_timer_create_args_t logtimer_args = {
        .callback = &log_timer_task,
        .name = "log_timer_task"
    };

    ESP_ERROR_CHECK(esp_timer_create(&logtimer_args, &logging_timer));
    set_logstate(LOGSTATE_LOGGING_STOPPED, -1);

    // Would need larger stack, but for now it's just using static buffers;
    xTaskCreate(log_download_task, "log_dl_task", 1024 * 8, NULL, 10, &log_download_task_handle);

    // flight_timer = xTimerCreate("flight_timer", pdMS_TO_TICKS(1000 * LIFTOFF_DURATION), pdFALSE, (void*)0, flight_timer_callback);
    const esp_timer_create_args_t flight_timer_args = {
        .callback = &flight_timer_callback,
        .name = "flight_timer"
    };

    ESP_ERROR_CHECK(esp_timer_create(&flight_timer_args, &flight_timer));


    int32_t prelog_buf_size, prelog_rate_hz;
    config_get_int(CONFIG_LOGGING_ARMED_RINGBUF_CAP_KEY, &prelog_buf_size);
    config_get_int(CONFIG_LOGGING_ARMED_RINGBUF_HZ_KEY, &prelog_rate_hz);

    prelog_init(&prelogger, &logger, prelog_buf_size);

    const esp_timer_create_args_t prelog_timer_args = {
        .callback = &prelog_recording_timer_task,
        .name = "prelog_timer"
    };

    ESP_ERROR_CHECK(esp_timer_create(&prelog_timer_args, &prelog_recording_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(prelog_recording_timer, 1000000 / prelog_rate_hz));
}

// TODO: Make a special extension to the frame manager to reduce the amount of memcpys?
static Resp_DownloadLog_Segment download_segment;
static void log_download_task(void* arg) {
    // Log download:
    // -> CMD_DownloadLog
    // <- ACK_DownloadLog_Segment * N
    // IGNORE IGNORE IGNORE IGNORE IGNORE-> ACK_DownloadLog_Segment // TBD whether this will be implemented!!!
    // <- ACK_Download_Complete
    // -> ACK_Download_Complete
    while (1) {
        // Wait for notification signalling to DownloadLog
        uint32_t not_val;
        xTaskNotifyWait(0, 0, &not_val, portMAX_DELAY);
        if (not_val == LOGDL_NOTIFY_START) {
            telemetry_enable = false;
            size_t current_address = logger.log_start_address;

            uint16_t total_crc = 0;
            int n_segments = 0;

            // TODO: Make SUPER SURE that log is totally frozen during this time (mutex lock)
            // Send download segments
            esp_err_t e = ESP_OK;
            while ((current_address != logger.log_end_address) && (e == ESP_OK)) {
                // Clear, not memsetting because that would be big with the data...
                download_segment.has_error = false;
                download_segment.length = 0;
                download_segment.segment_crc16 = 0;
                download_segment.start_address = 0;
                download_segment.error = 0;

                // Wrap!
                if (current_address == LOG_MEMORY_SIZE_B) current_address = 0;

                size_t segment_length = 0;
                if (logger.log_end_address > current_address) {
                    segment_length = min(logger.log_end_address - current_address, 4096);
                } else {
                    segment_length = min(LOG_MEMORY_SIZE_B - current_address, 4096);
                }

                // ESP_LOGI("LOGGER", "Downloading segment starting at address %d, %d bytes long.", current_address, segment_length);

                download_segment.data.size = segment_length;
                download_segment.start_address = current_address;
                download_segment.length = segment_length;
                e = logger_read_bytes_raw(&logger, current_address, segment_length, download_segment.data.bytes);

                uint16_t frame_crc = fmgr_util_crc16(download_segment.data.bytes, segment_length);
                download_segment.segment_crc16 = frame_crc;
                total_crc ^= frame_crc;

                if (e != ESP_OK) {
                    download_segment.has_error = true;
                    download_segment.error = e;
                    if (USB_AVAILABLE) {
                        link_send_datum(DatumTypeID_RESP_DownloadLog_Segment, Resp_DownloadLog_Segment_fields, &download_segment);
                        link_flush();
                    }
                    ESP_LOGE("LOGGER", "Error while logger_read_bytes_raw in log download: %s", esp_err_to_name(e));
                    break; // Break the downloading inner loop
                } else {
                    if (USB_AVAILABLE) {
                        link_send_datum(DatumTypeID_RESP_DownloadLog_Segment, Resp_DownloadLog_Segment_fields, &download_segment);
                    } else {
                        e = ESP_ERR_NOT_ALLOWED;
                        break;
                    }
                    // TODO: wait for ack here, fail download on timeout!
                }

                // esp_task_wdt_reset();
                // TODO: Feed the WDT during this task...
                // Or maybe sleep a little bit during the flush...
                n_segments++;
                current_address += segment_length;
            }

            // Success?
            Acknowledgement_Download_Complete ack;
            ack.has_error = e != ESP_OK;
            ack.error = e;
            ack.log_crc16 = total_crc;
            ESP_LOGI("LOGGER", "Log download completed! %d segments sent, error code: %s, full crc xor: %d", n_segments, esp_err_to_name(e), total_crc);
            link_send_datum(DatumTypeID_ACK_Download_Complete, Acknowledgement_Download_Complete_fields, &ack);
            link_flush();
            telemetry_enable = true;

        } else {
            continue;
        }
    }
}

SemaphoreHandle_t radio_interrupt_semaphore;
TaskHandle_t radio_handle_interrupt;
void IRAM_ATTR radio_handle_interrupt_fromisr(void* arg) {
    // xTaskResumeFromISR(radio_handle_interrupt);
    xSemaphoreGiveFromISR(radio_interrupt_semaphore, NULL);
}

void radio_handle_interrupt_task(void* arg) {
    while (1) {
        // vTaskSuspend(NULL);
        xSemaphoreTake(radio_interrupt_semaphore, portMAX_DELAY);
        sx127x_handle_interrupt((sx127x*)arg);
    }
}

QueueHandle_t radio_rx_dispatching_queue;
typedef struct radio_rx_data_t {
    sx127x* device;
    uint8_t* data;
    uint16_t data_length
} radio_rx_data_t;
void radio_rx_callback_dispatcher(sx127x* device, uint8_t* data, uint16_t data_length) {
    if (data_length > 256) {
        ESP_LOGE("RADIO", "Error dispatching receive callback: too much data!");
        return;
    }
    uint8_t* data_owned = (uint8_t*)calloc(1, data_length);
    memcpy(data_owned, data, data_length);

    radio_rx_data_t rxdat = {
        .device = device,
        .data = data_owned,
        .data_length = data_length
    };
    xQueueSend(radio_rx_dispatching_queue, &rxdat, portMAX_DELAY);
}

void radio_rx_callback_worker(void* args) {
    radio_rx_data_t arg;
    ESP_LOGI("RADIO", "Started receiving worker");

    while (1) {
        xQueueReceive(radio_rx_dispatching_queue, &arg, portMAX_DELAY);
        // sx127x* device = arg.device;
        uint8_t* data = arg.data;
        uint16_t data_length = arg.data_length;

        // gpio_set_level(PIN_LED_G, 0);

        // if (xSemaphoreTake(radio_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
        //     ESP_LOGE("RADIO", "Error acquiring radio for receive information!");
        // }

        // THIS IS NOT NECCESARY IF NO RSSI IS NEEDED!
        // if (xSemaphoreTake(tl_spi_hold_sem, 1000) != pdTRUE) {
        //     ESP_LOGE("RADIO", "Aborted Rx because bus is held.");
        //     continue
        // } else {
            // int16_t rssi;
            // ESP_ERROR_CHECK(sx127x_rx_get_packet_rssi(device, &rssi));
            // float snr;
            // ESP_ERROR_CHECK(sx127x_lora_rx_get_packet_snr(device, &snr));
            // int32_t frequency_error;
            // ESP_ERROR_CHECK(sx127x_rx_get_frequency_error(device, &frequency_error));

            // xSemaphoreGive(radio_mutex);
        xSemaphoreGive(tl_spi_hold_sem);

        fmgr_load_frame(&lora_incoming_fmgr, data, data_length);
        if (fmgr_check_crc(&lora_incoming_fmgr)) {
            fmgr_decode_frame(&lora_incoming_fmgr, link_lora_decode_datum);
        } else {
            ESP_LOGW("LINK_LoRa", "CRC failed on incoming packet, was (%d), should be (%d)", fmgr_get_cur_frame_crc(&lora_incoming_fmgr), fmgr_calculate_crc(&lora_incoming_fmgr));
        }

        // ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_CAD, SX127x_MODULATION_LORA, device));
        // }
        free(arg.data);
    }
}

// This can't be called from an ISR... right?
static void radio_txcomplete_callback(sx127x* device) {
    // ESP_LOGI("RADIO", "TX Completed.");
    // ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_CAD, SX127x_MODULATION_LORA, radio));
    ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_RX_CONT, SX127x_MODULATION_LORA, radio));

    // Signal
    xSemaphoreGive(lora_txdone_sem);
    gpio_set_level(PIN_LED_R, 0);
    // xSemaphoreGive(radio_mutex);
}

// void radio_cad_callback(sx127x* device, int cad_detected) {
//     if (cad_detected == 0) {
//         // ESP_LOGI("RADIO", "CAD not detected");
//         ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_CAD, SX127x_MODULATION_LORA, device));
//         return;
//     }

//     gpio_set_level(PIN_LED_G, 1);

//     // if (xSemaphoreTake(radio_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
//     //     ESP_LOGE("RADIO", "Error acquiring radio lock for receive, receive aborted! (lockup?)");
//     // }
//     // put into RX mode first to handle interrupt as soon as possible
//     ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_RX_CONT, SX127x_MODULATION_LORA, device));
//     ESP_LOGI("RADIO", "CAD detected\n");
// }

static int RADIO_CODINGRATE = SX127x_CR_4_5;
static void init_radio() {
    lora_txdone_sem = xSemaphoreCreateBinary();
    xSemaphoreGive(lora_txdone_sem);
    // radio_mutex = xSemaphoreCreateBinary();
    // xSemaphoreGive(radio_mutex);
    radio_rx_dispatching_queue = xQueueCreate(16, sizeof(radio_rx_data_t));

    radio_interrupt_semaphore = xSemaphoreCreateBinary();

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 10e6, // 10MHz is sx1276's max SPI freq
        .spics_io_num = PIN_RFM_CS,
        .queue_size = 16,
        .command_bits = 0,
        .address_bits = 8,
        .dummy_bits = 0,
        .mode = 0
    };
    ESP_ERROR_CHECK(spi_bus_add_device(TL_SPI, &dev_cfg, &radio_spi_device));

    gpio_reset_pin(PIN_RFM_RST);
    gpio_set_direction(PIN_RFM_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_RFM_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(PIN_RFM_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_ERROR_CHECK(sx127x_create(radio_spi_device, &radio));
    ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_SLEEP, SX127x_MODULATION_LORA, radio));

    int32_t freq = CONFIG_RADIO_FREQ_HZ_DEFAULT;
    config_get_int(CONFIG_RADIO_FREQ_HZ_KEY, &freq);
    ESP_ERROR_CHECK(sx127x_set_frequency((uint64_t)freq, radio)); // 915MHz
    ESP_ERROR_CHECK(sx127x_lora_reset_fifo(radio));
    ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_STANDBY, SX127x_MODULATION_LORA, radio));

    LoRa_Bandwidth_t bw_config = CONFIG_RADIO_BANDWIDTH_DEFAULT;
    config_get_enum(CONFIG_RADIO_BANDWIDTH_KEY, (int32_t*)&bw_config);

    ESP_ERROR_CHECK(sx127x_lora_set_bandwidth(config_convert_bandwidth(bw_config), radio));
    ESP_ERROR_CHECK(sx127x_lora_set_implicit_header(NULL, radio));

    int RADIO_SFS[] = {
        SX127x_SF_6,
        SX127x_SF_7,
        SX127x_SF_8,
        SX127x_SF_9,
        SX127x_SF_10,
        SX127x_SF_11,
        SX127x_SF_12
    };
    int32_t cfg_sf = 7;
    config_get_int(CONFIG_RADIO_LORA_SPREADING_FACTOR_KEY, &cfg_sf);
    ESP_ERROR_CHECK(sx127x_lora_set_modem_config_2(RADIO_SFS[cfg_sf - 6], radio));

    int RADIO_CRS[] = {
        SX127x_CR_4_5,
        SX127x_CR_4_6,
        SX127x_CR_4_7,
        SX127x_CR_4_8,
    };
    int32_t cfg_cr = 5;
    config_get_int(CONFIG_RADIO_LORA_CODING_RATE_KEY, &cfg_cr);
    RADIO_CODINGRATE = RADIO_CRS[cfg_cr - 5];

    // ESP_ERROR_CHECK(sx127x_lora_set_modem_config_1(SX127x_CR_4_5, radio));
    ESP_ERROR_CHECK(sx127x_lora_set_syncword(0x18, radio));
    ESP_ERROR_CHECK(sx127x_set_preamble_length(8, radio));

    sx127x_tx_set_callback(radio_txcomplete_callback, radio);
    sx127x_rx_set_callback(radio_rx_callback_dispatcher, radio);
    // sx127x_lora_cad_set_callback(radio_cad_callback, radio);

    ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_RX_CONT, SX127x_MODULATION_LORA, radio));
    // ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_CAD, SX127x_MODULATION_LORA, radio));

    BaseType_t task_code = xTaskCreatePinnedToCore(radio_handle_interrupt_task, "handle interrupt", 8196, radio, 3, &radio_handle_interrupt, xPortGetCoreID());
    if (task_code != pdPASS) {
        ESP_LOGE("RADIO", "Can't create task: %d", task_code);
        sx127x_destroy(radio);
        return;
    }

    xTaskCreate(radio_rx_callback_worker, "radio_rx_worker", 1024 * 8, NULL, 4, NULL);


    gpio_reset_pin(PIN_RFM_D0);
    gpio_set_direction((gpio_num_t)PIN_RFM_D0, GPIO_MODE_INPUT);
    gpio_pulldown_en((gpio_num_t)PIN_RFM_D0);
    gpio_pullup_dis((gpio_num_t)PIN_RFM_D0);
    gpio_set_intr_type((gpio_num_t)PIN_RFM_D0, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add((gpio_num_t)PIN_RFM_D0, radio_handle_interrupt_fromisr, (void*)radio);

    int32_t cfg_power = CONFIG_RADIO_POWER_DEFAULT;
    config_get_int(CONFIG_RADIO_POWER_KEY, &cfg_power);
    ESP_ERROR_CHECK(sx127x_tx_set_pa_config(SX127x_PA_PIN_BOOST, cfg_power, radio));
    // uint8_t paconfig_value = SX127x_PA_PIN_BOOST | 0b00001111 | 0b01110000;
    // sx127x_spi_write_register(0x09, &paconfig_value, 1, spi_device);
    ESP_ERROR_CHECK(sx127x_tx_set_ocp(true, 120, radio));

    // uint8_t data[1] = { 0 };
    // data[0] = 0b10000000 | 15;
    // sx127x_spi_write_register(REG_PA_DAC, data, 1, radio->spi_device);

    ESP_LOGI("RADIO", "Successfully initialized radio.");
}

// static void radio_test_tx() {
//     sx127x_tx_header_t header = {
//         .enable_crc = true,
//         .coding_rate = SX127x_CR_4_5 };
//     ESP_ERROR_CHECK(sx127x_lora_tx_set_explicit_header(&header, radio));
//     uint8_t data[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199 };
//     ESP_ERROR_CHECK(sx127x_lora_tx_set_for_transmission(data, sizeof(data), radio));
//     ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_TX, SX127x_MODULATION_LORA, radio));
//     ESP_LOGI("RADIO", "Starting TX");
// }

static void radio_tx(uint8_t* data, int len) {
    if (xSemaphoreTake(tl_spi_hold_sem, pdMS_TO_TICKS(5)) == pdTRUE) {
        // How to not TX while RXing...
        if (xSemaphoreTake(lora_txdone_sem, pdMS_TO_TICKS(5000)) != pdTRUE) {
            ESP_LOGE("RADIO", "Transmit timed out (5sec). Attempting to continue.");
            // xSemaphoreGive(lora_txdone_sem);
            xSemaphoreGive(tl_spi_hold_sem);
            return;
        }

        // ESP_LOGI("RADIO", "Starting TX: %d bytes.", len);

        gpio_set_level(PIN_LED_R, 1);

        sx127x_tx_header_t header = {
           .enable_crc = true,
           .coding_rate = RADIO_CODINGRATE };
        ESP_ERROR_CHECK(sx127x_lora_tx_set_explicit_header(&header, radio));
        if (len > 255) {
            ESP_LOGE("RADIO", "Message length overrun!");
            gpio_set_level(PIN_LED_R, 0);
            xSemaphoreGive(tl_spi_hold_sem);
            return;
        }


        // ESP_LOGI("RADIO", "Actually TXing");

        ESP_ERROR_CHECK(sx127x_lora_tx_set_for_transmission(data, (uint8_t)len, radio));
        ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_TX, SX127x_MODULATION_LORA, radio));
        xSemaphoreGive(tl_spi_hold_sem);
    } else {
        ESP_LOGE("RADIO", "Held too long, Tx failed.");
    }
}

// static void radio_tx_blocking(uint8_t* data, int len) {
//     radio_tx(data, len);
//     xSemaphoreTake(lora_txdone_sem, portMAX_DELAY);
// }



void start_autolog() {
    // This uses AC mode, expecting the rocket to be in steady-state (on the launch rod) when armed
    xSemaphoreTake(sensors_mutex, portMAX_DELAY);
    adxl375_set_thresh_act(&adxl, LIFTOFF_ACC_THRESHOLD_G * 1000);
    adxl375_set_act_mode(&adxl, false, true, true, true);
    adxl375_set_act_mode(&adxl, true, true, true, true);
    adxl375_enable_interrupts(&adxl, ADXL_INT_ACT);

    // TODO: Determine acceptable inact threshold for flight phase detection
    // FIXME: Inact might not work, because:
    // - Falling can be 0g
    // - Accelerometer will still have gravitational acceleration during landing
    // Implement a solution in sensor_task that checks the deltas of acceleration values, if they are changing under the noise threshold, we're good

    // Probably want to switch it to AC mode because 
    // adxl375_set_inact_mode(&adxl, false, true, true, true);
    // adxl375_set_thresh_inact(&adxl, LAND_THRESHOLD_G * 1000);
    // adxl375_set_time_inact(&adxl, LAND_TIME);
    xSemaphoreGive(sensors_mutex);

    set_logstate(LOGSTATE_LOGGING_AUTO_ARMED, 0);
}

void stop_autolog() {
    if (esp_timer_is_active(flight_timer)) esp_timer_stop(flight_timer);
    xSemaphoreTake(sensors_mutex, portMAX_DELAY);
    adxl375_set_act_mode(&adxl, false, false, false, false);
    adxl375_enable_interrupts(&adxl, 0);
    xSemaphoreGive(sensors_mutex);
}

// TODO: LOGSTATE_LOGGING_AUTO_FLIGHT -> LOGSTATE_LOGGING_AUTO_LANDED
static void flight_timer_callback(void* arg) {
    if (log_state == LOGSTATE_LOGGING_AUTO_LIFTOFF)
        set_logstate(LOGSTATE_LOGGING_AUTO_FLIGHT, 0);
}

static void adxl_int_worker(void* arg) {
    while (1) {
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
        xSemaphoreTake(sensors_mutex, portMAX_DELAY);
        // At ADXL ISR right now
        uint8_t intsrc;
        adxl375_get_int_source(&adxl, &intsrc);

        switch (log_state) {
            case LOGSTATE_LOGGING_AUTO_ARMED:
                if (intsrc & ADXL_INT_ACT) {
                    set_logstate(LOGSTATE_LOGGING_AUTO_LIFTOFF, 0);
                    // DONE: Start flight timer to switch to LOGSTATE_LOGGING_AUTO_FLIGHT
                    ESP_ERROR_CHECK(esp_timer_start_once(flight_timer, 1000000 * LIFTOFF_DURATION));

                    log_data_event_t* logevent = (log_data_event_t*)calloc(1, sizeof(log_data_event_t));
                    logevent->event = LOGDATA_EVENT_LIFTOFF;
                    logger_queue_log_data_now(&logger, LOG_DTYPE_DATA_EVENT, (uint8_t*)logevent, sizeof(*logevent));

                    Alert alt = {
                         .type = AlertType_ALT_Liftoff,
                            .has_data = false
                    };

                    // TODO: Make sure it's safe to flush here!
                    link_send_datum(DatumTypeID_INFO_Alert, Alert_fields, &alt);
                    link_flush();
                }
                break;
                // TODO: Inact interrupt for LANDED?
            default:
                break;
        }
        xSemaphoreGive(sensors_mutex);
    }
}

GPS current_gps;
uDeviceHandle_t gnssHandle;

#define PVT_MESSAGE_BUFFER_LENGTH  (92 + U_UBX_PROTOCOL_OVERHEAD_LENGTH_BYTES)
static char* gps_ubx_buf;
static int32_t TELEM_DIV = 1;

static void gps_ubx_callback(uDeviceHandle_t devHandle, const uGnssMessageId_t* pMessageId,
    int32_t errorCodeOrLength, void* pCallbackParam) {
    char* pBuffer = (char*)pCallbackParam;
    int32_t length;
    uGnssDec_t* pDec;
    uGnssDecUbxNavPvt_t* pUbxNavPvt;
    int64_t utcTimeNanoseconds;

    static int gpsctr = 0;

    (void)pMessageId;

    if (errorCodeOrLength >= 0) {
        // Read the message into our buffer
        length = uGnssMsgReceiveCallbackRead(devHandle, pBuffer, errorCodeOrLength);
        if (length >= 0) {
            // Call the uGnssDec() API to decode the message
            pDec = pUGnssDecAlloc(pBuffer, length);
            if ((pDec != NULL) && (pDec->errorCode == 0)) {
                // No need to check pDec->id (or pMessageId) here since we have
                // only asked for UBX-NAV-PVT messages.
                pUbxNavPvt = &(pDec->pBody->ubxNavPvt);

                utcTimeNanoseconds = uGnssDecUbxNavPvtGetTimeUtc(pUbxNavPvt);

                current_gps.lat = ((float)pUbxNavPvt->lat) / 1e7;
                current_gps.lon = ((float)pUbxNavPvt->lon) / 1e7;

                current_gps.alt = pUbxNavPvt->hMSL / 1000.0f;
                current_gps.utc_time = utcTimeNanoseconds;

                current_gps.has_fix_status = true;
                current_gps.fix_status = pUbxNavPvt->fixType;

                current_gps.has_sats_used = true;
                current_gps.sats_used = pUbxNavPvt->numSV;

                // DONE: Check that GGA works as the primary message
                log_data.gps_lat = current_gps.lat;
                log_data.gps_lon = current_gps.lon;
                log_data.gps_alt = current_gps.alt;
                log_data.flags |= LOG_FLAG_GPS_FRESH;

                // Since this runs at 10Hz, only send telemetry every 1Hz
                if ((gpsctr % TELEM_DIV) == 0) {
                    xTaskNotify(telemetry_task, 0, eNoAction);
                }
                gpsctr++;
            }
            uGnssDecFree(pDec);
        }
    } else {
        ESP_LOGI("GPS", "Empty or bad message received.");
    }
}

static void init_gps_worker(void* arg) {
    gpio_set_level(PIN_LED_R, 1);
    uGnssTransportHandle_t transportHandle;
    int32_t err = uPortInit();
    if (err) {
        ESP_LOGE("GPS", "UBXLIB Port init error: %" PRIi32, err);
        return;
    }
    err = uGnssInit();
    if (err) {
        ESP_LOGE("GPS", "UBXLIB GNSS init error: %" PRIi32, err);
        return;
    }


    transportHandle.uart = uPortUartOpen(UART_GPS, 9600, NULL, U_GNSS_UART_BUFFER_LENGTH_BYTES, PIN_GPS_SDI, PIN_GPS_SDO, -1, -1);
    err = uGnssAdd(U_GNSS_MODULE_TYPE_M8, U_GNSS_TRANSPORT_UART, transportHandle, -1, true, &gnssHandle);
    if (err) {
        ESP_LOGE("GPS", "UBXLIB GNSS add error: %" PRIi32, err);
        return;
    }

    // SET BAUD TO 115200!
    // This message was generated using u-center, it sets the module's UART1 to UBX output at 115200 baud
    // B5 62 06 00 14 00 01 00 00 00 D0 08 00 00 00 C2 01 00 23 00 01 00 00 00 00 00 DA 52
    uint8_t cfgmessage[] = {
        181, 98, 6, 0, 20, 0, 1, 0, 0, 0, 208, 8, 0, 0, 0, 194, 1, 0, 35, 0, 1, 0, 0, 0, 0, 0, 218, 82
    };
    uGnssMsgSend(gnssHandle, (const char*)cfgmessage, sizeof(cfgmessage));
    vTaskDelay(pdMS_TO_TICKS(3000));

    uGnssRemove(gnssHandle);
    uPortUartClose(transportHandle.uart);

    // ESP_LOGI("GPS", "Closing port and transitioning to 115200 baud");

    transportHandle.uart = uPortUartOpen(UART_GPS, 115200, NULL, U_GNSS_UART_BUFFER_LENGTH_BYTES, PIN_GPS_SDI, PIN_GPS_SDO, -1, -1);
    err = uGnssAdd(U_GNSS_MODULE_TYPE_M8, U_GNSS_TRANSPORT_UART, transportHandle, -1, true, &gnssHandle);
    if (err) {
        ESP_LOGE("GPS", "UBXLIB GNSS2 re-add error: %" PRIi32, err);
        return;
    }

    uGnssSetUbxMessagePrint(gnssHandle, DEBUG_PRINT_UBXLIB);
    uGnssCfgSetProtocolOut(gnssHandle, U_GNSS_PROTOCOL_NMEA, false);
    uGnssCfgSetProtocolOut(gnssHandle, U_GNSS_PROTOCOL_UBX, true);

    // Power up the GNSS module
    uGnssPwrOn(gnssHandle);

    uGnssMessageId_t messageId;
    messageId.type = U_GNSS_PROTOCOL_UBX;
    messageId.id.ubx = 0x0107; // The message class/ID of a UBX-NAV-PVT message
    if (uGnssCfgSetMsgRate(gnssHandle, &messageId, 1) != 0) {
        ESP_LOGW("GPS", "Error setting UBX NAV_PVT output");
    }

    config_get_int(CONFIG_TELEMETRY_DIVIDER_KEY, &TELEM_DIV);
    int32_t measprdms = CONFIG_GPS_NAV_PERIOD_MS_DEFAULT;
    int32_t measpernav = CONFIG_GPS_NAV_COUNT_DEFAULT;
    config_get_int(CONFIG_GPS_NAV_PERIOD_MS_KEY, &measprdms);
    config_get_int(CONFIG_GPS_NAV_COUNT_KEY, &measpernav);

    err = uGnssCfgSetRate(gnssHandle, measprdms, measpernav, -1);
    if (err) {
        ESP_LOGE("GPS", "UBXLIB GNSS rate error: %" PRIi32, err);
        return;
    }

    gps_ubx_buf = (char*)pUPortMalloc(PVT_MESSAGE_BUFFER_LENGTH);

    uGnssMsgReceiveStart(gnssHandle, &messageId, gps_ubx_callback, gps_ubx_buf);

    ESP_LOGI("GPS", "Successfully init GPS!");
    gpio_set_level(PIN_LED_R, 0);
    vTaskSuspend(NULL);
}

static void init_gps() {
    xTaskCreate(&init_gps_worker, "gps_init_task", 1024 * 4, NULL, 10, NULL);
}

static void battmon_task(void* arg) {
    int battmon_delay_ticks = pdMS_TO_TICKS(1000.0f / BATT_MON_HZ);
    while (true) {
        if (gpio_get_level(PIN_SDA) && MAX17048_Exists(&battery_monitor)) {
            float soc = MAX1708_SOC(&battery_monitor);
            if (soc <= 105.0f) {
                float vbatt = MAX17048_Voltage(&battery_monitor) / 1000.0f;
                bool on_battery = (!battery_monitor.timed_out) && (vbatt > 3.2f);
                if (on_battery) {
                    ESP_LOGI("BATT", "Battery SOC: %f", soc);
                    Battery datum;
                    datum.battery_voltage = vbatt;
                    datum.percentage = soc;

                    if (telemetry_enable)
                        link_send_datum(DatumTypeID_INFO_Battery, Battery_fields, &datum);
                }
            }
            vTaskDelay(battmon_delay_ticks);
        }
    }
}

esp_timer_handle_t sensor_timer;

void DEBUG_send_float(char* name, char* suffix, float value) {
    DebugDataOutput message;
    if ((strlen(name) < 64) && ((suffix == NULL) || (strlen(suffix) < 32)) && USB_AVAILABLE) {
        // memcpy(message.suffix)
        message.has_suffix = suffix != NULL;
        if (message.has_suffix) {
            memcpy(message.suffix, suffix, strlen(suffix) + 1);
        }

        memcpy(message.name, name, strlen(name) + 1);
        message.has_float_value = true;
        message.float_value = value;
        link_send_datum(DatumTypeID_INFO_Debug, DebugDataOutput_fields, &message);
    } else {
        // ESP_LOGE("DEBUG", "Too large of a name for debug value");
    }
}

void DEBUG_send_float3(char* name, char* suffix, float values[3]) {
    char namebuf[64];
    int namelen = strlen(name);
    if (namelen < (64 - 2)) {
        memcpy(namebuf, name, namelen);
        namebuf[namelen] = ':';
        namebuf[namelen + 2] = '\0';

        for (int i = 0; i < 3; ++i) {
            namebuf[namelen + 1] = 'X' + i;
            DEBUG_send_float(namebuf, suffix, values[i]);
        }
    } else {
        ESP_LOGE("DEBUG", "Too large of a name for debug vector");
    }
}

void DEBUG_output_task(void* arg) {
    bool lps22_on, lsm6dsm_on, adxl_on, lis3mdl_on, compass_on, orientation_on, worldacc_on;
    bool usb_output = false;
    config_get_bool(CONFIG_MISC_DEBUG_MONITOR_USB_KEY, &usb_output);

    config_get_bool(CONFIG_MISC_DEBUG_MONITOR_LPS22_KEY, &lps22_on);
    config_get_bool(CONFIG_MISC_DEBUG_MONITOR_ADXL375_KEY, &adxl_on);
    config_get_bool(CONFIG_MISC_DEBUG_MONITOR_LIS3MDL_KEY, &lis3mdl_on);
    config_get_bool(CONFIG_MISC_DEBUG_MONITOR_LSM6DSM_KEY, &lsm6dsm_on);
    config_get_bool(CONFIG_MISC_DEBUG_MONITOR_COMPASS_KEY, &compass_on);
    config_get_bool(CONFIG_MISC_DEBUG_MONITOR_ORIENTATION_KEY, &orientation_on);
    config_get_bool(CONFIG_MISC_DEBUG_MONITOR_WORLD_ACC_KEY, &worldacc_on);

    float debug_mon_hz;
    config_get_float(CONFIG_MISC_DEBUG_MONITOR_HZ_KEY, &debug_mon_hz);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000.0f / debug_mon_hz));
        if (lps22_on) {
            // ESP_LOGI("DEBUG-LPS22", "hPa:  %f", pressure_hPa_raw);
            DEBUG_send_float("LPS22 Raw Pressure", "hPa", pressure_hPa_raw);
            DEBUG_send_float("LPS22 Raw Altitude", "m", altitude_m_raw);
            DEBUG_send_float("LPS22 Filtered Altitude", "m", altitude_m);
            DEBUG_send_float("LPS22 Filtered V-Speed", "m/s", v_speed_m_s);
        }
        if (lsm6dsm_on) {
            // ESP_LOGI("DEBUG-LSM6DSM",
                // "Acceleration [g]:  %4.2f  %4.2f  %4.2f",
                // acceleration_g_raw[0], acceleration_g_raw[1], acceleration_g_raw[2]);
            DEBUG_send_float3("LSM6DSM Raw Acceleration", "g", (float*)acceleration_g_raw);
            // ESP_LOGI("DEBUG-LSM6DSM",
                // "Angular rate [dps]:  %4.2f  %4.2f  %4.2f",
                // angular_rate_dps_raw[0], angular_rate_dps_raw[1], angular_rate_dps_raw[2]);
            DEBUG_send_float3("LSM6DSM Raw Gyro", "dps", (float*)angular_rate_dps_raw);
        }
        if (lis3mdl_on) {
            // ESP_LOGI("DEBUG-LIS3MDL",
                // "Magnetic Field [mG]:  %4.2f  %4.2f  %4.2f",
                // magnetic_mG_raw[0], magnetic_mG_raw[1], magnetic_mG_raw[2]);
            DEBUG_send_float3("LIS3MDL Raw Magnetic", "mG", (float*)magnetic_mG_raw);
        }
        // ESP_LOGI("DEBUG-LIS3MDL", "Temperature [C]:  %4.2f\n", temperature_degC);
        if (adxl_on) {
            // ESP_LOGI("DEBUG-ADXL",
                // "Acceleration [200g]:  %4.2f  %4.2f  %4.2f",
                // adxl_acceleration_g_raw[0], adxl_acceleration_g_raw[1], adxl_acceleration_g_raw[2]);
            DEBUG_send_float3("ADXL375 Raw Acceleration", "h", (float*)adxl_acceleration_g_raw);
        }
        if (compass_on) {
            float heading_raw = (atan2(magnetic_mG[1], magnetic_mG[0]) / 3.14159265) * 180.0f;
            // ESP_LOGI("DEBUG-COMPASS", "Heading [deg]: %4.2f", heading_raw);
            DEBUG_send_float("LIS3MDL Compass", "deg", heading_raw);
        }
        if (orientation_on) {
            FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
            // ESP_LOGI("DEBUG-ORIENTATION", "Euler Angles [deg]: (roll): %4.2f, (pitch): %4.2f, (yaw): %4.2f", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
            DEBUG_send_float3("Orientation (Euler Angles)", "deg", euler.array);

        }
        if (worldacc_on) {
            FusionVector worldacc = FusionVectorMultiplyScalar(FusionAhrsGetEarthAcceleration(&ahrs), GRAVITY_G);
            // ESP_LOGI("DEBUG-WORLDACC",
            //     "World-Space Acceleration [g]:  North: %4.2f West: %4.2f Up: %4.2f",
            //     worldacc.axis.x, worldacc.axis.y, worldacc.axis.z);
            DEBUG_send_float3("Fused world-space acceleration", "m/s2", worldacc.array);
        }
        // ESP_LOGI("DEBUG-LIS3MDL", "Temperature [C]:  %4.2f\n", temperature_degC);
    }
}

static void telemetry_tx_task(void* arg) {
    while (1) {
        BaseType_t _res = xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
        if (telemetry_enable) {
            Altitude alt = {
                .has_v_speed = true,
                .alt_m = altitude_m,
                .v_speed = v_speed_m_s
            };

            // Flush link every 1 second
            link_send_datum(DatumTypeID_INFO_GPS, GPS_fields, &current_gps);
            link_send_datum(DatumTypeID_INFO_Altitude, Altitude_fields, &alt);
            link_flush();
        }
    }
}

static void sensor_output_task(void* arg) {
    while (1) {
        SensorData data;

        // Raw option mostly for calibration reasons!
        if (sensor_output_raw) {
            memcpy(data.lsm_acceleration_g, (float*)acceleration_g_raw, sizeof(acceleration_g_raw));
            memcpy(data.lsm_gyro_dps, (float*)angular_rate_dps_raw, sizeof(angular_rate_dps_raw));
            memcpy(data.lis_magnetic_mG, (float*)magnetic_mG_raw, sizeof(magnetic_mG_raw));
            memcpy(data.adxl_acceleration_g, (float*)adxl_acceleration_g_raw, sizeof(adxl_acceleration_g_raw));
            data.lps_pressure_hPa = pressure_hPa_raw;
        } else {
            memcpy(data.lsm_acceleration_g, (float*)acceleration_g, sizeof(acceleration_g));
            memcpy(data.lsm_gyro_dps, (float*)angular_rate_dps, sizeof(angular_rate_dps));
            memcpy(data.lis_magnetic_mG, (float*)magnetic_mG, sizeof(magnetic_mG));
            memcpy(data.adxl_acceleration_g, (float*)adxl_acceleration_g, sizeof(adxl_acceleration_g));
            data.lps_pressure_hPa = pressure_hPa;
        }

        FusionVector worldacc = FusionVectorMultiplyScalar(FusionAhrsGetEarthAcceleration(&ahrs), GRAVITY_G);
        memcpy(data.filtered_world_acceleration_m_s, (float*)worldacc.array, sizeof(worldacc.array));

        data.filtered_orientation = (Quaternion){ 0 };
        FusionQuaternion orientation = FusionAhrsGetQuaternion(&ahrs);
        // memcpy(data.filtered_orientation.components, orientation.array, sizeof(orientation.array));
        data.filtered_orientation.x = orientation.element.x;
        data.filtered_orientation.y = orientation.element.y;
        data.filtered_orientation.z = orientation.element.z;
        data.filtered_orientation.w = orientation.element.w;
        data.has_filtered_orientation = true;

        data.has_filtered_altimetry = true;
        // data.filtered_altimetry.
        data.filtered_altimetry.alt_m = altitude_m;
        data.filtered_altimetry.v_speed = v_speed_m_s;
        data.filtered_altimetry.has_v_speed = true;

        if (USB_AVAILABLE || TCP_AVAILABLE) {
            link_send_datum(DatumTypeID_INFO_SensorData, SensorData_fields, &data);
            // ESP_LOGW("SENSOR", "OUT!");
        } else {
            // Do literally nothing
        }

        vTaskDelay(pdMS_TO_TICKS(1000.0f / sensor_output_hz));
    }
}

esp_timer_handle_t once_per_second_timer;
static void once_per_second() {
    static uint32_t seconds = 0;

    // Flush logger every 5 seconds
    // DONE: Only do this while logging!
    if ((seconds % 5) == 0 && (log_state_hz != 0)) {
        ESP_LOGI("LOGGER", "Flushing NVS");
        logger_flush(&logger);

        if (telemetry_enable) {
            LogStatus stat;
            get_logstatus(&stat);
            link_send_datum(DatumTypeID_INFO_LogStatus, LogStatus_fields, &stat);
        }
    }

    seconds += 1;
}

static void init_nvs() {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    err = nvs_open("config", NVS_READWRITE, &nvs_config_handle);
    if (err != ESP_OK) {
        ESP_LOGI("CONFIG", "Error (%s) opening config NVS handle!\n", esp_err_to_name(err));
    }
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
    int32_t event_id, void* event_data) {
    // if (event_id == WIFI_EVENT_AP_STACONNECTED) {
    //     wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*)event_data;
    //      ESP_LOGI("WIFI", "station "MACSTR" join, AID=%d",
    //          MAC2STR(event->mac), event->aid);
    // } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
    //     wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*)event_data;
    //      ESP_LOGI("WIFI", "station "MACSTR" leave, AID=%d, reason=%d",
    //          MAC2STR(event->mac), event->aid, event->reason);
    // }
}

void init_wifi_ap(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        &wifi_event_handler,
        NULL,
        NULL));

    wifi_config_t wifi_config = {
        .ap = {
            // .ssid = (uint8_t*)WIFI_SSID,
            // .ssid_len = strlen(WIFI_SSID),
            .channel = WIFI_CHANNEL,
            // .password = (uint8_t*)WIFI_PASS,
            .max_connection = 4,
#ifdef CONFIG_ESP_WIFI_SOFTAP_SAE_SUPPORT
            .authmode = WIFI_AUTH_WPA3_PSK,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
#else /* CONFIG_ESP_WIFI_SOFTAP_SAE_SUPPORT */
            .authmode = WIFI_AUTH_WPA2_PSK,
#endif
            .pmf_cfg = {
                    .required = true,
            },
        },
    };

    memcpy(wifi_config.ap.ssid, WIFI_SSID, strlen(WIFI_SSID));
    wifi_config.ap.ssid_len = strlen(WIFI_SSID);
    memcpy(wifi_config.ap.password, WIFI_PASS, strlen(WIFI_PASS));
    if (strlen(WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI("WIFI", "wifi_init_softap finished. SSID:%s password:%s channel:%d",
        WIFI_SSID, WIFI_PASS, WIFI_CHANNEL);
}

char tcp_tx_writebuf[1024];
size_t tcp_tx_writebuf_fill = 0;

static bool tcp_write_byte(uint8_t b) {
    static uint8_t tmpbuf[2] = { 0 };
    int bufl = 1;
    if (b == 0) {
        tmpbuf[0] = USB_SER_ESC;
        tmpbuf[1] = USB_SER_ESC_NULL;
        bufl = 2;
    } else if (b == USB_SER_ESC) {
        tmpbuf[0] = USB_SER_ESC;
        tmpbuf[1] = USB_SER_ESC_ESC;
        bufl = 2;
    } else {
        tmpbuf[0] = b;
        bufl = 1;
    }
    if (tcp_tx_writebuf_fill + bufl >= sizeof(tcp_tx_writebuf)) {
        if (!tcp_flush()) {
            return false;
        }
    }
    memcpy(&tcp_tx_writebuf[tcp_tx_writebuf_fill], tmpbuf, bufl);
    tcp_tx_writebuf_fill += bufl;
    return true;
}

static bool tcp_flush() {
    // ESP_LOGI("LINK_TCP", "Starting flush");
    int to_write = tcp_tx_writebuf_fill;
    while (to_write > 0) {
        int written = send(TCP_ACTIVE_SOCK, tcp_tx_writebuf + (tcp_tx_writebuf_fill - to_write), to_write, 0);
        if (written < 0) {
            ESP_LOGE("LINK_TCP", "Error occurred during sending: errno %d", errno);
            tcp_tx_writebuf_fill = 0;
            return false;
        }
        to_write -= written;
    }
    tcp_tx_writebuf_fill = 0;
    // ESP_LOGI("LINK_TCP", "Done");
    return true;
}


static esp_err_t tcp_tx(uint8_t* data, int len) {
    if (!TCP_AVAILABLE) {
        return ESP_ERR_INVALID_STATE;
    } else {
        uint16_t len_b = len;
        // LSB first
        bool sent = tcp_write_byte((len_b) & 0xFF);
        sent &= tcp_write_byte((len_b >> 8) & 0xFF);

        for (int i = 0; i < len; ++i)
            sent &= tcp_write_byte(data[i]);

        tcp_flush();

        // uint8_t zero = 0x00;
        // for (int i = 0; i < 10; ++i)
        //     sent &= usb_serial_jtag_write_bytes(&zero, 1, pdMS_TO_TICKS(10));
        uint8_t zeros[10] = { 0 };
        send(TCP_ACTIVE_SOCK, zeros, sizeof(zeros), 0);

        if (!sent) {
            ESP_LOGE("LINK_TCP", "Error sending over TCP");
            return ESP_ERR_NOT_FINISHED;
        } else
            return ESP_OK;
    }
}

void link_wifi_tcp_task(void* arg) {
    ESP_LOGI("LINK_TCP", "Starting TCP task");
    char addr_str[128];
    int ip_protocol = 0;
    int keepAlive = 1;
    int keepIdle = 5;
    int keepInterval = 5;
    int keepCount = 3;
    struct sockaddr_storage dest_addr;

    int bufsize = 0;
    uint8_t* fmgr_in_buffer = fmgr_get_buffer(&tcp_incoming_fmgr, &bufsize);

    encoded_reader_t tcp_reader = { 0 };
    encoded_reader_init(&tcp_reader, fmgr_in_buffer, bufsize);

    config_get_int(CONFIG_WIFI_TCP_PORT_KEY, &LINK_TCP_PORT);

    // Get IPV4 address
    struct sockaddr_in* dest_addr_ip4 = (struct sockaddr_in*)&dest_addr;
    dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr_ip4->sin_family = AF_INET;
    dest_addr_ip4->sin_port = htons(LINK_TCP_PORT);
    ip_protocol = IPPROTO_IP;



    int listen_sock = socket(AF_INET, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0) {
        ESP_LOGE("LINK_WiFi", "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    ESP_LOGI("LINK_TCP", "Socket created");

    int err = bind(listen_sock, (struct sockaddr*)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE("LINK_TCP", "Socket unable to bind: errno %d", errno);
        goto CLEAN_UP;
    }
    ESP_LOGI("LINK_TCP", "Socket bound, port %" PRIi32, LINK_TCP_PORT);

    err = listen(listen_sock, 1);
    if (err != 0) {
        ESP_LOGE("LINK_TCP", "Error occurred during listen: errno %d", errno);
        goto CLEAN_UP;
    }

    uint8_t rx_buffer[5000];
    while (1) {
        ESP_LOGI("LINK_TCP", "Socket listening");

        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t addr_len = sizeof(source_addr);
        TCP_ACTIVE_SOCK = accept(listen_sock, (struct sockaddr*)&source_addr, &addr_len);
        if (TCP_ACTIVE_SOCK < 0) {
            ESP_LOGE("LINK_TCP", "Unable to accept connection: errno %d", errno);
            break;
        }

        // Set tcp keepalive option
        setsockopt(TCP_ACTIVE_SOCK, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
        setsockopt(TCP_ACTIVE_SOCK, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
        setsockopt(TCP_ACTIVE_SOCK, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
        setsockopt(TCP_ACTIVE_SOCK, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));

        if (source_addr.ss_family == PF_INET) {
            inet_ntoa_r(((struct sockaddr_in*)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
        }

        ESP_LOGI("LINK_TCP", "Socket accepted ip address: %s", addr_str);

        // do_retransmit(sock);
        // Do stuff with socket

        int len = 0;
        tcp_active = true;
        do {
            len = recv(TCP_ACTIVE_SOCK, rx_buffer, sizeof(rx_buffer) - 1, 0);
            if (len < 0) {
                ESP_LOGI("LINK_TCP", "Error reading from socket, errno: %d", errno);
                break;
            } else if (len == 0) {
                ESP_LOGI("LINK_TCP", "Connection closed");
                break;
            } else {
                // Read!!
                for (int i = 0; i < len; ++i) {
                    encoded_reader_give_byte(&tcp_reader, rx_buffer[i]);
                    if (encoded_reader_has_frame(&tcp_reader)) {
                        size_t frame_len = 0;
                        uint8_t* decoded_buffer = encoded_reader_get_frame(&tcp_reader, &frame_len);

                        ESP_LOGI("LINK_TCP", "Frame received over TCP, %d bytes.", frame_len);
                        fmgr_load_frame(&tcp_incoming_fmgr, decoded_buffer, frame_len);
                        bool ok = fmgr_decode_frame(&tcp_incoming_fmgr, link_tcp_decode_datum);
                        if (!ok)
                            ESP_LOGW("LINK_TCP", "Error decoding frame!");
                        fmgr_reset(&tcp_incoming_fmgr);

                        encoded_reader_reset(&tcp_reader);
                    }
                }
                // ESP_LOG_BUFFER_CHAR("LINK_TCP", rx_buffer, len);
            }
        } while (len != 0);
        tcp_active = false;

        shutdown(TCP_ACTIVE_SOCK, 0);
        close(TCP_ACTIVE_SOCK);
    }

CLEAN_UP:
    close(listen_sock);
    vTaskDelete(NULL);
}

void link_wifi_init() {
    init_wifi_ap();
    // Priority 11?
    xTaskCreate(link_wifi_tcp_task, "link_tcp_task", 1024 * 8, NULL, 10, NULL);
}

void app_main(void) {
    gpio_install_isr_service(0);

#if DEBUG_ON_EXP
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0,
        DEBUG_UART_RX, DEBUG_UART_TX,
        UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_set_baudrate(UART_NUM_0, DEBUG_UART_BAUD));
#endif

    fmgr_mutex = xSemaphoreCreateRecursiveMutex();
    tl_spi_hold_sem = xSemaphoreCreateMutex();
    sensors_mutex = xSemaphoreCreateMutex();

    fmgr_init(&lora_outgoing_fmgr, 255);
    fmgr_init(&lora_incoming_fmgr, 255);
    fmgr_init(&usb_outgoing_fmgr, USB_SERIAL_BUF_SIZE);
    fmgr_init(&usb_incoming_fmgr, USB_SERIAL_BUF_SIZE);
    fmgr_init(&tcp_outgoing_fmgr, USB_SERIAL_BUF_SIZE);
    fmgr_init(&tcp_incoming_fmgr, USB_SERIAL_BUF_SIZE);

    init_nvs();

    init_config("config");

    config_get_string(CONFIG_WIFI_SSID_KEY, WIFI_SSID);
    if (strlen(WIFI_SSID) == 0) {
        config_get_string(CONFIG_DEVICE_NAME_KEY, WIFI_SSID);
    }
    config_get_string(CONFIG_WIFI_PASSWORD_KEY, WIFI_PASS);

    init_usb();

    // TODO: Set default values if neccesary!!
    // Reboot will be required to set configuration!!
    // Make a sub-app that is a pop-out window from the telemetry app that allows configuration
    // TODO: Maybe in the future allow hot-reloading (or just quick-reboot) and changing of config over-the-air with LoRa

    init_leds();

    init_i2c();
    init_sensor_spi();
    init_tl_spi();

    init_logging();

    init_radio();

    init_battmon();

    config_get_bool(CONFIG_SENSORS_ADXL375_DISABLE_KEY, &ADXL_DISABLE);
    config_get_bool(CONFIG_SENSORS_LIS3MDL_DISABLE_KEY, &LIS3MDL_DISABLE);
    config_get_bool(CONFIG_SENSORS_LSM6DSM_DISABLE_KEY, &LSM6DSM_DISABLE);
    config_get_bool(CONFIG_SENSORS_LPS22_DISABLE_KEY, &LPS22_DISABLE);

    xTaskCreate(adxl_int_worker, "adxl_int", 1024 * 4, NULL, 2, &adxl_int_handler);
    if (!ADXL_DISABLE) {
        ESP_LOGI("SYS", "Initializing ADXL375");
        init_adxl375(&adxl_int_handler);
    }
    if (!LSM6DSM_DISABLE) {
        ESP_LOGI("SYS", "Initializing LSM6DSM");
        init_lsm6dsm();
    }
    if (!LIS3MDL_DISABLE) {
        ESP_LOGI("SYS", "Initializing LIS3MDL");
        init_lis3mdl();
    }
    if (!LPS22_DISABLE) {
        ESP_LOGI("SYS", "Initializing LPS22");
        init_lps22();
    }

    init_sensors();

    init_gps();

    config_get_bool(CONFIG_WIFI_ENABLED_KEY, &WIFI_ENABLED);
    // Init WiFi + TCP link
    if (WIFI_ENABLED) {
        link_wifi_init();
    } else {
        ESP_LOGI("SYS", "WIFI Disabled");
    }

    // Stack overflowing... when no battery...
    xTaskCreate(battmon_task, "battmon_task", 4 * 1024, NULL, 10, NULL);

    // Log the boot, also log sensor disabilities
    log_data_event_t* logevent = (log_data_event_t*)calloc(1, sizeof(log_data_event_t));
    logevent->event = LOGDATA_EVENT_BOOT;
    logevent->argument = ((uint16_t)(LIS3MDL_DISABLE & 1) << 0) | ((uint16_t)(LSM6DSM_DISABLE & 1) << 1) | ((uint16_t)(LPS22_DISABLE & 1) << 2) | ((uint16_t)(ADXL_DISABLE & 1) << 3);
    logger_queue_log_data_now(&logger, LOG_DTYPE_DATA_EVENT, (uint8_t*)logevent, sizeof(*logevent));
    logger_flush(&logger);

    bool debug_mon_on;
    config_get_bool(CONFIG_MISC_DEBUG_MONITOR_EN_KEY, &debug_mon_on);
    if (debug_mon_on)
        xTaskCreate(DEBUG_output_task, "debug_task", 4 * 1024, NULL, 10, NULL);

    xTaskCreate(telemetry_tx_task, "telemetry_tx_task", 4 * 1024, NULL, 10, &telemetry_task);

    const esp_timer_create_args_t sensor_timer_args = {
        .callback = &sensors_routine,
        .name = "sensors_routine"
    };

    ESP_ERROR_CHECK(esp_timer_create(&sensor_timer_args, &sensor_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(sensor_timer, 1000000 / SENSOR_HZ));

    const esp_timer_create_args_t pps_timer_args = {
        .callback = &once_per_second,
        .name = "once_per_second"
    };

    ESP_ERROR_CHECK(esp_timer_create(&pps_timer_args, &once_per_second_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(once_per_second_timer, 1000000));

    while (1) {
        /* Toggle the LED state */
        vTaskDelay(portMAX_DELAY);
    }
}