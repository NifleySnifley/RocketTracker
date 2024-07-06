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

#include "max17048.h"
#include "lps22hh_reg.h"
#include "lis3mdl_reg.h"
#include "lsm6dsm_reg.h"

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

////////////////// GLOBALS //////////////////
// Peripherals
i2c_master_bus_handle_t i2c_main_bus_handle;

// ICs
MAX17048_t battery_monitor;
sx127x* radio = NULL;

// Sensors
stmdev_ctx_t lps22;
spi_device_handle_t lps22_device;

stmdev_ctx_t lis3mdl;
spi_device_handle_t lis3mdl_device;

stmdev_ctx_t lsm6dsm;
spi_device_handle_t lsm6dsm_device;

// TODO: ADXL driver
spi_device_handle_t adxl_device;


////////////////// GLOBAL HANDLES //////////////////
fmgr_t lora_fmgr;
SemaphoreHandle_t lora_txdone_sem;
SemaphoreHandle_t fmgr_mutex;

esp_flash_t* ext_flash;
logger_t logger;

nvs_handle_t nvs_config_handle;

static TaskHandle_t telemetry_task;

////////////////// INIT FUNCTIONS //////////////////
static void init_leds(void) {
    gpio_reset_pin(PIN_LED_G);
    gpio_reset_pin(PIN_LED_R);

    /* Set the GPIO as a push/pull output */
    gpio_set_direction(PIN_LED_G, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_LED_R, GPIO_MODE_OUTPUT);
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

static void radio_tx(uint8_t* data, int len);
void lora_link_flush() {
    int ndatum = fmgr_get_n_datum_encoded(&lora_fmgr);
    if (ndatum > 0) {
        int size;

        uint8_t* data = fmgr_get_frame(&lora_fmgr, &size);
        ESP_LOGI("RADIO", "Starting TX: %d datum. %d bytes.", ndatum, size);

        radio_tx(data, size);
    }
}

void link_send_datum(DatumTypeID type, const pb_msgdesc_t* fields, const void* src_struct) {
    xSemaphoreTake(fmgr_mutex, portMAX_DELAY);
    // TODO: Switch between USB and LoRa accordingly

    // Keep trying!!!
    if (!fmgr_encode_datum(&lora_fmgr, type, fields, src_struct)) {
        // TODO: Encode this data into a backlog, then schedule to send asynchronously rather than block

        lora_link_flush();
        // Wait for TX done!
        xSemaphoreTake(lora_txdone_sem, portMAX_DELAY);
        if (!fmgr_encode_datum(&lora_fmgr, type, fields, src_struct)) {
            while (1) {
                ESP_LOGE("LORA", "Critical link error: clogged");
                vTaskDelay(pdMS_TO_TICKS(500));
            }
        }
    }

    xSemaphoreGive(fmgr_mutex);
}

static void init_sensor_spi() {
    ESP_LOGI("SYS", "Initializing sensor SPI...");
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_MISO,
        .mosi_io_num = PIN_MOSI,
        .sclk_io_num = PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    //Initialize the SPI bus
    esp_err_t e = spi_bus_initialize(SENSORS_SPI, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(e);

    gpio_reset_pin(PIN_ADXL_CS);
    gpio_reset_pin(PIN_LPS_CS);
    gpio_reset_pin(PIN_LSM6DSM_CS);
    gpio_reset_pin(PIN_LIS3MDL_CS);

    // gpio_set_direction(PIN_ADXL_CS, GPIO_MODE_OUTPUT);
    // gpio_set_direction(PIN_LPS_CS, GPIO_MODE_OUTPUT);
    // gpio_set_direction(PIN_LSM6DSM_CS, GPIO_MODE_OUTPUT);
    // gpio_set_direction(PIN_LIS3MDL_CS, GPIO_MODE_OUTPUT);

    // gpio_set_level(PIN_ADXL_CS, 1);
    // gpio_set_level(PIN_LPS_CS, 1);
    // gpio_set_level(PIN_LSM6DSM_CS, 1);
    // gpio_set_level(PIN_LIS3MDL_CS, 1);
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

static void init_logging() {
    const esp_flash_spi_device_config_t device_config = {
        .host_id = TL_SPI,
        .cs_id = 0,
        .cs_io_num = PIN_FLASH_CS,
        .io_mode = SPI_FLASH_DIO,
        .freq_mhz = TL_SPI_FREQ / 1000000, // TODO: FASTER!!!
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
}

TaskHandle_t radio_handle_interrupt;
void IRAM_ATTR radio_handle_interrupt_fromisr(void* arg) {
    xTaskResumeFromISR(radio_handle_interrupt);
}

void radio_handle_interrupt_task(void* arg) {
    while (1) {
        vTaskSuspend(NULL);
        sx127x_handle_interrupt((sx127x*)arg);
    }
}

// This can't be called from an ISR... right?
static void radio_txcomplete_callback(sx127x* arg) {
    ESP_LOGI("RADIO", "TX Completed.");
    // Signal
    fmgr_reset(&lora_fmgr);
    xSemaphoreGive(fmgr_mutex);
}

static void init_radio() {
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = TL_SPI_FREQ,
        .spics_io_num = PIN_RFM_CS,
        .queue_size = 16,
        .command_bits = 0,
        .address_bits = 8,
        .dummy_bits = 0,
        .mode = 0
    };
    spi_device_handle_t spi_device;
    ESP_ERROR_CHECK(spi_bus_add_device(TL_SPI, &dev_cfg, &spi_device));

    gpio_reset_pin(PIN_RFM_RST);
    gpio_set_direction(PIN_RFM_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_RFM_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(PIN_RFM_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_ERROR_CHECK(sx127x_create(spi_device, &radio));
    ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_SLEEP, SX127x_MODULATION_LORA, radio));
    ESP_ERROR_CHECK(sx127x_set_frequency(9.14e+8, radio)); // 915MHz
    ESP_ERROR_CHECK(sx127x_lora_reset_fifo(radio));
    ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_STANDBY, SX127x_MODULATION_LORA, radio));
    ESP_ERROR_CHECK(sx127x_lora_set_bandwidth(SX127x_BW_125000, radio));
    ESP_ERROR_CHECK(sx127x_lora_set_implicit_header(NULL, radio));
    ESP_ERROR_CHECK(sx127x_lora_set_modem_config_2(SX127x_SF_7, radio));
    // ESP_ERROR_CHECK(sx127x_lora_set_modem_config_1(SX127x_CR_4_5, radio));
    ESP_ERROR_CHECK(sx127x_lora_set_syncword(0x18, radio));
    ESP_ERROR_CHECK(sx127x_set_preamble_length(8, radio));
    sx127x_tx_set_callback(radio_txcomplete_callback, radio);

    BaseType_t task_code = xTaskCreatePinnedToCore(radio_handle_interrupt_task, "handle interrupt", 8196, radio, 2, &radio_handle_interrupt, xPortGetCoreID());
    if (task_code != pdPASS) {
        ESP_LOGE("RADIO", "Can't create task: %d", task_code);
        sx127x_destroy(radio);
        return;
    }


    gpio_reset_pin(PIN_RFM_D0);
    gpio_set_direction((gpio_num_t)PIN_RFM_D0, GPIO_MODE_INPUT);
    gpio_pulldown_en((gpio_num_t)PIN_RFM_D0);
    gpio_pullup_dis((gpio_num_t)PIN_RFM_D0);
    gpio_set_intr_type((gpio_num_t)PIN_RFM_D0, GPIO_INTR_POSEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add((gpio_num_t)PIN_RFM_D0, radio_handle_interrupt_fromisr, (void*)radio);

    ESP_ERROR_CHECK(sx127x_tx_set_pa_config(SX127x_PA_PIN_BOOST, 20, radio));
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
    sx127x_tx_header_t header = {
       .enable_crc = true,
       .coding_rate = SX127x_CR_4_5 };
    ESP_ERROR_CHECK(sx127x_lora_tx_set_explicit_header(&header, radio));
    if (len > 255) {
        ESP_LOGE("RADIO", "Message length overrun!");
    }
    ESP_ERROR_CHECK(sx127x_lora_tx_set_for_transmission(data, (uint8_t)len, radio));
    ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_TX, SX127x_MODULATION_LORA, radio));
}

static int32_t sensor_platform_write(void* handle, uint8_t Reg, const uint8_t* Bufp, uint16_t len) {
    spi_device_handle_t* dev = (spi_device_handle_t*)handle;
    spi_transaction_t t = {
        .length = 8 * len,
        .addr = Reg,
        .tx_buffer = Bufp,
        .rx_buffer = NULL
    };

    if (*dev == lis3mdl_device) {
        t.addr |= 1 << 6;
    }

    esp_err_t e = spi_device_transmit(*dev, &t);
    return e != ESP_OK;
}

static int32_t sensor_platform_read(void* handle, uint8_t Reg, uint8_t* Bufp, uint16_t len) {
    spi_device_handle_t* dev = (spi_device_handle_t*)handle;

    spi_transaction_t t = {
        .length = 8 * len,
        .addr = Reg | (1 << 7),
        .tx_buffer = NULL,
        .rx_buffer = Bufp,
    };

    if (*dev == lis3mdl_device) {
        t.addr |= 1 << 6;
    }

    esp_err_t e = spi_device_transmit(*dev, &t);

    return e != ESP_OK;
}

static void platform_delay(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

static void init_lps22() {
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SENSORS_SPI_FREQ,
        .mode = 0,
        .spics_io_num = PIN_LPS_CS,
        .queue_size = 7,
        .address_bits = 8,
        .command_bits = 0,
        .dummy_bits = 0,
    };
    esp_err_t e = spi_bus_add_device(SENSORS_SPI, &devcfg, &lps22_device);
    ESP_ERROR_CHECK(e);

    lps22.read_reg = sensor_platform_read;
    lps22.write_reg = sensor_platform_write;
    lps22.mdelay = platform_delay;
    lps22.handle = &lps22_device;

    platform_delay(10);

    uint8_t whoamI = 0;
    lps22hh_device_id_get(&lps22, &whoamI);

    if (whoamI != LPS22HH_ID) {
        ESP_LOGE("LPS22", "Error initializing sensor.");
    }

    /* Restore default configuration */
    lps22hh_reset_set(&lps22, PROPERTY_ENABLE);

    uint8_t rst;
    do {
        lps22hh_reset_get(&lps22, &rst);
        // vTaskDelay(pdMS_TO_TICKS(5));
    } while (rst);

    lps22hh_block_data_update_set(&lps22, PROPERTY_ENABLE);
    /* Set Output Data Rate */
    lps22hh_data_rate_set(&lps22, LPS22HH_200_Hz);
    lps22hh_pressure_offset_set(&lps22, 0);
}

static void init_lis3mdl() {
    spi_device_interface_config_t devcfg = {
        // TODO: Can this be increased on a per-device basis?
        .clock_speed_hz = SENSORS_SPI_FREQ,
        .mode = 0,
        .spics_io_num = PIN_LIS3MDL_CS,
        .queue_size = 7,
        .address_bits = 8,
        .command_bits = 0,
        .dummy_bits = 0,
    };
    esp_err_t e = spi_bus_add_device(SENSORS_SPI, &devcfg, &lis3mdl_device);
    ESP_ERROR_CHECK(e);

    lis3mdl.read_reg = sensor_platform_read;
    lis3mdl.write_reg = sensor_platform_write;
    lis3mdl.mdelay = platform_delay;
    lis3mdl.handle = &lis3mdl_device;

    platform_delay(20);

    uint8_t whoamI = 0;
    lis3mdl_device_id_get(&lis3mdl, &whoamI);

    if (whoamI != LIS3MDL_ID) {
        ESP_LOGE("LIS3MDL", "Error initializing sensor. (%d)", whoamI);
    }

    /* Restore default configuration */
    lis3mdl_reset_set(&lis3mdl, PROPERTY_ENABLE);

    uint8_t rst;
    do {
        lis3mdl_reset_get(&lis3mdl, &rst);
    } while (rst);

    /* Enable Block Data Update */
    lis3mdl_block_data_update_set(&lis3mdl, PROPERTY_ENABLE);
    /* Set Output Data Rate */
    lis3mdl_data_rate_set(&lis3mdl, LIS3MDL_HP_300Hz);
    /* Set full scale */
    lis3mdl_full_scale_set(&lis3mdl, LIS3MDL_16_GAUSS);
    /* Enable temperature sensor */
    lis3mdl_temperature_meas_set(&lis3mdl, PROPERTY_ENABLE);
    /* Set device in continuous mode */
    lis3mdl_operating_mode_set(&lis3mdl, LIS3MDL_CONTINUOUS_MODE);
}

static void init_lsm6dsm() {
    spi_device_interface_config_t devcfg = {
        // TODO: Can this be increased on a per-device basis?
        .clock_speed_hz = SENSORS_SPI_FREQ,
        .mode = 0,
        .spics_io_num = PIN_LSM6DSM_CS,
        .queue_size = 7,
        .address_bits = 8,
        .command_bits = 0,
        .dummy_bits = 0,
    };
    esp_err_t e = spi_bus_add_device(SENSORS_SPI, &devcfg, &lsm6dsm_device);
    ESP_ERROR_CHECK(e);

    lsm6dsm.read_reg = sensor_platform_read;
    lsm6dsm.write_reg = sensor_platform_write;
    lsm6dsm.mdelay = platform_delay;
    lsm6dsm.handle = &lsm6dsm_device;

    platform_delay(15);

    uint8_t whoamI = 0;
    lsm6dsm_device_id_get(&lsm6dsm, &whoamI);

    if (whoamI != LSM6DSM_ID) {
        ESP_LOGE("LSM6DSM", "Error initializing sensor. (%d)", whoamI);
    }

    /* Restore default configuration */
    lsm6dsm_reset_set(&lsm6dsm, PROPERTY_ENABLE);

    uint8_t rst;
    do {
        lsm6dsm_reset_get(&lsm6dsm, &rst);
    } while (rst);

    /*  Enable Block Data Update */
    lsm6dsm_block_data_update_set(&lsm6dsm, PROPERTY_ENABLE);
    /* Set Output Data Rate for Acc and Gyro */
    lsm6dsm_xl_data_rate_set(&lsm6dsm, LSM6DSM_XL_ODR_416Hz);
    lsm6dsm_gy_data_rate_set(&lsm6dsm, LSM6DSM_GY_ODR_416Hz);
    /* Set full scale */
    // TODO: What's best for orientation filter??
    // FIX: Keep this for Low-G stuff, switch over to the ADXL when this accelerometer has been exceeded 
    lsm6dsm_xl_full_scale_set(&lsm6dsm, LSM6DSM_8g);
    lsm6dsm_gy_full_scale_set(&lsm6dsm, LSM6DSM_2000dps);

    /* Configure filtering chain(No aux interface)
     * Accelerometer - analog filter
     */
    lsm6dsm_xl_filter_analog_set(&lsm6dsm, LSM6DSM_XL_ANA_BW_1k5Hz);
    /* Accelerometer - LPF1 path (LPF2 not used) */
    //lsm6dsm_xl_lp1_bandwidth_set(&dev_ctx, LSM6DSM_XL_LP1_ODR_DIV_4);
    /* Accelerometer - LPF1 + LPF2 path */
    lsm6dsm_xl_lp2_bandwidth_set(&lsm6dsm,
        LSM6DSM_XL_LOW_NOISE_LP_ODR_DIV_100);
    /* Accelerometer - High Pass / Slope path */
    //lsm6dsm_xl_reference_mode_set(&dev_ctx, PROPERTY_DISABLE);
    //lsm6dsm_xl_hp_bandwidth_set(&dev_ctx, LSM6DSM_XL_HP_ODR_DIV_100);
    /* Gyroscope - filtering chain */
    lsm6dsm_gy_band_pass_set(&lsm6dsm, LSM6DSM_HP_260mHz_LP1_STRONG);
}

#define GPS_UART_RX_BUF_SIZE        (1024)
static char gps_buf[GPS_UART_RX_BUF_SIZE + 1];
// static size_t gps_total_bytes;
// static char* gps_last_buf_end;
static QueueHandle_t gps_uart_q;

// const char GPS_INIT_SEQUENCE[] = 
// "$PUBX,40,GLL,1,1,1,1,1,0*5D\r\n"; // Set rate of GLL to 10Hz

GPS current_gps;
void gps_uart_task(void* args) {
    uart_event_t event;
    nmea_s* data;

    while (1) {
        if (xQueueReceive(gps_uart_q, &event, portMAX_DELAY)) {
            switch (event.type) {
                case UART_PATTERN_DET:
                {
                    int pos = uart_pattern_pop_pos(UART_GPS);
                    if (pos == -1) {
                        uart_flush_input(pos);
                    }

                    while (pos != -1) {
                        int len = uart_read_bytes(UART_GPS, gps_buf, pos, 1000 / portTICK_PERIOD_MS);
                        if (len > 0) {
                            // Convert GNSS message to GPS so it can be parsed
                            if (gps_buf[1 + 2] == 'N') {
                                gps_buf[1 + 2] = 'P';

                                gps_buf[len] = '\n';
                                data = nmea_parse(&gps_buf[1], len, 0);
                                if (data == NULL) {
                                    ESP_LOGE("GPS", "Failed to a NMEA sentence! Type: %.5s (%d)\n", &gps_buf[1] + 1, nmea_get_type(&gps_buf[1]));
                                    uart_flush_input(UART_GPS);
                                } else {
                                    if (data->errors != 0) {
                                        ESP_LOGW("GPS", "Sentence struct contains parse errors!\n");
                                    }

                                    if (NMEA_GPGLL == data->type) {
                                        nmea_gpgll_s* pos = (nmea_gpgll_s*)data;
                                        current_gps.lat = (pos->latitude.degrees + pos->latitude.minutes / 60.0f) * (pos->latitude.cardinal == 'S' ? -1.f : 1.f);
                                        current_gps.lon = (pos->longitude.degrees + pos->longitude.minutes / 60.0f) * (pos->longitude.cardinal == 'W' ? -1.f : 1.f);

                                        // Notify telemetry task that GPS data is ready.
                                        xTaskNotify(telemetry_task, 0, eNoAction);
                                    }
                                    if (NMEA_GPGSA == data->type) {
                                        nmea_gpgsa_s* gpgsa = (nmea_gpgsa_s*)data;

                                        current_gps.has_fix_status = true;
                                        current_gps.fix_status = gpgsa->fixtype;
                                    }
                                    if (NMEA_GPGGA == data->type) {
                                        nmea_gpgga_s* gpgga = (nmea_gpgga_s*)data;
                                        current_gps.alt = gpgga->altitude;
                                        current_gps.has_sats_used = true;
                                        current_gps.sats_used = gpgga->n_satellites;
                                        current_gps.utc_time = gpgga->time.tm_sec + 100000 * gpgga->time.tm_min + 10000000 * gpgga->time.tm_hour; // TODO: Add day/year/etc?
                                    }
                                }
                                nmea_free(data);
                            }
                        } else {
                            ESP_LOGE("GPS", "Pattern read fail.");
                        }
                        pos = uart_pattern_pop_pos(UART_GPS);
                    }
                }
                break;
                case UART_FIFO_OVF:
                    ESP_LOGW("GPS", "HW FIFO Overflow");
                    uart_flush(UART_GPS);
                    xQueueReset(gps_uart_q);
                    break;
                case UART_BUFFER_FULL:
                    ESP_LOGW("GPS", "Ring Buffer Full");
                    uart_flush(UART_GPS);
                    xQueueReset(gps_uart_q);
                    break;
                case UART_BREAK:
                    ESP_LOGW("GPS", "Rx Break");
                    break;
                case UART_PARITY_ERR:
                    ESP_LOGE("GPS", "Parity Error");
                    break;
                case UART_FRAME_ERR:
                    ESP_LOGE("GPS", "Frame Error");
                    break;
                default:
                    break;
            }
        }
    }
}

static void init_gps() {
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_GPS, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_GPS,
        PIN_GPS_SDI, PIN_GPS_SDO,
        UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_GPS, GPS_UART_RX_BUF_SIZE * 2, 0, 16, &gps_uart_q, 0));

    ESP_ERROR_CHECK(uart_enable_pattern_det_baud_intr(UART_GPS, '\n', 1, 9, 0, 0));
    ESP_ERROR_CHECK(uart_pattern_queue_reset(UART_GPS, 16));

    // TODO: Figure out how to configure the GPS to go at 10Hz
    // uart_write_bytes(UART_GPS, GPS_INIT_SEQUENCE, sizeof(GPS_INIT_SEQUENCE));
    ESP_ERROR_CHECK(uart_flush(UART_GPS));

    xTaskCreate(&gps_uart_task, "gps_uart_task", 1024 * 4, NULL, 10, NULL);
}

// TODO: Create mutex for fmgr
static void battmon_task(void* arg) {
    int battmon_delay_ticks = pdMS_TO_TICKS(1000.0f / BATT_MON_HZ);
    while (true) {
        if (MAX17048_Exists(&battery_monitor)) {
            float soc = MAX1708_SOC(&battery_monitor);
            float vbatt = MAX17048_Voltage(&battery_monitor) / 1000.0f;
            bool on_battery = (!battery_monitor.timed_out) && (vbatt > 3.2f);
            if (on_battery) {
                ESP_LOGI("BATT", "Battery SOC: %f", soc);
                Battery datum;
                datum.battery_voltage = vbatt;
                datum.percentage = soc;

                link_send_datum(DatumTypeID_INFO_Battery, Battery_fields, &datum);
            }
            vTaskDelay(battmon_delay_ticks);
        }
    }
}

static volatile float pressure_hPa;
static volatile float magnetic_mG[3];
static volatile float acceleration_g[3];
static volatile float angular_rate_dps[3];
static volatile float temperature_degC;


esp_timer_handle_t sensor_timer;
void sensors_routine(void* arg) {
    static uint32_t data_raw_pressure;
    static int16_t data_raw_magnetic[3];
    static int16_t data_raw_acceleration[3];
    static int16_t data_raw_angular_rate[3];
    static int16_t data_raw_temperature;

    ///////////////////////// LPS22 /////////////////////////
    lps22hh_reg_t lps_reg;
    lps22hh_read_reg(&lps22, LPS22HH_STATUS, (uint8_t*)&lps_reg, 1);

    if (lps_reg.status.p_da) {
        memset(&data_raw_pressure, 0x00, sizeof(uint32_t));
        lps22hh_pressure_raw_get(&lps22, &data_raw_pressure);
        pressure_hPa = lps22hh_from_lsb_to_hpa(data_raw_pressure);
    }

    ///////////////////////// LIS3MDL /////////////////////////
    uint8_t reg;
    /* Read output only if new value is available */
    lis3mdl_mag_data_ready_get(&lis3mdl, &reg);

    if (reg) {
        /* Read magnetic field data */
        memset(data_raw_magnetic, 0x00, 3 * sizeof(int16_t));
        lis3mdl_magnetic_raw_get(&lis3mdl, data_raw_magnetic);
        magnetic_mG[0] = 1000 * lis3mdl_from_fs16_to_gauss(
            data_raw_magnetic[0]);
        magnetic_mG[1] = 1000 * lis3mdl_from_fs16_to_gauss(
            data_raw_magnetic[1]);
        magnetic_mG[2] = 1000 * lis3mdl_from_fs16_to_gauss(
            data_raw_magnetic[2]);

        memset(&data_raw_temperature, 0x00, sizeof(int16_t));
        lis3mdl_temperature_raw_get(&lis3mdl, &data_raw_temperature);
        temperature_degC = lis3mdl_from_lsb_to_celsius(data_raw_temperature);
    }

    ///////////////////////// LSM6DSM /////////////////////////
    lsm6dsm_reg_t lsm_reg;
    /* Read output only if new value is available */
    lsm6dsm_status_reg_get(&lsm6dsm, &lsm_reg.status_reg);

    if (lsm_reg.status_reg.xlda) {
        /* Read acceleration field data */
        memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
        lsm6dsm_acceleration_raw_get(&lsm6dsm, data_raw_acceleration);
        acceleration_g[0] =
            lsm6dsm_from_fs2g_to_mg(data_raw_acceleration[0]) * 0.001f * 4.0f;
        acceleration_g[1] =
            lsm6dsm_from_fs2g_to_mg(data_raw_acceleration[1]) * 0.001f * 4.0f;
        acceleration_g[2] =
            lsm6dsm_from_fs2g_to_mg(data_raw_acceleration[2]) * 0.001f * 4.0f;
    }

    if (lsm_reg.status_reg.gda) {
        /* Read angular rate field data */
        memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
        lsm6dsm_angular_rate_raw_get(&lsm6dsm, data_raw_angular_rate);
        angular_rate_dps[0] =
            lsm6dsm_from_fs2000dps_to_mdps(data_raw_angular_rate[0]) * 0.001f;
        angular_rate_dps[1] =
            lsm6dsm_from_fs2000dps_to_mdps(data_raw_angular_rate[1]) * 0.001f;
        angular_rate_dps[2] =
            lsm6dsm_from_fs2000dps_to_mdps(data_raw_angular_rate[2]) * 0.001f;
    }
}

void debug_output_task(void* arg) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000.0f / DEBUG_MON_HZ));

#if DEBUG_MON_LPS22
        ESP_LOGI("DEBUG-LPS22", "hPa:  %f", pressure_hPa);
#endif
#if DEBUG_MON_LSM6DSM
        ESP_LOGI("DEBUG-LSM6DSM",
            "Acceleration [g]:  %4.2f  %4.2f  %4.2f",
            acceleration_g[0], acceleration_g[1], acceleration_g[2]);
        ESP_LOGI("DEBUG-LSM6DSM",
            "Angular rate [dps]:  %4.2f  %4.2f  %4.2f",
            angular_rate_dps[0], angular_rate_dps[1], angular_rate_dps[2]);
#endif
#if DEBUG_MON_LIS3MDL
        ESP_LOGI("DEBUG-LIS3MDL",
            "Magnetic Field [mG]:  %4.2f  %4.2f  %4.2f",
            magnetic_mG[0], magnetic_mG[1], magnetic_mG[2]);
        ESP_LOGI("DEBUG-LIS3MDL", "Temperature [C]:  %4.2f\n", temperature_degC);
#endif
    }
}

static void telemetry_tx_task(void* arg) {
    while (1) {
        // TODO: Check for GPS good (check fix level, DOPs, etc.)
        // TODO: Also send over a USB interface when possible (tinyusb?)

        BaseType_t res = xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
        if (res == pdTRUE) {
            // fmgr_encode_datum(&fmgr, DatumTypeID_INFO_GPS, GPS_fields, &current_gps);
            link_send_datum(DatumTypeID_INFO_GPS, GPS_fields, &current_gps);
            lora_link_flush();
        }
    }
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

static bool s_led_state = false;

void app_main(void) {
    init_nvs();
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
    fmgr_mutex = xSemaphoreCreateMutex();
    fmgr_init(&lora_fmgr, 256);

    init_battmon();

    ESP_LOGI("SYS", "Initializing LSM6DSM");
    init_lsm6dsm();

    ESP_LOGI("SYS", "Initializing LIS3MDL");
    init_lis3mdl();

    ESP_LOGI("SYS", "Initializing LPS22");
    init_lps22();

    // TODO: ADXL375

    init_gps();

    // Stack overflowing... when no battery...
    xTaskCreate(battmon_task, "battmon_task", 4 * 1024, NULL, 10, NULL);

#if DEBUG_MON_ENABLED
    xTaskCreate(debug_output_task, "debug_task", 4 * 1024, NULL, 10, NULL);
#endif


    xTaskCreate(telemetry_tx_task, "telemetry_tx_task", 4 * 1024, NULL, 10, &telemetry_task);

    const esp_timer_create_args_t sensor_timer_args = {
        .callback = &sensors_routine,
        .name = "sensors_routine"
    };

    ESP_ERROR_CHECK(esp_timer_create(&sensor_timer_args, &sensor_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(sensor_timer, 1000000 / SENSOR_HZ));

    while (1) {
        /* Toggle the LED state */
        s_led_state = !s_led_state;
        gpio_set_level(PIN_LED_G, s_led_state);
        gpio_set_level(PIN_LED_R, !s_led_state);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}