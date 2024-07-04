#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "pindefs.h"
#include "tracker_config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "memory.h"

#include "max17048.h"
#include "lps22hh_reg.h"
#include "lis3mdl_reg.h"
#include "lsm6dsm_reg.h"

////////////////// GLOBALS //////////////////
// Peripherals
i2c_master_bus_handle_t i2c_main_bus_handle;


// ICs
MAX17048_t battery_monitor;


// Sensors
stmdev_ctx_t lps22;
spi_device_handle_t lps22_device;

stmdev_ctx_t lis3mdl;
spi_device_handle_t lis3mdl_device;

stmdev_ctx_t lsm6dsm;
spi_device_handle_t lsm6dsm_device;

// TODO: ADXL driver
spi_device_handle_t adxl_device;


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

static void battmon_task(void* arg) {
    int battmon_delay_ticks = pdMS_TO_TICKS(1000.0f / BATT_MON_HZ);
    while (true) {
        float soc = MAX1708_SOC(&battery_monitor);
        bool on_battery = !battery_monitor.timed_out;
        if (on_battery) {
            ESP_LOGI("BATT", "Battery SOC: %f", soc);
        }
        vTaskDelay(battmon_delay_ticks);
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
        ESP_LOGI("DEBUG-LPS22", "hPa:  %f", pressure_hPa);
        ESP_LOGI("DEBUG-LSM6DSM",
            "Acceleration [g]:  %4.2f  %4.2f  %4.2f",
            acceleration_g[0], acceleration_g[1], acceleration_g[2]);
        ESP_LOGI("DEBUG-LSM6DSM",
            "Angular rate [dps]:  %4.2f  %4.2f  %4.2f",
            angular_rate_dps[0], angular_rate_dps[1], angular_rate_dps[2]);
        ESP_LOGI("DEBUG-LIS3MDL",
            "Magnetic Field [mG]:  %4.2f  %4.2f  %4.2f",
            magnetic_mG[0], magnetic_mG[1], magnetic_mG[2]);
        ESP_LOGI("DEBUG-LIS3MDL", "Temperature [C]:  %4.2f\n", temperature_degC);
    }
}

static bool s_led_state = false;

void app_main(void) {
    init_leds();

    init_i2c();
    init_sensor_spi();

    init_battmon();

    ESP_LOGI("SYS", "Initializing LSM6DSM");
    init_lsm6dsm();

    ESP_LOGI("SYS", "Initializing LIS3MDL");
    init_lis3mdl();

    ESP_LOGI("SYS", "Initializing LPS22");
    init_lps22();

    // Stack overflowing... when no battery...
    // xTaskCreate(battmon_task, "battmon_task", 2048, NULL, 10, NULL);

#if DEBUG_MON_ENABLED
    xTaskCreate(debug_output_task, "debug_task", 4 * 1024, NULL, 10, NULL);
#endif

    const esp_timer_create_args_t sensor_timer_args = {
        .callback = &sensors_routine,
        .name = "sensors_routine"
    };

    ESP_ERROR_CHECK(esp_timer_create(&sensor_timer_args, &sensor_timer));

    /* Start the timer */
    ESP_ERROR_CHECK(esp_timer_start_periodic(sensor_timer, 1000000 / SENSOR_HZ));

    while (1) {
        /* Toggle the LED state */
        s_led_state = !s_led_state;
        gpio_set_level(PIN_LED_G, s_led_state);
        gpio_set_level(PIN_LED_R, !s_led_state);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
