#include "sensors.h"
#include "configuration.h"
#include "Fusion.h"
#include "tracker_config.h"
#include "alt_filter.h"

// Sensors
stmdev_ctx_t lps22;
spi_device_handle_t lps22_device;

stmdev_ctx_t lis3mdl;
spi_device_handle_t lis3mdl_device;

stmdev_ctx_t lsm6dsm;
spi_device_handle_t lsm6dsm_device;

adxl375_handle_t adxl;
spi_device_handle_t adxl_device;
TaskHandle_t adxl_int_handler;

bool LIS3MDL_DISABLE = false;
bool LSM6DSM_DISABLE = false;
bool ADXL_DISABLE = false;
bool LPS22_DISABLE = false;

volatile float pressure_hPa;
volatile float magnetic_mG[3];
volatile float acceleration_g[3];
volatile float angular_rate_dps[3];
volatile float temperature_degC;
volatile float adxl_acceleration_g[3];

volatile float pressure_hPa_raw;
volatile float magnetic_mG_raw[3];
volatile float acceleration_g_raw[3];
volatile float angular_rate_dps_raw[3];
volatile float temperature_degC_raw;
volatile float adxl_acceleration_g_raw[3];
volatile float altitude_m_raw;


FusionAhrs ahrs;
volatile float altitude_m;
volatile float v_speed_m_s;
altimetry_filter_t alt_filter;

const FusionAxesAlignment SENSORS_ALIGNMENT = FusionAxesAlignmentNXNYPZ;

SemaphoreHandle_t sensors_mutex;

float apply_2pt_calibration(float value, calibration_2pt_t calibration) {
    return (value + calibration.offset) * calibration.scale;
}
void apply_2pt_calibrations_inplace(float* value, calibration_2pt_t* calibrations, int n) {
    for (int i = 0; i < n; ++i)
        value[i] = apply_2pt_calibration(value[i], calibrations[i]);
}

calibration_2pt_t adxl_acc_calib[3];
calibration_2pt_t lsm_acc_calib[3];
calibration_2pt_t lsm_gyr_calib[3];
calibration_2pt_t lis_mag_calib[3];
calibration_2pt_t lps_calib;
bool ahrs_no_magnetometer = false;
float acc_transition_low = CONFIG_SENSORS_ACC_TRANS_LO_DEFAULT;
float acc_transition_high = CONFIG_SENSORS_ACC_TRANS_LO_DEFAULT;

void init_sensors() {
    // ADXL375 Calibration
    config_get_float(CONFIG_CALIBRATION_ADXL375_X_OFFSET_KEY, &adxl_acc_calib[0].offset);
    config_get_float(CONFIG_CALIBRATION_ADXL375_X_SCALE_KEY, &adxl_acc_calib[0].scale);

    config_get_float(CONFIG_CALIBRATION_ADXL375_Y_OFFSET_KEY, &adxl_acc_calib[1].offset);
    config_get_float(CONFIG_CALIBRATION_ADXL375_Y_SCALE_KEY, &adxl_acc_calib[1].scale);

    config_get_float(CONFIG_CALIBRATION_ADXL375_Z_OFFSET_KEY, &adxl_acc_calib[2].offset);
    config_get_float(CONFIG_CALIBRATION_ADXL375_Z_SCALE_KEY, &adxl_acc_calib[2].scale);


    // LSM6DSM Acceleration Calibration
    config_get_float(CONFIG_CALIBRATION_LSM6DSM_ACCEL_X_OFFSET_KEY, &lsm_acc_calib[0].offset);
    config_get_float(CONFIG_CALIBRATION_LSM6DSM_ACCEL_X_SCALE_KEY, &lsm_acc_calib[0].scale);

    config_get_float(CONFIG_CALIBRATION_LSM6DSM_ACCEL_Y_OFFSET_KEY, &lsm_acc_calib[1].offset);
    config_get_float(CONFIG_CALIBRATION_LSM6DSM_ACCEL_Y_SCALE_KEY, &lsm_acc_calib[1].scale);

    config_get_float(CONFIG_CALIBRATION_LSM6DSM_ACCEL_Z_OFFSET_KEY, &lsm_acc_calib[2].offset);
    config_get_float(CONFIG_CALIBRATION_LSM6DSM_ACCEL_Z_SCALE_KEY, &lsm_acc_calib[2].scale);

    // LSM6DSM Gyro Calibration
    config_get_float(CONFIG_CALIBRATION_LSM6DSM_GYRO_X_OFFSET_KEY, &lsm_gyr_calib[0].offset);
    lsm_gyr_calib[0].scale = 1.0f;
    // config_get_float(&lsm_gyr_calib[0].scale, CONFIG_CALIBRATION_LSM6DSM_GYRO_X_SCALE_KEY);

    config_get_float(CONFIG_CALIBRATION_LSM6DSM_GYRO_Y_OFFSET_KEY, &lsm_gyr_calib[1].offset);
    lsm_gyr_calib[1].scale = 1.0f;
    // config_get_float(&lsm_gyr_calib[1].scale, CONFIG_CALIBRATION_LSM6DSM_GYRO_Y_SCALE_KEY);

    config_get_float(CONFIG_CALIBRATION_LSM6DSM_GYRO_Z_OFFSET_KEY, &lsm_gyr_calib[2].offset);
    lsm_gyr_calib[2].scale = 1.0f;
    // config_get_float(&lsm_gyr_calib[2].scale, CONFIG_CALIBRATION_LSM6DSM_GYRO_Z_SCALE_KEY);

    config_get_bool(CONFIG_SENSORS_AHRS_NO_MAG_KEY, &ahrs_no_magnetometer);

    // LIS3MDL Calibration
    config_get_float(CONFIG_CALIBRATION_LIS3MDL_X_OFFSET_KEY, &lis_mag_calib[0].offset);
    config_get_float(CONFIG_CALIBRATION_LIS3MDL_X_SCALE_KEY, &lis_mag_calib[0].scale);

    config_get_float(CONFIG_CALIBRATION_LIS3MDL_Y_OFFSET_KEY, &lis_mag_calib[1].offset);
    config_get_float(CONFIG_CALIBRATION_LIS3MDL_Y_SCALE_KEY, &lis_mag_calib[1].scale);

    config_get_float(CONFIG_CALIBRATION_LIS3MDL_Z_OFFSET_KEY, &lis_mag_calib[2].offset);
    config_get_float(CONFIG_CALIBRATION_LIS3MDL_Z_SCALE_KEY, &lis_mag_calib[2].scale);

    // LPS22 Calibration
    config_get_float(CONFIG_CALIBRATION_LPS22_OFFSET_KEY, &lps_calib.offset);
    lps_calib.scale = 1.0f;

    config_get_float(CONFIG_SENSORS_ACC_TRANS_LO_KEY, &acc_transition_low);
    config_get_float(CONFIG_SENSORS_ACC_TRANS_HI_KEY, &acc_transition_high);

    FusionAhrsInitialise(&ahrs);
    // FusionAhrsSetSettings(&ahrs, &settings);
    FusionAhrsReset(&ahrs);


    float vertical_acceleration_stdev, altimetry_stdev;
    config_get_float(CONFIG_SENSORS_ALTITUDE_STDEV_KEY, &altimetry_stdev);
    config_get_float(CONFIG_SENSORS_VERT_ACCEL_STDEV_KEY, &vertical_acceleration_stdev);

    altimetry_filter_init(&alt_filter, 1.0f / SENSOR_HZ, vertical_acceleration_stdev, altimetry_stdev);
}

void init_sensor_spi() {
    ESP_LOGI("SYS", "Initializing sensor SPI...");
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_MISO,
        .mosi_io_num = PIN_MOSI,
        .sclk_io_num = PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };

    gpio_reset_pin(PIN_SCK);
    gpio_reset_pin(PIN_MOSI);
    gpio_reset_pin(PIN_MISO);

    // gpio_set_direction(PIN_MISO, GPIO_MODE_INPUT);
    // gpio_pullup_en(PIN_MISO);

    //Initialize the SPI bus
    esp_err_t e = spi_bus_initialize(SENSORS_SPI, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(e);

    gpio_reset_pin(PIN_ADXL_CS);
    gpio_reset_pin(PIN_LPS_CS);
    gpio_reset_pin(PIN_LSM6DSM_CS);
    gpio_reset_pin(PIN_LIS3MDL_CS);

    gpio_set_direction(PIN_ADXL_CS, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_LPS_CS, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_LSM6DSM_CS, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_LIS3MDL_CS, GPIO_MODE_OUTPUT);

    gpio_set_level(PIN_ADXL_CS, 1);
    gpio_set_level(PIN_LPS_CS, 1);
    gpio_set_level(PIN_LSM6DSM_CS, 1);
    gpio_set_level(PIN_LIS3MDL_CS, 1);
}

int32_t sensor_platform_write(void* handle, uint8_t Reg, const uint8_t* Bufp, uint16_t len) {
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

int32_t sensor_platform_read(void* handle, uint8_t Reg, uint8_t* Bufp, uint16_t len) {
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

void platform_delay(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}


// TODO: make this do nothing when not autolog
// Handles LOGSTATE_LOGGING_AUTO_ARMED -> LOGSTATE_LOGGING_AUTO_LIFTOFF transition for autolog
IRAM_ATTR void adxl_isr(void* arg) {
    // xTaskNotifyFromISR(adxl_int_handler, 0, eNoAction, NULL);
    xTaskNotifyFromISR(*((TaskHandle_t*)arg), 0, eNoAction, NULL);
}

void init_adxl375(TaskHandle_t* interrupt_task) {
    spi_device_interface_config_t devcfg = {
        // TODO: Can this be increased on a per-device basis?
        .clock_speed_hz = 5e6,
        .mode = 3, // CPOL=1, CPHA=1
        .spics_io_num = PIN_ADXL_CS,
        .queue_size = 7,
        .address_bits = 8,
        .command_bits = 0,
        .dummy_bits = 0,
    };

    esp_err_t e = spi_bus_add_device(SENSORS_SPI, &devcfg, &adxl_device);
    // ESP_ERROR_CHECK(e);

    platform_delay(10);

    e = ESP_ERR_NOT_FOUND;

    e = adxl375_init(&adxl, &adxl_device, PIN_ADXL_CS);
    if (e != ESP_OK) {
        ESP_LOGE("ADXL", "Error initializing sensor");
        ADXL_DISABLE = true;
        return;
    }

    adxl375_set_interrupts_pin2(&adxl, 0); // All interrupts on pin 1
    adxl375_enable_interrupts(&adxl, ADXL_INT_ACT | ADXL_INT_INACT);
    adxl375_set_act_mode(&adxl, false, false, false, false);
    adxl375_set_inact_mode(&adxl, false, false, false, false);


    gpio_reset_pin(PIN_ADXL_INT1);
    gpio_set_direction(PIN_ADXL_INT1, GPIO_MODE_INPUT);

    gpio_set_intr_type(PIN_ADXL_INT1, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add(PIN_ADXL_INT1, adxl_isr, (void*)interrupt_task);

    adxl375_set_autosleep(&adxl, false);

    adxl375_set_bw_rate(&adxl, ADXL_RATE_200, false);
    adxl375_set_mode(&adxl, ADXL_POWER_MEASURE);
}

void init_lps22() {
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 8e6, // "Reccomended" SPI frequency
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
        LPS22_DISABLE = true;
        return;
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
    lps22hh_lp_bandwidth_set(&lps22, LPS22HH_LPF_ODR_DIV_2);
    // lps22hh_data_rate_set(&lps22, LPS22HH_75_Hz_LOW_NOISE);

    // lps22hh_pressure_offset_set(&lps22, (int16_t)(-63.0527133333 * 1048576.0f)); // error*1048576.0f
}

void init_lis3mdl() {
    spi_device_interface_config_t devcfg = {
        // TODO: Can this be increased on a per-device basis?
        .clock_speed_hz = 5e6,
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
        LIS3MDL_DISABLE = true;
        return;
    }

    /* Restore default configuration */
    lis3mdl_reset_set(&lis3mdl, PROPERTY_ENABLE);

    uint8_t rst;
    do {
        lis3mdl_reset_get(&lis3mdl, &rst);
    } while (rst);

    /* Enable Block Data Update */
    lis3mdl_block_data_update_set(&lis3mdl, PROPERTY_ENABLE);
    // lis3mdl_spi_mode_set(&lis3mdl, LIS3MDL_SPI_4_WIRE)

    /* Set Output Data Rate */
    lis3mdl_data_rate_set(&lis3mdl, LIS3MDL_HP_300Hz); // lower noise from HP?
    /* Set full scale */
    lis3mdl_full_scale_set(&lis3mdl, LIS3MDL_4_GAUSS);
    /* Enable temperature sensor */
    lis3mdl_temperature_meas_set(&lis3mdl, PROPERTY_DISABLE);
    /* Set device in continuous mode */
    lis3mdl_operating_mode_set(&lis3mdl, LIS3MDL_CONTINUOUS_MODE);
}

void init_lsm6dsm() {
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10e6,
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
        LSM6DSM_DISABLE = true;
        return;
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
    lsm6dsm_xl_full_scale_set(&lsm6dsm, LSM6DSM_16g);
    lsm6dsm_gy_full_scale_set(&lsm6dsm, LSM6DSM_2000dps);

    /* Configure filtering chain(No aux interface)
     * Accelerometer - analog filter
     */
     // lsm6dsm_xl_filter_analog_set(&lsm6dsm, LSM6DSM_XL_ANA_BW_1k5Hz);
     /* Accelerometer - LPF1 path (LPF2 not used) */
    // lsm6dsm_xl_lp1_bandwidth_set(&lsm6dsm, LSM6DSM_XL_LP1_ODR_DIV_2);
    /* Accelerometer - LPF1 + LPF2 path */

   // NOTE: No LPF!!!
   // lsm6dsm_xl_lp2_bandwidth_set(&lsm6dsm,
   //     LSM6DSM_XL_LOW_NOISE_LP_ODR_DIV_9);

   /* Accelerometer - High Pass / Slope path */
   //lsm6dsm_xl_reference_mode_set(&dev_ctx, PROPERTY_DISABLE);
   //lsm6dsm_xl_hp_bandwidth_set(&dev_ctx, LSM6DSM_XL_HP_ODR_DIV_100);
   /* Gyroscope - filtering chain */
    // lsm6dsm_gy_band_pass_set(&lsm6dsm, LSM6DSM_HP_260mHz_LP1_STRONG);
    // No gyro bandpass
}

float clamp(float value, float min, float max) {
    return (value > max) ? max : ((value < min) ? min : value);
}

FusionVector get_interpolated_acceleration() {
    FusionVector lsm_accel, adxl_accel, interpolated_accel;
    memcpy(lsm_accel.array, (float*)acceleration_g, sizeof(lsm_accel.array));
    memcpy(adxl_accel.array, (float*)adxl_acceleration_g, sizeof(adxl_accel.array));

    if (ADXL_DISABLE) {
        return lsm_accel;
    } else if (LSM6DSM_DISABLE) {
        return adxl_accel;
    }

    // if (FusionVectorMagnitudeSquared(adxl_accel))
    for (int axis = 0; axis < 3; axis++) {
        float abs_avg = fabsf((lsm_accel.array[axis] + adxl_accel.array[axis]) * 0.5f);

        float factor = clamp((abs_avg - acc_transition_low) / (acc_transition_high - acc_transition_low), 0.0f, 1.0f);
        interpolated_accel.array[axis] = lsm_accel.array[axis] * (1.0f - factor) + factor * adxl_accel.array[axis];
    }

    return interpolated_accel;
}

static float pressure_hPa_to_alt_m(float hPa) {
    return 44330.f * (1 - powf(hPa / 1013.25f, 0.190284f));
}

void sensors_routine(void* arg) {
    static uint32_t data_raw_pressure;
    static int16_t data_raw_magnetic[3];
    static int16_t data_raw_acceleration[3];
    static int16_t data_raw_angular_rate[3];
    static int16_t data_raw_acceleration_adxl[3];
    // static int16_t data_raw_temperature;


    if (xSemaphoreTake(sensors_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {

        ///////////////////////// LPS22 /////////////////////////
        if (!LPS22_DISABLE) {
            lps22hh_reg_t lps_reg;
            lps22hh_read_reg(&lps22, LPS22HH_STATUS, (uint8_t*)&lps_reg, 1);

            if (lps_reg.status.p_da) {
                memset(&data_raw_pressure, 0x00, sizeof(uint32_t));
                lps22hh_pressure_raw_get(&lps22, &data_raw_pressure);
                pressure_hPa_raw = lps22hh_from_lsb_to_hpa(data_raw_pressure);

                // CALIBRATE!
                pressure_hPa = apply_2pt_calibration(pressure_hPa_raw, lps_calib);

                log_data.lps_press_raw = data_raw_pressure;
                log_data.flags |= LOG_FLAG_PRESS_FRESH;

                altitude_m_raw = pressure_hPa_to_alt_m(pressure_hPa);
                altimetry_filter_correct(&alt_filter, altitude_m_raw);
            }
        }

        ///////////////////////// LIS3MDL /////////////////////////
        if (!LIS3MDL_DISABLE) {
            uint8_t reg;
            /* Read output only if new value is available */
            lis3mdl_mag_data_ready_get(&lis3mdl, &reg);

            if (reg) {
                /* Read magnetic field data */
                memset(data_raw_magnetic, 0x00, 3 * sizeof(int16_t));
                lis3mdl_magnetic_raw_get(&lis3mdl, data_raw_magnetic);
                // +X global is sensor Y
                magnetic_mG_raw[0] = 1000 * lis3mdl_from_fs4_to_gauss(
                    data_raw_magnetic[0]);
                // +Y global is -sensor X
                magnetic_mG_raw[1] = 1000 * lis3mdl_from_fs4_to_gauss(
                    data_raw_magnetic[1]);
                // +Z global is sensor Z
                magnetic_mG_raw[2] = 1000 * lis3mdl_from_fs4_to_gauss(
                    data_raw_magnetic[2]);


                // CALIBRATE!
                memcpy((float*)magnetic_mG, (float*)magnetic_mG_raw, sizeof(magnetic_mG));
                apply_2pt_calibrations_inplace((float*)magnetic_mG, lis_mag_calib, 3);

                // memset(&data_raw_temperature, 0x00, sizeof(int16_t));
                // lis3mdl_temperature_raw_get(&lis3mdl, &data_raw_temperature);
                // temperature_degC = lis3mdl_from_lsb_to_celsius(data_raw_temperature);

                // log_data.lis_mag_raw
                memcpy(log_data.lis_mag_raw, data_raw_magnetic, sizeof(data_raw_magnetic));
                log_data.flags |= LOG_FLAG_MAG_FRESH;
            }
            lis3mdl_operating_mode_set(&lis3mdl, LIS3MDL_CONTINUOUS_MODE); // THIS FIXES IT!!!
        }

        ///////////////////////// LSM6DSM /////////////////////////
        if (!LSM6DSM_DISABLE) {
            lsm6dsm_reg_t lsm_reg;
            /* Read output only if new value is available */
            lsm6dsm_status_reg_get(&lsm6dsm, &lsm_reg.status_reg);

            if (lsm_reg.status_reg.xlda) {
                /* Read acceleration field data */
                memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
                lsm6dsm_acceleration_raw_get(&lsm6dsm, data_raw_acceleration);
                // X global is sensor Y
                acceleration_g_raw[0] =
                    lsm6dsm_from_fs16g_to_mg(data_raw_acceleration[0]) * 0.001f;
                // Y global is -sensor X
                acceleration_g_raw[1] =
                    lsm6dsm_from_fs16g_to_mg(data_raw_acceleration[1]) * 0.001f;
                // Z global is sensor Z
                acceleration_g_raw[2] =
                    lsm6dsm_from_fs16g_to_mg(data_raw_acceleration[2]) * 0.001f;

                // CALIBRATE!
                memcpy((float*)acceleration_g, (float*)acceleration_g_raw, sizeof(acceleration_g));
                apply_2pt_calibrations_inplace((float*)acceleration_g, lsm_acc_calib, 3);


                memcpy(log_data.lsm_acc_raw, data_raw_acceleration, sizeof(data_raw_acceleration));
                log_data.flags |= LOG_FLAG_LSM_ACC_FRESH;
            }

            if (lsm_reg.status_reg.gda) {
                /* Read angular rate field data */
                memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
                lsm6dsm_angular_rate_raw_get(&lsm6dsm, data_raw_angular_rate);
                // TODO: Confirm that this inversion is neccesary with the axes inversion of X and Y
                angular_rate_dps_raw[0] =
                    lsm6dsm_from_fs2000dps_to_mdps(data_raw_angular_rate[0]) * 0.001f;
                angular_rate_dps_raw[1] =
                    lsm6dsm_from_fs2000dps_to_mdps(data_raw_angular_rate[1]) * 0.001f;
                angular_rate_dps_raw[2] =
                    lsm6dsm_from_fs2000dps_to_mdps(data_raw_angular_rate[2]) * 0.001f;

                // CALIBRATE!
                memcpy((float*)angular_rate_dps, (float*)angular_rate_dps_raw, sizeof(angular_rate_dps));
                apply_2pt_calibrations_inplace((float*)angular_rate_dps, lsm_gyr_calib, 3);

                memcpy(log_data.lsm_gyr_raw, data_raw_angular_rate, sizeof(data_raw_angular_rate));
                log_data.flags |= LOG_FLAG_LSM_GYR_FRESH;
            }
        }

        if (!ADXL_DISABLE) {
            uint8_t adxl_intsrcs;
            adxl375_get_int_source(&adxl, &adxl_intsrcs);
            if (adxl_intsrcs & ADXL_INT_DRDY) {
                /* Read angular rate field data */
                memset(data_raw_acceleration_adxl, 0x00, 3 * sizeof(int16_t));
                adxl375_get_acceleration_raw(&adxl, data_raw_acceleration_adxl);

                // Global X is sensor Y
                adxl_acceleration_g_raw[0] = (data_raw_acceleration_adxl[0] * 49) * 0.001f;
                // Global Y is -sensor X
                adxl_acceleration_g_raw[1] = (data_raw_acceleration_adxl[1] * 49) * 0.001f;
                // Z is Z
                adxl_acceleration_g_raw[2] = (data_raw_acceleration_adxl[2] * 49) * 0.001f;

                // CALIBRATE!
                memcpy((float*)adxl_acceleration_g, (float*)adxl_acceleration_g_raw, sizeof(adxl_acceleration_g));
                apply_2pt_calibrations_inplace((float*)adxl_acceleration_g, adxl_acc_calib, 3);

                memcpy(log_data.adxl_acc_raw, data_raw_acceleration_adxl, sizeof(data_raw_acceleration_adxl));
                log_data.flags |= LOG_FLAG_ADXL_ACC_FRESH;
            }
            xSemaphoreGive(sensors_mutex);
        }

    }

    FusionVector gyro, accel, mag;
    memcpy(gyro.array, (float*)angular_rate_dps, sizeof(gyro.array));
    memcpy(accel.array, (float*)acceleration_g, sizeof(accel.array));
    // accel = get_interpolated_acceleration(); // TODO: Rigorously test!!!
    memcpy(mag.array, (float*)magnetic_mG, sizeof(mag.array));

    /*
        Axes alignment describing the sensor axes relative to the body axes.
        For example, if the body X axis is aligned with the sensor Y axis and the body Y axis is aligned with sensor X axis
        but pointing the opposite direction then alignment is +Y-X+Z.
    */
    gyro = FusionAxesSwap(gyro, SENSORS_ALIGNMENT);
    accel = FusionAxesSwap(accel, SENSORS_ALIGNMENT);
    mag = FusionAxesSwap(mag, SENSORS_ALIGNMENT);

    if (ahrs_no_magnetometer || LIS3MDL_DISABLE) {
        FusionAhrsUpdateNoMagnetometer(&ahrs, gyro, accel, 1.0f / SENSOR_HZ);
    } else {
        FusionAhrsUpdate(&ahrs, gyro, accel, mag, 1.0f / SENSOR_HZ);
    }

    // FusionAhrsUpdate(&ahrs, gyro, accel, mag, 1.0f / SENSOR_HZ);

    FusionQuaternion orientation = FusionAhrsGetQuaternion(&ahrs);
    memcpy(log_data.orientation_quat, orientation.array, sizeof(orientation.array));

    altimetry_filter_update(&alt_filter, &ahrs);


    altitude_m = altimetry_filter_get_filtered_altitude(&alt_filter);
    v_speed_m_s = altimetry_filter_get_filtered_vspeed(&alt_filter);
    log_data.filtered_altitude_m = altitude_m;
}