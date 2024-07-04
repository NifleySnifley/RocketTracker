#include "pindefs.h"
#include "Fusion.h"
#include <stdbool.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
// #include "bmp390/bmp3.h"
// #include "bmp390/bmp3_interface.h"
#include "lis3mdl/lis3mdl_reg.h"
#include "lsm6dsox/lsm6dsox_reg.h"
#include "lps22hh/lps22hh_reg.h"
#include "max17048/max17048.h"
#include "memory.h"
#include "esp_timer.h"
#include "vt_linalg"
#include "vt_kalman"

#define I2C_MAIN_PORT I2C_NUM_0

#define SENSOR_HZ 256
#define SENSOR_PERIOD (1.0f/SENSOR_HZ) // replace this with actual sample period

#define CONV_mg_TO_m_s (0.001f * 9.81f)

i2c_master_bus_handle_t i2c_main_bus_handle;

i2c_master_dev_handle_t bmp390_dev_handle;
i2c_master_dev_handle_t lis3mdl_dev_handle;
i2c_master_dev_handle_t lsm6dsox_dev_handle;
i2c_master_dev_handle_t lps22_dev_handle;
i2c_master_dev_handle_t max17048_dev_handle;

MAX17048_t max17048;
static bool onBattery = false;

static int32_t i2c_platform_write(void* handle, uint8_t Reg, const uint8_t* Bufp, uint16_t len) {

	uint8_t* data = (uint8_t*)malloc(len + 1);
	data[0] = Reg;
	for (int i = 0; i < len; i++) {
		data[i + 1] = Bufp[i];
	}
	esp_err_t e = i2c_master_transmit(*((i2c_master_dev_handle_t*)handle), data, len + 1, 50);
	free(data);
	return e != ESP_OK;
}
static int32_t i2c_platform_read(void* handle, uint8_t Reg, uint8_t* Bufp, uint16_t len) {
	if (i2c_master_transmit_receive(*((i2c_master_dev_handle_t*)handle), &Reg, 1, Bufp, len, 50) == ESP_OK) {
		return 0;
	}
	return 2;
}

static void platform_delay(uint32_t ms) {
	vTaskDelay(pdMS_TO_TICKS(ms));
}

// bmp3_dev bmp;
stmdev_ctx_t lis3mdl;
stmdev_ctx_t lsm6dsox;
stmdev_ctx_t lps22hh;

static volatile float magnetic_mG[3];
static volatile float acceleration_g[3];
static volatile float angular_rate_dps[3];
// static struct bmp3_data data = { 0 };
FusionAhrs ahrs;
FusionVector earth_accel;

using namespace vt;
// state = [[pos],[vel]], integrate vel for pos
numeric_matrix<2, 2> F({ {1.0,0.00390625f},
						{0, 1.0} });

numeric_matrix<2, 1> B({ {7.62939453e-06f},{3.90625000e-03f} });
numeric_matrix<1, 2> H({ {1.0} });
numeric_matrix<2, 2> Q = numeric_matrix<2, 2>::diagonals(1.3940354099852605e-05);
numeric_matrix<1, 1> R = numeric_matrix<1, 1>::diagonals(0.06322944764207783);
numeric_vector<2> x0; // {x, v, a}

kalman_filter_t<2, 1, 1> kf(F, B, H, Q, R, x0);

// float get_altitude() {
// 	return 44330.f * (1 - powf(data.pressure / 101325.f, 0.190284f));
// }

void sensor_read(void* arg) {
	int8_t rslt;
	struct bmp3_status status = { { 0 } };
	int16_t data_raw_magnetic[3];
	int16_t data_raw_acceleration[3];
	int16_t data_raw_angular_rate[3];

	static bool magFresh = false;

	//////////////////////////// BMP390 ////////////////////////////
	rslt = bmp3_get_status(&status, &bmp);
	bmp3_check_rslt("bmp3_get_status", rslt);

	/* Read temperature and pressure data iteratively based on data ready interrupt */
	if ((rslt == BMP3_OK) && (status.intr.drdy == BMP3_ENABLE)) {
		/*
		 * First parameter indicates the type of data to be read
		 * BMP3_PRESS_TEMP : To read pressure and temperature data
		 * BMP3_TEMP       : To read only temperature data
		 * BMP3_PRESS      : To read only pressure data
		 */
		rslt = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data, &bmp);
		bmp3_check_rslt("bmp3_get_sensor_data", rslt);

		// 	/* NOTE : Read status register again to clear data ready interrupt status */
		rslt = bmp3_get_status(&status, &bmp);
		bmp3_check_rslt("bmp3_get_status", rslt);
	}

	//////////////////////////// LIS3MDL ////////////////////////////
	uint8_t reg;
	/* Read output only if new value is available */
	lis3mdl_mag_data_ready_get(&lis3mdl, &reg);

	if (reg) {
		/* Read magnetic field data */
		memset(data_raw_magnetic, 0x00, 3 * sizeof(int16_t));
		lis3mdl_magnetic_raw_get(&lis3mdl, data_raw_magnetic);
		magnetic_mG[0] = 1000 * lis3mdl_from_fs4_to_gauss(
			data_raw_magnetic[0]);
		magnetic_mG[1] = 1000 * lis3mdl_from_fs4_to_gauss(
			data_raw_magnetic[1]);
		magnetic_mG[2] = 1000 * lis3mdl_from_fs4_to_gauss(
			data_raw_magnetic[2]);

		bool magFresh = false;
	}

	//////////////////////////// LSM6DSOX ////////////////////////////

	/* Read output only if new xl value is available */
	lsm6dsox_xl_flag_data_ready_get(&lsm6dsox, &reg);

	if (reg) {
		/* Read acceleration field data */
		memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
		lsm6dsox_acceleration_raw_get(&lsm6dsox, data_raw_acceleration);
		acceleration_g[0] =
			lsm6dsox_from_fs2_to_mg(data_raw_acceleration[0]) * 0.001f;
		acceleration_g[1] =
			lsm6dsox_from_fs2_to_mg(data_raw_acceleration[1]) * 0.001f;
		acceleration_g[2] =
			lsm6dsox_from_fs2_to_mg(data_raw_acceleration[2]) * 0.001f;
	}

	lsm6dsox_gy_flag_data_ready_get(&lsm6dsox, &reg);

	if (reg) {
		/* Read angular rate field data */
		memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
		lsm6dsox_angular_rate_raw_get(&lsm6dsox, data_raw_angular_rate);
		angular_rate_dps[0] =
			lsm6dsox_from_fs2000_to_mdps(data_raw_angular_rate[0]) * 0.001f;
		angular_rate_dps[1] =
			lsm6dsox_from_fs2000_to_mdps(data_raw_angular_rate[1]) * 0.001f;
		angular_rate_dps[2] =
			lsm6dsox_from_fs2000_to_mdps(data_raw_angular_rate[2]) * 0.001f;
	}

	const FusionVector gyroscope = { angular_rate_dps[0], angular_rate_dps[1], angular_rate_dps[2] }; // replace this with actual gyroscope data in degrees/s
	const FusionVector accelerometer = { acceleration_g[0],acceleration_g[1],acceleration_g[2] }; // replace this with actual accelerometer data in g
	FusionVector magnetometer = { magnetic_mG[0],magnetic_mG[1],magnetic_mG[2] }; // replace this with actual magnetometer data in arbitrary units

	if (magFresh) {
		FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, SENSOR_PERIOD);
	} else {
		FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SENSOR_PERIOD);
	}

	// TODO: For rocketry this will prolly need to account for gravity...?
	earth_accel = FusionVectorMultiplyScalar(
		FusionAhrsGetEarthAcceleration(&ahrs),
		9.81);

	numeric_vector<1> z({ {get_altitude()} });
	numeric_vector<1> u({ {earth_accel.axis.z} });
	kf.update(z);
	kf.predict(u);
}

void dataout_task(void* pvArgs) {
	while (true) {
		vTaskDelay(pdMS_TO_TICKS(100));
		// ESP_LOGI("LSM6DSOX",
		// 	"Acceleration [g]:%4.2f\t%4.2f\t%4.2f",
			// acceleration_m_s[0], acceleration_m_s[1], acceleration_m_s[2]);
		// const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

		// ESP_LOGI("AHRS", "Roll %0.1f, Pitch %0.1f, Yaw %0.1f", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
		// ESP_LOGI("IMU", "Vertical accel (m/s2): %f", earth_accel.axis.z);
		// ESP_LOGI("KF", "Filtered altitude (m): %f", kf.state_vector[0]);
		// printf("%f, %f, %f\n", get_altitude(), earth_accel.axis.z, kf.state_vector[0]);


		ESP_LOGI("BATT", "SOC = %f", MAX1708_SOC(&max17048) * 100.f);
	}
}

// void init_bmp390() {
// 	int8_t rslt = bmp3_interface_init(&bmp, BMP3_I2C_INTF);
// 	bmp3_check_rslt("bmp3_interface_init", rslt);

// 	rslt = bmp3_init(&bmp);
// 	bmp3_check_rslt("bmp3_init", rslt);

// 	struct bmp3_settings settings = { 0 };
// 	settings.int_settings.drdy_en = BMP3_ENABLE;
// 	settings.int_settings.latch = BMP3_ENABLE;
// 	settings.press_en = BMP3_ENABLE;
// 	settings.temp_en = BMP3_ENABLE;

// 	settings.odr_filter.press_os = BMP3_OVERSAMPLING_2X;
// 	settings.odr_filter.temp_os = BMP3_OVERSAMPLING_2X;
// 	settings.odr_filter.odr = BMP3_ODR_100_HZ;
// 	settings.op_mode = BMP3_MODE_NORMAL;


// 	uint16_t settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS | BMP3_SEL_ODR |
// 		BMP3_SEL_DRDY_EN | BMP3_SEL_LATCH;

// 	rslt = bmp3_set_sensor_settings(settings_sel, &settings, &bmp);
// 	bmp3_check_rslt("bmp3_set_sensor_settings", rslt);

// 	rslt = bmp3_set_op_mode(&settings, &bmp);
// 	bmp3_check_rslt("bmp3_set_op_mode", rslt);
// }

void init_lis3mdl() {
	i2c_device_config_t lis3mdl_dev_config = {
			.dev_addr_length = I2C_ADDR_BIT_LEN_7,
			.device_address = LIS3MDL_I2C_ADD_L >> 1,
			.scl_speed_hz = 400000,
	};


	ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_main_bus_handle, &lis3mdl_dev_config, &lis3mdl_dev_handle));
	// TODO: FIX for SPI!!
	// lis3mdl.write_reg = platform_write;
	// lis3mdl.read_reg = platform_read;
	lis3mdl.mdelay = platform_delay;
	lis3mdl.handle = &lis3mdl_dev_handle;

	static uint8_t whoamI, rst;

	lis3mdl_device_id_get(&lis3mdl, &whoamI);

	if (whoamI != LIS3MDL_ID)
		ESP_LOGE("LIS3MDL", "Error: LIS3MDL not found");

	/* Restore default configuration */
	lis3mdl_reset_set(&lis3mdl, PROPERTY_ENABLE);

	do {
		lis3mdl_reset_get(&lis3mdl, &rst);
	} while (rst);

	/* Enable Block Data Update */
	lis3mdl_block_data_update_set(&lis3mdl, PROPERTY_ENABLE);
	/* Set Output Data Rate */
	lis3mdl_data_rate_set(&lis3mdl, LIS3MDL_MP_560Hz);
	/* Set full scale */
	lis3mdl_full_scale_set(&lis3mdl, LIS3MDL_4_GAUSS);
	/* Enable temperature sensor */
	lis3mdl_temperature_meas_set(&lis3mdl, PROPERTY_ENABLE);
	/* Set device in continuous mode */
	lis3mdl_operating_mode_set(&lis3mdl, LIS3MDL_CONTINUOUS_MODE);
}

void init_lsm6dsox() {
	i2c_device_config_t lsm6dsox_dev_config = {
			.dev_addr_length = I2C_ADDR_BIT_LEN_7,
			.device_address = LSM6DSOX_I2C_ADD_L >> 1,
			.scl_speed_hz = 400000,
	};

	ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_main_bus_handle, &lsm6dsox_dev_config, &lsm6dsox_dev_handle));

	// TODO: FIX for SPI!!
	// lsm6dsox.write_reg = platform_write;
	// lsm6dsox.read_reg = platform_read;
	lsm6dsox.mdelay = platform_delay;
	lsm6dsox.handle = &lsm6dsox_dev_handle;

	static uint8_t whoamI, rst;

	platform_delay(20);
	/* Check device ID */
	lsm6dsox_device_id_get(&lsm6dsox, &whoamI);

	if (whoamI != LSM6DSOX_ID) {
		ESP_LOGE("LSM6DSOX", "Error: LSM6DSOX not found");
		return;
	}

	/* Restore default configuration */
	lsm6dsox_reset_set(&lsm6dsox, PROPERTY_ENABLE);

	do {
		lsm6dsox_reset_get(&lsm6dsox, &rst);
	} while (rst);

	/* Disable I3C interface */
	lsm6dsox_i3c_disable_set(&lsm6dsox, LSM6DSOX_I3C_DISABLE);
	/* Enable Block Data Update */
	lsm6dsox_block_data_update_set(&lsm6dsox, PROPERTY_ENABLE);
	/* Set Output Data Rate */
	lsm6dsox_xl_data_rate_set(&lsm6dsox, LSM6DSOX_XL_ODR_833Hz);
	lsm6dsox_gy_data_rate_set(&lsm6dsox, LSM6DSOX_GY_ODR_833Hz);
	/* Set full scale */
	lsm6dsox_xl_full_scale_set(&lsm6dsox, LSM6DSOX_2g);
	lsm6dsox_gy_full_scale_set(&lsm6dsox, LSM6DSOX_2000dps);
	/* Configure filtering chain(No aux interface)
	 * Accelerometer - LPF1 + LPF2 path
	 */
	lsm6dsox_xl_hp_path_on_out_set(&lsm6dsox, LSM6DSOX_LP_ODR_DIV_100);
	lsm6dsox_xl_filter_lp2_set(&lsm6dsox, PROPERTY_ENABLE);
}

static bool max17048_timeout = false;

void max17048_write_i2c(uint8_t Address, void* data, uint8_t amount) {
	uint8_t* DatatoSend = (uint8_t*)data;

	esp_err_t e = i2c_master_transmit(max17048_dev_handle, DatatoSend, amount, 50);
	// return e != ESP_OK;
	if (e != ESP_OK) {
		max17048_timeout = true;
	}
}

void max17048_read_i2c(uint8_t Address, void* Register, uint8_t amount, uint8_t Sizereg) {
	uint8_t* DatatoSend = (uint8_t*)Register;
	uint8_t recvbuffer[2];
	esp_err_t e = i2c_master_transmit_receive(max17048_dev_handle, DatatoSend, 1, recvbuffer, amount, 50);

	DatatoSend[0] = recvbuffer[0];
	DatatoSend[1] = recvbuffer[1];
	if (e != ESP_OK) {
		max17048_timeout = true;
	}
}

bool init_max17048() {
	i2c_device_config_t max17048_dev_config = {
				.dev_addr_length = I2C_ADDR_BIT_LEN_7,
				.device_address = MAX17048_ADDR_SLAVE >> 1,
				.scl_speed_hz = 400000,
	};

	ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_main_bus_handle, &max17048_dev_config, &max17048_dev_handle));

	MAX17048_Init(&max17048, max17048_write_i2c, max17048_read_i2c, 0x36 << 1);
	// max17048.Address = 0x36 << 1;
	// max17048.Read = max17048_read_i2c;
	// max17048.Write = max17048_write_i2c;


	return max17048_timeout;
}

esp_timer_handle_t sensor_timer;

extern "C" void app_main() {
	const FusionAhrsSettings settings = {
			.convention = FusionConventionNwu,
			.gain = 0.5f,
			.gyroscopeRange = 2000.0f, /* replace this with actual gyroscope range in degrees/s */
			.accelerationRejection = 90.0f,
			.magneticRejection = 90.0f,
			.recoveryTriggerPeriod = 5 * SENSOR_HZ, /* 5 seconds */
	};
	FusionAhrsSetSettings(&ahrs, &settings);
	FusionAhrsInitialise(&ahrs);

	i2c_master_bus_config_t i2c_mst_config = {
		.i2c_port = I2C_MAIN_PORT,
		.sda_io_num = PIN_SDA,
		.scl_io_num = PIN_SCL,
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.glitch_ignore_cnt = 7,
	};
	i2c_mst_config.flags.enable_internal_pullup = false;

	ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &i2c_main_bus_handle));

	onBattery = init_max17048();

	// init_bmp390();

	// init_lis3mdl();

	// init_lsm6dsox();

	xTaskCreate(dataout_task, "dataout_task", 1024 * 4, NULL, 10, NULL);

	const esp_timer_create_args_t sensor_timer_args = {
		.callback = &sensor_read,
		.name = "sensor_read"
	};

	ESP_ERROR_CHECK(esp_timer_create(&sensor_timer_args, &sensor_timer));

	/* Start the timer */
	ESP_ERROR_CHECK(esp_timer_start_periodic(sensor_timer, 1000000 / SENSOR_HZ));
}