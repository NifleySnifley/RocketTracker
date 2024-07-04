#include "esp_log.h"
#include "driver/i2c_master.h"
#include "bmp3.h"
#include "bmp3_defs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bmp3_interface.h"
#include "memory.h"
#include <stdio.h>
#include <unistd.h>

extern i2c_master_bus_handle_t bus_handle;
extern i2c_master_dev_handle_t bmp390_dev_handle;

/*!
	 *  @brief Function to select the interface between SPI and I2C.
	 *
	 *  @param[in] bmp3      : Structure instance of bmp3_dev
	 *  @param[in] intf     : Interface selection parameter
	 *
	 *  @return Status of execution
	 *  @retval 0 -> Success
	 *  @retval < 0 -> Failure Info
	 */
BMP3_INTF_RET_TYPE bmp3_interface_init(struct bmp3_dev* bmp3, uint8_t intf) {
	if (intf == BMP3_I2C_INTF) {
		i2c_device_config_t bmp390_dev_cfg = {
			.dev_addr_length = I2C_ADDR_BIT_LEN_7,
			.device_address = BMP3_ADDR_I2C_SEC,
			.scl_speed_hz = 400000,
		};

		ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &bmp390_dev_cfg, &bmp390_dev_handle));

		bmp3->read = bmp3_i2c_read;
		bmp3->write = bmp3_i2c_write;
		bmp3->intf = BMP3_I2C_INTF;
		bmp3->delay_us = bmp3_delay_us;
		bmp3->intf_ptr = &bmp390_dev_handle;
		return BMP3_INTF_RET_SUCCESS;
	} else {
		return 1;
	}
}

/*!
 *  @brief Function for reading the sensor's registers through I2C bus.
 *
 *  @param[in] reg_addr     : Register address.
 *  @param[out] reg_data    : Pointer to the data buffer to store the read data.
 *  @param[in] len          : No of bytes to read.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return Status of execution
 *  @retval = BMP3_INTF_RET_SUCCESS -> Success
 *  @retval != BMP3_INTF_RET_SUCCESS  -> Failure Info
 *
 */
BMP3_INTF_RET_TYPE bmp3_i2c_read(uint8_t reg_addr, uint8_t* reg_data, uint32_t len, void* intf_ptr) {
	if (i2c_master_transmit_receive(bmp390_dev_handle, &reg_addr, 1, reg_data, len, 50) == ESP_OK) {
		return BMP3_INTF_RET_SUCCESS;
	}
	return 2;
}

/*!
 *  @brief Function for writing the sensor's registers through I2C bus.
 *
 *  @param[in] reg_addr     : Register address.
 *  @param[in] reg_data     : Pointer to the data buffer whose value is to be written.
 *  @param[in] len          : No of bytes to write.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return Status of execution
 *  @retval = BMP3_INTF_RET_SUCCESS -> Success
 *  @retval != BMP3_INTF_RET_SUCCESS  -> Failure Info
 *
 */
BMP3_INTF_RET_TYPE bmp3_i2c_write(uint8_t reg_addr, const uint8_t* reg_data, uint32_t len, void* intf_ptr) {
	uint8_t* data = malloc(len + 1);
	data[0] = reg_addr;
	for (int i = 0; i < len; i++) {
		data[i + 1] = reg_data[i];
	}
	esp_err_t e = i2c_master_transmit(bmp390_dev_handle, data, len + 1, 50);
	free(data);
	return e != ESP_OK;
}

/*!
 * @brief This function provides the delay for required time (Microsecond) as per the input provided in some of the
 * APIs.
 *
 *  @param[in] period       : The required wait time in microsecond.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return void.
 *
 */
void bmp3_delay_us(uint32_t period, void* intf_ptr) {
	// usleep(period);//vTaskDelay(pdMS_TO_TICKS(period / 1000));
}

/*!
 *  @brief Prints the execution status of the APIs.
 *
 *  @param[in] api_name : Name of the API whose execution status has to be printed.
 *  @param[in] rslt     : Error code returned by the API whose execution status has to be printed.
 *
 *  @return void.
 */
void bmp3_check_rslt(const char api_name[], int8_t rslt) {
	if (rslt != 0)
		ESP_LOGE("BMP3", "%s = %d", api_name, rslt);
}