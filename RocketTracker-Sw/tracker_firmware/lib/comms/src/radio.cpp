#include "radio.h"
#include <stdint.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_types.h"
#include "esp_event.h"
#include "esp_err.h"

#define SPI_AQUIRE HAL_wait_spi(); spi_lock = true;
#define SPI_RELEASE spi_lock = false;

bool RFM97_LoRa::messageAvailable() {
	if (state == RFM97_RadioState::RX_FINISHED) {
		state = RFM97_RadioState::RX_WAITING;
		return true;
	}

	return false;
}

void RFM97_LoRa::reset() {
	// DONE: Platform code for GPIO reset
	gpio_set_level(rst, 0);
	vTaskDelay(10);
	gpio_set_level(rst, 1);
}

void RFM97_LoRa::HAL_wait_spi() {
	while (spi_lock) {
		vTaskDelay(10);
	}
}

/// Sets register reg to value
/// @param reg register address to write to
/// @param value value to write
void RFM97_LoRa::HAL_spiwrite(uint8_t reg, uint8_t value) {
	SPI_AQUIRE;

	// DONE: Platform code for SPI
	gpio_set_level(cs, 0);

	spi_transaction_t t{
		.flags = SPI_TRANS_USE_TXDATA,
		.length = 2 * 8,
	};

	t.tx_data[0] = reg | 0b10000000;
	t.tx_data[1] = value;

	ESP_ERROR_CHECK(spi_device_transmit(spi_inst, &t));

	gpio_set_level(cs, 1);
	SPI_RELEASE;
}

/// Writes len bytes stored in dataptr to registers starting at start or into the FIFO
/// @param start first register address to start writing at
/// @param dataptr data to write to consecutive addresses
/// @param len amount of bytes to write
void RFM97_LoRa::HAL_spiwrite(uint8_t start, uint8_t* dataptr, size_t len) {
	SPI_AQUIRE;

	// DONE: Platform code for SPI
	gpio_set_level(cs, 0);

	spi_transaction_t t{
		.flags = SPI_TRANS_USE_TXDATA,
		.length = 1 * 8,
	};

	t.tx_data[0] = start | 0b10000000;

	ESP_ERROR_CHECK(spi_device_transmit(spi_inst, &t));

	t.flags = 0;
	t.length = len * 8;
	t.tx_buffer = dataptr;
	t.rx_buffer = NULL;

	ESP_ERROR_CHECK(spi_device_transmit(spi_inst, &t));

	// for (size_t i = 0; i < len; ++i)
	// 	spi_write_blocking(spi_inst, &dataptr[i], 1);
	gpio_set_level(cs, 1);
	SPI_RELEASE;
}

/// Reads from register at address reg
/// @param reg register address to read from
/// @return value stored at address
uint8_t RFM97_LoRa::HAL_spiread(uint8_t reg) {
	SPI_AQUIRE;
	uint8_t rx[2];

	// DONE: Platform code for SPI
	gpio_set_level(cs, 0);
	spi_transaction_t t{
		.flags = SPI_TRANS_USE_TXDATA,
		.length = 2 * 8,
		.rx_buffer = rx
	};

	t.tx_data[0] = reg & 0b01111111;
	t.tx_data[1] = 0xFF;

	ESP_ERROR_CHECK(spi_device_transmit(spi_inst, &t));

	gpio_set_level(cs, 1);
	SPI_RELEASE;

	return rx[1];
}

/// Reads len bytes into dataout starting at address reg
/// @param start register address to start from
/// @param dataout buffer to store read bytes into (must be at least len bytes)
/// @param len number of bytes to read
void RFM97_LoRa::HAL_spiread(uint8_t start, uint8_t* dataout, size_t len) {
	SPI_AQUIRE;
	gpio_set_level(cs, 0);

	spi_transaction_t t{
		.flags = SPI_TRANS_USE_TXDATA,
		.length = 1 * 8,
	};

	t.tx_data[0] = start & 0b01111111;

	ESP_ERROR_CHECK(spi_device_transmit(spi_inst, &t));

	t.flags = 0;
	t.length = len * 8;
	t.rx_buffer = dataout;
	t.tx_buffer = NULL;

	ESP_ERROR_CHECK(spi_device_transmit(spi_inst, &t));

	gpio_set_level(cs, 1);
	SPI_RELEASE;
}

/// Writes n bytes to the FIFO
/// @param bytes bytes to write (must be at least n long)
/// @param n number of bytes to write
/// @param offset where to start writing
/// @return true if parameters are valid, if not, writing is aborted
bool RFM97_LoRa::writeFIFO(uint8_t* bytes, size_t n) {
	ASSURE(n <= 256);
	ASSURE(n > 0);

	// HAL_spiwrite(SX1276_REG_FIFO_TX_BASEADDR, 0x00);
	HAL_spiwrite(SX1276_REG_FIFO_ADDR_PTR, 0x00);

	HAL_spiwrite(SX1276_REG_FIFO, bytes, n);

	return true;
}

#if HAL_ENABLE_DMA
bool RFM97_LoRa::HAL_writeFIFO_DMA(uint8_t* bytes, size_t n) {
	ASSURE(n <= 256);
	ASSURE(n > 0);

	write(SX1276_REG_FIFO_TX_BASEADDR, 0x00);
	write(SX1276_REG_FIFO_ADDR_PTR, 0x00);

	uint8_t w = SX1276_REG_FIFO | 0b10000000;

	SPI_AQUIRE;
	// TODO: Platform code for DMA
	return false;

	return true;
}
#endif

/// Reads n bytes from the FIFO
/// @param bytes buffer to store the read bytes (must have capacity for n bytes)
/// @param n number of bytes to read
/// @param offset where to start reading
/// @return true if parameters are valid, if not, reading is aborted
bool RFM97_LoRa::readFIFO(uint8_t* bytes, size_t n) {
	ASSURE(n <= 256);
	ASSURE(n > 0);

	HAL_spiwrite(SX1276_REG_FIFO_ADDR_PTR, HAL_spiread(SX1276_REG_FIFO_RX_CURRENTADDR));
	HAL_spiread(SX1276_REG_FIFO, bytes, n);

	return true;
}

#if HAL_ENABLE_DMA
bool RFM97_LoRa::HAL_readFIFO_DMA(volatile uint8_t* bytes, size_t n) {
	ASSURE(n <= 256);
	ASSURE(n > 0);

	uint8_t w = SX1276_REG_FIFO & 0b01111111;

	// write(SX1276_REG_FIFO_RX_BASEADDR, 0x00);
	// write(SX1276_REG_FIFO_ADDR_PTR, 0x00);

	SPI_AQUIRE;
	// TODO: Platform code for DMA
	return false;

	return true;
}
#endif

// Make sure all of the FIFO pointers are correct
void RFM97_LoRa::initFIFO() {
	HAL_spiwrite(SX1276_REG_FIFO_RX_BASEADDR, 0x00);
	HAL_spiwrite(SX1276_REG_FIFO_TX_BASEADDR, 0x00);
	HAL_spiwrite(SX1276_REG_FIFO_ADDR_PTR, 0x00);
}

RFM97_LoRa::RFM97_LoRa(spi_host_device_t SPI, gpio_num_t CS, gpio_num_t DIO0, gpio_num_t RST, gpio_num_t STX, gpio_num_t SRX, gpio_num_t SCK, bool use_dma) : spi_bus(SPI), cs(CS), rst(RST), dio0(DIO0), sck(SCK), stx(STX), srx(SRX), use_dma(use_dma) {}

bool RFM97_LoRa::HAL_spiInit(uint baud) {
	// TODO: Platform code SPI init
	spi_device_handle_t spi;
	spi_bus_config_t buscfg = {
		.mosi_io_num = stx,
		.miso_io_num = srx,
		.sclk_io_num = sck,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
	};
	spi_device_interface_config_t devcfg = {
		.mode = 0,                                //SPI mode 0
		.clock_speed_hz = 10 * 1000 * 1000,   	  // 10MHz
		.spics_io_num = -1,               //CS pin
		.queue_size = 7,                          //We want to be able to queue 7 transactions at a time
	};

	ESP_ERROR_CHECK(spi_bus_initialize(spi_bus, &buscfg, SPI_DMA_CH_AUTO));
	//Attach the LCD to the SPI bus
	ESP_ERROR_CHECK(spi_bus_add_device(spi_bus, &devcfg, &spi_inst));

	gpio_reset_pin(cs);
	gpio_set_direction(cs, GPIO_MODE_OUTPUT);
	gpio_set_level(cs, 1);
	return true;
}

bool RFM97_LoRa::HAL_gpioInit() {
	gpio_reset_pin(rst);
	gpio_reset_pin(dio0);
	gpio_set_direction(rst, GPIO_MODE_OUTPUT);
	gpio_set_direction(dio0, GPIO_MODE_INPUT);

	return true;
}

void radio_service(void* arg) {
	RFM97_LoRa* radio = (RFM97_LoRa*)arg;

	int v;
	while (1) {
		if (xQueueReceive(radio->intq, &v, portMAX_DELAY)) {
			radio->ISR_A();
		}
	}
}

// LongRangeMode
bool RFM97_LoRa::init() {
	// TODO: Platform code GPIO initialization
	HAL_gpioInit();

	// SPI Init
	ASSURE(HAL_spiInit(1000 * 1000));

	reset();

	// Make sure its the right chip and its connected properly...
	ASSURE(HAL_spiread(SX1276_REG_VERSION) == 0x12);

	// Sleep and then set into LoRa mode
	HAL_spiwrite(SX1276_REG_OPMODE, 0b00001000);
	vTaskDelay(10);
	HAL_spiwrite(SX1276_REG_OPMODE, 0b00001000 | SX1276_OPMODE_MASK_LORAMODE);
	setMode(SX1276_MODE_STDBY);

	// Set any neccesary registers to work properly
	configure();

	// Set frequency to 915mhz
	ASSURE(setFreq(915.0));

	// Set to the highest power setting available (20dBm with power amp)
	ASSURE(setPower(20, true));

	// Set bandwidth to 125kHz, the usual LoRa bandwidth 
	// Other options are 500, 250, 62.5, etc.
	ASSURE(setBandwidth(SX1276_BANDWIDTH_125KHZ));

	// Set spreading factor to 12 (4096 chirps per bit) for best decoding.
	ASSURE(setSpreadingFactor(7));

	// Set coding rate denominator to 5
	ASSURE(setCodingRate(5));

	setPreambleLen(8);

	// Set header mode to explicit
	setHeaderMode(false);

	// Enable CRC
	setCrcMode(true);

	// Syncword of 0x18
	setSyncWord(0x18);

	// Max gain
	setGain(SX1276_GAIN_MAX);

	// Set DIO0 to interrupt on RX done
	ASSURE(configDIO(0, SX1276_DIO0_RX_DONE));

	// Set overcurrent protection to 120mA
	setOCP(true, 15);

	state = RFM97_RadioState::STANDBY;

	intq = xQueueCreate(10, sizeof(int));
	xTaskCreate(radio_service, "radio_service", 1024 * 2, (void*)this, configMAX_PRIORITIES, NULL);

	return true;
}

void RFM97_LoRa::configure() {
	// No FHSS!
	HAL_spiwrite(SX1276_REG_HOPPERIOD, 0x00);

	// Enable all interrupts
	HAL_spiwrite(SX1276_REG_IRQFLAGSMASK, 0x00);

	// Set FIFO pointers
	initFIFO();
}

/// @param func ISR function to be called when DIO0 rises. the ISR MUST call onInterrupt() for proper behavior of the radio.
void RFM97_LoRa::setISRA(void (*func)(void*)) {
	// TODO: Platform code
	// gpio_intr_enable(dio0);
	gpio_set_intr_type(dio0, GPIO_INTR_POSEDGE);
	gpio_install_isr_service(0);
	ESP_ERROR_CHECK(gpio_isr_handler_add(dio0, func, NULL));
}

/// @param func ISR function to be called when DMA completes. the ISR MUST call onInterrupt() for proper behavior of the radio.
void RFM97_LoRa::setISRB(void (*func)(void*)) {
	// TODO: Platform code
	// irq_set_exclusive_handler(DMA_IRQ_0, func);
}


void RFM97_LoRa::standby() {
	setMode(SX1276_MODE_STDBY);
	state = RFM97_RadioState::STANDBY;
}

void RFM97_LoRa::sleep() {
	setMode(SX1276_MODE_SLEEP);
	state = RFM97_RadioState::SLEEP;
}

// Clears all IRQ flags by writing ones
void RFM97_LoRa::clearIRQ() {
	HAL_spiwrite(SX1276_REG_IRQFLAGS, 0xFF);
}

void RFM97_LoRa::setMode(uint8_t modebits) {
	HAL_spiwrite(SX1276_REG_OPMODE, (HAL_spiread(SX1276_REG_OPMODE) & (~SX1276_OPMODE_MASK_MODE)) | modebits);
}

// Frf
bool RFM97_LoRa::setFreq(double freq_mhz) {
	// Make sure it's in the ISM band (RFM97CW can go outside!)
	ASSURE(freq_mhz < SX1276_FRF_MAX && freq_mhz > SX1276_FRF_MIN);

	uint32_t FRF = (freq_mhz * (uint32_t(1) << 19)) / 32.0;

	// write registers
	HAL_spiwrite(SX1276_REG_FRF_MSB, (FRF & 0xFF0000) >> 16);
	HAL_spiwrite(SX1276_REG_FRF_MID, (FRF & 0x00FF00) >> 8);
	HAL_spiwrite(SX1276_REG_FRF_LSB, FRF & 0x0000FF);

	return true;
}

// PaSelect and PaConfig
bool RFM97_LoRa::setPower(uint8_t power, bool pa_boost) {
	uint8_t paconfig_val = 0x00;
	bool padac_highpower = false;

	standby();

	if (pa_boost) {
		ASSURE(power > 2 && (power <= 17 || power == 20));
		paconfig_val |= SX1276_PASELECT_PABOOST;
		paconfig_val |= SX1276_PACONFIG_MAXPOWER;

		if (power == 20) {
			paconfig_val |= 0x0F;
			padac_highpower = true;
		} else {
			paconfig_val |= power - 2;
		}
	} else {
		ASSURE(power > -3 && power <= 14);
		// paconfig_val &= ~SX1276_PASELECT_PABOOST;

		if (power < 0) {
			paconfig_val |= SX1276_PACONFIG_LOWPOWER;
			paconfig_val |= power + 3;
		} else {
			paconfig_val |= SX1276_PACONFIG_MAXPOWER;
			paconfig_val |= power;
		}
	}

	HAL_spiwrite(SX1276_REG_PACONFIG, paconfig_val);
	HAL_spiwrite(SX1276_REG_PADAC, padac_highpower ? SX1276_PADAC_HIGHPOWER : SX1276_PADAC_LOWPOWER);

	return true;
}

// Bw
bool RFM97_LoRa::setBandwidth(uint32_t bw_mode) {
	ASSURE(bw_mode == SX1276_BANDWIDTH_500KHZ ||
		bw_mode == SX1276_BANDWIDTH_250KHZ ||
		bw_mode == SX1276_BANDWIDTH_125KHZ ||
		bw_mode == SX1276_BANDWIDTH_62_5_KHZ ||
		bw_mode == SX1276_BANDWIDTH_41_7_KHZ ||
		bw_mode == SX1276_BANDWIDTH_31_25_KHZ ||
		bw_mode == SX1276_BANDWIDTH_20_8_KHZ ||
		bw_mode == SX1276_BANDWIDTH_15_6_KHZ ||
		bw_mode == SX1276_BANDWIDTH_10_4_KHZ ||
		bw_mode == SX1276_BANDWIDTH_7_8_KHZ);

	/*
		Signal bandwidth:
		7.8 kHz   -> 0000
		10.4 kHz  -> 0001
		15.6 kHz  -> 0010
		20.8kHz   -> 0011
		31.25 kHz -> 0100
		41.7 kHz  -> 0101
		62.5 kHz  -> 0110
		125 kHz   -> 0111
		250 kHz   -> 1000
		500 kHz   -> 1001
	*/

	// Set bandwidth setting in modemconfig 1
	HAL_spiwrite(SX1276_REG_MODEMCONFIG_1, (HAL_spiread(SX1276_REG_MODEMCONFIG_1) & (~SX1276_MODEMCFG_MASK_BANDWIDTH)) | bw_mode << 4);

	// automatic HBWO
	if (bw_mode == SX1276_BANDWIDTH_500KHZ) enableHBWO();

	return true;
}

// ImplicitHeaderModeOn
void RFM97_LoRa::setHeaderMode(bool implicit = false) {
	HAL_spiwrite(SX1276_REG_MODEMCONFIG_1, (HAL_spiread(SX1276_REG_MODEMCONFIG_1) & (~SX1276_MODEMCFG_MASK_HEADERMODE)) | implicit);
}

// SpreadingFactor
bool RFM97_LoRa::setSpreadingFactor(uint8_t spreading_factor = 7) {
	ASSURE(spreading_factor >= SX1276_SF_MIN && spreading_factor <= SX1276_SF_MAX);

	/*
		SF rate (expressed as a base-2 logarithm):
		6 -> 64 chips / symbol [REQUIRES ADDITIONAL CONFIGURATION]
		7 -> 128 chips / symbol
		8 -> 256 chips / symbol
		9 -> 512 chips / symbol
		10 -> 1024 chips / symbol
		11 -> 2048 chips / symbol
		12 -> 4096 chips / symbol
	*/

	// write to the 4 highest bits
	HAL_spiwrite(SX1276_REG_MODEMCONFIG_2, (HAL_spiread(SX1276_REG_MODEMCONFIG_2) & (~SX1276_MODEMCFG_MASK_SPREADINGFACTOR)) | spreading_factor << 4);

	return true;
}

// CodingRate
bool RFM97_LoRa::setCodingRate(uint8_t cr_denom = 5) {
	ASSURE(cr_denom >= SX1276_CR_MIN && cr_denom <= SX1276_CR_MAX);

	/*
		Error coding rate:
		4/5 -> 001
		4/6 -> 010
		4/7 -> 011
		4/8 -> 100
	*/

	HAL_spiwrite(SX1276_REG_MODEMCONFIG_1, (HAL_spiread(SX1276_REG_MODEMCONFIG_1) & (~SX1276_MODEMCFG_MASK_CODINGRATE)) | (cr_denom - 4));
	return true;
}

// RxPayloadCrcOn
void RFM97_LoRa::setCrcMode(bool crc_en = false) {
	/*
		Enable CRC generation and check on payload:
		CRC disable -> 0
		CRC enable  -> 1
		If CRC is needed, RxPayloadCrcOn should be set:
		- in Implicit header mode: on Tx and Rx side
		- in Explicit header mode: on the Tx side alone (recovered from the
		header in Rx side)
	*/
	HAL_spiwrite(SX1276_REG_MODEMCONFIG_2, (HAL_spiread(SX1276_REG_MODEMCONFIG_2) & (~SX1276_MODEMCFG_MASK_CRC)) | (crc_en ? SX1276_CRC_ON : SX1276_CRC_OFF));
}

// SymbTimeout
bool RFM97_LoRa::setTimeout(uint16_t timeout_symbols = 0x64) {
	ASSURE(timeout_symbols < 1024); // Make sure the value fits in 10 bits

	/*
		`Ts` is the time taken by a single symbol

		TimeOut = SymbTimeout * Ts
	*/

	uint8_t timeout_msb = timeout_symbols & 0x300;
	uint8_t timeout_lsb = timeout_symbols & 0xFF;

	// Write the LSB to the dedicated register and the MSB to the two dedicated bits in modemconfig 2
	HAL_spiwrite(SX1276_REG_SYMBTIMEOUT_LSB, timeout_lsb);
	HAL_spiwrite(SX1276_REG_MODEMCONFIG_2, (HAL_spiread(SX1276_REG_MODEMCONFIG_2) & (~SX1276_MODEMCFG_MASK_SYMTIMEOUTMSB)) | timeout_msb);

	return true;
}

// RegPreambleMsb, RegPreambleLsb
void RFM97_LoRa::setPreambleLen(uint16_t preamble_len = 0x08) {
	HAL_spiwrite(SX1276_REG_PREAMBLE_LSB, (uint8_t)(preamble_len & 0xFF));
	HAL_spiwrite(SX1276_REG_PREAMBLE_MSB, (uint8_t)((preamble_len & 0xFF00) >> 8));
}

// LnaGain
bool RFM97_LoRa::setGain(uint8_t gain = 1) {
	ASSURE(gain > 0 && gain <= 6);

	HAL_spiwrite(SX1276_REG_LNA, (HAL_spiread(SX1276_REG_LNA) & (~SX1276_LNA_MASK_GAIN)) | (gain << 5));

	return true;
}

// LnaBoostHf
void RFM97_LoRa::setLNABoostHF(bool boost = false) {
	HAL_spiwrite(SX1276_REG_LNA, (HAL_spiread(SX1276_REG_LNA) & (~SX1276_LNA_MASK_BOOSTHF)) | (boost ? 0b11 : 0));
}

// AgcAutoOn
void RFM97_LoRa::setAGC(bool agc = 0) {
	HAL_spiwrite(SX1276_REG_MODEMCONFIG_3, (HAL_spiread(SX1276_REG_MODEMCONFIG_3) & (~SX1276_MODEMCFG_MASK_AGC)) | (agc ? SX1276_AGC_ON : SX1276_AGC_OFF));
}

// LowDataRateOptimize
void RFM97_LoRa::setLDRO(bool ldro = 0) {
	HAL_spiwrite(SX1276_REG_MODEMCONFIG_3, (HAL_spiread(SX1276_REG_MODEMCONFIG_3) & (~SX1276_MODEMCFG_MASK_LDRO)) | (ldro ? SX1276_LDRO_ON : SX1276_LDRO_OFF));
}

// HighBWOptimize1
// See errata note section 2.1
// NOTE: Automatically enabled by setBandwidth(SX1276_BANDWIDTH_500KHZ)
void RFM97_LoRa::enableHBWO() {
	HAL_spiwrite(0x36, 0x02);
	HAL_spiwrite(0x3A, 0x64);
}

// Dio[0-5]Mapping
bool RFM97_LoRa::configDIO(uint8_t dio, uint8_t mode) {
	ASSURE(dio < 6 && mode < 3);
	// Either DioMapping1 or DioMapping2
	if (dio < 4) {
		uint8_t shift = (3 - dio) * 2;
		HAL_spiwrite(SX1276_REG_DIOMAPPING1, (HAL_spiread(SX1276_REG_DIOMAPPING1) & ~(0b11 << shift)) | (mode << shift));
	} else {
		uint8_t shift = (3 - (dio - 4)) * 2;
		HAL_spiwrite(SX1276_REG_DIOMAPPING2, (HAL_spiread(SX1276_REG_DIOMAPPING2) & ~(0b11 << shift)) | (mode << shift));
	}

	return true;
}

// DetectionThreshold
// TODO
void RFM97_LoRa::setDetectionThreshold() {

}

// SyncWord
void RFM97_LoRa::setSyncWord(uint8_t syncword = 0x18) {
	HAL_spiwrite(SX1276_REG_SYNCWORD, syncword);
}

// InvertIQ RX and TX
// TODO
void RFM97_LoRa::setInvIQ(bool inverted) {

}

// PacketRssi
/// @return the RSSI (dBm) of the last received packet
int16_t RFM97_LoRa::getRSSI() {
	return -157 + HAL_spiread(SX1276_REG_PKTRSSIVALUE);
}

// PacketSnr
/// @return the SNR of the last received packet
float RFM97_LoRa::getSNR() {
	return float(HAL_spiread(SX1276_REG_PKTSNRVALUE)) / 4.0f;
}

// Sets the overcurrent protection limit
void RFM97_LoRa::setOCP(bool enabled, uint8_t trim = 0x0b) {
	/*
		  trim       max current          formula
		0 to 15  |  45 to 120 mA  |  45 + 5*OcpTrim [mA]
		16 to 27 |  130 to 240 mA |  -30 + 10*OcpTrim [mA]
		27+      |  240 mA        |  240 mA
	*/

	HAL_spiwrite(SX1276_REG_OCP, (enabled ? SX1276_OCP_MASK_EN : 0x00) | (trim & SX1276_OCP_MASK_TRIM));
}

// Called by a ISR on DIO0
// Updates the radio's state machine based on radio IRQs
void RFM97_LoRa::ISR_A() {
	clearIRQ();
	// printf("ISR_A\n");
	switch (state) {
		case RFM97_RadioState::TX_WAITING:
			clearIRQ();
			if (tx_state_go_rx) startReceiving();
			else {
				standby();
				state = RFM97_RadioState::STANDBY;
			}
			break;
			// case RFM97_RadioState::RX_FINISHED:
		case RFM97_RadioState::RX_WAITING:
			receive();
			break;
		default:
			return;
	};
}

// void RFM97_LoRa::ISR_B() {
// 	hw_clear_bits(&dma_hw->ints0, 1u << dma_RX || 1u << dma_TX);
// 	gpio_put(cs, 1);
// 	SPI_RELEASE;

// 	if (state == RFM97_RadioState::RX_DMA) {
// 		state = RFM97_RadioState::RX_FINISHED;

// 	} else if (state == RFM97_RadioState::TX_DMA) {
// 		setMode(SX1276_MODE_TX); // Transmit
// 		state = RFM97_RadioState::TX_WAITING;
// 	}
// }

void RFM97_LoRa::startReceiving() {
	// wait_for_safe_state();

	standby();
	state = RFM97_RadioState::RX_WAITING;
	configDIO(0, SX1276_DIO0_RX_DONE);
	setMode(SX1276_MODE_RXCONTINUOUS);
}

bool RFM97_LoRa::transmit(uint8_t* data, size_t len, bool return_to_rx = true) {
	ASSURE(len > 0 && len <= 256);
	tx_state_go_rx = return_to_rx;

	// Wait for safe state to transmit, no way around it, maybe add a fail-poll mode?
	// wait_for_safe_state();

	standby();
	ASSURE(configDIO(0, SX1276_DIO0_TX_DONE));
	clearIRQ();

	HAL_spiwrite(SX1276_REG_PAYLOADLENGTH, (uint8_t)len);

#if HAL_ENABLE_DMA
	if (use_dma) {
		HAL_writeFIFO_DMA(data, len);
		state = RFM97_RadioState::TX_DMA;
		return true; // ISR_B handles transmit
	} else {
		writeFIFO(data, len);
		state = RFM97_RadioState::TX_WAITING;
		setMode(SX1276_MODE_TX); // Transmit
	}
#else
	writeFIFO(data, len);
	state = RFM97_RadioState::TX_WAITING;
	setMode(SX1276_MODE_TX); // Transmit
#endif

	// // Wait for the transmit done IRQ
	// // TODO: improve for async receiving (start receive and prep IRQ state to cleanup when done)
	// while (state != RFM97_RadioState::TX_FINISHED) {
	// 	// sleep_ms(1);
	// 	// Busy-wait
	// }

	// clearIRQ();
	// standby();
	// state = RFM97_RadioState::STANDBY;

	return true;
}

// TODO: DMA based and IRQ chaining
// dio0 IRQ (dma start) -> DMA done IRQ (set rxdone state)
void RFM97_LoRa::receive() {
	lastRxLen = HAL_spiread(SX1276_REG_RX_NB_BYTES);

#if HAL_ENABLE_DMA
	if (use_dma) {
		HAL_readFIFO_DMA(rxbuf, lastRxLen);
		state = RFM97_RadioState::RX_DMA;
	} else {
		readFIFO((uint8_t*)rxbuf, lastRxLen);
		state = RFM97_RadioState::RX_FINISHED;
	}
#else
	readFIFO((uint8_t*)rxbuf, lastRxLen);
	state = RFM97_RadioState::RX_FINISHED;
#endif
}

bool RFM97_LoRa::applyConfig(RFM97_LoRa_config cfg) {
	ASSURE(setFreq(cfg.frf));
	ASSURE(setBandwidth(cfg.bw_mode));
	ASSURE(setCodingRate(cfg.cr));
	ASSURE(setSpreadingFactor(cfg.sf));
	ASSURE(setPower(cfg.power, true));
	return true;
}

void RFM97_LoRa::wait_for_safe_state() {
	while (!(state == RFM97_RadioState::RX_WAITING || state == RFM97_RadioState::STANDBY)) {
		vTaskDelay(10);
		//__wfe();
	};
}