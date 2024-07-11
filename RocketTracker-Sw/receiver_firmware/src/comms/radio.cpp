#include <hardware/spi.h>
#include <hardware/gpio.h>
#include "radio.h"
#include <stdint.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/sync.h"
#include "pinout.h"
#include "tusb.h"

#define SPI_AQUIRE wait_spi(); spi_lock = true;
#define SPI_RELEASE spi_lock = false;

bool RFM97_LoRa::messageAvailable() {
	if (state == RFM97_RadioState::RX_FINISHED) {
		state = RFM97_RadioState::RX_WAITING;
		return true;
	}

	return false;
}

void RFM97_LoRa::reset() {
	gpio_put(rst, 0);
	sleep_ms(10);
	gpio_put(rst, 1);
}

void RFM97_LoRa::wait_spi() {
	while (spi_lock) {
		__wfe();
	}
}

/// Sets register reg to value
/// @param reg register address to write to
/// @param value value to write
void RFM97_LoRa::write(uint8_t reg, uint8_t value) {
	SPI_AQUIRE;
	uint8_t reg_b = reg | 0b10000000;
	gpio_put(cs, 0);
	spi_write_blocking(spi_inst, &reg_b, 1); // Addr and wnr bit set to 1 for write
	spi_write_blocking(spi_inst, &value, 1);
	gpio_put(cs, 1);
	SPI_RELEASE;
}

/// Writes len bytes stored in dataptr to registers starting at start or into the FIFO
/// @param start first register address to start writing at
/// @param dataptr data to write to consecutive addresses
/// @param len amount of bytes to write
void RFM97_LoRa::write(uint8_t start, uint8_t* dataptr, size_t len) {
	SPI_AQUIRE;
	uint8_t start_b = start | 0b10000000;
	gpio_put(cs, 0);
	spi_write_blocking(spi_inst, &start_b, 1);
	for (size_t i = 0; i < len; ++i)
		spi_write_blocking(spi_inst, &dataptr[i], 1);
	gpio_put(cs, 1);
	SPI_RELEASE;
}

/// Reads from register at address reg
/// @param reg register address to read from
/// @return value stored at address
uint8_t RFM97_LoRa::read(uint8_t reg) {
	SPI_AQUIRE;
	uint8_t reg_b = reg & 0b01111111;
	gpio_put(cs, 0);
	spi_write_blocking(spi_inst, &reg_b, 1); // Addr and wnr bit set to 0 for read
	uint8_t value;
	spi_read_blocking(spi_inst, 0xFF, &value, 1);
	gpio_put(cs, 1);
	SPI_RELEASE;

	return value;
}

/// Reads len bytes into dataout starting at address reg
/// @param start register address to start from
/// @param dataout buffer to store read bytes into (must be at least len bytes)
/// @param len number of bytes to read
void RFM97_LoRa::read(uint8_t start, uint8_t* dataout, size_t len) {
	SPI_AQUIRE;
	uint8_t start_b = start & 0b01111111;
	gpio_put(cs, 0);
	spi_write_blocking(spi_inst, &start_b, 1); // Addr and wnr bit set to 0 for read
	spi_read_blocking(spi_inst, 0xFF, dataout, len);
	gpio_put(cs, 1);
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

	// write(SX1276_REG_FIFO_TX_BASEADDR, 0x00);
	write(SX1276_REG_FIFO_ADDR_PTR, 0x00);
	write(SX1276_REG_FIFO, bytes, n);

	return true;
}


bool RFM97_LoRa::writeFIFO_DMA(uint8_t* bytes, size_t n) {
	ASSURE(n <= 256);
	ASSURE(n > 0);

	// write(SX1276_REG_FIFO_TX_BASEADDR, 0x00);
	write(SX1276_REG_FIFO_ADDR_PTR, 0x00);

	uint8_t w = SX1276_REG_FIFO | 0b10000000;

	SPI_AQUIRE;
	gpio_put(cs, 0);
	spi_write_blocking(spi_inst, &w, 1);

	hw_clear_bits(&dma_hw->ch[dma_RX].al1_ctrl, DMA_CH0_CTRL_TRIG_INCR_WRITE_BITS);
	hw_clear_bits(&dma_hw->ch[dma_RX].al1_ctrl, DMA_CH0_CTRL_TRIG_INCR_READ_BITS);

	hw_set_bits(&dma_hw->ch[dma_TX].al1_ctrl, DMA_CH0_CTRL_TRIG_INCR_READ_BITS);
	hw_clear_bits(&dma_hw->ch[dma_TX].al1_ctrl, DMA_CH0_CTRL_TRIG_INCR_WRITE_BITS);

	dma_channel_transfer_to_buffer_now(dma_RX, &dma_dummy, n);
	dma_channel_transfer_from_buffer_now(dma_TX, bytes, n);

	return true;
}

/// Reads n bytes from the FIFO
/// @param bytes buffer to store the read bytes (must have capacity for n bytes)
/// @param n number of bytes to read
/// @param offset where to start reading
/// @return true if parameters are valid, if not, reading is aborted
bool RFM97_LoRa::readFIFO(uint8_t* bytes, size_t n) {
	ASSURE(n <= 256);
	ASSURE(n > 0);

	// write(SX1276_REG_FIFO_RX_BASEADDR, 0x00);
	write(SX1276_REG_FIFO_ADDR_PTR, read(SX1276_REG_FIFO_RX_CURRENTADDR));
	read(SX1276_REG_FIFO, bytes, n);

	return true;
}

bool RFM97_LoRa::readFIFO_DMA(volatile uint8_t* bytes, size_t n) {
	ASSURE(n <= 256);
	ASSURE(n > 0);

	uint8_t w = SX1276_REG_FIFO & 0b01111111;

	// write(SX1276_REG_FIFO_RX_BASEADDR, 0x00);
	// write(SX1276_REG_FIFO_ADDR_PTR, 0x00);

	write(SX1276_REG_FIFO_ADDR_PTR, read(SX1276_REG_FIFO_RX_CURRENTADDR));

	SPI_AQUIRE;
	gpio_put(cs, 0);
	spi_write_blocking(spi_inst, &w, 1);
	// read(SX1276_REG_FIFO, (uint8_t*)bytes, n);

	hw_set_bits(&dma_hw->ch[dma_RX].al1_ctrl, DMA_CH0_CTRL_TRIG_INCR_WRITE_BITS);
	hw_clear_bits(&dma_hw->ch[dma_RX].al1_ctrl, DMA_CH0_CTRL_TRIG_INCR_READ_BITS);

	hw_clear_bits(&dma_hw->ch[dma_TX].al1_ctrl, DMA_CH0_CTRL_TRIG_INCR_READ_BITS);
	hw_clear_bits(&dma_hw->ch[dma_TX].al1_ctrl, DMA_CH0_CTRL_TRIG_INCR_WRITE_BITS);

	dma_dummy = 0xFF;
	dma_channel_transfer_to_buffer_now(dma_RX, bytes, n);
	dma_channel_transfer_from_buffer_now(dma_TX, &dma_dummy, n);

	return true;
}

// Make sure all of the FIFO pointers are correct
void RFM97_LoRa::initFIFO() {
	write(SX1276_REG_FIFO_RX_BASEADDR, 0x00);
	write(SX1276_REG_FIFO_TX_BASEADDR, 0x00);
	write(SX1276_REG_FIFO_ADDR_PTR, 0x00);
}

RFM97_LoRa::RFM97_LoRa(spi_inst_t* SPI, int CS, int DIO0, int DIO3, int RST, int STX, int SRX, int SCK, bool use_dma) : spi_inst(SPI), cs(CS), rst(RST), dio0(DIO0), dio3(DIO3), sck(SCK), stx(STX), srx(SRX), use_dma(use_dma) {}

bool RFM97_LoRa::spiInit(uint baud) {
	ASSURE(spi_init(spi_inst, baud));
	gpio_set_function(srx, GPIO_FUNC_SPI);
	gpio_set_function(sck, GPIO_FUNC_SPI);
	gpio_set_function(stx, GPIO_FUNC_SPI);
	// Make the SPI pins available to picotool
	// bi_decl(bi_3pins_with_func(srx, stx, sck, GPIO_FUNC_SPI));

	// Chip select is active-low, so we'll initialise it to a driven-high state
	gpio_init(cs);
	gpio_put(cs, 1);
	gpio_set_dir(cs, GPIO_OUT);
	// Make the CS pin available to picotool
	// bi_decl(bi_1pin_with_name(cs, "SPI CS"));

	return true;
}

// LongRangeMode
bool RFM97_LoRa::init() {
	gpio_init(dio0);
	gpio_init(rst);
	gpio_set_dir(dio0, GPIO_IN);
	gpio_set_dir(rst, GPIO_OUT);

	// SPI Init
	ASSURE(spiInit(1000 * 1000));

	dma_TX = dma_claim_unused_channel(true);
	dma_RX = dma_claim_unused_channel(true);

	// DMA Init
	dma_channel_config cfg = dma_channel_get_default_config(dma_TX);
	channel_config_set_transfer_data_size(&cfg, DMA_SIZE_8);
	channel_config_set_dreq(&cfg, spi_get_dreq(spi_inst, true));
	channel_config_set_write_increment(&cfg, false);
	dma_channel_configure(dma_TX, &cfg, &spi_get_hw(spi_inst)->dr, 0, 0, false);

	cfg = dma_channel_get_default_config(dma_RX);
	channel_config_set_transfer_data_size(&cfg, DMA_SIZE_8);
	channel_config_set_dreq(&cfg, spi_get_dreq(spi_inst, false));
	channel_config_set_read_increment(&cfg, false);
	dma_channel_configure(dma_RX, &cfg, 0, &spi_get_hw(spi_inst)->dr, 0, false);

	dma_channel_set_irq0_enabled(dma_RX, true);
	dma_channel_set_irq0_enabled(dma_TX, false);
	irq_set_enabled(DMA_IRQ_0, true);

	reset();

	// Make sure its the right chip and its connected properly...
	ASSURE(read(SX1276_REG_VERSION) == 0x12);

	gpio_put(cs, 1);

	// Sleep and then set into LoRa mode
	write(SX1276_REG_OPMODE, 0b00001000);
	sleep_ms(10);
	write(SX1276_REG_OPMODE, 0b00001000 | SX1276_OPMODE_MASK_LORAMODE);
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
	return true;
}

void RFM97_LoRa::configure() {
	// No FHSS!
	write(SX1276_REG_HOPPERIOD, 0x00);

	// Enable all interrupts
	write(SX1276_REG_IRQFLAGSMASK, 0x00);

	// Set FIFO pointers
	initFIFO();
}

/// @param func ISR function to be called when DIO0 rises. the ISR MUST call onInterrupt() for proper behavior of the radio.
void RFM97_LoRa::setISRA(gpio_irq_callback_t func) {
	gpio_set_irq_callback(func);
	gpio_set_irq_enabled(dio0, GPIO_IRQ_EDGE_RISE, true);
	gpio_set_irq_enabled(dio3, GPIO_IRQ_EDGE_RISE, true);
	// gpio_set_irq_enabled_with_callback(dio0, GPIO_IRQ_EDGE_RISE, true, func);
	irq_set_enabled(IO_IRQ_BANK0, true);
}

/// @param func ISR function to be called when DMA completes. the ISR MUST call onInterrupt() for proper behavior of the radio.
void RFM97_LoRa::setISRB(irq_handler_t func) {
	irq_set_exclusive_handler(DMA_IRQ_0, func);
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
	write(SX1276_REG_IRQFLAGS, 0xFF);
}

void RFM97_LoRa::setMode(uint8_t modebits) {
	write(SX1276_REG_OPMODE, (read(SX1276_REG_OPMODE) & (~SX1276_OPMODE_MASK_MODE)) | modebits);
}

// Frf
bool RFM97_LoRa::setFreq(double freq_mhz) {
	// Make sure it's in the ISM band (RFM97CW can go outside!)
	ASSURE(freq_mhz < SX1276_FRF_MAX && freq_mhz > SX1276_FRF_MIN);

	uint32_t FRF = (freq_mhz * (uint32_t(1) << 19)) / 32.0;

	// write registers
	write(SX1276_REG_FRF_MSB, (FRF & 0xFF0000) >> 16);
	write(SX1276_REG_FRF_MID, (FRF & 0x00FF00) >> 8);
	write(SX1276_REG_FRF_LSB, FRF & 0x0000FF);

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

	write(SX1276_REG_PACONFIG, paconfig_val);
	write(SX1276_REG_PADAC, padac_highpower ? SX1276_PADAC_HIGHPOWER : SX1276_PADAC_LOWPOWER);

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
	write(SX1276_REG_MODEMCONFIG_1, (read(SX1276_REG_MODEMCONFIG_1) & (~SX1276_MODEMCFG_MASK_BANDWIDTH)) | bw_mode << 4);

	// automatic HBWO
	if (bw_mode == SX1276_BANDWIDTH_500KHZ) enableHBWO();

	return true;
}

// ImplicitHeaderModeOn
void RFM97_LoRa::setHeaderMode(bool implicit = false) {
	write(SX1276_REG_MODEMCONFIG_1, (read(SX1276_REG_MODEMCONFIG_1) & (~SX1276_MODEMCFG_MASK_HEADERMODE)) | implicit);
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
	write(SX1276_REG_MODEMCONFIG_2, (read(SX1276_REG_MODEMCONFIG_2) & (~SX1276_MODEMCFG_MASK_SPREADINGFACTOR)) | spreading_factor << 4);

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

	write(SX1276_REG_MODEMCONFIG_1, (read(SX1276_REG_MODEMCONFIG_1) & (~SX1276_MODEMCFG_MASK_CODINGRATE)) | (cr_denom - 4));
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
	write(SX1276_REG_MODEMCONFIG_2, (read(SX1276_REG_MODEMCONFIG_2) & (~SX1276_MODEMCFG_MASK_CRC)) | (crc_en ? SX1276_CRC_ON : SX1276_CRC_OFF));
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
	write(SX1276_REG_SYMBTIMEOUT_LSB, timeout_lsb);
	write(SX1276_REG_MODEMCONFIG_2, (read(SX1276_REG_MODEMCONFIG_2) & (~SX1276_MODEMCFG_MASK_SYMTIMEOUTMSB)) | timeout_msb);

	return true;
}

// RegPreambleMsb, RegPreambleLsb
void RFM97_LoRa::setPreambleLen(uint16_t preamble_len = 0x08) {
	write(SX1276_REG_PREAMBLE_LSB, (uint8_t)(preamble_len & 0xFF));
	write(SX1276_REG_PREAMBLE_MSB, (uint8_t)((preamble_len & 0xFF00) >> 8));
}

// LnaGain
bool RFM97_LoRa::setGain(uint8_t gain = 1) {
	ASSURE(gain > 0 && gain <= 6);

	write(SX1276_REG_LNA, (read(SX1276_REG_LNA) & (~SX1276_LNA_MASK_GAIN)) | (gain << 5));

	return true;
}

// LnaBoostHf
void RFM97_LoRa::setLNABoostHF(bool boost = false) {
	write(SX1276_REG_LNA, (read(SX1276_REG_LNA) & (~SX1276_LNA_MASK_BOOSTHF)) | (boost ? 0b11 : 0));
}

// AgcAutoOn
void RFM97_LoRa::setAGC(bool agc = 0) {
	write(SX1276_REG_MODEMCONFIG_3, (read(SX1276_REG_MODEMCONFIG_3) & (~SX1276_MODEMCFG_MASK_AGC)) | (agc ? SX1276_AGC_ON : SX1276_AGC_OFF));
}

// LowDataRateOptimize
void RFM97_LoRa::setLDRO(bool ldro = 0) {
	write(SX1276_REG_MODEMCONFIG_3, (read(SX1276_REG_MODEMCONFIG_3) & (~SX1276_MODEMCFG_MASK_LDRO)) | (ldro ? SX1276_LDRO_ON : SX1276_LDRO_OFF));
}

// HighBWOptimize1
// See errata note section 2.1
// NOTE: Automatically enabled by setBandwidth(SX1276_BANDWIDTH_500KHZ)
void RFM97_LoRa::enableHBWO() {
	write(0x36, 0x02);
	write(0x3A, 0x64);
}

// Dio[0-5]Mapping
bool RFM97_LoRa::configDIO(uint8_t dio, uint8_t mode) {
	ASSURE(dio < 6 && mode < 3);
	// Either DioMapping1 or DioMapping2
	if (dio < 4) {
		uint8_t shift = (3 - dio) * 2;
		write(SX1276_REG_DIOMAPPING1, (read(SX1276_REG_DIOMAPPING1) & ~(0b11 << shift)) | (mode << shift));
	} else {
		uint8_t shift = (3 - (dio - 4)) * 2;
		write(SX1276_REG_DIOMAPPING2, (read(SX1276_REG_DIOMAPPING2) & ~(0b11 << shift)) | (mode << shift));
	}

	return true;
}

// DetectionThreshold
// TODO
void RFM97_LoRa::setDetectionThreshold() {

}

// SyncWord
void RFM97_LoRa::setSyncWord(uint8_t syncword = 0x18) {
	write(SX1276_REG_SYNCWORD, syncword);
}

// InvertIQ RX and TX
// TODO
void RFM97_LoRa::setInvIQ(bool inverted) {

}

// PacketRssi
/// @return the RSSI (dBm) of the last received packet
int16_t RFM97_LoRa::getRSSI() {
	return -157 + read(SX1276_REG_PKTRSSIVALUE);
}

// PacketSnr
/// @return the SNR of the last received packet
float RFM97_LoRa::getSNR() {
	return float(read(SX1276_REG_PKTSNRVALUE)) / 4.0f;
}

// Sets the overcurrent protection limit
void RFM97_LoRa::setOCP(bool enabled, uint8_t trim = 0x0b) {
	/*
		  trim       max current          formula
		0 to 15  |  45 to 120 mA  |  45 + 5*OcpTrim [mA]
		16 to 27 |  130 to 240 mA |  -30 + 10*OcpTrim [mA]
		27+      |  240 mA        |  240 mA
	*/

	write(SX1276_REG_OCP, (enabled ? SX1276_OCP_MASK_EN : 0x00) | (trim & SX1276_OCP_MASK_TRIM));
}

// Called by a ISR on DIO0
// Updates the radio's state machine based on radio IRQs
void RFM97_LoRa::ISR_A(uint gpio, uint32_t events) {
	if (gpio == dio3) {
		ISR_C();
		return;
	} else {
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
			case RFM97_RadioState::RX_IN_PROGRESS:
				if (read(SX1276_REG_IRQFLAGS) & (1 << 5)) {
					clearIRQ();
					// CRC error!!!
				} else {
					clearIRQ();
					receive();
				}
				break;
			default:
				return;
		};
	}
}

// Called from ISR a
void RFM97_LoRa::ISR_C() {
	clearIRQ();
	if (state == RFM97_RadioState::RX_WAITING) {
		state = RFM97_RadioState::RX_IN_PROGRESS;
	}
}

void RFM97_LoRa::ISR_B() {
	hw_clear_bits(&dma_hw->ints0, 1u << dma_RX || 1u << dma_TX);
	gpio_put(cs, 1);
	SPI_RELEASE;

	if (state == RFM97_RadioState::RX_DMA) {
		state = RFM97_RadioState::RX_FINISHED;

	} else if (state == RFM97_RadioState::TX_DMA) {
		setMode(SX1276_MODE_TX); // Transmit
		state = RFM97_RadioState::TX_WAITING;
	}
}

void RFM97_LoRa::startReceiving() {
	// wait_for_safe_state();

	standby();
	state = RFM97_RadioState::RX_WAITING;
	configDIO(0, SX1276_DIO0_RX_DONE);
	configDIO(3, SX1276_DIO3_VALID_HEADER);
	setMode(SX1276_MODE_RXCONTINUOUS);
}

bool RFM97_LoRa::transmit(uint8_t* data, size_t len, bool return_to_rx = true) {
	ASSURE(len > 0 && len <= 256);
	tx_state_go_rx = return_to_rx;

	// Wait for safe state to transmit, no way around it, maybe add a fail-poll mode?
	wait_for_safe_state();

	standby();
	ASSURE(configDIO(0, SX1276_DIO0_TX_DONE));
	clearIRQ();

	write(SX1276_REG_PAYLOADLENGTH, (uint8_t)len);

	if (use_dma) {
		writeFIFO_DMA(data, len);
		state = RFM97_RadioState::TX_DMA;
		return true; // ISR_B handles transmit
	} else {
		writeFIFO(data, len);
		state = RFM97_RadioState::TX_WAITING;
		setMode(SX1276_MODE_TX); // Transmit
	}

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
	lastRxLen = read(SX1276_REG_RX_NB_BYTES);

	if (use_dma) {
		readFIFO_DMA(rxbuf, lastRxLen);
		state = RFM97_RadioState::RX_DMA;
	} else {
		readFIFO((uint8_t*)rxbuf, lastRxLen);
		state = RFM97_RadioState::RX_FINISHED;
	}
}

bool RFM97_LoRa::applyConfig(RFM97_LoRa_config cfg) {
	ASSURE(setFreq(cfg.frf));
	ASSURE(setBandwidth(cfg.bw_mode));
	// ASSURE(setCodingRate(cfg.cr));
	ASSURE(setSpreadingFactor(cfg.sf));
	ASSURE(setPower(cfg.power, true));
	return true;
}

void RFM97_LoRa::wait_for_safe_state() {
	int timeout = 1000 / 5;
	while (!(state == RFM97_RadioState::RX_WAITING || state == RFM97_RadioState::STANDBY) && timeout) {
		// __wfe();
		sleep_ms(5);
		// tud_cdc_n_write_str(ITF_VGPS, "safe locked\n");
		// tud_cdc_n_write_flush(ITF_VGPS);
		timeout--;
	};
}