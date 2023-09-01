#ifndef RADIO_H
#define RADIO_H

#include <hardware/spi.h>
#include <hardware/dma.h>
#include <hardware/irq.h>
#include <hardware/gpio.h>

#define SX1276_REG_FIFO 0x00
#define SX1276_REG_OPMODE 0x01
// RESERVED 0x02
// RESERVED 0x03 
// RESERVED 0x04
// RESERVED 0x05
#define SX1276_REG_FRF_MSB 0x06
#define SX1276_REG_FRF_MID 0x07
#define SX1276_REG_FRF_LSB 0x08
#define SX1276_REG_PACONFIG 0x09
#define SX1276_REG_PARAMP 0x0A
#define SX1276_REG_OCP 0x0B
#define SX1276_REG_LNA 0x0C
#define SX1276_REG_FIFO_ADDR_PTR 0x0D
#define SX1276_REG_FIFO_TX_BASEADDR 0x0E
#define SX1276_REG_FIFO_RX_BASEADDR 0x0F
#define SX1276_REG_FIFO_RX_CURRENTADDR 0x10
#define SX1276_REG_IRQFLAGSMASK 0x11
#define SX1276_REG_IRQFLAGS 0x12
#define SX1276_REG_RX_NB_BYTES 0x13
#define SX1276_REG_RXHEADERCNT_MSB 0x14
#define SX1276_REG_RXHEADERCNT_LSB 0x15
#define SX1276_REG_RXPACKETCNT_MSB 0x16
#define SX1276_REG_RXPACKETCNT_LSB 0x17
#define SX1276_REG_MODEMSTAT 0x18
#define SX1276_REG_PKTSNRVALUE 0x19
#define SX1276_REG_PKTRSSIVALUE 0x1A
#define SX1276_REG_RSSIVALUE 0x1B
#define SX1276_REG_HOPCHANNEL 0x1C
#define SX1276_REG_MODEMCONFIG_1 0x1D
#define SX1276_REG_MODEMCONFIG_2 0x1E
#define SX1276_REG_SYMBTIMEOUT_LSB 0x1F
#define SX1276_REG_PREAMBLE_MSB 0x20
#define SX1276_REG_PREAMBLE_LSB 0x21
#define SX1276_REG_PAYLOADLENGTH 0x22
#define SX1276_REG_MAXPAYLOADLENGTH 0x23
#define SX1276_REG_HOPPERIOD 0x24
#define SX1276_REG_FIFORXBYTE_ADDR 0x25 
#define SX1276_REG_MODEMCONFIG_3 0x26
// RESERVED 0x27
#define SX1276_REG_FEI_MSB 0x28
#define SX1276_REG_FEI_MID 0x29
#define SX1276_REG_FEI_LSB 0x2A
// RESERVED 0x2B
#define SX1276_REG_RSSI_WIDEBAND 0x2C
// RESERVED 0x2D
// RESERVED 0x2E
#define SX1276_REG_IFFREQ1 0x2F
#define SX1276_REG_IFFREQ2 0x30
#define SX1276_REG_DETECTOPTIMIZE 0x31
// RESERVED 0x32
#define SX1276_REG_INVERTIQ 0x33
// RESERVED 0x34
// RESERVED 0x35
#define SX1276_REG_HIGHBWOPTIMIZE1 0x36
#define SX1276_REG_DETECTIONTHRESHOLD 0x37
// RESERVED 0x38
#define SX1276_REG_SYNCWORD 0x39
#define SX1276_REG_HIGHBWOPTIMIZE2 0x3A
#define SX1276_REG_INVERTIQ2 0x3B
// RESERVED 0x3C
// RESERVED 0x3D
// RESERVED 0x3E
// RESERVED 0x3F
#define SX1276_REG_DIOMAPPING1 0x40
#define SX1276_REG_DIOMAPPING2 0x41
#define SX1276_REG_VERSION 0x42
// UNUSED 0x44
#define SX1276_REG_TCXO 0x4B
#define SX1276_REG_PADAC 0x4D
#define SX1276_REG_FORMERTEMP 0x5B
// UNUSED 0x5D
#define SX1276_REG_AGCREF 0x61
#define SX1276_REG_AGCTHRESH1 0x62
#define SX1276_REG_AGCTHRESH2 0x63
#define SX1276_REG_AGCTHRESH3 0x64
#define SX1276_REG_PLL 0x17


#define SX1276_MODE_SLEEP 0b000
#define SX1276_MODE_STDBY 0b001
#define SX1276_MODE_FSTX 0b010
#define SX1276_MODE_TX 0b011
#define SX1276_MODE_FSRX 0b100
#define SX1276_MODE_RXCONTINUOUS 0b101
#define SX1276_MODE_RXSINGLE 0b110
#define SX1276_MODE_CAD 0b111

#define SX1276_PASELECT_PABOOST 0b10000000
#define SX1276_PASELECT_RFO     0b00000000

#define SX1276_PACONFIG_LOWPOWER            0b00100000
#define SX1276_PACONFIG_MAXPOWER            0b01110000

#define SX1276_PADAC_HIGHPOWER 0x87
#define SX1276_PADAC_LOWPOWER  0x84

#define SX1276_TXCONTINUOUS_ON  0b00001000
#define SX1276_TXCONTINUOUS_OFF 0b00000000

#define SX1276_CRC_ON  0b00000100
#define SX1276_CRC_OFF 0b00000000

#define SX1276_LDRO_ON  0b00001000
#define SX1276_LDRO_OFF 0b00000000

#define SX1276_AGC_ON  0b00000100
#define SX1276_AGC_OFF 0b00000000


#define SX1276_OPMODE_MASK_MODE             0b00000111
#define SX1276_OPMODE_MASK_LFMODE           0b00001000
#define SX1276_OPMODE_MASK_SHAREDREGS       0b01000000
#define SX1276_OPMODE_MASK_LORAMODE         0b10000000

#define SX1276_PACONFIG_MASK_PASELECT       0b10000000
#define SX1276_PACONFIG_MASK_MAXPOWER       0b01110000
#define SX1276_PACONFIG_MASK_OUTPUTPOWER    0b00001111

#define SX1276_MODEMCFG_MASK_BANDWIDTH      0b11110000
#define SX1276_MODEMCFG_MASK_CODINGRATE     0b00001110
#define SX1276_MODEMCFG_MASK_HEADERMODE     0b00000001
#define SX1276_MODEMCFG_MASK_SPREADINGFACTOR 0b11110000
#define SX1276_MODEMCFG_MASK_SYMTIMEOUTMSB  0b00000011
#define SX1276_MODEMCFG_MASK_CRC            0b00000100
#define SX1276_MODEMCFG_MASK_TXCONTINUOUS   0b00001000
#define SX1276_MODEMCFG_MASK_LDRO           0b00001000
#define SX1276_MODEMCFG_MASK_AGC            0b00000100

#define SX1276_LNA_MASK_GAIN        0b11100000
#define SX1276_LNA_MASK_BOOSTLF     0b00011000
#define SX1276_LNA_MASK_BOOSTHF     0b00000011

#define SX1276_OCP_MASK_EN      0b00100000
#define SX1276_OCP_MASK_TRIM    0b00011111


#define SX1276_BANDWIDTH_7_8_KHZ        0b0000
#define SX1276_BANDWIDTH_10_4_KHZ       0b0001
#define SX1276_BANDWIDTH_15_6_KHZ       0b0010
#define SX1276_BANDWIDTH_20_8_KHZ       0b0011
#define SX1276_BANDWIDTH_31_25_KHZ      0b0100
#define SX1276_BANDWIDTH_41_7_KHZ       0b0101
#define SX1276_BANDWIDTH_62_5_KHZ       0b0110
#define SX1276_BANDWIDTH_125KHZ         0b0111
#define SX1276_BANDWIDTH_250KHZ         0b1000
#define SX1276_BANDWIDTH_500KHZ         0b1001

#define SX1276_GAIN_MAX 1
#define SX1276_GAIN_MIN 6

#define SX1276_DIO0_RX_DONE 0b00
#define SX1276_DIO0_TX_DONE 0b01
#define SX1276_DIO0_CAD_DONE 0b10

#define SX1276_DIO1_RX_TIMEOUT 0b00
#define SX1276_DIO1_FHSS_CHANGE_CHANNEL 0b01
#define SX1276_DIO1_CAD_DETECTED 0b10

#define SX1276_DIO2_FHSS_CHANGE_CHANNEL 0b00

#define SX1276_DIO3_CAD_DONE 0b00
#define SX1276_DIO3_VALID_HEADER 0b01
#define SX1276_DIO3_PAYLOAD_CRC_ERR 0b10

#define SX1276_DIO4_CAD_DETECTED 0b00
#define SX1276_DIO4_PLL_LOCK 0b01

#define SX1276_DIO5_MODEREADY 0b00
#define SX1276_DIO5_CLKOUT 0b01

#define SX1276_IRQ_CRC_MASK 0b00100000

#define ASSURE(x) if (!(x)) return false;


#define SX1276_FRF_MIN 903.0
#define SX1276_FRF_MAX 927.0
#define SX1276_CR_MIN 5
#define SX1276_CR_MAX 8
#define SX1276_SF_MIN 6
#define SX1276_SF_MAX 12


enum class RFM97_RadioState {
	TX_WAITING,
	TX_DMA,
	TX_FINISHED,
	RX_WAITING,
	RX_DMA,
	RX_FINISHED,
	SLEEP,
	STANDBY,
};

typedef struct RFM97_LoRa_config {
	double frf;
	int power;
	uint8_t sf;
	uint8_t cr;
	uint8_t bw_mode;
} RFM97_LoRa_config;

class RFM97_LoRa {
private:
	// GPIO connections
	int rst, dio0;
	int srx, stx, cs, sck;
	spi_inst_t* spi_inst;
	bool use_dma = true;

	volatile RFM97_RadioState state;

	int dma_RX, dma_TX;
	uint8_t dma_dummy = 0xFF; // Need an address with blank data for reading SPI with DMA
	volatile bool spi_lock;

	bool tx_state_go_rx;

	void wait_spi();

public:
	volatile uint8_t rxbuf[256];
	volatile uint8_t lastRxLen;

	RFM97_LoRa(spi_inst_t* SPI, int CS, int DIO0, int RST, int STX, int SRX, int SCK, bool use_dma = true);

	bool messageAvailable();

	void reset();

	/// Sets register reg to value
	/// @param reg register address to write to
	/// @param value value to write
	void write(uint8_t reg, uint8_t value);

	/// Writes len bytes stored in dataptr to registers starting at start or into the FIFO
	/// @param start first register address to start writing at
	/// @param dataptr data to write to consecutive addresses
	/// @param len amount of bytes to write
	void write(uint8_t start, uint8_t* dataptr, size_t len);

	/// Reads from register at address reg
	/// @param reg register address to read from
	/// @return value stored at address
	uint8_t read(uint8_t reg);

	/// Reads len bytes into dataout starting at address reg
	/// @param start register address to start from
	/// @param dataout buffer to store read bytes into (must be at least len bytes)
	/// @param len number of bytes to read
	void read(uint8_t start, uint8_t* dataout, size_t len);

	/// Writes n bytes to the FIFO
	/// @param bytes bytes to write (must be at least n long)
	/// @param n number of bytes to write
	/// @param offset where to start writing
	/// @return true if parameters are valid, if not, writing is aborted
	bool writeFIFO(uint8_t* bytes, size_t n);

	bool writeFIFO_DMA(uint8_t* bytes, size_t n);

	/// Reads n bytes from the FIFO
	/// @param bytes buffer to store the read bytes (must have capacity for n bytes)
	/// @param n number of bytes to read
	/// @param offset where to start reading
	/// @return true if parameters are valid, if not, reading is aborted
	bool readFIFO(uint8_t* bytes, size_t n);

	bool readFIFO_DMA(volatile uint8_t* bytes, size_t n);

	// Make sure all of the FIFO pointers are correct
	void initFIFO();

	bool spiInit(uint baud);

	// LongRangeMode
	bool init();

	void configure();

	/// @param func ISR function to be called when DIO0 rises. the ISR MUST call ISR_A() for proper behavior of the radio.
	void setISRA(gpio_irq_callback_t func);
	/// @param func ISR function to be called when DIO0 rises. the ISR MUST call ISR_B() for proper behavior of the radio.
	void setISRB(irq_handler_t func);

	void standby();

	void sleep();

	// Clears all IRQ flags by writing ones
	void clearIRQ();

	void setMode(uint8_t modebits);

	// Frf
	bool setFreq(double freq_mhz);

	// PaSelect and PaConfig
	bool setPower(uint8_t power, bool pa_boost);

	// Bw
	bool setBandwidth(uint32_t bw_mode);

	// ImplicitHeaderModeOn
	void setHeaderMode(bool implicit);

	// SpreadingFactor
	bool setSpreadingFactor(uint8_t spreading_factor);

	// CodingRate
	bool setCodingRate(uint8_t cr_denom);

	// RxPayloadCrcOn
	void setCrcMode(bool crc_en);

	// SymbTimeout
	bool setTimeout(uint16_t timeout_symbols);

	// RegPreambleMsb, RegPreambleLsb
	void setPreambleLen(uint16_t preamble_len);

	// LnaGain
	bool setGain(uint8_t gain);

	// LnaBoostHf
	void setLNABoostHF(bool boost);

	// AgcAutoOn
	void setAGC(bool agc);

	// LowDataRateOptimize
	void setLDRO(bool ldro);

	// HighBWOptimize1
	// See errata note section 2.1
	// NOTE: Automatically enabled by setBandwidth(SX1276_BANDWIDTH_500KHZ)
	void enableHBWO();

	// Dio[0-5]Mapping
	bool configDIO(uint8_t dio, uint8_t mode);

	// DetectionThreshold
	// TODO
	void setDetectionThreshold();

	// SyncWord
	void setSyncWord(uint8_t syncword);

	// InvertIQ RX and TX
	// TODO
	void setInvIQ(bool inverted);

	// PacketRssi
	/// @return the RSSI (dBm) of the last received packet
	int16_t getRSSI();

	// PacketSnr
	/// @return the SNR of the last received packet
	float getSNR();
	// Sets the overcurrent protection limit
	void setOCP(bool enabled, uint8_t trim);

	// Called by a ISR on DIO0
	// Updates the radio's state machine based on radio IRQs
	void ISR_A(uint gpio, uint32_t events);

	// Used by DMA controller
	void ISR_B();

	void startReceiving();

	bool transmit(uint8_t* data, size_t len, bool return_to_rx);

	// Doesn't actually receive, but collects recently received data
	void receive();

	bool applyConfig(RFM97_LoRa_config cfg);

	void wait_for_safe_state();
};

#endif