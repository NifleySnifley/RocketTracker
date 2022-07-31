#include <SPI.h>

// class RFM97_LoRa_RL : public RFM97 {
// private:
//     bool txflag, rxflag;

// public:
//     uint8_t data[256];

//     RFM97_LoRa_RL(Module* mod) : RFM97(mod) {}

//     bool init() {
//         reset();
//         // 915.0, 125.0, 7U, 5U, 18, 20, 8U, 0

//         /*
//             915.0,  // Freq
//             125.0,  // 125.0,  // Bandwidth
//             7U,    // Spreading factor
//             5U,     // Coding rate denominator
//             0x12,   // LoRa Sync word
//             10,     // TX Power
//             8U,     // Preamble length
//             0       // Gain (0 for AGC)
//         */

//         return begin(
//             915.0, 125.0, 7U, 5U, 18, 20, 8U, 0
//         ) == RADIOLIB_ERR_NONE;
//     }

//     // Enables interrupt on DIO0 when a message is fully received
//     int16_t enableRXInterrupt() {
//         return _mod->SPIsetRegValue(RADIOLIB_SX127X_REG_DIO_MAPPING_1, RADIOLIB_SX127X_DIO0_PACK_PAYLOAD_READY, 7, 6);
//     }

//     int16_t transmit(uint8_t* data, size_t len) {
//         txflag = true;
//         int status = RFM97::transmit(data, len, 0);
//         startReceive(255, RADIOLIB_SX127X_RXCONTINUOUS);
//         return status;
//     }

//     bool received() {
//         if (rxflag) {
//             rxflag = false;
//             return true;
//         }
//         return false;
//     };

//     bool transmitted() {
//         if (txflag) {
//             txflag = false;
//             return true;
//         }
//         return false;
//     };

//     void detectTransmit() {
//         txflag = true;
//     }

//     void detectReceive() {
//         rxflag = true;
//     }
// };

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

#define ASSURE(x) if (!(x)) return false;

enum class RFM97_RadioState {
    TX_WAITING,
    TX_FINISHED,
    RX_WAITING,
    RX_FINISHED,
    SLEEP,
    STANDBY,
};

class RFM97_LoRa {
public:
    int cs, rst, dio0;
    SPISettings settings;

    uint8_t rxbuf[256];
    uint8_t lastRxLen;

    volatile RFM97_RadioState state = RFM97_RadioState::SLEEP;

    bool messageAvailable() {
        if (state == RFM97_RadioState::RX_FINISHED) {
            state = RFM97_RadioState::RX_WAITING;
            return true;
        }

        return false;
    }

    void reset() {
        digitalWrite(rst, LOW);
        delay(10);
        digitalWrite(rst, HIGH);
    }

    /// Sets register reg to value
    /// @param reg register address to write to
    /// @param value value to write
    void write(uint8_t reg, uint8_t value) {
        SPI.beginTransaction(settings);
        digitalWrite(cs, LOW);
        SPI.transfer(reg | 0b10000000); // Addr and wnr bit set to 1 for write
        SPI.transfer(value);
        digitalWrite(cs, HIGH);
        SPI.endTransaction();
    }

    /// Writes len bytes stored in dataptr to registers starting at start or into the FIFO
    /// @param start first register address to start writing at
    /// @param dataptr data to write to consecutive addresses
    /// @param len amount of bytes to write
    void write(uint8_t start, uint8_t* dataptr, size_t len) {
        SPI.beginTransaction(settings);
        digitalWrite(cs, LOW);
        SPI.transfer(start | 0b10000000); // Addr and wnr bit set to 1 for write
        for (size_t i = 0; i < len; ++i)
            SPI.transfer(dataptr[i]);
        digitalWrite(cs, HIGH);
        SPI.endTransaction();
    }

    /// Reads from register at address reg
    /// @param reg register address to read from
    /// @return value stored at address
    uint8_t read(uint8_t reg) {
        SPI.beginTransaction(settings);
        digitalWrite(cs, LOW);
        SPI.transfer(reg | (0 << 7)); // Addr and wnr bit set to 0 for read
        uint8_t value = SPI.transfer(0xFF);
        digitalWrite(cs, HIGH);
        SPI.endTransaction();

        return value;
    }

    /// Reads len bytes into dataout starting at address reg
    /// @param start register address to start from
    /// @param dataout buffer to store read bytes into (must be at least len bytes)
    /// @param len number of bytes to read
    void read(uint8_t start, uint8_t* dataout, size_t len) {
        SPI.beginTransaction(settings);
        digitalWrite(cs, LOW);
        SPI.transfer(start | (0 << 7)); // Addr and wnr bit set to 0 for read
        for (size_t i = 0; i < len; ++i)
            dataout[i] = SPI.transfer(0xFF);
        digitalWrite(cs, HIGH);
        SPI.endTransaction();
    }

    /// Writes n bytes to the FIFO
    /// @param bytes bytes to write (must be at least n long)
    /// @param n number of bytes to write
    /// @param offset where to start writing
    /// @return true if parameters are valid, if not, writing is aborted
    bool writeFIFO(uint8_t* bytes, size_t n, uint8_t offset = 0) {
        ASSURE(offset + n < 256);
        ASSURE(n > 0);

        write(SX1276_REG_FIFO_ADDR_PTR, offset);
        write(SX1276_REG_FIFO, bytes, n);

        return true;
    }

    /// Reads n bytes from the FIFO
    /// @param bytes buffer to store the read bytes (must have capacity for n bytes)
    /// @param n number of bytes to read
    /// @param offset where to start reading
    /// @return true if parameters are valid, if not, reading is aborted
    bool readFIFO(uint8_t* bytes, size_t n, uint8_t offset = 0) {
        ASSURE(offset + n < 256);
        ASSURE(offset > 0 && n > 0);

        write(SX1276_REG_FIFO_ADDR_PTR, offset);
        read(SX1276_REG_FIFO, bytes, n);

        return true;
    }

    // Make sure all of the FIFO pointers are correct
    void initFIFO() {
        write(SX1276_REG_FIFO_RX_BASEADDR, 0x00);
        write(SX1276_REG_FIFO_TX_BASEADDR, 0x00);
    }

    RFM97_LoRa(int CS, int DIO0, int RST) : cs(CS), rst(RST), dio0(DIO0), settings(2000000, MSBFIRST, SPI_MODE0) {}

    // LongRangeMode
    bool init() {
        pinMode(cs, OUTPUT);
        pinMode(dio0, INPUT);
        pinMode(rst, OUTPUT);

        SPI.begin();

        reset();

        // Make sure its the right chip and its connected properly...
        ASSURE(read(SX1276_REG_VERSION) == 0x12);

        digitalWrite(cs, HIGH);

        // Sleep and then set into LoRa mode
        write(SX1276_REG_OPMODE, 0b00001000);
        delay(10);
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

    void configure() {
        // No FHSS!
        write(SX1276_REG_HOPPERIOD, 0x00);

        // Enable all interrupts
        write(SX1276_REG_IRQFLAGSMASK, 0x00);

        // Set FIFO pointers
        initFIFO();
    }

    /// @param func ISR function to be called when DIO0 rises. the ISR MUST call onInterrupt() for proper behavior of the radio.
    void setISR(void(*func)(void)) {
        attachInterrupt(digitalPinToInterrupt(dio0), func, RISING);
    }

    void standby() {
        setMode(SX1276_MODE_STDBY);
        state = RFM97_RadioState::STANDBY;
    }

    void sleep() {
        setMode(SX1276_MODE_SLEEP);
        state = RFM97_RadioState::SLEEP;
    }

    // Clears all IRQ flags by writing ones
    void clearIRQ() {
        write(SX1276_REG_IRQFLAGS, 0xFF);
    }

    void setMode(uint8_t modebits) {
        write(SX1276_REG_OPMODE, (read(SX1276_REG_OPMODE) & (~SX1276_OPMODE_MASK_MODE)) | modebits);
    }

    // Frf
    bool setFreq(uint32_t freq_mhz) {
        // TODO: Fix this with more realistic limits
        ASSURE(freq_mhz < 1000.0 && freq_mhz > 400.0);

        uint32_t FRF = (freq_mhz * (uint32_t(1) << 19)) / 32.0;

        // write registers
        write(SX1276_REG_FRF_MSB, (FRF & 0xFF0000) >> 16);
        write(SX1276_REG_FRF_MID, (FRF & 0x00FF00) >> 8);
        write(SX1276_REG_FRF_LSB, FRF & 0x0000FF);

        return true;
    }

    // PaSelect and PaConfig
    bool setPower(uint8_t power, bool pa_boost) {
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
    bool setBandwidth(uint32_t bw_mode) {
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
    void setHeaderMode(bool implicit = false) {
        write(SX1276_REG_MODEMCONFIG_1, (read(SX1276_REG_MODEMCONFIG_1) & (~SX1276_MODEMCFG_MASK_HEADERMODE)) | implicit);
    }

    // SpreadingFactor
    bool setSpreadingFactor(uint8_t spreading_factor = 7) {
        ASSURE(spreading_factor > 6 && spreading_factor <= 12);

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
    bool setCodingRate(uint8_t cr_denom = 5) {
        ASSURE(cr_denom >= 5 && cr_denom <= 8);

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
    void setCrcMode(bool crc_en = false) {
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
    bool setTimeout(uint16_t timeout_symbols = 0x64) {
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
    void setPreambleLen(uint16_t preamble_len = 0x08) {
        write(SX1276_REG_PREAMBLE_LSB, (uint8_t)(preamble_len & 0xFF));
        write(SX1276_REG_PREAMBLE_MSB, (uint8_t)((preamble_len & 0xFF00) >> 8));
    }

    // LnaGain
    bool setGain(uint8_t gain = 1) {
        ASSURE(gain > 0 && gain <= 6);

        write(SX1276_REG_LNA, (read(SX1276_REG_LNA) & (~SX1276_LNA_MASK_GAIN)) | (gain << 5));

        return true;
    }

    // LnaBoostHf
    void setLNABoostHF(bool boost = false) {
        write(SX1276_REG_LNA, (read(SX1276_REG_LNA) & (~SX1276_LNA_MASK_BOOSTHF)) | (boost ? 0b11 : 0));
    }

    // AgcAutoOn
    void setAGC(bool agc = 0) {
        write(SX1276_REG_MODEMCONFIG_3, (read(SX1276_REG_MODEMCONFIG_3) & (~SX1276_MODEMCFG_MASK_AGC)) | (agc ? SX1276_AGC_ON : SX1276_AGC_OFF));
    }

    // LowDataRateOptimize
    void setLDRO(bool ldro = 0) {
        write(SX1276_REG_MODEMCONFIG_3, (read(SX1276_REG_MODEMCONFIG_3) & (~SX1276_MODEMCFG_MASK_LDRO)) | (ldro ? SX1276_LDRO_ON : SX1276_LDRO_OFF));
    }

    // HighBWOptimize1
    // See errata note section 2.1
    // NOTE: Automatically enabled by setBandwidth(SX1276_BANDWIDTH_500KHZ)
    void enableHBWO() {
        write(0x36, 0x02);
        write(0x3A, 0x64);
    }

    // Dio[0-5]Mapping
    bool configDIO(uint8_t dio, uint8_t mode) {
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
    void setDetectionThreshold() {

    }

    // SyncWord
    void setSyncWord(uint8_t syncword = 0x18) {
        write(SX1276_REG_SYNCWORD, syncword);
    }

    // InvertIQ RX and TX
    // TODO
    void setInvIQ(bool inverted) {

    }

    // PacketRssi
    /// @return the RSSI (dBm) of the last received packet
    int16_t getRSSI() {
        return -157 + read(SX1276_REG_PKTRSSIVALUE);
    }

    // PacketSnr
    /// @return the SNR of the last received packet
    float getSNR() {
        return float(read(SX1276_REG_PKTSNRVALUE)) / 4.0f;
    }

    // Sets the overcurrent protection limit
    void setOCP(bool enabled, uint8_t trim = 0x0b) {
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
    void onInterrupt() {
        switch (state) {
            case RFM97_RadioState::TX_WAITING:
                state = RFM97_RadioState::TX_FINISHED;
                break;
                // case RFM97_RadioState::RX_FINISHED:
            case RFM97_RadioState::RX_WAITING:
                receive();
                break;
            default:
                return;
        };
        clearIRQ();
    }

    void startReceiving() {
        standby();
        state = RFM97_RadioState::RX_WAITING;
        configDIO(0, SX1276_DIO0_RX_DONE);
        setMode(SX1276_MODE_RXCONTINUOUS);
    }

    bool transmit(uint8_t* data, size_t len, bool return_to_rx = false) {
        ASSURE(len > 0 && len < 256);

        standby();
        clearIRQ();

        write(SX1276_REG_PAYLOADLENGTH, (uint8_t)len);
        writeFIFO(data, len, 0);

        ASSURE(configDIO(0, SX1276_DIO0_TX_DONE));
        clearIRQ();

        state = RFM97_RadioState::TX_WAITING;
        setMode(SX1276_MODE_TX);

        // Wait for the transmit done IRQ
        while (state != RFM97_RadioState::TX_FINISHED) {}

        clearIRQ();
        standby();
        state = RFM97_RadioState::STANDBY;

        if (return_to_rx) startReceiving();

        return true;
    }

    // Doesn't actually receive, but collects recently received data
    void receive() {
        lastRxLen = read(SX1276_REG_RX_NB_BYTES);
        readFIFO(rxbuf, lastRxLen);
        state = RFM97_RadioState::RX_FINISHED;
    }

    /// @return true if a message was successfully received in the timeout period, false if timed out
    // TODO: Fix using the real-time modem status register and/or IRQ flags register and the symbol timeout
    // bool waitForRX(uint32_t timeout_ms = -1) {
    //     state = RFM97_RadioState::RX_WAITING;
    //     setMode(SX1276_MODE_RXSINGLE);
    //     configDIO(0, SX1276_DIO0_RX_DONE);

    //     uint32_t time_started = millis();
    //     while (state != RFM97_RadioState::RX_FINISHED) {
    //         // Timed out
    //         if ((millis() - time_started) >= timeout_ms) {
    //             standby();
    //             return false;
    //         }
    //     };
    // }
};