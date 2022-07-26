#include <RadioLib.h>
#include <SPI.h>

class RFM97_LoRa : public RFM97 {
private:
    bool txflag, rxflag;

public:
    uint8_t data[256];

    RFM97_LoRa(Module* mod) : RFM97(mod) {}

    bool init() {
        reset();
        // 915.0, 125.0, 7U, 5U, 18, 20, 8U, 0

        /*
            915.0,  // Freq
            125.0,  // 125.0,  // Bandwidth
            7U,    // Spreading factor
            5U,     // Coding rate denominator
            0x12,   // LoRa Sync word
            10,     // TX Power
            8U,     // Preamble length
            0       // Gain (0 for AGC)
        */

        return begin(
            915.0, 125.0, 7U, 5U, 18, 20, 8U, 0
        ) == RADIOLIB_ERR_NONE;
    }

    // Enables interrupt on DIO0 when a message is fully received
    int16_t enableRXInterrupt() {
        return _mod->SPIsetRegValue(RADIOLIB_SX127X_REG_DIO_MAPPING_1, RADIOLIB_SX127X_DIO0_PACK_PAYLOAD_READY, 7, 6);
    }

    int16_t transmit(uint8_t* data, size_t len) {
        txflag = true;
        int status = RFM97::transmit(data, len, 0);
        startReceive(255, RADIOLIB_SX127X_RXCONTINUOUS);
        return status;
    }

    bool received() {
        if (rxflag) {
            rxflag = false;
            return true;
        }
        return false;
    };

    bool transmitted() {
        if (txflag) {
            txflag = false;
            return true;
        }
        return false;
    };

    void detectTransmit() {
        txflag = true;
    }

    void detectReceive() {
        rxflag = true;
    }
};

// class RFM97_LoRa {
// private:
//     bool received() {

//     }

//     bool transmitted() {

//     }

//     void write(uint8_t reg, uint8_t value) {

//     }

//     void write(uint8_t start, uint8_t* dataptr, size_t len) {

//     }

//     uint8_t read(uint8_t reg) {

//     }

//     void read(uint8_t start, uint8_t* dataout, size_t len) {

//     }


// public:
//     RFM97_LoRa(int CS, int DIO0, int RST) {

//     }

//     bool init() {

//     }

//     void onInterrupt() {

//     }

//     void transmit(uint8_t* data, size_t len) {

//     }

//     void receive(uint8_t* buffer, uint8_t* len) {

//     }
// };