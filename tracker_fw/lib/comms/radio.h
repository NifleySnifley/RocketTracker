#include <RadioLib.h>

class RFM97_LoRa : public RFM97 {
private:
    bool txflag, rxflag;

public:
    uint8_t data[256];

    RFM97_LoRa(Module* mod) : RFM97(mod) {}

    bool init() {
        reset();
        // forceLDRO(false);
        // 915.0, 125.0, 7U, 5U, 18, 13, 8U, 0
        return begin(
            915.0,  // Freq
            125.0,  // Bandwidth
            7U,     // Spreading factor
            5U,     // Coding rate denominator
            0x12,     // LoRa Sync word
            13,     // TX Power
            8U,     // Preamble length
            0       // Gain (0 for AGC)
        ) == RADIOLIB_ERR_NONE;
    }

    // Enables interrupt on DIO0 when a message is fully received
    int16_t enableRXInterrupt() {
        return _mod->SPIsetRegValue(RADIOLIB_SX127X_REG_DIO_MAPPING_1, RADIOLIB_SX127X_DIO0_PACK_PAYLOAD_READY, 7, 6);
    }

    int16_t transmit(uint8_t* data, size_t len) {
        txflag = true;
        return RFM97::transmit(data, len, 0);
    }

    bool received() {
        if (rxflag) {
            txflag = false;
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