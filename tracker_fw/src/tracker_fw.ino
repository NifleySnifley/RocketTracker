#include <Arduino.h>
#include <RadioLib.h>
#include <Adafruit_GPS.h>

#include <pb_common.h>
#include <pb_decode.h>
#include <pb_encode.h>

#include "protocol.h"
#include "radio.h"
#include "messages.pb.h"
#include "configuration.h"
#include "errors.h"

#define GPS_serial Serial3
#define DEBUG 1 
#define ASSERT_ERR(x,e) if (!(x)) { return e; }
#define CHECK_ERR(e) if (err != ERR_NONE) { handleError(e, __FILE__, __LINE__); }

// RFM97 has the following connections:
// CS pin:    10
// DIO0 pin:  2
// RESET pin: 3
RFM97_LoRa radio = RFM97_LoRa(10, 2, 3);
Adafruit_GPS GPS(&GPS_serial);

GPSData lastGPSreading;

// Global configuration table for storing nonvolatile configuration data
Configuration config;

void radio_ISR(void) {
    radio.onInterrupt();
}

void setup() {
#if DEBUG 
    cleanConfig();
#endif
    readConfig(&config);

    // Serial can be used for higher speed communication using the same protocol as the radio
    Serial.begin(115200);
    while (!Serial) {}

    // Configure GPS
    if (!GPS.begin(9600))
        Serial.println("Error initializing GPS!");
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

    // Configure radio
    if (radio.init())
        Serial.println("Radio initialized successfully");
    else Serial.println("Radio error");
    radio.setISR(radio_ISR);
    radio.startReceiving();
}

void printGPS() {
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);

    Serial.print("\nTime: ");
    if (GPS.hour < 10) { Serial.print('0'); }
    Serial.print(GPS.hour, DEC); Serial.print(':');
    if (GPS.minute < 10) { Serial.print('0'); }
    Serial.print(GPS.minute, DEC); Serial.print(':');
    if (GPS.seconds < 10) { Serial.print('0'); }
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    if (GPS.milliseconds < 10) {
        Serial.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
        Serial.print("0");
    }
    Serial.println(GPS.milliseconds);
}

RadioMessage rx_buf;
RadioMessage tx_buf;

void loop() {
    static uint32_t lastSend = 0, lastRecv = 0;
    uint32_t now = millis();

    // Parse GPS message(or at least try to)
    if (GPS_serial.available() > 0) {
        GPS.read();
        if (GPS.newNMEAreceived()) {
            GPS.parse(GPS.lastNMEA());
            // printGPS();
            lastGPSreading = {
                .latitude = (double)GPS.latitudeDegrees,
                .longitude = (double)GPS.longitudeDegrees,
                .altitude = (double)GPS.altitude,
            };
        }
    }

    if ((millis() - lastSend) > 2000) {
        Serial.println("Sending message");

        LocationData loc;
        loc.gps_reading = lastGPSreading;
        // TODO: Add more sensors
        loc.has_magnetometer_reading = false;
        loc.has_altitude = false;

        pb_ostream_t stream = pb_ostream_from_buffer(tx_buf.data, sizeof(tx_buf.data));
        pb_encode(&stream, LocationData_fields, &loc);
        tx_buf.message_type = MSGTYPE_BROADCAST;
        tx_buf.message_category = BM_POSITION_BROADCAST;
        // Log other data, transmit messages, etc.

        radio.transmit((uint8_t*)&tx_buf, sizeof(RadioMessage) - 1, true);

        lastSend = millis();
    }

    if (radio.messageAvailable()) {
        // Serial.println("Received a message!");
        // Serial.print("RSSI: "); Serial.println(radio.getRSSI());
        // Serial.print("SNR: "); Serial.println(radio.getSNR());

        // Crude deserialization of the messagge into the standardized packet format
        memcpy(radio.rxbuf, (void*)&rx_buf, min(radio.lastRxLen, sizeof(RadioMessage) - 1));

        // Process message
        uint16_t err = messageReceived(rx_buf, radio.getRSSI(), radio.getSNR());
        CHECK_ERR(err);
    }
}

// Called when a message is received
uint16_t messageReceived(const RadioMessage& message, int RSSI, float SNR) {
    // Ignore messages sent by other trackers
    if (message.sender == 1) return;

    // If the message isn't a broadcast (ID of 0), ignore it if it doesn't have this tracker's ID.
    if (message.tracker_id != 0 && message.tracker_id != config.id) return;

    switch (message.message_type) {
        case MSGTYPE_CONFIG_GET:
            pb_ostream_t stream = pb_ostream_from_buffer(tx_buf.data, sizeof(tx_buf.data));
            pb_encode(&stream, Configuration_fields, &config);

            tx_buf.message_type = MSGTYPE_CONFIG_GET;
            tx_buf.message_category = CM_REPLY;

            // Reply with the current configuration
            radio.transmit((uint8_t*)&tx_buf, sizeof(RadioMessage) - 1, true);
            break;
        case MSGTYPE_CONFIG_SET:
            if (message.message_category == CM_ASK) {
                // Decode sent config
                pb_istream_t stream = pb_istream_from_buffer(message.data, sizeof(message.data));
                ASSERT_ERR(pb_decode(&stream, Configuration_fields, &config), ERR_INVALID_PROTOBUF);
                writeConfig(config);
            }
            break;
        default:
            break;
    }

    return ERR_NONE;
}

void handleError(uint16_t err, const char* file, uint32_t line) {
    Serial.print("ERROR #"); Serial.print(err); Serial.print(" in "); Serial.print(file); Serial.print(" at line "); Serial.println(line);
}