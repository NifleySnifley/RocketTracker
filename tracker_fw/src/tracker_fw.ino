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

#define GPS_serial Serial3
#define DEBUG 1 

// RFM97 has the following connections:
// CS pin:    10
// DIO0 pin:  2
// RESET pin: 3
RFM97_LoRa radio = new Module(10, 2, 3);
// RFM97_LoRa radio(10, 2, 3);
Adafruit_GPS GPS(&GPS_serial);

GPSData lastGPSreading;

// Global configuration table for storing nonvolatile configuration data
Configuration config;

void recv_ISR(void) {
    if (radio.transmitted()) return;
    radio.detectReceive();
    radio.readData(radio.data, 255);
}

void setup() {
#if DEBUG 
    cleanConfig();
#endif
    readConfig(&config);

    Serial.begin(115200);
    while (!Serial) {}
    if (!GPS.begin(9600))
        Serial.println("Error initializing GPS!");

    if (radio.init())
        Serial.println("Radio initialized successfully");
    radio.enableRXInterrupt();
    radio.setDio0Action(recv_ISR);
    // radio.setOutputPower(17); // Max boosted transmission power
    // radio.forceLDRO(false);
    // radio.explicitHeader();

    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

    Serial.println("Initialization complete!");
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

RadioMessage radio_buf;

void loop() {
    static uint32_t lastSend = 0, lastRecv = 0;
    static bool missed = false;
    uint32_t now = millis();

    // Parse GPS message (or at least try to)
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

    if ((millis() - lastSend) > 10000) {
        Serial.println("Sending message");

        LocationData loc;
        loc.gps_reading = lastGPSreading;
        loc.has_magnetometer_reading = false;

        pb_ostream_t stream = pb_ostream_from_buffer(radio_buf.data, sizeof(radio_buf.data));
        pb_encode(&stream, LocationData_fields, &loc);
        // Log other data, transmit messages, etc.

        // Idk, this is bad
        // radio.transmit((uint8_t*)&radio_buf, sizeof(radio_buf));
        radio.data[0] = radio.data[1] = radio.data[2] = radio.data[3] = 0xFF;
        for (int i = 4; i < 255; ++i)
            radio.data[i] = 'E';

        radio.transmit(radio.data, 255);
        lastSend = millis();
        missed = true;
    }

    // delay(1000);
    if (radio.received()) {
        missed = false;
        lastRecv = millis();
        Serial.println("Received a radio message!");
        Serial.print("RSSI: "); Serial.println(radio.getRSSI());
        Serial.print("SNR: "); Serial.println(radio.getSNR());
        // Serial.println("Data:");
        // for (int i = 0; i < 252; ++i) {
        //     Serial.print("\t");
        //     Serial.println(buf[i], HEX);
        // }
    }

    if ((millis() - lastSend) > 5000 && missed) {
        missed = false;
        Serial.println("Missed acknowledgement!");
    }

    // String str;
    // int state = radio.receive(str);
    // if (state == RADIOLIB_ERR_NONE) {
    //     // packet was successfully received
    //     Serial.println(F("success!"));

    //     // print the data of the packet
    //     Serial.print(F("[RF69] Data:\t\t"));
    //     Serial.println(str);

    //     // print RSSI (Received Signal Strength Indicator)
    //     // of the last received packet
    //     Serial.print(F("[RF69] RSSI:\t\t"));
    //     Serial.print(radio.getRSSI());
    //     Serial.println(F(" dBm"));
    // } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
    //     // timeout occurred while waiting for a packet
    //     Serial.println(F("timeout!"));

    // } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
    //     // packet was received, but is malformed
    //     Serial.println(F("CRC error!"));

    // } else {
    //     // some other error occurred
    //     Serial.print(F("failed, code "));
    //     Serial.println(state);
    // }
}
