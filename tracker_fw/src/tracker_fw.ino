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
// RFM97_LoRa_RL radio = new Module(10, 2, 3);
RFM97_LoRa radio = RFM97_LoRa(10, 2, 3);
Adafruit_GPS GPS(&GPS_serial);

GPSData lastGPSreading;

// Global configuration table for storing nonvolatile configuration data
Configuration config;

uint8_t data[256];

void radio_ISR(void) {
    radio.onInterrupt();
}

void setup() {
#if DEBUG 
    cleanConfig();
#endif
    readConfig(&config);

    Serial.begin(115200);
    // while (!Serial) {}

    if (!GPS.begin(9600))
        Serial.println("Error initializing GPS!");

    if (radio.init())
        Serial.println("Radio initialized successfully");
    else Serial.println("Radio error");
    radio.setISR(radio_ISR);
    radio.startReceiving();

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
        loc.has_magnetometer_reading = false;

        pb_ostream_t stream = pb_ostream_from_buffer(radio_buf.data, sizeof(radio_buf.data));
        pb_encode(&stream, LocationData_fields, &loc);
        // Log other data, transmit messages, etc.

        // Idk, this is bad
        // radio.transmit((uint8_t*)&radio_buf, sizeof(radio_buf));
        data[0] = data[1] = data[2] = data[3] = 0xFF;
        for (int i = 4; i < 255; ++i)
            data[i] = 'E';

        radio.transmit(data, 255, true);
        lastSend = millis();
    }

    if (radio.messageAvailable()) {
        Serial.println("Received a message!");
        Serial.print("RSSI: "); Serial.println(radio.getRSSI());
        Serial.print("SNR: "); Serial.println(radio.getSNR());
    }
}
