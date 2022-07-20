#include <Arduino.h>
#include <RadioLib.h>
#include <Adafruit_GPS.h>
#include <pb_common.h>
#include <pb_decode.h>
#include <pb_encode.h>

#include "protocol.h"
#include "messages.pb.h"
#include "configuration.pb.h"

#define GPS_serial Serial3

Configuration config;

// RFM97 has the following connections:
// CS pin:    10
// DIO0 pin:  2
// RESET pin: 3
RFM97 radio = new Module(10, 2, 3);
Adafruit_GPS GPS(&GPS_serial);

GPSData lastGPSreading;

void setup() {
    Serial.begin(115200);
    GPS.begin(9600);
    radio.begin();
    radio.setNodeAddress(config.addr);

    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
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

// TODO: Writes config to EEPROM
void writeConfig() {}

// TODO: Reads config from EEPROM or loads the default
void readConfig() {}

RadioMessage radio_buf;

void loop() {
    // Parse GPS message (or at least try to)
    if (GPS_serial.available() > 0) {
        GPS.read();
        if (GPS.newNMEAreceived()) {
            GPS.parse(GPS.lastNMEA());
            printGPS();
            lastGPSreading = {
                .latitude = (double)GPS.latitudeDegrees,
                .longitude = (double)GPS.longitudeDegrees,
                .altitude = (double)GPS.altitude,
            };
        }
    }

    pb_ostream_t stream = pb_ostream_from_buffer(radio_buf.data, sizeof(radio_buf.data));
    pb_encode(&stream, GPSData_fields, &lastGPSreading);
    // Log other data, transmit messages, etc.
}
