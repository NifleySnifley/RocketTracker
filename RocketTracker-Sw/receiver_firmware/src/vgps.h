#ifndef VGPS_H
#define VGPS_H

#include "protocol.pb.h"
#include "tusb.h"
#include "math.h"
#include <stdint.h>

extern char nmea_buffer[256];
extern char rx_buffer[256];

#define NMEA_ENDING_LEN 6 // *, Checksum (2), CR, LF, ZERO

int gps_deg_to_dmsint(double degrees, char* nwse_out);

uint8_t nmea_checksum(char* message, int len);

void write_fake_gps(GPS_Info* gps);
#endif