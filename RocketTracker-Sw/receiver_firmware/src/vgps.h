#ifndef VGPS_H
#define VGPS_H

#include "protocol.pb.h"
#include "tusb.h"
#include "math.h"
#include <stdint.h>

extern char nmea_buffer[256];
extern char rx_buffer[256];
extern int rx_num;
extern bool vgps_msg;

enum VGPS_CMDTYPE {
	VGPS_SET_FRF,
	VGPS_SET_BW,
	VGPS_SET_POWER,
	VGPS_SET_SF,
	VGPS_SET_CR,
};

#define NMEA_ENDING_LEN 6 // *, Checksum (2), CR, LF, ZERO

int gps_deg_to_dmsint(double degrees, char* nwse_out);

uint8_t nmea_checksum(char* message, int len);

void write_fake_gps(GPS* gps);

void vgps_char_rx_cb(char c);

#endif