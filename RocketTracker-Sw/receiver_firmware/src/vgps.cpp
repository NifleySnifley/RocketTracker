#include "vgps.h"
#include "pinout.h"
#include "global.h"
#include <stdlib.h>
#include "comms/radio.h"

char nmea_buffer[256];
char rx_buffer[256];
int rx_num = 0;
bool vgps_msg = false;

int gps_deg_to_dmsint(double degrees, char* nwse_out) {
	if (degrees < 0) {
		degrees *= -1;
		if (nwse_out != NULL) {
			*nwse_out = (*nwse_out == 'W' || *nwse_out == 'E') ? 'W' : 'S';
		}
	}
	float decimal = fmod(degrees, 1.0);
	int d = round(degrees);
	float m = decimal * 60.0;
	return d * 100 * 100000 + round(m * 100000);
}

uint8_t nmea_checksum(char* message, int len) {
	uint8_t checksum;
	for (int i = 1; i < len; ++i)
		checksum ^= nmea_buffer[i];
	return checksum;
}

void write_fake_gps(GPS_Info* gps) {
	char ns = 'N', ew = 'E';
	int lat_dms = gps_deg_to_dmsint(gps->lat, &ns);
	int lon_dms = gps_deg_to_dmsint(gps->lon, &ew);
	int sentence_len;
	uint8_t checksum;

	// GPGGA
	sentence_len = snprintf(
		nmea_buffer,
		sizeof(nmea_buffer) - NMEA_ENDING_LEN,
		"$GPGGA,%06d,%d.%05d,%c,%05d.%05d,%c,%d,%02d,%.1f,%.1f,M,%.1f,M,,",
		gps->utc_time / 1000,
		// gps->utc_time % 1000,
		lat_dms / 100000,
		lat_dms % 100000,
		ns,
		lon_dms / 100000,
		lon_dms % 100000,
		ew,
		1, // 1 = Fixed
		15,//gps->has_sats_used ? gps->sats_used : 0,
		0.9, // HDOP
		gps->alt,
		0.0 //gps->alt
	);
	nmea_buffer[sentence_len] = '\0';

	checksum = nmea_checksum(nmea_buffer, sentence_len);
	snprintf(&nmea_buffer[sentence_len], NMEA_ENDING_LEN, "*%2X\r\n", checksum);

	tud_cdc_n_write_str(ITF_VGPS, nmea_buffer);
	tud_cdc_n_write_flush(ITF_VGPS);

	// Misc. stuff to make the computer think the GPS has accurate satellite info
	// tud_cdc_n_write_str(ITF_VGPS, "$GPGSA,A,3,11,13,20,18,15,29,05,,,,,,2.26,1.16,1.93*0A\r\n");
	// tud_cdc_n_write_str(ITF_VGPS, "$GPGSV,4,1,15,05,67,048,29,07,00,039,,11,21,093,10,13,39,123,24*70\r\n");
	// tud_cdc_n_write_str(ITF_VGPS, "$GPGSV,4,2,15,15,34,171,21,16,04,333,,18,30,292,25,20,36,058,21*71\r\n");
	// tud_cdc_n_write_str(ITF_VGPS, "$GPGSV,4,3,15,23,13,237,,25,11,220,,26,13,306,11,29,74,253,25*70\r\n");
	// tud_cdc_n_write_str(ITF_VGPS, "$GPGSV,4,4,15,30,00,066,,46,27,227,,48,30,223,*42\r\n");
	// tud_cdc_n_write_flush(ITF_VGPS);

	// GPGLL
	sentence_len = snprintf(
		nmea_buffer,
		sizeof(nmea_buffer) - NMEA_ENDING_LEN,
		"$GPGLL,%d.%05d,%c,%05d.%05d,%c,%06d.%03d,A,A",
		lat_dms / 100000,
		lat_dms % 100000,
		ns,
		lon_dms / 100000,
		lon_dms % 100000,
		ew,
		gps->utc_time / 1000,
		gps->utc_time % 1000
	);

	checksum = nmea_checksum(nmea_buffer, sentence_len);
	snprintf(&nmea_buffer[sentence_len], NMEA_ENDING_LEN, "*%2X\r\n", checksum);

	tud_cdc_n_write_str(ITF_VGPS, nmea_buffer);
	tud_cdc_n_write_flush(ITF_VGPS);

	// GPRMC
	sentence_len = snprintf(
		nmea_buffer,
		sizeof(nmea_buffer) - NMEA_ENDING_LEN,
		"$GPRMC,%06d.%03d,A,%d.%05d,%c,%05d.%05d,%c,,,%d,,,A",
		gps->utc_time / 1000,
		gps->utc_time % 1000,
		lat_dms / 100000,
		lat_dms % 100000,
		ns,
		lon_dms / 100000,
		lon_dms % 100000,
		ew,
		200823 // Date...
	);
	nmea_buffer[sentence_len] = '\0';

	checksum = nmea_checksum(nmea_buffer, sentence_len);
	snprintf(&nmea_buffer[sentence_len], NMEA_ENDING_LEN, "*%2X\r\n", checksum);

	tud_cdc_n_write_str(ITF_VGPS, nmea_buffer);
	tud_cdc_n_write_flush(ITF_VGPS);
}

void vgps_char_rx_cb(char c) {
	if (rx_num == sizeof(rx_buffer)) {
		rx_num == 0; // safety...
	}

	if (c == '$') {
		// NMEA restart short-circuit
		rx_num = 0;
	} else if (c == '\n' || c == '\r') {
		if (rx_num == 0) return;
		// End of message
		// rx_buffer[rx_num++] = '\r';
		// rx_buffer[rx_num++] = '\n';
		rx_buffer[rx_num++] = '\0';

		printf("Parsing: %s\n", rx_buffer);

		char* seg = strtok(rx_buffer, " ,");
		int i = 0;

		VGPS_CMDTYPE ctype;

		while (seg != NULL) {
			if (i == 0) {
				if (strcmp(seg, "PFRF") == 0)
					ctype = VGPS_SET_FRF;
				else if (strcmp(seg, "PSF") == 0)
					ctype = VGPS_SET_SF;
				else if (strcmp(seg, "PCR") == 0)
					ctype = VGPS_SET_CR;
				else if (strcmp(seg, "PPOW") == 0)
					ctype = VGPS_SET_POWER;
				else if (strcmp(seg, "PBW") == 0)
					ctype = VGPS_SET_BW;
				else
					goto lineread_done;
				printf("Ctype = %d\n", ctype);
			} else {
				int iarg;
				double farg;
				switch (ctype) {
					case VGPS_SET_FRF:
						farg = strtod(seg, NULL);
						printf("FRF = %f\n", farg);
						if (farg < SX1276_FRF_MAX && farg > SX1276_FRF_MIN) {
							radioconfig.frf = farg;
							radioconfig_updated = true;
						}
						break;

					case VGPS_SET_BW:
						radioconfig_updated = true;
						printf("BW\n");
						// Oh boy 
						// I'm so sorry...
						if (strcmp(seg, "7.8") == 0) {
							radioconfig.bw_mode = SX1276_BANDWIDTH_7_8_KHZ;
						} else if (strcmp(seg, "10.4") == 0) {
							radioconfig.bw_mode = SX1276_BANDWIDTH_10_4_KHZ;
						} else if (strcmp(seg, "15.6") == 0) {
							radioconfig.bw_mode = SX1276_BANDWIDTH_15_6_KHZ;
						} else if (strcmp(seg, "20.8") == 0) {
							radioconfig.bw_mode = SX1276_BANDWIDTH_20_8_KHZ;
						} else if (strcmp(seg, "31.25") == 0) {
							radioconfig.bw_mode = SX1276_BANDWIDTH_31_25_KHZ;
						} else if (strcmp(seg, "41.7") == 0) {
							radioconfig.bw_mode = SX1276_BANDWIDTH_41_7_KHZ;
						} else if (strcmp(seg, "62.5") == 0) {
							radioconfig.bw_mode = SX1276_BANDWIDTH_62_5_KHZ;
						} else if (strcmp(seg, "125") == 0) {
							radioconfig.bw_mode = SX1276_BANDWIDTH_125KHZ;
						} else if (strcmp(seg, "250") == 0) {
							radioconfig.bw_mode = SX1276_BANDWIDTH_250KHZ;
						} else if (strcmp(seg, "500") == 0) {
							radioconfig.bw_mode = SX1276_BANDWIDTH_500KHZ;
						} else {
							radioconfig_updated = false;
						}
						break;

					case VGPS_SET_CR:
						iarg = atoi(seg);
						printf("CR = %d\n", iarg);
						if (iarg < SX1276_CR_MAX && iarg > SX1276_CR_MIN) {
							radioconfig.cr = iarg;
							radioconfig_updated = true;
						}
						break;

					case VGPS_SET_POWER:
						iarg = atoi(seg);
						printf("POW = %d\n", iarg);
						if ((iarg < 2 && iarg <= 17) || iarg == 20) {
							radioconfig.power = iarg;
							radioconfig_updated = true;
						}
						break;

					case VGPS_SET_SF:
						iarg = atoi(seg);
						printf("SF = %d\n", iarg);
						if (iarg < SX1276_SF_MAX && iarg > SX1276_SF_MIN) {
							radioconfig.sf = iarg;
							radioconfig_updated = true;
						}
						break;

					default:
						break;
				}

				goto lineread_done;
			}

			++i;
			seg = strtok(NULL, " ,");
		}

		// tud_cdc_n_write_str(ITF_TELEM, rx_buffer);
		// tud_cdc_n_write_flush(ITF_TELEM);

	lineread_done:
		rx_num = 0;
	} else {
		// Add another character
		rx_buffer[rx_num++] = c;
	}
}
