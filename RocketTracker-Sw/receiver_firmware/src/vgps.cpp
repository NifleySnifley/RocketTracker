#include "vgps.h"
#include "pinout.h"

char nmea_buffer[256];
char rx_buffer[256];

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