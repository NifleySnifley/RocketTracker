#include <stdio.h>
#include "frame_manager.h"

int main(int argc, char const* argv[]) {
	GPS_Info gpsinfo;
	gpsinfo.alt = 200.0;
	gpsinfo.fix_status = 1;
	gpsinfo.utc_time = 161229487;
	gpsinfo.lat = 46.24897362189417;
	gpsinfo.lon = -92.43390039973657;
	gpsinfo.has_fix_status = true;
	gpsinfo.has_sats_used = false;

	Altitude_Info altinfo;
	altinfo.alt_m = 500.65;
	altinfo.v_speed = 1002.6;
	altinfo.has_v_speed = true;

	Orientation_Info orientinfo;
	orientinfo.orientation_x = 1.41;
	orientinfo.orientation_y = 1.41;
	orientinfo.orientation_z = 1.41;

	Battery_Info battinfo;
	battinfo.battery_voltage = 3.65;
	battinfo.charging = false;

	FrameManager f;

	f.encode_datum(MessageTypeID_TLM_GPS_Info, GPS_Info_fields, &gpsinfo);
	f.encode_datum(MessageTypeID_TLM_Altitude_Info, Altitude_Info_fields, &altinfo);
	f.encode_datum(MessageTypeID_TLM_Orientation_Info, Orientation_Info_fields, &orientinfo);
	f.encode_datum(MessageTypeID_TLM_Battery_Info, Battery_Info_fields, &battinfo);

	int datalen;
	uint8_t* data = f.get_frame(&datalen);

	FILE* file = fopen("./testmessage.bin", "wb");
	fwrite(data, 1, datalen, file);
	fclose(file);

	printf("Encoded %d datum segments into a frame %d bytes long\n", f.n_datum_encoded, f.cur_frame_len);

	return 0;
}
