#define PICO_BOOT_STAGE2_CHOOSE_W25Q080 1
#define PICO_FLASH_SIZE_BYTES (4 * 1024 * 1024)
#define PICO_FLASH_SPI_CLKDIV 2
#define PICO_XOSC_STARTUP_DELAY_MULTIPLIER 64

#include "bsp/board.h"
#include "tusb.h"

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "comms/radio.h"
#include "pinout.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "display/ssd1306.h"
#include <pb_encode.h>
#include <pb_decode.h>
#include "protocol.pb.h"
#include "comms/comms_lib/frame_manager.h"
#include "tusb.h"
#include "math.h"
#include "vgps.h"
#include "global.h"

static const RFM97_LoRa_config RFM97_CFG_DEFAULT{
	.frf = 914.0,
	.power = 20,
	.sf = 7,
	.cr = 5,
	.bw_mode = SX1276_BANDWIDTH_125KHZ
};

RFM97_LoRa radio(spi0, RFM97CW_CS, RFM97CW_DIO0, RFM97CW_DIO3, RFM97CW_RST, RFM97CW_MOSI, RFM97CW_MISO, RFM97CW_SCK, true);
// FrameManager lora_fmg;
// FrameManager usb_fmg;
uint8_t fmgr_buf1[256] = { 0 };
FrameManager fmg(fmgr_buf1, 256);
RFM97_LoRa_config radioconfig = RFM97_CFG_DEFAULT;
bool radioconfig_updated = false;

absolute_time_t gps_last_update = at_the_end_of_time;
bool gps_fresh = false;
GPS current_gps;

RadioRxStatus radio_stat;
auto_init_mutex(rdata_mutex);

absolute_time_t ledr_d = get_absolute_time(), ledg_d = get_absolute_time(), send_d = get_absolute_time();

void ISR_A(uint gpio, uint32_t events) {
	// gpio_put(PIN_LED_G, 1);
	radio.ISR_A(gpio, events);
}

void ISR_B() {
	// gpio_put(PIN_LED_G, 1);
	radio.ISR_B();
}

#define MS_SEND 2000
#define MS_LED 100
#define MS_VGPS_NMEA 1500 // 1.5 seconds
#define GPS_STALE_US 10 * 1000 * 1000 // 10 seconds

// Runs on core 2, display/UI
void core2() {
	ssd1306_t display;

	gpio_init(PIN_SDA0);
	gpio_set_function(PIN_SDA0, GPIO_FUNC_I2C);
	gpio_pull_up(PIN_SDA0);

	gpio_init(PIN_SCL0);
	gpio_set_function(PIN_SCL0, GPIO_FUNC_I2C);
	gpio_pull_up(PIN_SCL0);

	i2c_init(i2c0, 400000);

	display.external_vcc = false;
	if (!ssd1306_init(&display, 128, 32, 0x3C, i2c0)) {
		while (1) {}
	}
	ssd1306_clear(&display);

	char textbuf[128];
	for (;;) {
		ssd1306_clear(&display);
		mutex_enter_blocking(&rdata_mutex);
		{
			snprintf(&textbuf[0], 64, "RSSI: %d", radio_stat.RSSI);
			snprintf(&textbuf[64], 64, "SNR: %0.2f", radio_stat.SNR);
		}
		mutex_exit(&rdata_mutex);
		ssd1306_draw_string(&display, 0, 0, 1, &textbuf[0]);
		ssd1306_draw_string(&display, 0, 10, 1, &textbuf[64]);
		ssd1306_show(&display);
	}
}

#define ESC 0xFF
#define ESC_ESC 0x01
#define ESC_NULL 0x02

void write_b_esc(uint8_t b) {
	if (b == 0) {
		tud_cdc_n_write_char(ITF_TELEM, ESC);
		tud_cdc_n_write_char(ITF_TELEM, ESC_NULL);
	} else if (b == ESC) {
		tud_cdc_n_write_char(ITF_TELEM, ESC);
		tud_cdc_n_write_char(ITF_TELEM, ESC_ESC);
	} else {
		tud_cdc_n_write_char(ITF_TELEM, b);
	}
}

void write_frame_raw(uint8_t* frame_data, int len) {
	uint8_t l = len;
	// 16-bit length
	write_b_esc(l);
	write_b_esc(0);

	for (int i = 0; i < len; ++i)
		write_b_esc(frame_data[i]);

	for (int i = 0; i < 10; ++i)
		tud_cdc_n_write_char(ITF_TELEM, '\0');

	tud_cdc_n_write_flush(ITF_TELEM);
}

uint8_t telem_txbuf[256];
uint16_t telem_txsize = 0;
void telem_rx_cb() {
	static int telem_txidx = 0;
	// 0 ready, 1 reading data, 2 esc, 3 done UNUSED
	static int telem_rawtx_state = 0;
	static int telem_rawtx_state_ret = 0;
	static uint8_t val_unesc;

	uint8_t byte = (uint8_t)tud_cdc_n_read_char(ITF_TELEM);

	// Reset FSM on zero byte!
	if (byte == 0) {
		telem_txidx = 0;
		telem_rawtx_state = 0;
		telem_txidx = 0;
		// printf("Zero rst\r\n");
		return;
	}

	// On esc, save state and go to escape state
	if (byte == ESC) {
		telem_rawtx_state_ret = telem_rawtx_state;
		telem_rawtx_state = 2;
		// printf("ESC\n");
		return;
		// If in esc state, set val unesc to the unescaped!
	} else if (telem_rawtx_state == 2) {
		val_unesc = (byte == ESC_ESC) ? ESC : 0;
		telem_rawtx_state = telem_rawtx_state_ret; // Pop state
		// printf("UNESC: %d\n", val_unesc);
	} else {
		// Otherwise the value is just the byte
		val_unesc = byte;
	}

	switch (telem_rawtx_state) {
		case 0:
			telem_txsize = val_unesc;

			telem_rawtx_state = 4;
			telem_txidx = 0;
			// printf("Got len %d\n", telem_txsize);
			break;
		case 4:
			telem_txsize |= ((uint16_t)val_unesc) << 8;

			telem_rawtx_state = 1;
			telem_txidx = 0;
			// printf("Got len %d\n", telem_txsize);
			break;
		case 1:
			if (telem_txidx >= sizeof(telem_txbuf))
				return;
			// printf("Read byte %d to idx %d\n", val_unesc, telem_txidx);
			telem_txbuf[telem_txidx++] = val_unesc;
			if (telem_txidx >= telem_txsize) {
				// telem_rawtx_state = 3;

				// tud_cdc_n_write_str(ITF_VGPS, "tx start\n");
				radio.transmit(telem_txbuf, telem_txsize, true);
				// printf("Send %d bytes\n", telem_txsize);
				// write_frame_raw(telem_txbuf, telem_txsize);
				ledr_d = make_timeout_time_ms(MS_LED);
				telem_rawtx_state = 0;
				// tud_cdc_n_write_str(ITF_VGPS, "tx done\n");
				// tud_cdc_n_write_flush(ITF_VGPS);

				telem_txidx = 0;
				// printf("Message full!\n");
			}
			break;
	}
}

void read_datum(int i, DatumTypeID id, int len, uint8_t* data) {
	if (id == DatumTypeID_INFO_GPS) {
		pb_istream_t stream = pb_istream_from_buffer(data, len);
		pb_decode(&stream, GPS_fields, &current_gps);
		gps_fresh = true;
	}
}

void test_end() {
	while (1) {
		gpio_put(PIN_LED_G, 1);
		tud_task();
		sleep_ms(1);
	}
}

int main() {
	// FrameManager fmg = FrameManager(256);
	// lora_fmg = FrameManager(256);
	// usb_fmg = FrameManager(5000);
	tud_init(BOARD_TUD_RHPORT);
	stdio_usb_init();

	// multicore_launch_core1(core2);

	gpio_init(PIN_LED_R);
	gpio_set_dir(PIN_LED_R, GPIO_OUT);
	gpio_init(PIN_LED_G);
	gpio_set_dir(PIN_LED_G, GPIO_OUT);
	gpio_put(PIN_LED_G, 0);
	gpio_put(PIN_LED_R, 1);

	radio.init();
	radio.setISRA(ISR_A);
	radio.setISRB(ISR_B);
	radio.setPower(20, true); // Low-er power for testing :)
	radio.setFreq(914.0);

	radio.applyConfig(radioconfig);

	radio.startReceiving();

	while (true) {
		tud_task();
		absolute_time_t t = get_absolute_time();

		if (radioconfig_updated) {
			radioconfig_updated = false;
			// char dbuf[128];
			// snprintf(dbuf, sizeof(dbuf), "%f, %d, %d\r\n", radioconfig.frf, radioconfig.sf, radioconfig.cr);
			// tud_cdc_n_write_str(ITF_VGPS, dbuf);
			// tud_cdc_n_write_flush(ITF_VGPS);

			radio.wait_for_safe_state();
			radio.standby();
			radio.applyConfig(radioconfig);
			radio.initFIFO();
			radio.startReceiving();
		}

		// if (absolute_time_diff_us(t, send_d) < 0) {
		// 	GPS_Info gpsinfo;
		// 	gpsinfo.alt = 200.0;
		// 	gpsinfo.fix_status = 1;
		// 	gpsinfo.utc_time = 161229487;
		// 	gpsinfo.lat = 46.24897362189417;
		// 	gpsinfo.lon = -92.43390039973657;
		// 	gpsinfo.has_fix_status = true;
		// 	gpsinfo.has_sats_used = false;

		// 	Altitude_Info altinfo;
		// 	altinfo.alt_m = 500.65;
		// 	altinfo.v_speed = 1002.6;
		// 	altinfo.has_v_speed = true;

		// 	Orientation_Info orientinfo;
		// 	orientinfo.orientation_x = 1.41;
		// 	orientinfo.orientation_y = 1.41;
		// 	orientinfo.orientation_z = 1.41;

		// 	Battery_Info battinfo;
		// 	battinfo.battery_voltage = 3.65;
		// 	battinfo.charging = false;

		// 	fmg.reset();
		// 	fmg.encode_datum(MessageTypeID_TLM_GPS_Info, GPS_Info_fields, &gpsinfo);
		// 	fmg.encode_datum(MessageTypeID_TLM_Altitude_Info, Altitude_Info_fields, &altinfo);
		// 	fmg.encode_datum(MessageTypeID_TLM_Orientation_Info, Orientation_Info_fields, &orientinfo);
		// 	fmg.encode_datum(MessageTypeID_TLM_Battery_Info, Battery_Info_fields, &battinfo);

		// 	int s = 0;
		// 	uint8_t* data = fmg.get_frame(&s);

		// 	write_frame_raw(data, s);
		// 	radio.transmit(data, s, true);

		// 	send_d = make_timeout_time_ms(MS_SEND);
		// 	ledr_d = make_timeout_time_ms(MS_LED);
		// }

		if (radio.messageAvailable()) {
			fmg.reset();

			fmg.load_frame((uint8_t*)radio.rxbuf, radio.lastRxLen);

			// test_end();

			// radio_stat.crc_valid = true;
			if (fmg.check_crc()) {
				// radio_stat.crc_valid = false;

				// Valid CRC
				fmg.decode_frame(read_datum);
				ledg_d = make_timeout_time_ms(MS_LED);
				write_frame_raw((uint8_t*)radio.rxbuf, radio.lastRxLen);

				// RX Stuff, CRC or not
				mutex_enter_blocking(&rdata_mutex);
				{
					radio_stat.RSSI = radio.getRSSI();
					radio_stat.SNR = radio.getSNR();
				}
				mutex_exit(&rdata_mutex);

				fmg.reset();
				fmg.encode_datum(DatumTypeID_STATUS_RadioRxStatus, RadioRxStatus_fields, &radio_stat);
				int len;
				uint8_t* dat = fmg.get_frame(&len);
				write_frame_raw(dat, len);
			}
		}

		if (gps_fresh && tud_cdc_n_connected(ITF_TELEM)) {
			gps_fresh = false;
			write_fake_gps(&current_gps);
		}

		t = get_absolute_time();
		gpio_put(PIN_LED_R, absolute_time_diff_us(get_absolute_time(), ledr_d) > 0);
		gpio_put(PIN_LED_G, absolute_time_diff_us(get_absolute_time(), ledg_d) > 0);
	}

	return 0;
}