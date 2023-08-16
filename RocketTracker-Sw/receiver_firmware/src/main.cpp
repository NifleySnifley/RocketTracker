#define PICO_BOOT_STAGE2_CHOOSE_W25Q080 1
// #define PICO_BOOT_STAGE2_CHOOSE_GENERIC_03H 1
#define PICO_FLASH_SIZE_BYTES (4 * 1024 * 1024)
#define PICO_FLASH_SPI_CLKDIV 2
#define PICO_XOSC_STARTUP_DELAY_MULTIPLIER 64

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

RFM97_LoRa radio(spi0, RFM97CW_CS, RFM97CW_DIO0, RFM97CW_RST, RFM97CW_MOSI, RFM97CW_MISO, RFM97CW_SCK);

uint8_t payload[] = "Hello World";

float radio_snr;
int radio_rssi;
auto_init_mutex(rdata_mutex);

void dio0_isr(uint gpio, uint32_t events) {
	radio.onInterrupt(gpio, events);
}

#define MS_SEND 2000
#define MS_LED 100
#define MSG_TXT "Testing sending some data!"

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
			snprintf(&textbuf[0], 64, "RSSI: %d", radio_rssi);
			snprintf(&textbuf[64], 64, "SNR: %0.2f", radio_snr);
		}
		mutex_exit(&rdata_mutex);
		ssd1306_draw_string(&display, 0, 0, 1, &textbuf[0]);
		ssd1306_draw_string(&display, 0, 10, 1, &textbuf[64]);
		ssd1306_show(&display);
	}
}

int main() {
	stdio_init_all();

	multicore_launch_core1(core2);

	gpio_init(PIN_LED_R);
	gpio_set_dir(PIN_LED_R, GPIO_OUT);
	gpio_init(PIN_LED_G);
	gpio_set_dir(PIN_LED_G, GPIO_OUT);

	radio.init();
	radio.setISR(dio0_isr);
	radio.startReceiving();
	radio.setFreq(915.0);

	absolute_time_t ledr_d = get_absolute_time(), ledg_d = get_absolute_time(), send_d = make_timeout_time_ms(MS_SEND);
	while (true) {
		absolute_time_t t = get_absolute_time();

		if (absolute_time_diff_us(get_absolute_time(), send_d) < 0) {
			Raw_Datum msg = (Raw_Datum){
				{strlen(MSG_TXT), MSG_TXT}
			}; // (I hate C++ inline struct initialization)

			uint8_t buffer[256];
			pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
			pb_encode(&stream, Raw_Datum_fields, &msg);

			printf("Sending... ");
			radio.transmit(payload, sizeof(payload), true);
			printf("Sent!\n");
			send_d = make_timeout_time_ms(MS_SEND);

			ledr_d = make_timeout_time_ms(MS_LED);
		}

		if (radio.messageAvailable()) {
			mutex_enter_blocking(&rdata_mutex);
			{
				radio_snr = radio.getSNR();
				radio_rssi = radio.getRSSI();
			}
			mutex_exit(&rdata_mutex);

			printf("Packet (%d) received, SNR: %f, RSSI: %d\n", radio.lastRxLen, radio.getSNR(), radio.getRSSI());
			printf("Hex: ");
			for (int i = 0; i < radio.lastRxLen; ++i)
				printf("%02x ", radio.rxbuf[i]);
			printf("\n");
			printf("String: %.*s", radio.lastRxLen, radio.rxbuf);
			printf("\n");
			ledg_d = make_timeout_time_ms(100);
		}

		t = get_absolute_time();
		gpio_put(PIN_LED_R, absolute_time_diff_us(get_absolute_time(), ledr_d) > 0);
		gpio_put(PIN_LED_G, absolute_time_diff_us(get_absolute_time(), ledg_d) > 0);
	}

	return 0;
}