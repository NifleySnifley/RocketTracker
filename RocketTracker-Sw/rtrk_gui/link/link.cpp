#include <boost/system.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <string>
#include <vector>
#include <iostream>
#include "link.hpp"
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include "windows.h"
#endif

#include "frame_manager_v2.hpp"

using namespace boost;

#ifdef _WIN32
std::vector<std::string> enumerate_serial_ports() {
	std::vector<std::string> serialList;
	std::string COMName("COM"), queryName("");
	CHAR bufferTragetPath[5000];
	std::string tmp;
	DWORD path_size(0);

	for (int i(0); i < 255; i++) {
		queryName = COMName + std::to_string(i);

		path_size = QueryDosDeviceA(queryName.c_str(), bufferTragetPath, 5000);
		if (path_size != 0) {
			serialList.push_back(queryName);
		}
	}
	return serialList;
}
#endif
// TODO: Implement for *NIX

SerialLink::SerialLink() : io(0), keepalive_timer(io) {
	this->main_port = new asio::serial_port(this->io);
}

std::vector<std::string> SerialLink::get_available_ports() {
	return enumerate_serial_ports();
}

void SerialLink::rx_callback(const boost::system::error_code& error, std::size_t n) {
	static int usb_recv_frameidx = 0;
	// 0 read len MSB, 4 read len LSB, 1 reading data, 2 esc, 3 done UNUSED
	static int usb_recv_state = 0;
	static int usb_recv_state_ret = 0;
	static uint16_t frame_size;
	static uint8_t val_unesc;

	if (error) {
		std::cerr << error.to_string() << std::endl;
	} else {
		std::cout << "Read " << n << " bytes" << std::endl;


		for (int i = 0; i < n;++i) {
			uint8_t byte = buffer[i];

			// Reset FSM on zero byte!
			if (byte == 0) {
				usb_recv_frameidx = 0;
				usb_recv_state = 0;
				continue;
			}

			// On esc, save state and go to escape state
			if (byte == USB_SER_ESC) {
				usb_recv_state_ret = usb_recv_state;
				usb_recv_state = 2;
				// printf("ESC\n");
				continue;
				// If in esc state, set val unesc to the unescaped!
			} else if (usb_recv_state == 2) {
				val_unesc = (byte == USB_SER_ESC_ESC) ? USB_SER_ESC : 0;
				usb_recv_state = usb_recv_state_ret; // Pop state
				// printf("UNESC: %d\n", val_unesc);
			} else {
				// Otherwise the value is just the byte
				val_unesc = byte;
			}

			switch (usb_recv_state) {
				// Receive MSB of frame length
				case 0:
					frame_size = val_unesc;

					usb_recv_state = 4;
					break;
				case 4:
					frame_size |= (uint16_t)val_unesc << 8;

					usb_recv_state = 1;
					usb_recv_frameidx = 0;
					// ESP_LOGI("LINK_USB", "Got frame size: %d", frame_size);
					break;

				case 1:
					// ESP_LOGI("LINK_USB", "buffer[%d] = %d", usb_recv_frameidx, val_unesc);
					buffer[usb_recv_frameidx++] = val_unesc;
					if (usb_recv_frameidx >= frame_size) {
						usb_recv_state = 3;

						this->frame_rx_callback(frame_size);

						usb_recv_state = 0;
						usb_recv_frameidx = 0;
					}
					break;
			}
		}

		// Start reading again
		this->main_port->async_read_some(
			boost::asio::buffer(this->buffer),
			boost::bind(
				&SerialLink::rx_callback,
				this,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred
			)
		);
	}
}

void SerialLink::frame_rx_callback(uint16_t n) {
	std::cout << "Received frame, length = " << n << std::endl;
	FrameManager2 fmgr(this->buffer, (size_t)n);
	for (const auto& [a, b] : fmgr.get_datums()) {
		this->rx_signal(a, b);
	}
}

SerialLink::~SerialLink() {
	if (this->main_port != nullptr && this->main_port->is_open()) {
		this->main_port->close();
	}
}

void SerialLink::run() {
	this->io.run();
}

void SerialLink::keep_alive(const boost::system::error_code& error) {
	uint8_t alive[2] = { 0 };
	this->main_port->async_write_some(
		asio::buffer(alive, sizeof(alive)),
		[this](boost::system::error_code ec, std::size_t /*length*/) {
			if (ec) {
				std::cout << "Error writing to serial port" << std::endl;
			}
		}
	);

	// TODO: Implement sending zero bytes to keepalive
	this->keepalive_timer.expires_at(this->keepalive_timer.expires_at() + boost::posix_time::milliseconds(250));
	this->keepalive_timer.async_wait(boost::bind(
		&SerialLink::keep_alive,
		this,
		boost::asio::placeholders::error
	));
}

void SerialLink::send_frame(FrameManager2* frame) {
	uint8_t* data = 0;
	size_t len = 0;
	data = frame->get_data(&len);

	this->main_port->async_write_some(
		asio::buffer(data, len),
		[this](boost::system::error_code ec, std::size_t /*length*/) {
			if (ec) {
				std::cout << "Error writing frame to serial port" << std::endl;
			}
		}
	);
}

bool SerialLink::open_port(std::string port) {
	if (this->main_port->is_open()) this->main_port->close();
	try {
		this->main_port->open(port);
		// FIXME: Need to flush input!
		this->main_port->async_read_some(
			boost::asio::buffer(this->buffer),
			boost::bind(
				&SerialLink::rx_callback,
				this,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred
			)
		);
		// Start keeping the port alive
		this->keepalive_timer.expires_at(boost::posix_time::microsec_clock::local_time() + boost::posix_time::milliseconds(250));
		this->keepalive_timer.async_wait(boost::bind(
			&SerialLink::keep_alive,
			this,
			boost::asio::placeholders::error
		));
		return true;
	}
	catch (...) {
		return false;
	}
}

void SerialLink::close() {
	if (this->main_port->is_open()) {
		this->main_port->close();
	}
}