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
using namespace std::placeholders;



#ifdef _WIN32

HANDLE open_serial_port(std::string portname, uint32_t baud_rate) {
	std::string device = "\\\\.\\" + portname;

	HANDLE port = CreateFileA(device.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL,
		OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
	if (port == INVALID_HANDLE_VALUE)
	{
		std::cerr << device << std::endl;;
		return INVALID_HANDLE_VALUE;
	}

	// Flush away any bytes previously read or written.
	BOOL success = FlushFileBuffers(port);
	if (!success)
	{
		std::cerr << "Failed to flush serial port" << std::endl;
		CloseHandle(port);
		return INVALID_HANDLE_VALUE;
	}

	// Configure read and write operations to time out after 0 ms.
	COMMTIMEOUTS timeouts = { 0 };
	timeouts.ReadIntervalTimeout = MAXDWORD;
	timeouts.ReadTotalTimeoutConstant = 0;
	timeouts.ReadTotalTimeoutMultiplier = 0;
	timeouts.WriteTotalTimeoutConstant = 0;
	timeouts.WriteTotalTimeoutMultiplier = 0;

	success = SetCommTimeouts(port, &timeouts);
	if (!success)
	{
		std::cerr << "Failed to set serial timeouts" << std::endl;
		CloseHandle(port);
		return INVALID_HANDLE_VALUE;
	}

	// Set the baud rate and other options.
	DCB state = { 0 };
	state.DCBlength = sizeof(DCB);
	state.BaudRate = baud_rate;
	state.ByteSize = 8;
	state.Parity = NOPARITY;
	state.StopBits = ONESTOPBIT;
	success = SetCommState(port, &state);
	if (!success)
	{
		std::cerr << "Failed to set serial settings" << std::endl;
		CloseHandle(port);
		return INVALID_HANDLE_VALUE;
	}

	return port;
}

SSIZE_T read_port(HANDLE port, uint8_t* buffer, size_t size)
{
	DWORD received;
	BOOL success = ReadFile(port, buffer, size, &received, NULL);
	if (!success)
	{
		std::cerr << "Failed to read from port" << std::endl;
		return -1;
	}
	return received;
}

int write_port(HANDLE port, uint8_t* buffer, size_t size)
{
	DWORD written;
	BOOL success = WriteFile(port, buffer, size, &written, NULL);
	if (!success)
	{
		std::cerr << "Failed to write to port"<<std::endl;
		return -1;
	}
	if (written != size)
	{
		std::cerr << "Failed to write all bytes to port" <<std::endl;
		return -1;
	}
	return 0;
}

bool close_port(HANDLE port) {
	return CloseHandle(port);
}

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
	//this->main_port = new asio::serial_port(this->io);
	this->rx_buf = std::vector<uint8_t>();
	this->serial_port = nullptr;
}

std::vector<std::string> SerialLink::get_available_ports() {
	return enumerate_serial_ports();
}

void SerialLink::rx_callback(size_t n) {
	//std::cout << "Read " << n << " bytes" << std::endl;
	for (int i = 0; i < n; ++i) {
		uint8_t byte = this->serial_buffer[i];
		if (byte == 0) {
			if (this->rx_buf.size() > 0) {
				// Actual data has arrived!
				std::vector<uint8_t> decoded_frame = cobs_decode(this->rx_buf.data(), this->rx_buf.size());
				if (decoded_frame.size() < 6) {
					std::cerr << "Corrupted frame data received!" << std::endl;
					this->rx_buf.clear();
					continue;
				}
				uint16_t length_decoded = decoded_frame[0] | (decoded_frame[1] << 8);
				if (decoded_frame.size() == (length_decoded + sizeof(uint16_t))) {
					//std::cout << "Received frame, length = " << length_decoded << std::endl;
					frame_rx_callback(decoded_frame.data() + 2, decoded_frame.size() - 2);
				}
			}
			// After reading, clear!
			this->rx_buf.clear();
		} else {
			this->rx_buf.push_back(byte);
		}
	}


		// Start reading again
		/*this->main_port->async_read_some(
			boost::asio::buffer(this->serial_buffer),
			boost::bind(
				&SerialLink::rx_callback,
				this,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred
			)
		);*/
	
}

void SerialLink::frame_rx_callback(uint8_t* data, size_t size) {
	FrameManager2 fmgr(data, size);
	for (const auto& [a, b] : fmgr.get_datums()) {
		//std::cout << a << std::endl;
		this->rx_signal(a, b);
	}
	std::memset(this->serial_buffer, 0, sizeof(this->serial_buffer));
}

SerialLink::~SerialLink() {
	/*if (this->main_port != nullptr && this->main_port->is_open()) {
		this->main_port->close();
	}*/
	//this->thread_stop = true;
	//this->io_thread.join();
}

void SerialLink::run() {
	this->io.run();

	if (this->serial_port != nullptr) {
		while (1) {
			SSIZE_T n = read_port(this->serial_port, this->serial_buffer, sizeof(this->serial_buffer));
			if (n > 0) {
				this->rx_callback((size_t)n);
			}
			else {
				break;
			}
		}
	}

	// if (this->main_port->is_open()) {
	// 	size_t n = this->main_port->read_some(asio::buffer(this->buffer));
	// 	// this->rx_callback(, n);
	// }

}

void SerialLink::keep_alive(const boost::system::error_code& error) {
	uint8_t alive[2] = { 0 };
	if (this->serial_port != nullptr) {
		write_port(this->serial_port, alive, sizeof(alive));
	}
	/*this->main_port->async_write_some(
		asio::buffer(alive, sizeof(alive)),
		[this](boost::system::error_code ec, std::size_t) {
			if (ec) {
				std::cout << "Error writing to serial port" << std::endl;
			}
		}
	);*/

	// TODO: Implement sending zero bytes to keepalive
	this->keepalive_timer.expires_at(this->keepalive_timer.expires_at() + boost::posix_time::milliseconds(250));
	this->keepalive_timer.async_wait(boost::bind(
		&SerialLink::keep_alive,
		this,
		boost::asio::placeholders::error
	));
}

void hexdump(void* ptr, int buflen) {
	unsigned char* buf = (unsigned char*)ptr;
	int i, j;
	for (i = 0; i < buflen; i += 16) {
		printf("%06x: ", i);
		for (j = 0; j < 16; j++)
			if (i + j < buflen)
				printf("%02x ", buf[i + j]);
			else
				printf("   ");
		printf(" ");
		for (j = 0; j < 16; j++)
			if (i + j < buflen)
				printf("%c", isprint(buf[i + j]) ? buf[i + j] : '.');
		printf("\n");
	}
}

std::vector<uint8_t> cobs_encode(uint8_t* data, size_t len) {
	std::vector<uint8_t> encoded = std::vector<uint8_t>();
	encoded.reserve(len);

	for (int i = 0; i < len; ++i) {
		uint8_t b = data[i];
		if (b == USB_SER_ESC) {
			encoded.push_back(USB_SER_ESC);
			encoded.push_back(USB_SER_ESC_ESC);
		} else if (b == 0) {
			encoded.push_back(USB_SER_ESC);
			encoded.push_back(USB_SER_ESC_NULL);
		} else {
			encoded.push_back(b);
		}
	}

	return encoded;
}

std::vector<uint8_t> cobs_decode(uint8_t* data, size_t len) {
	std::vector<uint8_t> decoded = std::vector<uint8_t>();
	decoded.reserve(len);

	size_t i = 0;
	while (i < len) {
		if (data[i] == USB_SER_ESC) {
			decoded.push_back(data[i + 1] == USB_SER_ESC_ESC ? USB_SER_ESC : 0x00);
			i += 2;
		} else {
			decoded.push_back(data[i]);
			i += 1;
		}
	}

	return decoded;
}

void SerialLink::send_frame(FrameManager2* frame) {
	size_t frame_len = 0;
	uint8_t* frame_data = frame->get_data(&frame_len);

	std::vector<uint8_t> encoded_frame = cobs_encode(frame_data, frame_len);

	uint8_t frame_len_raw[2];
	frame_len_raw[0] = frame_len & 0xFF;
	frame_len_raw[1] = (frame_len >> 8) & 0xFF;

	std::vector<uint8_t> len_encoded_cobs = cobs_encode(frame_len_raw, 2);

	size_t buf_size = 4 + len_encoded_cobs.size() + encoded_frame.size();
	uint8_t* sendbuf = (uint8_t*)malloc(buf_size);

	sendbuf[0] = 0x00;
	sendbuf[1] = 0x00;
	sendbuf[buf_size - 1] = 0x00;
	sendbuf[buf_size - 2] = 0x00;

	std::memcpy(&sendbuf[2], len_encoded_cobs.data(), len_encoded_cobs.size());
	std::memcpy(&sendbuf[2 + len_encoded_cobs.size()], encoded_frame.data(), encoded_frame.size());

	if (this->serial_port != nullptr) {
		size_t n = write_port(this->serial_port, sendbuf, buf_size);
		std::cout << "Wrote " << n << " bytes." << std::endl;
	}
	
	/*this->main_port->async_write_some(
		asio::buffer(sendbuf, buf_size),
		[this, sendbuf](boost::system::error_code ec, std::size_t) {
			if (ec) {
				std::cout << "Error writing frame to serial port" << std::endl;
			}
			free(sendbuf);
		}
	);*/
}

bool SerialLink::open_port(std::string port) {
	this->serial_port = open_serial_port(port, 1000000);

	this->keepalive_timer.expires_at(boost::posix_time::microsec_clock::local_time() + boost::posix_time::milliseconds(250));
	this->keepalive_timer.async_wait(boost::bind(
		&SerialLink::keep_alive,
		this,
		boost::asio::placeholders::error
	));
	return true;
}

void SerialLink::close() {
	// CLOSE!!!
	if (this->serial_port != nullptr) {
		close_port(this->serial_port);
		this->serial_port = nullptr;
	}
}