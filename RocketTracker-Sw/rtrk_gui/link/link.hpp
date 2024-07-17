#ifndef LINK_H
#define LINK_H

#include "boost/asio.hpp"
#include "boost/system.hpp"
#include "boost/signals2.hpp"
#include "protocol.pb.h"
#include "link/frame_manager_v2.hpp"

#define USB_SERIAL_BUF_SIZE 1024*5
#define USB_SER_ESC 0xFF
#define USB_SER_ESC_ESC 0x01
#define USB_SER_ESC_NULL 0x02

class Link {
public:
	boost::signals2::signal<void(DatumTypeID id, std::shared_ptr<std::vector<uint8_t>> payload)> rx_signal;

	virtual void send_frame(FrameManager2* frame) = 0;
};

class SerialLink : public Link {
private:
	boost::asio::serial_port* main_port;
	boost::asio::io_service io;

	boost::asio::deadline_timer keepalive_timer;

	uint8_t buffer[USB_SERIAL_BUF_SIZE];

	void keep_alive(const boost::system::error_code& error);

public:
	SerialLink();

	std::vector<std::string> get_available_ports();

	bool open_port(std::string portname);

	void close();

	void rx_callback(const boost::system::error_code& error, std::size_t n);

	void frame_rx_callback(uint16_t length); // Stored in buffer

	void run();

	void send_frame(FrameManager2* frame);

	~SerialLink();
};

#endif