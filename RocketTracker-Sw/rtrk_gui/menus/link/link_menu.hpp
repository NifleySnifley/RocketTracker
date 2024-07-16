#ifndef LINK_MENU_H
#define LINK_MENU_H

#include "menu_interface.hpp"
#include <boost/asio.hpp>
#include <boost/system.hpp>
#include <boost/signals2.hpp>
#include <tuple>
#include "protocol.pb.h"
#include "frame_manager_v2.hpp"

#define USB_SERIAL_BUF_SIZE 1024*5
#define USB_SER_ESC 0xFF
#define USB_SER_ESC_ESC 0x01
#define USB_SER_ESC_NULL 0x02


// TODO: Seperate "Link" from a "connection menu"
class LinkMenu : public IMenu {
private:
	boost::asio::serial_port* main_port;
	boost::asio::serial_port* secondary_port;
	boost::asio::io_service io;

	boost::signals2::signal<void(DatumTypeID id, void* datum)> rx_signal;

	std::vector<std::string> available_ports = std::vector<std::string>();

	bool is_main_connected = false, is_secondary_connected = false;

	uint8_t buffer[5000];

public:
	LinkMenu();

	void update_serialports();

	void deinit();
	void init();

	void rx_callback(const boost::system::error_code& error, std::size_t n);

	void frame_rx_callback(uint16_t length); // Stored in buffer

	void render_gui(bool* open);

	~LinkMenu();
};

#endif