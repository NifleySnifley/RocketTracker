#ifndef LINK_MENU_H
#define LINK_MENU_H

#include "menu_interface.hpp"
#include <boost/asio.hpp>
#include <boost/system.hpp>

enum LinkType {
	ReceiverUSB,
	TrackerUSB
};

class LinkMenu : public IMenu {
private:
	boost::asio::serial_port* main_port;
	boost::asio::serial_port* secondary_port;
	boost::asio::io_service io;

	std::vector<std::string> available_ports = std::vector<std::string>();

	LinkType type = ReceiverUSB;

	bool is_main_connected = false, is_secondary_connected = false;

	uint8_t buffer[5000];

public:
	LinkMenu();

	void update_serialports();

	void deinit();
	void init();

	void rx_callback(const boost::system::error_code& error, std::size_t n);

	void render_gui(bool* open);

	~LinkMenu();
};

#endif