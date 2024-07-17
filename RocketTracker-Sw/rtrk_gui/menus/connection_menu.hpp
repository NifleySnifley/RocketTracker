#ifndef LINK_MENU_H
#define LINK_MENU_H

#include "menu_interface.hpp"
#include <boost/asio.hpp>
#include <boost/system.hpp>
#include <boost/signals2.hpp>
#include <tuple>
#include "protocol.pb.h"
#include "link/frame_manager_v2.hpp"
#include "link/link.hpp"


// TODO: Seperate "Link" from a "connection menu"
class ConnectionMenu : public IMenu {
private:
	std::vector<std::string> available_ports = std::vector<std::string>();
	SerialLink* link;

public:
	ConnectionMenu(SerialLink* link);

	void update_serialports();

	void deinit();

	void init();

	void render_gui(bool* open);

	~ConnectionMenu();
};

#endif