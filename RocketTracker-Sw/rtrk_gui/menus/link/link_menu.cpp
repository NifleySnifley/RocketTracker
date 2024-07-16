#include "link/link_menu.hpp"
#include "imgui.h"
#include <boost/system.hpp>
#include <boost/bind.hpp>
#include <string>
#include <vector>
#include <iostream>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include "windows.h"
#endif

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

using namespace boost;

const char* link_types[2] = {
	"Receiver (USB)",
	"Tracker (USB)"
};

LinkMenu::LinkMenu() : io(4) {
	this->main_port = new asio::serial_port(this->io);
	// this->secondary_port = new asio::serial_port(this->io);
}

void LinkMenu::update_serialports() {
	this->available_ports.clear();
	this->available_ports.push_back("<Select Port>");
	std::vector<std::string> ports = enumerate_serial_ports();
	this->available_ports.insert(available_ports.end(), ports.begin(), ports.end());
}

void LinkMenu::rx_callback(const boost::system::error_code& error, std::size_t n) {
	if (error) {
		std::cerr << error.to_string() << std::endl;
	} else {
		std::cout << "Read " << n << " bytes" << std::endl;


		// Start reading again
		this->main_port->async_read_some(
			boost::asio::buffer(this->buffer),
			boost::bind(
				&LinkMenu::rx_callback,
				this,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred
			)
		);
	}
}

void LinkMenu::init() {
	update_serialports();
}

void LinkMenu::render_gui(bool* isopen) {
	static int port_idx = 0;
	this->io.run();

	if (*isopen) {
		ImGui::Begin("Link Configuration", isopen);

		// ImGui::Combo("Connection Type", (int*)&this->type, link_types, IM_ARRAYSIZE(link_types));

		if (ImGui::Button("Refresh Ports")) {
			update_serialports();
		}

		const char* combo_preview_value = this->available_ports[port_idx].c_str();

		bool newport = false;

		if (ImGui::BeginCombo("Serial Port", combo_preview_value, 0)) {
			for (int n = 0; n < this->available_ports.size(); n++) {
				const bool is_selected = (port_idx == n);
				if (ImGui::Selectable(this->available_ports[n].c_str(), is_selected)) {
					port_idx = n;
					newport = true;
				}

				// Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
				if (is_selected)
					ImGui::SetItemDefaultFocus();
			}
			ImGui::EndCombo();
		}

		if (newport && port_idx != 0) {
			if (this->main_port->is_open()) this->main_port->close();
			this->main_port->open(this->available_ports[port_idx]);
			this->main_port->async_read_some(
				boost::asio::buffer(this->buffer),
				boost::bind(
					&LinkMenu::rx_callback,
					this,
					boost::asio::placeholders::error,
					boost::asio::placeholders::bytes_transferred
				)
			);
		}

		ImGui::End();
	}
}

void LinkMenu::deinit() {
	if (this->main_port != nullptr && this->main_port->is_open()) {
		this->main_port->close();
	}
}

LinkMenu::~LinkMenu() {
	delete this->main_port;
}