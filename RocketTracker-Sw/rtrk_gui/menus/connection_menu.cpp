#include "connection_menu.hpp"
#include "imgui.h"
#include <boost/system.hpp>
#include <boost/bind.hpp>
#include <string>
#include <vector>
#include <iostream>
#include "link/link.hpp"

const char* link_types[2] = {
	"Receiver (USB)",
	"Tracker (USB)"
};

ConnectionMenu::ConnectionMenu(SerialLink* link) : link(link) {
	// this->secondary_port = new asio::serial_port(this->io);
}

void ConnectionMenu::update_serialports() {
	this->available_ports.clear();
	this->available_ports.push_back("<Select Port>");
	std::vector<std::string> ports = this->link->get_available_ports();
	this->available_ports.insert(available_ports.end(), ports.begin(), ports.end());
}

void ConnectionMenu::init() {
	update_serialports();
}

void ConnectionMenu::render_gui(bool* isopen) {
	static size_t port_idx = 0;

	if (*isopen) {
		ImGui::Begin("Connection", isopen);

		// ImGui::Combo("Connection Type", (int*)&this->type, link_types, IM_ARRAYSIZE(link_types));

		if (ImGui::Button("Refresh Ports")) {
			update_serialports();
		}

		port_idx = std::min(port_idx, this->available_ports.size() - 1);

		const char* combo_preview_value = this->available_ports[port_idx].c_str();

		bool newport = false;

		ImGui::Text("Serial Port:");
		if (ImGui::BeginCombo("##Serial Port", combo_preview_value, 0)) {
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
			if (!this->link->open_port(this->available_ports[port_idx])) {
				std::cerr << "Error opening serial port!" << std::endl;
			}
		} else if (port_idx == 0) {
			this->link->close();
		}

		ImGui::End();
	}
}

void ConnectionMenu::deinit() {

}

ConnectionMenu::~ConnectionMenu() {

}