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
				&LinkMenu::rx_callback,
				this,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred
			)
		);
	}
}

void LinkMenu::frame_rx_callback(uint16_t n) {
	std::cout << "Received frame, length = " << n << std::endl;
	FrameManager2 fmgr(this->buffer, (size_t)n);
	for (const auto& [a, b] : fmgr.get_datums()) {
		std::cout << "Datum type: " << a << ", Length: " << b->size() << std::endl;
	}
}

void LinkMenu::init() {
	update_serialports();
}

void LinkMenu::render_gui(bool* isopen) {
	static size_t port_idx = 0;
	this->io.run();

	if (*isopen) {
		ImGui::Begin("Link Configuration", isopen);

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
			if (this->main_port->is_open()) this->main_port->close();
			try {
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
			catch (...) {
				std::cerr << "Error opening serial port '" << this->available_ports[port_idx] << "'" << std::endl;
			}
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