#include "map_menu.hpp"

using namespace std::placeholders;

MapMenu::MapMenu(SerialLink* link) : link(link) {
	link->rx_signal.connect(std::bind(&MapMenu::datum_callback, this, _1, _2));
}

void MapMenu::deinit() {

}

void MapMenu::init() {

}

void MapMenu::render_gui(bool* open) {

}

void MapMenu::datum_callback(DatumTypeID id, std::shared_ptr<std::vector<uint8_t>> payload) {

}

MapMenu::~MapMenu() {

}