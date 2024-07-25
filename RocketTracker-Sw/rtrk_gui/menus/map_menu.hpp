#pragma once

#include "menu_interface.hpp"
#include <boost/signals2.hpp>
#include <tuple>
#include "protocol.pb.h"
#include "link/frame_manager_v2.hpp"
#include "link/link.hpp"
//#include "imosm.h"
//#include "imosm_rich.h"

//namespace ImOsm {
//	namespace Rich {
//		class RichMapPlot;
//		class MarkStorage;
//		class MarkEditorWidget;
//		class DistanceCalcWidget;
//		class DestinationCalcWidget;
//	} // namespace Rich
//} // namespace ImOsm

// TODO: Seperate "Link" from a "connection menu"
class MapMenu : public IMenu {
private:
	SerialLink* link;

	//std::shared_ptr<ImOsm::Rich::RichMapPlot> _mapPlot;
	//std::shared_ptr<ImOsm::Rich::MarkStorage> _markStorage;
	////std::unique_ptr<ImOsm::Rich::MarkEditorWidget> _markEditorWidget;
	////std::unique_ptr<ImOsm::Rich::DistanceCalcWidget> _distanceCalcWidget;
	////std::unique_ptr<ImOsm::Rich::DestinationCalcWidget> _destinationCalcWidget;
	//std::unique_ptr<ImOsm::TileSourceWidget> _tileSourceWidget;
	//std::unique_ptr<ImOsm::TileGrabberWidget> _tileGrabberWidget;

public:
	MapMenu(SerialLink* link);

	void deinit();

	void init();

	void render_gui(bool* open);

	void datum_callback(DatumTypeID id, std::shared_ptr<std::vector<uint8_t>> payload);

	~MapMenu();
};