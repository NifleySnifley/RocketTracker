#ifndef DEBUG_VIEW_MENU_H
#define DEBUG_VIEW_MENU_H

#include "menu_interface.hpp"
#include <boost/asio.hpp>
#include <boost/system.hpp>
#include <boost/signals2.hpp>
#include <tuple>
#include "protocol.pb.h"
#include "link/frame_manager_v2.hpp"
#include "link/link.hpp"
#include "imgui.h"

class LogDisplay {
private:
	ImGuiTextBuffer Buf;
	ImGuiTextFilter Filter;
	ImVector<int> LineOffsets; // Index to lines offset. We maintain this with AddLog() calls.
	
public:
	bool AutoScroll = true;  // Keep scrolling if already at the bottom.

	LogDisplay();

	void Clear();

	void AddLog(const char* fmt, ...);

	void Draw(const char* title, bool* p_open = NULL);
};

enum DatumDisplayType {
	Bytes,
	Json
};

class DebugViewer : public IMenu {
private:
	Link* link;
	LogDisplay log;
	DatumDisplayType disptype;

public:
	DebugViewer(Link* link);

	void datum_callback(DatumTypeID id, std::shared_ptr<std::vector<uint8_t>> payload);

	void deinit();

	void init();

	void render_gui(bool* open);

	// ~DebugViewer();
};

#endif