#ifndef SENSOR_DATA_VIEWER_H
#define SENSOR_DATA_VIEWER_H

#include "protocol.pb.h"
#include "include_imgui.h"
#include <functional>
#include <boost/system.hpp>
#include <boost/bind.hpp>
#include <string>
#include <vector>
#include <iostream>
#include "link/link.hpp"
#include "menu_interface.hpp"
#include "boost/circular_buffer.hpp"
#include <map>
#include <vector>
#include "main.h"

struct TimeSeries {
	boost::circular_buffer<float> data;
	boost::circular_buffer<float> timestamps;
	std::string name = "unnamed";
	std::string suffix = ""; // Units

	TimeSeries(size_t capacity, std::string name, std::string suffix = "");
	TimeSeries();

	void clear();

	void add_data_now(float datapoint);
	void add_data(float datapoint, float timestamp);
	void record_data(std::string name, float value, std::string suffix);

	void resize(size_t num_datapoints);

	void remove_back(size_t n = 1);
};

class DataViewer : public IMenu {
private:
	Link* link;
	std::map<std::string, TimeSeries> series;
	std::map<std::string, bool> series_enabled;
	int series_size = 12000; // 1 minute of 200hz

public:
	DataViewer(Link* link);

	void datum_callback(DatumTypeID id, std::shared_ptr<std::vector<uint8_t>> payload);

	void record_data(std::string name, float value);
	void record_data(std::string name, float value, std::string suffix);
	void set_units(std::string name, std::string units);

	void deinit();

	void init();

	void set_debug_datarate(uint32_t rate_hz);

	void render_gui(bool* open);
};


#endif