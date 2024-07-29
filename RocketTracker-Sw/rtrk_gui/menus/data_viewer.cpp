#include "data_viewer.hpp"
#include "boost/bind.hpp"
#include "protocol.pb.h"
#include "implot.h"
#include <format>

using namespace boost;

TimeSeries::TimeSeries() {
	this->data = circular_buffer<float>(10);
	this->data = circular_buffer<float>(10);
}

TimeSeries::TimeSeries(size_t size, std::string name, std::string suffix) : data(size), timestamps(size), name(name), suffix(suffix) {

}

void TimeSeries::clear() {
	this->timestamps.clear();
	this->data.clear();
}

void TimeSeries::add_data(float data, float time) {
	this->data.push_back(data);
	this->timestamps.push_back(time);
}

void TimeSeries::resize(size_t num_datapoints) {
	if (this->data.size() > num_datapoints) {
		this->remove_back(this->data.size() - num_datapoints);
	}
	this->data.set_capacity(num_datapoints);
	this->timestamps.set_capacity(num_datapoints);
}

void TimeSeries::add_data_now(float data) {
	posix_time::time_duration diff = posix_time::microsec_clock::local_time() - app_start_time;
	float seconds = diff.total_microseconds() * 1e-6f;
	this->add_data(data, seconds);
}

void TimeSeries::remove_back(size_t n) {
	for (int i = 0; i < n; ++i) {
		this->timestamps.pop_back();
		this->data.pop_back();
	}
}


using namespace boost::placeholders;

DataViewer::DataViewer(Link* link) {
	this->link = link;
	// TODO: Make it adjustable size
	this->series = std::map<std::string, TimeSeries>();
}

void DataViewer::record_data(std::string name, float value) {
	if (this->series.find(name) == this->series.end()) {
		// Add new series
		this->series.emplace(name, TimeSeries(this->series_size, name, ""));
	}
	this->series[name].add_data_now(value);
}

void DataViewer::record_data(std::string name, float value, std::string suffix) {
	this->record_data(name, value);
	this->set_units(name, suffix);
}

void DataViewer::set_units(std::string name, std::string units) {
	if (this->series.find(name) == this->series.end()) {
		// Add a new series
		this->series.emplace(name, TimeSeries(this->series_size, name, units));
	}
	this->series[name].suffix = units;
}

void DataViewer::datum_callback(DatumTypeID id, std::shared_ptr<std::vector<uint8_t>> payload) {
	if (id == DatumTypeID::INFO_SensorData) {
		// TODO: Handle incoming data! (add to ringbuffer)
		SensorData s;
		if (s.ParseFromArray(payload->data(), payload->size())) {
			// TODO: Save each data point in things
			this->record_data("ADXL375 Acceleration (X)", s.adxl_acceleration_g(0), "g");
			this->record_data("ADXL375 Acceleration (Y)", s.adxl_acceleration_g(1), "g");
			this->record_data("ADXL375 Acceleration (Z)", s.adxl_acceleration_g(2), "g");

			this->record_data("LSM6DSM Acceleration (X)", s.lsm_acceleration_g(0), "g");
			this->record_data("LSM6DSM Acceleration (Y)", s.lsm_acceleration_g(1), "g");
			this->record_data("LSM6DSM Acceleration (Z)", s.lsm_acceleration_g(2), "g");

			this->record_data("LSM6DSM Rotational Speed (X)", s.lsm_gyro_dps(0), "deg/s");
			this->record_data("LSM6DSM Rotational Speed (Y)", s.lsm_gyro_dps(1), "deg/s");
			this->record_data("LSM6DSM Rotational Speed (Z)", s.lsm_gyro_dps(2), "deg/s");

			this->record_data("LIS3MDL Magnetic Field (X)", s.lis_magnetic_mg(0), "mG");
			this->record_data("LIS3MDL Magnetic Field (Y)", s.lis_magnetic_mg(1), "mG");
			this->record_data("LIS3MDL Magnetic Field (Z)", s.lis_magnetic_mg(2), "mG");

			this->record_data("LPS22 Pressure", s.lps_pressure_hpa(), "hPa");
		}
	}
}

void DataViewer::deinit() {

}

void DataViewer::init() {
	this->link->rx_signal.connect(boost::bind(&DataViewer::datum_callback, this, _1, _2));
}

void DataViewer::set_debug_datarate(uint32_t rate_hz) {
	FrameManager2 f2(256);

	Command_ConfigSensorOutput command;
	command.set_rate_hz(rate_hz);
    command.set_raw(false);

	f2.encode_datum(DatumTypeID::CMD_ConfigSensorOutput, &command);
	this->link->send_frame(&f2);
}

void DataViewer::render_gui(bool* open) {
	static bool display_adxl, display_lsm, display_lps, display_lis, display_alt;

	if (*open) {
		ImGui::Begin("Sensor Data Viewer", open);

		// if (ImGui::TreeNode("Data Sources")) {
		// 	ImGui::Selectable("LPS22", &display_lps);
		// 	ImGui::Selectable("LSM6DSM", &display_lsm);
		// 	ImGui::Selectable("LIS3MDL", &display_lis);
		// 	ImGui::Selectable("ADXL375", &display_adxl);
		// 	ImGui::Selectable("Altitude", &display_alt);

		// 	ImGui::TreePop();
		// }

		// TODO: More robust comparison check...
		if (this->series_enabled.size() != this->series.size()) {
			for (const auto& [k, v] : this->series) {
				if (this->series_enabled.find(k) != this->series_enabled.end()) {
					this->series_enabled[k] = false;
				}
			}
		}

		// TODO: Value display of most recent aswell
		if (ImGui::TreeNode("Current Sensor Values")) {
			for (const auto& [k, v] : this->series) {
				ImGui::Text("%s = %f %s", v.name.c_str(), v.data[v.data.size() - 1], v.suffix.c_str());
			}

			ImGui::TreePop();
		}

		if (ImGui::DragInt("Buffer Size", &this->series_size, 10.0f, 0, 240000, "%d", 0)) {
			for (auto& [k, v] : this->series) {
				v.resize(this->series_size);
			}
		}

		ImGui::Separator();

		static int rate_idx = 0;

		uint32_t datarates[] = { 0, 60, 200 };
		if (ImGui::Combo("Debug data output rate", &rate_idx, "Disabled" "\0" "60Hz" "\0" "200Hz" "\0\0")) {
			this->set_debug_datarate(datarates[rate_idx]);
			std::cout << "Set output rate to " << datarates[rate_idx] << "Hz" << std::endl;
		}


		if (ImGui::BeginCombo("Data Sources", "select sources", 0)) {
			for (const auto& [a, b] : this->series_enabled) {
				ImGui::Selectable(a.c_str(), &this->series_enabled[a]);
			}
			ImGui::EndCombo();
		}

		// static float test = 0.0;
		// ImGui::DragFloat("test input", &test);
		// this->record_data("test", test);

		/*
		if (ImPlot::BeginPlot("##Scrolling", ImVec2(-1,150))) {
		ImPlot::SetupAxes(NULL, NULL, flags, flags);
		ImPlot::SetupAxisLimits(ImAxis_X1,t - history, t, ImGuiCond_Always);
		ImPlot::SetupAxisLimits(ImAxis_Y1,0,1);
		ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL,0.5f);
		ImPlot::PlotShaded("Mouse X", &sdata1.Data[0].x, &sdata1.Data[0].y, sdata1.Data.size(), -INFINITY, 0, sdata1.Offset, 2 * sizeof(float));
		ImPlot::PlotLine("Mouse Y", &sdata2.Data[0].x, &sdata2.Data[0].y, sdata2.Data.size(), 0, sdata2.Offset, 2*sizeof(float));
		ImPlot::EndPlot();
	}*/

		if (ImPlot::BeginPlot("Data")) {
			ImPlot::SetupAxis(ImAxis_X1, nullptr, ImPlotAxisFlags_AutoFit);
			for (auto& [k, v] : this->series) {
				if (this->series_enabled[k]) {

					std::ostringstream ss;
					ss << v.name;
					if (!v.suffix.empty()) {
						ss << " (" << v.suffix << ")";
					}
					std::string title = ss.str();

					/*auto [data_min, data_max] = std::minmax_element(v.data.begin(), v.data.end());

					if (scaleX)
						ImPlot::SetupAxisLimits(ImAxis_X1, v.timestamps[0], v.timestamps[v.timestamps.size()-1], ImGuiCond_Always);
					if (scaleY)
						*/
						// ImPlotAxisFlags_AutoFit ImPlot::SetupAxisLimits(ImAxis_Y1, *data_min, *data_max, ImGuiCond_Always);
					
					ImPlot::PlotLine(title.c_str(), v.timestamps.linearize(), v.data.linearize(), v.data.size());
				}
			}

			ImPlot::EndPlot();
		}


		ImGui::End();
	}
}
