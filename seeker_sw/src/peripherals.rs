use std::{
    sync::{Arc, Mutex},
    thread::JoinHandle,
};

use eframe::{
    egui,
    egui::{Context, Ui, Window},
    Frame,
};
use serde::{Deserialize, Serialize};

pub trait GUIItem {
    fn render_window(&mut self, ctx: &Context, frame: &mut Frame);
    fn render_widget(&mut self, ctx: &Context, ui: &mut Ui);
}

#[derive(Copy, Clone, Serialize, Deserialize, Default)]
pub struct Radio {
    pub channel: u8,
    pub network: u8,
    pub encryption_k: [u8; 16],
}

impl GUIItem for Radio {
    fn render_window(&mut self, ctx: &Context, frame: &mut Frame) {
        Window::new("Radio Configuration")
            .resizable(true)
            .default_width(280.0)
            .show(ctx, |ui| {
                self.render_widget(ctx, ui);
            });
    }

    fn render_widget(&mut self, ctx: &Context, ui: &mut Ui) {
        ui.label("Rocket channel: ");
        ui.add(egui::DragValue::new(&mut self.channel).clamp_range(0u8..=255));
        ui.end_row();

        ui.label("Rocket network #: ");
        ui.add(egui::DragValue::new(&mut self.network).clamp_range(0u8..=255));
        ui.end_row();
    }
}

pub enum CompassDirection {
    North,
    South,
    East,
    West
}

pub struct GPSData {
    latitude: f64,
    latitude_dir: CompassDirection,
    longitude: f64,
    longitude_dir: CompassDirection,
    satellites_fixed: usize,
    altitude: f64,
    fixed: bool,
    utc_time: f64,
}

impl Default for GPSData {
    fn default() -> Self {
        GPSData {
            latitude: 0.0,
            longitude: 0.0,
            latitude_dir: CompassDirection::North,
            longitude_dir: CompassDirection::West,
            altitude: 0.0,
            fixed: false,
            utc_time: 0.0,
            satellites_fixed: 0
        }
    }
}

// #[derive(Serialize, Deserialize)]
pub struct GPS {
    serial_port: String,
    //pub data: Arc<Mutex<GPSData>>,
}

impl GPS {
    pub fn new(port: String) -> Self {
        let mut gps = GPS {
            serial_port: port,
      //      gps_data: Arc::new(Mutex::new(GPSData::default())),
        };

        //std::thread::spawn(|| GPS::listener_fn());

        gps
    }

    //pub fn listener_fn(port: String, data_record: Arc<Mutex<GPSData>>) {}
}

impl GUIItem for GPS {
    fn render_window(&mut self, ctx: &Context, frame: &mut Frame) {
        Window::new("GPS").show(ctx, |ui| {
            self.render_widget(ctx, ui);
        });
    }

    fn render_widget(&mut self, ctx: &Context, ui: &mut Ui) {
        ui.heading("GPS Info");

        //let ports = serialport::available_ports().expect("No ports found!");
        //ui.label(format!("{} ports found", ports.len()));

        //for port in ports {
        //    ui.collapsing(port.port_name, |ui| {
        //        ui.label(match (port.port_type) {
        //            SerialPortType::UsbPort(info) => format!(
        //                "USB Serial {}",
        //                info.serial_number.unwrap_or("Unknown".to_owned())
        //            ),
        //            _ => "Unknown".to_string(),
        //        });
        //    });
        //}
    }
}
