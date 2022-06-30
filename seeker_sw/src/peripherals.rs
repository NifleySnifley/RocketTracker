#![allow(dead_code, non_camel_case_types, unused_must_use)]

use crate::configuration::SeekerConfiguration;
use crate::widgets::CompassWidget;

use std::{
    collections::*,
    error::Error,
    fs::File,
    io::{BufRead, BufReader},
    sync::{Arc, Mutex},
    thread::{sleep, JoinHandle, Thread},
    time::Duration,
};

use eframe::{
    egui::{self, RichText},
    egui::{Context, Ui, Window},
    epaint::PaintCallbackInfo,
    Frame,
};
use egui_glow;
use egui_glow::{glow, Painter};
use serde::{Deserialize, Serialize};

use nmea::{self, *};

#[cfg(target_arch = "arm")]
use rppal::i2c::I2c;

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
    West,
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
    debug: String,
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
            satellites_fixed: 0,
            debug: String::new(),
        }
    }
}

// #[derive(Serialize, Deserialize)]
pub struct GPS {
    serial_port: String,
    data: Arc<Mutex<GPSData>>,
}

#[cfg(target_arch = "arm")]
impl GPS {
    pub fn new(port: String) -> Self {
        let mut gps = GPS {
            serial_port: port,
            data: Arc::new(Mutex::new(GPSData::default())),
        };

        let th_port = gps.serial_port.clone();
        let th_dat = gps.data.clone();
        std::thread::spawn(move || GPS::listener_fn(th_port, th_dat));

        gps
    }

    pub fn listener_fn(port: String, data_record: Arc<Mutex<GPSData>>) {
        let device_file = File::open(port).expect("Error, unable to open GPS device file.");
        let mut readbuf = BufReader::new(device_file);
        while (true) {
            let mut lbuf = String::new();
            readbuf
                .read_line(&mut lbuf)
                .expect("Error, unable to read from GPS device file.");

            if (lbuf.len() <= 1) {
                continue;
            }

            let parsed = nmea::parse(lbuf.as_bytes()).unwrap();
            match parsed {
                ParseResult::GGA(data) => {
                    data_record.lock().as_mut().unwrap().debug =
                        format!("Lat: {}\r", data.latitude.unwrap_or(0.0f64));
                }
                _ => {}
            }
        }
    }
}

#[cfg(not(target_arch = "arm"))]
impl GPS {
    pub fn new(port: String) -> Self {
        let mut gpsdata = GPSData::default();

        gpsdata.latitude = 40.7128;
        gpsdata.longitude = 74.0060;
        gpsdata.latitude_dir = CompassDirection::North;
        gpsdata.longitude_dir = CompassDirection::West;
        gpsdata.satellites_fixed = 7;
        gpsdata.altitude = 100f64;

        let mut gps = GPS {
            serial_port: port,
            data: Arc::new(Mutex::new(gpsdata)),
        };

        gps
    }
}

impl GUIItem for GPS {
    fn render_window(&mut self, ctx: &Context, frame: &mut Frame) {
        Window::new("GPS").show(ctx, |ui| {
            self.render_widget(ctx, ui);
        });
    }

    fn render_widget(&mut self, ctx: &Context, ui: &mut Ui) {
        ui.code(RichText::new(self.data.lock().unwrap().debug.to_string()));

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

#[repr(u8)]
#[derive(Copy, Clone)]
pub enum LIS3MDL_REG {
    WHO_AM_I_A = 0x0F,
    CTRL_REG1 = 0x20,
    CTRL_REG2 = 0x21,
    CTRL_REG3 = 0x22,
    CTRL_REG4 = 0x23,
    CTRL_REG5 = 0x24,
    STATUS_REG = 0x27,
    OUT_X_L = 0x28,
    OUT_X_H = 0x29,
    OUT_Y_L = 0x2A,
    OUT_Y_H = 0x2B,
    OUT_Z_L = 0x2C,
    OUT_Z_H = 0x2D,
    TEMP_OUT_L = 0x2E,
    TEMP_OUT_H = 0x2F,
    INT_CFG = 0x30,
    INT_SRC = 0x31,
    INT_THS_L = 0x32,
    INT_THS_H = 0x33,
}

#[cfg(target_arch = "arm")]
pub struct Compass {
    i2c_conn: I2c,
    heading: f64,
    heading_avg_size: usize,
    heading_samples: VecDeque<f64>,
    widget: Arc<Mutex<CompassWidget>>,
    mag_vec: [f64; 3],
    calibration_offsets: [f64; 3],
}

#[cfg(not(target_arch = "arm"))]
pub struct Compass {
    heading: f64,
    widget: Arc<Mutex<CompassWidget>>,
}

#[cfg(target_arch = "arm")]
impl<'a> Compass {
    pub fn new(cc: &'a eframe::CreationContext<'a>) -> Self {
        let i2c_conn = I2c::with_bus(1).expect("Error, failed to open I2c");
        let mut compass = Self {
            i2c_conn,
            heading: 0f64,
            heading_avg_size: 20,
            heading_samples: VecDeque::new(),
            widget: Arc::new(Mutex::new(CompassWidget::new(cc.gl.as_ref()))),
            mag_vec: [0f64; 3],
            calibration_offsets: [0f64; 3], // TODO: Add calibration function instead of using these magic numbers
        };

        let cfg = SeekerConfiguration::load().unwrap();
        compass.calibration_offsets = cfg.magnetometer_offsets;

        compass.init_sensor();
        return compass;
    }

    pub fn run_calibration(&mut self) -> Result<(), Box<dyn Error>> {
        let mut samples: Vec<&[f64; 3]> = vec![];
        let mut cfg_file = SeekerConfiguration::load()?;
        // loop {}

        // Process samples:
        // - Remove 1% outliers
        // - Find center of ellipsoid
        // - Save to configuration file

        cfg_file.magnetometer_offsets = [1f64, 2f64, 3f64];
        cfg_file.save()?;

        Ok(())
    }

    fn init_sensor(&mut self) {
        self.i2c_conn.set_slave_address(0x1C); // LSM303AGR triple axis magnetometer + accelerometer

        // Startup sequence from datasheet
        self.reg_write(LIS3MDL_REG::CTRL_REG1, 0b01010000);
        self.reg_write(LIS3MDL_REG::CTRL_REG2, 0b00000000);
        self.reg_write(LIS3MDL_REG::CTRL_REG3, 0b00000000);
        self.reg_write(LIS3MDL_REG::CTRL_REG4, 0b00000000);
        self.reg_write(LIS3MDL_REG::CTRL_REG5, 0b00000000);
    }

    fn reg_write(&mut self, reg: LIS3MDL_REG, data: u8) {
        self.i2c_conn
            .write(&[reg as u8, data])
            .expect("Error, unable to interface with I2C bus");
    }

    fn reg_read(&mut self, reg: LIS3MDL_REG) -> u8 {
        let mut buf = [0u8; 1];
        self.i2c_conn
            .write_read(&[reg as u8], &mut buf)
            .expect("Error, unable to interface with I2C bus");
        buf[0]
    }

    pub fn read_sensor(&mut self) {
        // self.mag_vec[0] = ((self.reg_read(LIS3MDL_REG::OUT_X_H) as i16) << 8) | (self.reg_read(LIS3MDL_REG::OUT_X_L) as i16);
        // self.mag_vec[1] = ((self.reg_read(LIS3MDL_REG::OUT_Y_H) as i16) << 8) | (self.reg_read(LIS3MDL_REG::OUT_Y_L) as i16);
        // self.mag_vec[2] = ((self.reg_read(LIS3MDL_REG::OUT_Z_H) as i16) << 8) | (self.reg_read(LIS3MDL_REG::OUT_Z_L) as i16);

        /*
            [2863, 3329, 2851]
            [-4428, -3809, -4650]
            [-782.5 -240.  -899.5]
        */

        let mut buf = [0u8; 6];
        self.i2c_conn
            .write_read(&[LIS3MDL_REG::OUT_X_L as u8], &mut buf)
            .expect("Error, unable to interface with I2C bus");
        self.mag_vec[0] =
            (((buf[1] as i16) << 8) | (buf[0] as i16)) as f64 - self.calibration_offsets[0];
        self.mag_vec[1] =
            (((buf[3] as i16) << 8) | (buf[2] as i16)) as f64 - self.calibration_offsets[1];
        self.mag_vec[2] =
            (((buf[5] as i16) << 8) | (buf[4] as i16)) as f64 - self.calibration_offsets[2];

        let mut calculated_heading = (self.mag_vec[1]).atan2(self.mag_vec[0]);
        if calculated_heading < 0.0 {
            calculated_heading = (2.0 * std::f64::consts::PI) + calculated_heading;
        }
        if (self.heading_samples.len() < self.heading_avg_size) {
            for i in 0..self.heading_avg_size {
                self.heading_samples.push_front(calculated_heading);
            }
        } else {
            self.heading_samples.pop_back();
            self.heading_samples.push_front(calculated_heading);
        }

        // Make heading the running average of the last <heading_avg_size> values read from the magnetometer
        // Really helps make the readout more smooth
        self.heading = self.heading_samples.iter().sum::<f64>() / (self.heading_avg_size as f64);
        self.heading += std::f64::consts::PI;

        // println!(
        //     "{},{},{},{}",
        //     self.mag_vec[0],
        //     self.mag_vec[1],
        //     self.mag_vec[2],
        //     self.heading.to_degrees()
        // );
    }
}

#[cfg(not(target_arch = "arm"))]
impl<'a> Compass {
    pub fn new(cc: &'a eframe::CreationContext<'a>) -> Self {
        Self {
            heading: 0f64,
            widget: Arc::new(Mutex::new(CompassWidget::new(cc.gl.as_ref()))),
        }
    }

    fn read_sensor(&mut self) {
        self.heading = self.heading + (std::f64::consts::PI * 2f64) / 120f64;
        self.heading = self.heading.rem_euclid(std::f64::consts::PI * 2f64)
    }
}

impl GUIItem for Compass {
    fn render_window(&mut self, ctx: &Context, frame: &mut Frame) {
        Window::new("Compass").show(ctx, |ui| {
            self.render_widget(ctx, ui);
            egui::Frame::canvas(ui.style()).show(ui, |ui| {
                self.custom_painting(ui);
            });
        });
    }

    fn render_widget(&mut self, ctx: &Context, ui: &mut Ui) {
        // Read i2c magnetometer
        self.read_sensor();
        ui.label(format!("Heading: {:.1}", self.heading.to_degrees()));
        // println!("Sensor: {}", self.sensor_val);
        // ui.add(egui::DragValue::new(&mut self.sensor_val));
        // Display visual compass
    }
}

pub struct CallbackFn {
    f: Box<dyn Fn(PaintCallbackInfo, &Painter) + Sync + Send>,
}

impl CallbackFn {
    pub fn new<F: Fn(PaintCallbackInfo, &Painter) + Sync + Send + 'static>(callback: F) -> Self {
        let f = Box::new(callback);
        CallbackFn { f }
    }
}

impl Compass {
    fn custom_painting(&mut self, ui: &mut egui::Ui) {
        let (rect, response) =
            ui.allocate_exact_size(egui::Vec2::splat(300.0), egui::Sense::drag());

        // self.angle += response.drag_delta().x * 0.01;

        // Clone locals so we can move them into the paint callback:
        let heading = self.heading;
        let widget = self.widget.clone();

        // let cb = egui_glow::painter::CallbackFn::new(move |_info, painter| {
        //     widget.lock().unwrap().paint(painter.gl(), heading);
        // });

        let callback = egui::PaintCallback {
            rect,
            callback: std::sync::Arc::new(move |_info, render_ctx| {
                if let Some(painter) = render_ctx.downcast_ref::<egui_glow::Painter>() {
                    widget.lock().unwrap().paint(painter.gl(), heading);
                } else {
                    eprintln!("Can't do custom painting because we are not using a glow context");
                }
            }),
        };
        ui.painter().add(callback);
    }
}
