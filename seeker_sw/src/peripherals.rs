#![allow(dead_code, non_camel_case_types, unused_must_use)]

use crate::configuration::SeekerConfiguration;
use crate::widgets::CompassWidget;

use std::{
    collections::*,
    error::Error,
    fmt::{self, Display},
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
use serde_json;

use chrono::{NaiveDate, NaiveTime, TimeZone, Utc};
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

#[derive(Debug)]
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
    fix_time: NaiveTime,
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
            fix_time: NaiveTime::from_hms(0, 0, 0),
            satellites_fixed: 0,
            debug: String::new(),
        }
    }
}

//°
impl Display for GPSData {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{}: {}° {:?}, {}° {:?} at {} UTC using {} satellites",
            if self.fixed { "Fixed" } else { "NOT FIXED" },
            self.latitude,
            self.latitude_dir,
            self.longitude,
            self.longitude_dir,
            self.fix_time,
            self.satellites_fixed
        )
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
        let gps = GPS {
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
        loop {
            let mut lbuf = String::new();
            readbuf
                .read_line(&mut lbuf)
                .expect("Error, unable to read from GPS device file.");

            if (lbuf.len() <= 1) {
                continue;
            }

            let nmea_result = nmea::parse(lbuf.as_bytes());

            // TODO: Add more supported messages
            if let (Ok(parsed)) = nmea_result {
                match parsed {
                    ParseResult::GGA(data) => {
                        let mut record = data_record.lock();
                        if let Some(latitude) = data.latitude {
                            record.as_mut().unwrap().latitude = latitude
                        };
                        if let Some(longitude) = data.longitude {
                            record.as_mut().unwrap().longitude = longitude
                        };
                        if let Some(satellites) = data.fix_satellites {
                            record.as_mut().unwrap().satellites_fixed = satellites as usize
                        };
                        if let Some(alt) = data.altitude {
                            record.as_mut().unwrap().altitude = alt as f64
                        };
                        if let Some(time) = data.fix_time {
                            record.as_mut().unwrap().fix_time = time
                        };

                        record.as_mut().unwrap().fixed = match data.fix_type.unwrap() {
                            nmea::FixType::Gps
                            | nmea::FixType::DGps
                            | nmea::FixType::Pps
                            | nmea::FixType::Rtk
                            | nmea::FixType::FloatRtk
                            | nmea::FixType::Estimated => true,
                            _ => false,
                        };
                    }
                    ParseResult::GLL(data) => {
                        let mut record = data_record.lock();
                        record.as_mut().unwrap().latitude = data.latitude.unwrap_or(0f64);
                        record.as_mut().unwrap().longitude = data.longitude.unwrap_or(0f64);
                        record.as_mut().unwrap().fixed = data.valid;
                        record.as_mut().unwrap().fix_time = data.fix_time;
                    }
                    _ => {}
                }
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
        ui.label(self.data.lock().unwrap().to_string());

        // let ports = serialport::available_ports().expect("No ports found!");
        // ui.label(format!("{} ports found", ports.len()));

        // for port in ports {
        //    ui.collapsing(port.port_name, |ui| {
        //        ui.label(match (port.port_type) {
        //            SerialPortType::UsbPort(info) => format!(
        //                "USB Serial {}",
        //                info.serial_number.unwrap_or("Unknown".to_owned())
        //            ),
        //            _ => "Unknown".to_string(),
        //        });
        //    });
        // }
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

// TODO: Readout and GUI for Gyro
// TODO: Tilt compensation for compass
// TODO: Implement calibration

#[cfg(target_arch = "arm")]
pub struct Imu9DOF {
    mag_i2c_conn: I2c,
    heading: f64,
    heading_avg_size: usize,
    heading_samples: VecDeque<f64>,
    compass_widget: Arc<Mutex<CompassWidget>>,
    mag_vec: [f64; 3],
    calibration_offsets: [f64; 3],
    heading_offset: f64,
}

#[cfg(not(target_arch = "arm"))]
pub struct Imu9DOF {
    heading: f64,
    compass_widget: Arc<Mutex<CompassWidget>>,
}

#[cfg(target_arch = "arm")]
impl<'a> Imu9DOF {
    pub fn new(cc: &'a eframe::CreationContext<'a>) -> Self {
        let mag_i2c_conn = I2c::with_bus(1).expect("Error, failed to open I2c");
        let mut imu = Self {
            mag_i2c_conn,
            heading: 0f64,
            heading_avg_size: 20,
            heading_samples: VecDeque::new(),
            compass_widget: Arc::new(Mutex::new(CompassWidget::new(cc.gl.as_ref()))),
            mag_vec: [0f64; 3],
            calibration_offsets: [0f64; 3],
            heading_offset: 0f64,
        };

        let cfg = SeekerConfiguration::load().unwrap();
        imu.calibration_offsets = cfg.magnetometer_offsets;
        imu.heading_offset = cfg.heading_offset;

        imu.init_sensors();
        return imu;
    }

    pub fn run_calibration(&mut self) -> Result<(), Box<dyn Error>> {
        let samples: Vec<&[f64; 3]> = vec![];
        let cfg_file = SeekerConfiguration::load()?;

        // Collect samples until console input
        loop {}

        // Process samples:
        // - Remove 1% outliers
        // - Find center of ellipsoid ((min+max)/2)
        // - Save to configuration file

        cfg_file.magnetometer_offsets = [1f64, 2f64, 3f64];
        cfg_file.save()?;

        Ok(())
    }

    fn init_sensors(&mut self) {
        self.mag_i2c_conn.set_slave_address(0x1C); // LSM303AGR triple axis magnetometer + accelerometer

        // Startup sequence from datasheet
        self.reg_write_m(LIS3MDL_REG::CTRL_REG1, 0b01010000);
        self.reg_write_m(LIS3MDL_REG::CTRL_REG2, 0b00000000);
        self.reg_write_m(LIS3MDL_REG::CTRL_REG3, 0b00000000);
        self.reg_write_m(LIS3MDL_REG::CTRL_REG4, 0b00000000);
        self.reg_write_m(LIS3MDL_REG::CTRL_REG5, 0b00000000);
    }

    fn reg_write_m(&mut self, reg: LIS3MDL_REG, data: u8) {
        self.mag_i2c_conn
            .write(&[reg as u8, data])
            .expect("Error, unable to interface with I2C bus");
    }

    fn reg_read_m(&mut self, reg: LIS3MDL_REG) -> u8 {
        let mut buf = [0u8; 1];
        self.mag_i2c_conn
            .write_read(&[reg as u8], &mut buf)
            .expect("Error, unable to interface with I2C bus");
        buf[0]
    }

    pub fn read_sensors(&mut self) {
        /*
            [2863, 3329, 2851]
            [-4428, -3809, -4650]
            [-782.5 -240.  -899.5]
        */

        let mut buf = [0u8; 6];
        self.mag_i2c_conn
            .write_read(&[LIS3MDL_REG::OUT_X_L as u8], &mut buf)
            .expect("Error, unable to interface with I2C bus");
        self.mag_vec[0] =
            (((buf[1] as i16) << 8) | (buf[0] as i16)) as f64 - self.calibration_offsets[0];
        self.mag_vec[1] =
            (((buf[3] as i16) << 8) | (buf[2] as i16)) as f64 - self.calibration_offsets[1];
        self.mag_vec[2] =
            (((buf[5] as i16) << 8) | (buf[4] as i16)) as f64 - self.calibration_offsets[2];

        // DONE: Fix weird spinning when there are some negative values

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
        let min_heading = self.heading_samples.iter().cloned().fold(360.0, f64::min);
        let max_heading = self.heading_samples.iter().cloned().fold(0.0, f64::max);
        self.heading = self
            .heading_samples
            .iter()
            .map(|s| {
                if (*s > (min_heading + std::f64::consts::PI)) {
                    (*s - (2.0 * std::f64::consts::PI))
                } else {
                    *s
                }
            })
            .sum::<f64>()
            / (self.heading_avg_size as f64)
            + self.heading_offset;
    }
}

#[cfg(not(target_arch = "arm"))]
impl<'a> Imu9DOF {
    pub fn new(cc: &'a eframe::CreationContext<'a>) -> Self {
        Self {
            heading: 0f64,
            compass_widget: Arc::new(Mutex::new(CompassWidget::new(cc.gl.as_ref()))),
        }
    }

    fn read_sensor(&mut self) {
        self.heading = self.heading + (std::f64::consts::PI * 2f64) / 120f64;
        self.heading = self.heading.rem_euclid(std::f64::consts::PI * 2f64)
    }
}

impl GUIItem for Imu9DOF {
    fn render_window(&mut self, ctx: &Context, frame: &mut Frame) {
        Window::new("Compass").show(ctx, |ui| {
            self.render_widget(ctx, ui);
            egui::Frame::canvas(ui.style()).show(ui, |ui| {
                self.custom_painting(ui);
            });
        });
    }

    fn render_widget(&mut self, ctx: &Context, ui: &mut Ui) {
        self.read_sensors();
        ui.label(format!("Heading: {:.1}", self.heading.to_degrees()));
    }
}

impl Imu9DOF {
    fn custom_painting(&mut self, ui: &mut egui::Ui) {
        let (rect, response) =
            ui.allocate_exact_size(egui::Vec2::splat(300.0), egui::Sense::drag());

        // self.angle += response.drag_delta().x * 0.01;

        // Clone locals so we can move them into the paint callback:
        let heading = self.heading;
        let widget = self.compass_widget.clone();

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
