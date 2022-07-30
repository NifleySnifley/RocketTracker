#![allow(dead_code, non_camel_case_types, unused_must_use)]

use crate::configuration::SeekerConfiguration;
use crate::widgets::CompassWidget;

use std::{
    collections::*,
    error::Error,
    fmt::{self, Display},
    fs::File,
    io::Cursor,
    io::{self, BufRead, BufReader},
    sync::{
        mpsc::{self, Receiver, Sender},
        Arc, Mutex,
    },
    thread::{self, sleep, JoinHandle, Thread},
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

use average::{Estimate, Quantile};

#[cfg(target_arch = "arm")]
use rppal::i2c::I2c;

use byteorder::{BigEndian, LittleEndian, ReadBytesExt};

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

#[repr(u8)]
#[derive(Copy, Clone)]
enum LSM6DSOX_REG {
    FUNC_CFG_ACCESS = 0x01,
    PIN_CTRL = 0x02,
    S4S_TPH_L = 0x04,
    S4S_TPH_H = 0x05,
    S4S_RR = 0x06,
    FIFO_CTRL1 = 0x07,
    FIFO_CTRL2 = 0x08,
    FIFO_CTRL3 = 0x09,
    FIFO_CTRL4 = 0x0A,
    COUNTER_BDR_REG1 = 0x0B,
    COUNTER_BDR_REG2 = 0x0C,
    INT1_CTRL = 0x0D,
    INT2_CTRL = 0x0E,
    WHO_AM_I = 0x0F,
    CTRL1_XL = 0x10,
    CTRL2_G = 0x11,
    CTRL3_C = 0x12,
    CTRL4_C = 0x13,
    CTRL5_C = 0x14,
    CTRL6_C = 0x15,
    CTRL7_G = 0x16,
    CTRL8_XL = 0x17,
    CTRL9_XL = 0x18,
    CTRL10_C = 0x19,
    ALL_INT_SRC = 0x1A,
    WAKE_UP_SRC = 0x1B,
    TAP_SRC = 0x1C,
    D6D_SRC = 0x1D,
    STATUS_REG = 0x1E,
    OUT_TEMP_L = 0x20,
    OUT_TEMP_H = 0x21,
    OUTX_L_G = 0x22,
    OUTX_H_G = 0x23,
    OUTY_L_G = 0x24,
    OUTY_H_G = 0x25,
    OUTZ_L_G = 0x26,
    OUTZ_H_G = 0x27,
    OUTX_L_A = 0x28,
    OUTX_H_A = 0x29,
    OUTY_L_A = 0x2A,
    OUTY_H_A = 0x2B,
    OUTZ_L_A = 0x2C,
    OUTZ_H_A = 0x2D,
    EMB_FUNC_STATUS_MAINPAGE = 0x35,
    FSM_STATUS_A_MAINPAGE = 0x36,
    FSM_STATUS_B_MAINPAGE = 0x37,
    MLC_STATUS_MAINPAGE = 0x38,
    STATUS_MASTER_MAINPAGE = 0x39,
    FIFO_STATUS1 = 0x3A,
    FIFO_STATUS2 = 0x3B,
    TIMESTAMP0 = 0x40,
    TIMESTAMP1 = 0x41,
    TIMESTAMP2 = 0x42,
    TIMESTAMP3 = 0x43,
    UI_STATUS_REG_OIS = 49,
    UI_OUTX_L_G_OIS = 0x4A,
    UI_OUTX_H_G_OIS = 0x4B,
    UI_OUTY_L_G_OIS = 0x4C,
    UI_OUTY_H_G_OIS = 0x4D,
    UI_OUTZ_L_G_OIS = 0x4E,
    UI_OUTZ_H_G_OIS = 0x4F,
    UI_OUTX_L_A_OIS = 0x50,
    UI_OUTX_H_A_OIS = 0x51,
    UI_OUTY_L_A_OIS = 0x52,
    UI_OUTY_H_A_OIS = 0x53,
    UI_OUTZ_L_A_OIS = 0x54,
    UI_OUTZ_H_A_OIS = 0x55,
    TAP_CFG0 = 0x56,
    TAP_CFG1 = 0x57,
    TAP_CFG2 = 0x58,
    TAP_THS_6D = 0x59,
    INT_DUR2 = 0x5A,
    WAKE_UP_THS = 0x5B,
    WAKE_UP_DUR = 0x5C,
    FREE_FALL = 0x5D,
    MD1_CFG = 0x5E,
    MD2_CFG = 0x5F,
    S4S_ST_CMD_CODE = 60,
    S4S_DT_REG = 0x61,
    I3C_BUS_AVB = 0x62,
    INTERNAL_FREQ_FINE = 63,
    UI_INT_OIS = 0x6F,
    UI_CTRL1_OIS = 0x70,
    UI_CTRL2_OIS = 0x71,
    UI_CTRL3_OIS = 0x72,
    X_OFS_USR = 0x73,
    Y_OFS_USR = 0x74,
    Z_OFS_USR = 0x75,
    FIFO_DATA_OUT_TAG = 0x78,
    FIFO_DATA_OUT_X_L = 0x79,
    FIFO_DATA_OUT_X_H = 0x7A,
    FIFO_DATA_OUT_Y_L = 0x7B,
    FIFO_DATA_OUT_Y_H = 0x7C,
    FIFO_DATA_OUT_Z_L = 0x7D,
    FIFO_DATA_OUT_Z_H = 0x7E,
}

#[derive(Default, Debug)]
pub struct IMUReading {
    raw_accel: [f64; 3],
    raw_gyro: [f64; 3],
}

// TODO: Readout and GUI for Gyro
// TODO: Proper filtered gyro readout
// TODO: Tilt compensation for compass using gyro
//        - Get gyro tilt as a matrix, then apply the inverse to the magnetometer reading
// DONE: Implement calibration
// TODO: Map server in JS with websocket

#[cfg(target_arch = "arm")]
pub struct Imu9DOF {
    i2c_conn: I2c,
    heading: f64,
    heading_avg_size: usize,
    heading_samples: VecDeque<f64>,
    compass_widget: Arc<Mutex<CompassWidget>>,
    mag_vec: [f64; 3],
    calibration_offsets: [f64; 3],
    heading_offset: f64,

    imu_reading: Arc<Mutex<IMUReading>>,
}

#[cfg(not(target_arch = "arm"))]
pub struct Imu9DOF {
    heading: f64,
    compass_widget: Arc<Mutex<CompassWidget>>,
}

#[cfg(target_arch = "arm")]
impl<'a> Imu9DOF {
    pub fn new(cc: &'a eframe::CreationContext<'a>) -> Self {
        let i2c_conn = I2c::with_bus(1).expect("Error, failed to open I2c");
        let mut imu = Self {
            i2c_conn,
            heading: 0f64,
            heading_avg_size: 20,
            heading_samples: VecDeque::new(),
            compass_widget: Arc::new(Mutex::new(CompassWidget::new(cc.gl.as_ref()))),
            mag_vec: [0f64; 3],
            calibration_offsets: [0f64; 3],
            heading_offset: 0f64,

            imu_reading: Arc::new(Mutex::new(IMUReading::default())),
        };

        let cfg = SeekerConfiguration::load().unwrap();
        imu.calibration_offsets = cfg.magnetometer_offsets;
        imu.heading_offset = cfg.heading_offset;

        imu.init_sensors();
        return imu;
    }

    pub fn run_calibration(&mut self) -> Result<(), Box<dyn Error>> {
        let mut samples: Vec<[f64; 3]> = vec![];
        let mut cfg_file = SeekerConfiguration::load()?;

        println!("Starting calibration\nPlease move the magnetometer around as much as possible\nPress return when finished...");

        let (tx, rx): (Sender<()>, Receiver<()>) = mpsc::channel();
        thread::spawn(move || {
            let mut input = String::new();
            io::stdin().read_line(&mut input); // Blocks here
            tx.send(()).unwrap(); // Notify the loop that it should quit
        });

        let mut sample_ct = 0usize;
        const quant_threshold: f64 = 0.005;
        let mut quantiles_lo = [
            Quantile::new(quant_threshold),
            Quantile::new(quant_threshold),
            Quantile::new(quant_threshold),
        ];

        let mut quantiles_hi = [
            Quantile::new(1.0 - quant_threshold),
            Quantile::new(1.0 - quant_threshold),
            Quantile::new(1.0 - quant_threshold),
        ];

        // Make sure to reset calbration values to zero before sampling
        self.calibration_offsets = [0f64; 3];

        // Collect samples until quit signal
        loop {
            if rx.try_recv().is_ok() {
                print!("Data collection done!                       \n");
                break;
            }
            print!("{sample_ct} data points collected\r");

            self.read_sensors();
            for i in 0..3 {
                quantiles_lo[i].add(self.mag_vec[i]);
                quantiles_hi[i].add(self.mag_vec[i]);
            }
            samples.push(self.mag_vec.clone());

            sample_ct += 1;
        }

        println!("Finished calibration!");

        let quantiles_calcd_lo: Vec<f64> = quantiles_lo.iter().map(|q| q.quantile()).collect();
        let quantiles_calcd_hi: Vec<f64> = quantiles_hi.iter().map(|q| q.quantile()).collect();

        // Process samples:
        // - Remove 1% outliers
        // - Find center of ellipsoid ((min+max)/2)
        // - Save to configuration file

        cfg_file.magnetometer_offsets = [
            (quantiles_calcd_lo[0] + quantiles_calcd_hi[0]) / 2f64,
            (quantiles_calcd_lo[1] + quantiles_calcd_hi[1]) / 2f64,
            (quantiles_calcd_lo[2] + quantiles_calcd_hi[2]) / 2f64,
        ];
        self.calibration_offsets = cfg_file.magnetometer_offsets.clone();

        println!("Calibration offsets: {:?}", cfg_file.magnetometer_offsets);

        cfg_file.save()?;

        Ok(())
    }

    fn init_sensors(&mut self) {
        // Startup sequence from datasheet (LIS3MDL)
        self.reg_write_m(LIS3MDL_REG::CTRL_REG1, 0b01010000);
        self.reg_write_m(LIS3MDL_REG::CTRL_REG2, 0b00000000);
        self.reg_write_m(LIS3MDL_REG::CTRL_REG3, 0b00000000);
        self.reg_write_m(LIS3MDL_REG::CTRL_REG4, 0b00000000);
        self.reg_write_m(LIS3MDL_REG::CTRL_REG5, 0b00000000);

        // Initialize accelerometer and gyro of LSM6DSOX IMU
        // NOTE: Gyro: 1000dps, Accelerometer: 16g
        self.reg_write_i(LSM6DSOX_REG::CTRL1_XL, 0b0101_01_1_0);
        self.reg_write_i(LSM6DSOX_REG::CTRL2_G, 0b0101_10_1_0);

        // TODO: Gyro multithreading for better integration?
        // TODO: Better I2C error handling
    }

    fn read_imu(&mut self, imu_data: Arc<Mutex<IMUReading>>) {
        let mut readout = imu_data.lock().unwrap();
        let mut buf = [0u8; 12];

        self.i2c_conn
            .write_read(&[LSM6DSOX_REG::OUTX_L_G as u8], &mut buf)
            .expect("Error, unable to interface with I2C bus");

        let mut reader = Cursor::new(buf);

        // Read out and convert to radians/second
        readout.raw_gyro[0] =
            (reader.read_i16::<LittleEndian>().unwrap() as f64) / (i16::MAX as f64) * 17.45330; //(((buf[1] as i16) << 8) | (buf[0] as i16)) as f64;
        readout.raw_gyro[1] =
            (reader.read_i16::<LittleEndian>().unwrap() as f64) / (i16::MAX as f64) * 17.45330;
        readout.raw_gyro[2] =
            (reader.read_i16::<LittleEndian>().unwrap() as f64) / (i16::MAX as f64) * 17.45330;

        // Read and convert to m/s^2
        readout.raw_accel[0] =
            (reader.read_i16::<LittleEndian>().unwrap() as f64) / (i16::MAX as f64) * 156.9064;
        readout.raw_accel[1] =
            (reader.read_i16::<LittleEndian>().unwrap() as f64) / (i16::MAX as f64) * 156.9064;
        readout.raw_accel[2] =
            (reader.read_i16::<LittleEndian>().unwrap() as f64) / (i16::MAX as f64) * 156.9064;
    }

    fn reg_write_m(&mut self, reg: LIS3MDL_REG, data: u8) {
        self.i2c_conn.set_slave_address(0x1C);
        self.i2c_conn
            .write(&[reg as u8, data])
            .expect("Error, unable to interface with I2C bus");
    }

    fn reg_read_m(&mut self, reg: LIS3MDL_REG) -> u8 {
        self.i2c_conn.set_slave_address(0x1C);
        let mut buf = [0u8; 1];
        self.i2c_conn
            .write_read(&[reg as u8], &mut buf)
            .expect("Error, unable to interface with I2C bus");
        buf[0]
    }

    fn reg_write_i(&mut self, reg: LSM6DSOX_REG, data: u8) {
        self.i2c_conn.set_slave_address(0x6A);
        self.i2c_conn
            .write(&[reg as u8, data])
            .expect("Error, unable to interface with I2C bus");
    }

    fn reg_read_i(&mut self, reg: LSM6DSOX_REG) -> u8 {
        self.i2c_conn.set_slave_address(0x6A);
        let mut buf = [0u8; 1];
        self.i2c_conn
            .write_read(&[reg as u8], &mut buf)
            .expect("Error, unable to interface with I2C bus");
        buf[0]
    }

    pub fn read_sensors(&mut self) {
        self.i2c_conn.set_slave_address(0x6A);

        self.read_imu(self.imu_reading.clone());

        self.i2c_conn.set_slave_address(0x1C); // LSM303AGR triple axis magnetometer + accelerometer
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
        ui.label(format!("IMU: {:?}", self.imu_reading.lock().unwrap()));
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
