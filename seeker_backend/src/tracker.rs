use bluer::Device;
use chrono::{DateTime, Utc};

use crate::protos::*;

struct Tracker {
    last_report: Option<FlightUpdate>,
    pub reports: Vec<FlightUpdate>,
    pub last_report_ts: DateTime<Utc>,
    bluetooth_conn: Option<Device>,
    pub id: u32,
    pub is_active: bool,
}

impl Tracker {
    pub fn new(id: u32) -> Self {
        Tracker {
            last_report: None,
            reports: vec![],
            last_report_ts: Utc::now(),
            bluetooth_conn: None,
            id,
            is_active: true,
        }
    }

    pub fn handle_report(&mut self, report: &FlightUpdate) {
        self.last_report = Some(report.clone());
        self.reports.push(report.clone());
        self.last_report_ts = Utc::now();
        self.is_active = true;
    }

    pub fn update(&mut self) {
        self.is_active = ((Utc::now() - self.last_report_ts).num_minutes() <= 5)
            || self.bluetooth_conn.is_some();
    }

    pub fn get_last_report(&self) -> Result<FlightUpdate, ()> {
        self.last_report.clone().ok_or(())
    }

    pub fn save_as_flight_data(&self) {
        let fields: Vec<String> = ["longitude", "latitude", "GPS_altitude", "baro_altitude"]
            .iter()
            .map(|e| e.to_string())
            .collect();
        let data_rows: Vec<Vec<String>> = self
            .reports
            .iter()
            .map(|r| {
                let mut row = vec![];
                if let Some(gps) = &r.gps_reading {
                    row.push(gps.longitude.to_string());
                    row.push(gps.latitude.to_string());
                    row.push(gps.altitude.to_string());
                } else {
                    for _ in 0..3 {
                        row.push("NA".to_owned());
                    }
                }

                if let Some(alt) = &r.altitude {
                    row.push(alt.alt_m.to_string());
                } else {
                    row.push("NA".to_owned());
                }

                row
            })
            .collect();
    }
}
