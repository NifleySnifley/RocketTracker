use std::collections::HashMap;
use std::fs::File;
use std::io::BufReader;
use std::path::Path;
use std::{fs, io};
use uuid::Uuid;

use chrono::serde::ts_seconds;
use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct FlightLogMetadata {
    pub tracker_id: u32,
    pub id: Uuid,
    #[serde(with = "ts_seconds")]
    pub time_received: DateTime<Utc>,
    pub length: usize,
    pub recorded_fields: Vec<String>,
}

impl Default for FlightLogMetadata {
    fn default() -> Self {
        Self {
            tracker_id: 0,
            id: Uuid::new_v4(),
            time_received: chrono::offset::Utc::now(),
            length: 0,
            recorded_fields: vec!["timestamp".to_string()],
        }
    }
}

pub struct FlightDataRecord {
    // Vec of rows
    data: Vec<Vec<String>>,
    fields: Vec<String>,
}

fn cmp_vecs<T: PartialEq>(a: &Vec<T>, b: &Vec<T>) -> bool {
    let matching = a.iter().zip(b.iter()).filter(|&(a, b)| a == b).count();
    matching == a.len() && matching == b.len()
}

impl FlightDataRecord {
    pub fn new(fields: Vec<String>, data: Vec<Vec<String>>) -> Self {
        FlightDataRecord { data, fields }
    }

    pub fn from_hashmaps(data: Vec<HashMap<String, String>>) -> Result<Self, String> {
        let keys = data[0].keys().cloned().collect();
        let mut out_data: Vec<Vec<String>> = vec![];
        for entry in data {
            if !cmp_vecs(&entry.keys().cloned().collect(), &keys) {
                return Err("Error, all entries to `data` must contain the same keys".to_owned());
            }
            out_data.push(entry.values().cloned().collect())
        }

        Ok(FlightDataRecord::new(keys, out_data))
    }

    pub fn to_string_csv(&self) -> String {
        let mut csv = String::new();
        csv += &self.fields.join(",");
        for row in self.data.iter() {
            csv += &row.join(",");
        }

        csv
    }
}

// TODO Make a struct
pub fn get_flights(flights_path: &Path) -> io::Result<Vec<FlightLogMetadata>> {
    Ok(fs::read_dir(flights_path)?
        .into_iter()
        .filter(|d| d.is_ok())
        .filter_map(|d| {
            let dir_path = d.unwrap().path();

            let flight_metadata: Option<FlightLogMetadata> = serde_json::from_reader(
                BufReader::new(File::open(dir_path.join("flight.json")).ok()?),
            )
            .ok();
            flight_metadata
        })
        .collect())
}

pub fn create_empty_flight(flights_path: &Path, flight: &FlightLogMetadata) -> io::Result<()> {
    let uuid = Uuid::new_v4();
    let mut flight = flight.clone();
    flight.id = uuid;

    let flight_dir = flights_path.join(flight.id.to_string());
    fs::create_dir(&flight_dir)?;

    serde_json::to_writer(
        io::BufWriter::new(File::create(flight_dir.join("flight.json"))?),
        &flight,
    )
    .map_err(|e| io::Error::new(io::ErrorKind::Other, format!("Serde error: {}", e)))?;

    Ok(())
}

pub fn create_flight(
    flights_path: &Path,
    flight: &FlightLogMetadata,
    data: FlightDataRecord,
) -> io::Result<()> {
    let uuid = Uuid::new_v4();
    let mut flight = flight.clone();
    flight.id = uuid;

    let flight_dir = flights_path.join(flight.id.to_string());
    fs::create_dir(&flight_dir)?;

    flight.recorded_fields = data.fields.clone();

    serde_json::to_writer(
        io::BufWriter::new(File::create(flight_dir.join("flight.json"))?),
        &flight,
    )
    .map_err(|e| io::Error::new(io::ErrorKind::Other, format!("Serde error: {}", e)))?;

    fs::write(flight_dir.join("flight.csv"), data.to_string_csv())?;

    Ok(())
}
