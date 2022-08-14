use std::fs::File;
use std::io::BufReader;
use std::path::Path;
use std::{fs, io};
use uuid::Uuid;

use chrono::serde::ts_seconds;
use chrono::{DateTime, Local, NaiveDateTime, Utc};
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct FlightLogMetadata {
    pub id: Uuid,
    #[serde(with = "ts_seconds")]
    pub time_received: DateTime<Utc>,
    pub length: usize,
    pub recorded_fields: Vec<String>,
}

impl Default for FlightLogMetadata {
    fn default() -> Self {
        Self {
            id: Uuid::new_v4(),
            time_received: chrono::offset::Utc::now(),
            length: 0,
            recorded_fields: vec!["timestamp".to_string()],
        }
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
            return flight_metadata;
        })
        .collect())
}

pub fn create_flight(flights_path: &Path, flight: &FlightLogMetadata) -> io::Result<()> {
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
