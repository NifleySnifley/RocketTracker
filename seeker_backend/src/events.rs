use event_derive::WriteableEventData;
use serde::{Deserialize, Serialize};
use std::fmt::Debug;

pub trait EventData {}
pub trait WriteableEventData: EventData + Debug + Clone {}

#[derive(Clone, Serialize, Deserialize, Debug, WriteableEventData)]
pub struct GPSData {
    longitude: f64,
    latitude: f64,
    altitude: Option<f64>,
}

#[derive(Clone, Serialize, Deserialize, Debug, WriteableEventData)]
pub struct IMUData {}

#[derive(Clone, Serialize, Deserialize, Debug, WriteableEventData)]
pub struct LocalizationData {}
#[derive(Clone, Serialize, Deserialize, Debug, WriteableEventData)]
pub struct InFlightData {}

#[derive(Clone, Serialize, Deserialize, Debug, WriteableEventData)]
pub struct FlightStatusData {}

#[derive(Clone, Serialize, Deserialize, Debug, WriteableEventData)]
pub struct DebugData {
    message: String,
}

impl DebugData {
    pub fn with_data(data: String) -> Self {
        Self { message: data }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(tag = "event_type", content = "data")]
pub enum Event {
    InFlightUpdate(InFlightData),
    FlightStatusUpdate(FlightStatusData),
    LocalizationUpdate(LocalizationData),

    IMUUpdate(IMUData),
    GPSUpdate(GPSData),

    Debug(DebugData),
}
