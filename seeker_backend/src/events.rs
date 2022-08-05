use serde::{Deserialize, Serialize};

#[typetag::serde(tag = "event_type")]
pub trait EventData {}
#[derive(Serialize, Deserialize)]
pub struct GPSData {}
#[typetag::serde]
impl EventData for GPSData {}

#[derive(Serialize, Deserialize)]
pub struct LocalizationData {}
#[typetag::serde]
impl EventData for LocalizationData {}

#[derive(Serialize, Deserialize)]
pub struct InFlightData {}
#[typetag::serde]
impl EventData for InFlightData {}

#[derive(Serialize, Deserialize)]
pub struct FlightStatusData {}
#[typetag::serde]
impl EventData for FlightStatusData {}

#[derive(Serialize, Deserialize)]
pub struct DebugData {
    message: String,
}
#[typetag::serde]
impl EventData for DebugData {}

impl DebugData {
    pub fn with_data(data: String) -> Self {
        Self { message: data }
    }
}

pub enum RemoteUpdate {
    InFlightUpdate(InFlightData),
    GPSUpdate(GPSData),
    FlightStatusUpdate(FlightStatusData),
    LocalizationUpdate(LocalizationData),
    Debug(DebugData),
}
