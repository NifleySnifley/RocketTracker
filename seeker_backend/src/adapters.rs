use async_trait::async_trait;
use tokio::task::JoinHandle;

#[async_trait]
pub trait GPSAdapter {
    /// Returns a tuple of (latitude, longitude) containing the last position of the recorded GPS fix
    fn get_coords() -> (f64, f64);
    /// Returns the altitude of the last recorded GPS fix (meters)
    fn get_alt() -> f64;
    /// Returns the UTC time of the last recorded GPS fix
    fn get_time() -> chrono::NaiveTime;
    /// Returns the state of the last recorded GPS fix
    fn get_fix() -> nmea::FixType;
    /// Returns the number of satellites used in the last recorded GPS fix
    fn get_n_satellites() -> usize;

    /// Spawns an asynchronous worker to read data from a serial (NMEA) GPS
    async fn start_reader() -> JoinHandle<Result<(), String>>;
}

#[async_trait]
pub trait IMUAdapter {
    /// Returns the current orientation in (x,y,z) euler angles
    fn get_orientation_euler() -> (f64, f64, f64);
    /// Returns a vector (x,y,z) of the latest magnetometer reading
    fn get_mag_vector() -> (f64, f64, f64);

    /// Starts an asynchronous reader to read from the sensor
    async fn start_reader() -> JoinHandle<Result<(), String>>;
}

#[repr(u8)]
#[derive(Debug, Clone, Copy)]
#[allow(dead_code)]
pub enum PacketTypeIden {
    Broadcast,
    DataDownlink,
    TransactionInit,
    TransactionEnd,
    TransactionData,
    ConfigSet,
    ConfigGet,
    Ping,
    ConnectionConfig,
    GetInfo,
}

#[derive(Debug, Clone, Copy)]
pub struct Packet {
    pub type_iden: PacketTypeIden,
    pub category: u8,
    pub sender_type: u8,
    _reserved: u8,
    pub sender_id: u32,
    pub receiver_id: u32,
    pub message_index: u32,
    pub data: [u8; 240],
}

impl Packet {
    pub fn new(
        type_iden: PacketTypeIden,
        category: u8,
        src: u32,
        dest: u32,
        msg_idx: u32,
        data: &Vec<u8>,
    ) -> Result<Self, ()> {
        if data.len() > 240 {
            Err(())
        } else {
            let mut data = data.clone();
            data.resize(240, 0);
            Ok(Packet {
                type_iden,
                category,
                sender_type: 0,
                sender_id: src,
                receiver_id: dest,
                message_index: msg_idx,
                data: data.try_into().unwrap(),
                _reserved: 0,
            })
        }
    }

    pub fn to_bytes(&self) -> Vec<u8> {
        let mut bytes = vec![];

        bytes.push(self.type_iden as u8);
        bytes.push(self.category as u8);
        bytes.push(self.sender_type as u8);
        bytes.extend(self.sender_id.to_le_bytes());
        bytes.extend(self.receiver_id.to_le_bytes());
        bytes.extend(self.message_index.to_le_bytes());
        bytes.extend(self.data);

        bytes
    }
}

pub struct ConnectionInfo {
    pub remote_id: u32,
    pub state: ConnectionState,
}

#[derive(Debug, Clone)]
#[allow(dead_code)]
pub enum ConnectionError {
    Timeout,
    InvalidKey,
    GenericError(String),
}

#[derive(Debug, Clone, Copy)]
#[allow(dead_code)]
pub enum ConnectionState {
    Disconnected,
    Standard,
    HighSpeed,
}

#[async_trait]
pub trait TrackerConnector {
    /// Attempts to connect to a tracker using the connector
    /// ### Arguments:
    /// - target_id: ID of the device you are requesting to connect to
    /// - target_key: optional security key to send to the device
    /// - timeout: timeout for the connection request
    async fn try_connect(
        target_id: usize,
        target_key: Option<usize>,
        timout: std::time::Duration,
    ) -> Result<(), ConnectionError>;

    fn get_connection_info() -> ConnectionInfo;

    /// Starts an asynchronous worker to handle incoming data
    /// This includes receiving messages that are not necessarily data frames
    async fn start_worker() -> JoinHandle<Result<(), String>>;

    /// Basic function to send byte data over the connection
    async fn send_raw(data: Vec<u8>) -> Result<(), String>;

    /// Sends
    async fn send(data: Packet) -> Result<(), String>;

    // Asynchronously receive data from the connection
    async fn receive(timout: std::time::Duration) -> Result<Vec<u8>, String>;
}
