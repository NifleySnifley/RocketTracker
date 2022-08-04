use async_trait::async_trait;
use tokio::task::JoinHandle;

#[async_trait]
pub trait GPSAdapter {
    fn get_coords() -> (f64, f64);
    fn get_alt() -> f64;
    fn get_time() -> chrono::NaiveTime;
    fn get_fix() -> nmea::FixType;
    fn get_n_satellites() -> usize;

    async fn start_reader() -> JoinHandle<Result<(), String>>;
}

#[async_trait]
pub trait IMUAdapter {
    // Returns the current orientation in (x,y,z) euler angles
    fn get_orientation_euler() -> (f64, f64, f64);
    // Returns a vector (x,y,z) of the latest magnetometer reading
    fn get_mag_vector() -> (f64, f64, f64);

    // Starts an asynchronous reader to read from the sensor
    async fn start_reader() -> JoinHandle<Result<(), String>>;
}

pub struct ConnectionInfo {
    remote_id: u32,
}

#[derive(Debug, Clone)]
pub enum ConnectionError {
    Timeout,
    InvalidKey,
    GenericError(String),
}

#[derive(Debug, Clone, Copy)]

pub enum ConnectionState {
    Disconnected,
    Standard,
    HighSpeed,
}

#[async_trait]
pub trait TrackerConnector {
    // target_id: ID of the device you are requesting to connect to
    // target_key: optional security key to send to the device
    // timeout: timeout for the connection request
    async fn try_connect(
        target_id: usize,
        target_key: Option<usize>,
        timout: std::time::Duration,
    ) -> Result<ConnectionInfo, ConnectionError>;

    fn get_connection_state() -> ConnectionState;

    // Starts an asynchronous worker to handle incoming data
    // This includes receiving messages that are not necessarily data frames
    async fn start_worker() -> JoinHandle<Result<(), String>>;

    // Basic function to send some byte data over the connection
    async fn send(data: Vec<u8>) -> Result<(), String>;

    // Asynchronously receive data from the connection
    async fn receive(timout: std::time::Duration) -> Result<Vec<u8>, String>;
}
