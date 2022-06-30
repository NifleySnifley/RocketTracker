use serde::{Deserialize, Serialize};
use serde_json;
use std::error::Error;
use std::fs::File;
use std::io::BufReader;
use std::{env, fs};

static CFG_VERS: f32 = 0.1;

#[derive(Serialize, Deserialize, Debug)]
pub struct SeekerConfiguration {
    pub magnetometer_offsets: [f64; 3],
    pub heading_offset: f64,
    pub radio_network: u8,
    pub radio_id: u8,
    version: f32,
}

impl Default for SeekerConfiguration {
    fn default() -> Self {
        Self {
            magnetometer_offsets: [0f64; 3],
            heading_offset: 0f64,
            radio_network: 0,
            radio_id: 0,
            version: CFG_VERS,
        }
    }
}

impl SeekerConfiguration {
    pub fn load() -> Result<SeekerConfiguration, Box<dyn Error>> {
        let mut conf = Self::default();
        conf.reload()?;

        Ok(conf)
    }

    pub fn reload(&mut self) -> Result<(), Box<dyn Error>> {
        // Find the configuration file in the same directory as the executable instead of just a relative path
        let mut configpath = env::current_exe()?;
        configpath.pop();
        configpath.push("config.json");

        // Create config file if it doesn't exist
        if !configpath.exists() {
            let default_cfg = SeekerConfiguration::default();
            fs::write(&configpath, serde_json::to_string_pretty(&default_cfg)?)?;
        }

        let reader = BufReader::new(File::open(configpath)?);
        *self = serde_json::from_reader(reader)?;

        Ok(())
    }

    pub fn save(&self) -> Result<(), Box<dyn Error>> {
        let mut configpath = env::current_exe()?;
        configpath.pop();
        configpath.push("config.json");

        fs::write(&configpath, serde_json::to_string_pretty(&self)?)?;
        Ok(())
    }
}
