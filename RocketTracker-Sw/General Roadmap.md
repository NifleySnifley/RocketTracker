## Fixes:

- [ ] Identify where there are critical regions in the code and mark them accordingly, if neccesary

## Improvements:

- [ ] Set the proper max spi frequency for each device individually

- [ ] Increase GPS baudrate

- [ ] Increase GPS update rate (it is capable of 10Hz, currently using 1Hz default)

- [ ] Log parsing in the CLI tool, output in CSV, JSON, binary, as well as pickle

## Features:

- [x] Flight phase detection & autologging
  
  - [ ] Landed detection

- [ ] Configuration parameters, unified set, get, list, etc. interface
  
  - [ ] Accessible from wherever in the firmware, easily
  
  - [ ] FUTURE: Hot reloading? nah... just cause a panic reboot!!!

- [ ] Calibration of sensors via configuration parameters

- [ ] On-board sensor fusion (Madgewick AHRS, altimetry Kalman filter)

- [ ] Debug sensor data output over USB (use log format, so orientation, etc. can be shown in the application, fairly fast update rate)
  
  - [ ] Use this to make an interactive calibration tool!

- [ ] **Improved GUI!!!**
  
  - Use flutter?
  
  - Needs offline maps and nice realtime graphs
  
  - Configuration menu
  
  - Status notifications
  
  - Easy buttons for sending commands, etc.
  
  - Log downloading and visualization with the GUI!

- [ ] *Fix DIO3 not being connected on the PCB!*
