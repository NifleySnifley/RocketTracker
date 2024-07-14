## Fixes:

- [ ] Identify where there are critical regions in the code and mark them accordingly, if neccesary!!!!!
- [x] Make a way to turn off telemetry for stuff like log download & log deletion, like a flag or something for "telemetry hold"
- [ ] Investigate places where there could be race conditions, etc.

## Improvements:

- [ ] Set the proper max spi frequency for each device individually

- [ ] Increase GPS baudrate

- [ ] Increase GPS update rate (it is capable of 10Hz, currently using 1Hz default)

- [ ] Log parsing in the CLI tool, output in CSV, JSON, binary, as well as pickle

- [ ] Logging of the first few seconds before liftoff with a ringbuffer

- [ ] More advanced flight state detection!

- [ ] More reliable sensor filtering! (IIR/Lp/Hp filters, kalman altitude, etc.)
  
  - Need the following to be reliable:
    
    - Altitude
    
    - Vertical Speed
    
    - Acceleration (full DR)
    
    - Rocket orientation
    
    - 

## Features:

- [x] Flight phase detection & autologging
  
  - [ ] Landed detection

- [x] **Live altitude**

- [x] **Live autolog status updates!**

- [ ] Configuration parameters, unified set, get, list, etc. interface
  
  - [ ] Accessible from wherever in the firmware, easily
  
  - [ ] FUTURE: Hot reloading? nah... just cause a panic reboot!!!

- [ ] Calibration of sensors via configuration parameters
  
  - [ ] **Autocalibration:** calibrate the ADXL with the LSM's accelerometer.
    
    - [ ] One-point calibration is easy, make a routine for two-point calibration though (two board orientations)

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

# Milestones:

1. Basic functionality (testing 7/13)

2. Configurability (config system & interface)

3. Reliability & Quality ***every feature needs rigorous testing from this point!***
   
   - Handling of errors & edge cases
   
   - **Unit tests! Can you do this with qemu?**
   
   - **Focus on reliable software development processes**
   
   - Link: Repeatability, consistency, acknowledgement, fail-safe when packets drop
     
     - Any cricical, identical command should do the same thing if ran once, twice, or N times, ideally
   
   - **Treat this like a real aerospace system! failure is NOT an option!!! Everything needs to be reliable and fail-safe.**
   
   - Existing features that include reliability-focuses improvements:
     
     - Link system (DIO3)
     
     - Sensors (calibration & autocalibration, procedure for doing calibration)
     
     - Altitude & orientation filtering for flight state detection, etc.
     
     - Autolog activity/launch detection
     
     - Sensor filtering (DSP, lowpasses, etc.)
     
     - Altitude at start detection, altimetry relative to start
   
   - **Find ways to easily test flight functionalities, and/or simulate them**
     
     - SITL simulation with simulink + fake imperfect sensors + qemu or something like that? Nothing too complicated...

4. User friendliness
   
   - Usability, configurability, and ease of use
   
   - In-flight notifications
   
   - **Documentation**
   
   - **GUI software**
   
   - Make it a useable product not just for me

5. Featureness
   
   - Dual deploy!
     
     - Included in firmware, enable/disable and set up with config system
   
   - Uses of servo (homing steering? etc.)
   
   - More telemetry info (orientation, etc.)
   
   - Visualizations in GUI and log analysis tools
   
   - 
