# Tracker

## Features:

- ESP32 SoC with WiFi and Bluetooth for downloading flight logs

- 8Mb log storage (about 104 minutes of logging)

- 3 I2C peripheral ports

- SMA Antenna port (or wire antenna)

- Voltage regulator supporting up to 7 cell LiPo (1-2 cell reccomended)

## Power useage

| Part        | Current useage min (mA) | Current useage max |
| ----------- | ----------------------- | ------------------ |
| ESP32-WROOM | 160                     | 260                |
| RFM97CW     | 120                     | 140                |
| FGPMMOPA6H  | 20                      | 25                 |

## Assembly:

1. ESP32-WROOM module

2. GPS

3. Qwiic (JST-SH) connectors

4. LEDs and respective resistors

5. RFM97

6. Voltage regulator

7. Capacitors

8. Programming headers

9. Antenna or SMA connector

**Alternate (fast) procedure:**

1. Reflow all components on the top (ESP32, GPS, etc.)

2. Solder the bottom SMD components with paste and hot air gun

3. Hand solder the radio and THT components

## BOM:

| Ref         | Qnty | Value               | Cmp name            | Footprint                                                  | Description                                                                                                                                                                   | Vendor                                                                                              |
| ----------- | ---- | ------------------- | ------------------- | ---------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------- |
| C1, C3,     | 2    | 10uF                | C_Small             | Capacitor_SMD:C_1210_3225Metric_Pad1.33x2.70mm_HandSolder  | Unpolarized capacitor, small symbol                                                                                                                                           | Digikey                                                                                             |
| C2,         | 1    | .1uF                | C_Small             | Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder  | Unpolarized capacitor, small symbol                                                                                                                                           | Digikey                                                                                             |
| D1, D2,     | 2    | LED                 | LED                 | LED_SMD:LED_0603_1608Metric_Pad1.05x0.95mm_HandSolder      | Light emitting diode                                                                                                                                                          | Digikey                                                                                             |
| J1,         | 1    | Conn_02x03_Odd_Even | Conn_02x03_Odd_Even | Connector_PinHeader_1.27mm:PinHeader_2x03_P1.27mm_Vertical | Generic connector, double row, 02x03, odd/even pin numbering scheme (row 1 odd numbers, row 2 even numbers), script generated (kicad-library-utils/schlib/autogen/connector/) | Digikey                                                                                             |
| J2,         | 1    | Conn_02x05_Odd_Even | Conn_02x05_Odd_Even | Connector_PinHeader_1.27mm:PinHeader_2x05_P1.27mm_Vertical | Generic connector, double row, 02x05, odd/even pin numbering scheme (row 1 odd numbers, row 2 even numbers), script generated (kicad-library-utils/schlib/autogen/connector/) | Digikey                                                                                             |
| J4,         | 1    | Conn_Coaxial        | Conn_Coaxial        | Connector_Coaxial:SMA_Amphenol_132289_EdgeMount            | coaxial connector (BNC, SMA, SMB, SMC, Cinch/RCA, LEMO, ...)                                                                                                                  | Digikey                                                                                             |
| J5,         | 1    | Conn_01x02          | Conn_01x02          | Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical | Generic connector, single row, 01x02, script generated (kicad-library-utils/schlib/autogen/connector/)                                                                        | Digikey                                                                                             |
| J6, J7, J8, | 3    | Conn_01x04          | Conn_01x04          | Parts:Connector_Qwiic_Vertical                             | Generic connector, single row, 01x04, script generated (kicad-library-utils/schlib/autogen/connector/)                                                                        | Digikey                                                                                             |
| R1, R2,     | 2    | R_Small             | R_Small             | Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder   | Resistor, small symbol                                                                                                                                                        | Digikey                                                                                             |
| U1,         | 1    | ESP32-WROOM-32      | ESP32-WROOM-32      | RF_Module:ESP32-WROOM-32                                   | RF Module, ESP32-D0WDQ6 SoC, Wi-Fi 802.11b/g/n, Bluetooth, BLE, 32-bit, 2.7-3.6V, onboard antenna, SMD                                                                        | [SparkFun](https://www.sparkfun.com/products/17830)                                                 |
| U2,         | 1    | RFM97CW-915S2       | RFM97CW-915S2       | RF_Module:HOPERF_RFM9XW_SMD                                | Low power long range transceiver module, SPI and parallel interface, 915 MHz, spreading factor 6 to12, bandwith 7.8 to 500kHz, -111 to -139 dBm, SMD-16, DIP-16               | [SparkFun](https://www.sparkfun.com/products/18084)                                                 |
| U3,         | 1    | FGPMMOPA6H          | FGPMMOPA6H          | Parts:FGPMMOPA6H                                           | "Adafruit Ultimate MTK3333 GPS"                                                                                                                                               | Adafruit, [Cheap Source](https://www.tinyosshop.com/index.php?route=product/product&product_id=840) |
| U5,         | 1    | LD1117V33           | LD1117V33           | Package_TO_SOT_SMD:TO-252-2                                | 1.5A 35V Adjustable Linear Regulator, TO-220                                                                                                                                  | Digikey                                                                                             |
| U4,         | 1    | BMI160              | BMI160              | Package_LGA:Bosch_LGA-14_3x2.5mm_P0.5mm                    | Small, low power inertial measurement unit, LGA-14                                                                                                                            | Chip Shortage                                                                                       |
| R3, R4,     | 2    | 10k                 | R_Small             | Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder   | Resistor, small symbol                                                                                                                                                        | Digikey                                                                                             |

# Tracker (BaseStationHAT)

## BOM:

| Ref     | Qnty | Value                        | Cmp name                 | Footprint                                                  | Description                                                                                                                                                                   | Vendor                                              |
| ------- | ---- | ---------------------------- | ------------------------ | ---------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------- |
| AE1,    | 1    | Antenna_Chip                 | Antenna_Chip             | RF_Antenna:Pulse_W3011                                     | Ceramic chip antenna with pin for PCB trace                                                                                                                                   | Digikey                                             |
| BT1,    | 1    | Battery_Cell                 | Battery_Cell             | Battery:BatteryHolder_Keystone_3000_1x12mm                 | Single-cell battery                                                                                                                                                           | [SparkFun](https://www.sparkfun.com/products/10592) |
| C1,     | 1    | 3.3pF                        | C_Small                  | Capacitor_SMD:C_0402_1005Metric_Pad0.74x0.62mm_HandSolder  | Unpolarized capacitor, small symbol                                                                                                                                           | Digikey                                             |
| C2,     | 1    | .1uF                         | C_Small                  | Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder  | Unpolarized capacitor, small symbol                                                                                                                                           | Digikey                                             |
| C3,     | 1    | 10uF                         | C_Small                  | Capacitor_SMD:C_1210_3225Metric_Pad1.33x2.70mm_HandSolder  | Unpolarized capacitor, small symbol                                                                                                                                           | Digikey                                             |
| J1,     | 1    | Screw_Terminal_01x02         | Screw_Terminal_01x02     | TerminalBlock:TerminalBlock_bornier-2_P5.08mm              | Generic screw terminal, single row, 01x02, script generated (kicad-library-utils/schlib/autogen/connector/)                                                                   | Digikey                                             |
| J2, J4, | 2    | Conn_Coaxial                 | Conn_Coaxial             | CONREVSMA003:LINX_CONREVSMA003.062                         | coaxial connector (BNC, SMA, SMB, SMC, Cinch/RCA, LEMO, ...)                                                                                                                  | Digikey                                             |
| J3,     | 1    | 40HAT                        | OX40HAT                  | Connector_PinSocket_2.54mm:PinSocket_2x20_P2.54mm_Vertical | 20x2 (40 pin) female pin socket, 2.54mm pitch (Raspberry pi receptacle)                                                                                                       | Anny supplier                                       |
| J5,     | 1    | Conn_02x06_Counter_Clockwise | Conn_02x06_Odd_Even      | Connector_PinHeader_2.54mm:PinHeader_2x06_P2.54mm_Vertical | Generic connector, double row, 02x06, odd/even pin numbering scheme (row 1 odd numbers, row 2 even numbers), script generated (kicad-library-utils/schlib/autogen/connector/) | Digikey                                             |
| J6,     | 1    | Conn_01x10                   | Conn_01x10               | Connector_PinHeader_2.54mm:PinHeader_1x10_P2.54mm_Vertical | Generic connector, single row, 01x10, script generated (kicad-library-utils/schlib/autogen/connector/)                                                                        | Digikey                                             |
| J7, J8, | 2    | Conn_01x04                   | Conn_01x04               | Parts:Connector_Qwiic_Vertical                             | Generic connector, single row, 01x04, scrhttps://www.sparkfun.com/products/16766ipt generated (kicad-library-utils/schlib/autogen/connector/)                                 | [SparkFun]()                                        |
| JP1,    | 1    | Jumper_3_Bridged12           | Jumper_3_Bridged12       | Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical | Jumper, 3-pole, pins 1+2 closed/bridged                                                                                                                                       | Digikey                                             |
| JP2,    | 1    | SolderJumper_3_Open          | SolderJumper_3_Open      | Jumper:SolderJumper-3_P1.3mm_Open_Pad1.0x1.5mm             | Solder Jumper, 3-pole, open                                                                                                                                                   | N/A                                                 |
| JP3,    | 1    | SolderJumper_3_Bridged12     | SolderJumper_3_Bridged12 | Jumper:SolderJumper-3_P1.3mm_Bridged12_Pad1.0x1.5mm        | 3-pole Solder Jumper, pins 1+2 closed/bridged                                                                                                                                 | N/A                                                 |
| R1,     | 1    | 1k                           | R_Small                  | Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder   | Resistor, small symbol                                                                                                                                                        | Digikey                                             |
| R2,     | 1    | 10k                          | R_Small                  | Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder   | Resistor, small symbol                                                                                                                                                        | Digikey                                             |
| U1,     | 1    | RFM97CW-915S2                | RFM97CW-915S2            | RF_Module:HOPERF_RFM9XW_SMD                                | Low power long range transceiver module, SPI and parallel interface, 915 MHz, spreading factor 6 to12, bandwith 7.8 to 500kHz, -111 to -139 dBm, SMD-16, DIP-16               | [SparkFun](https://www.sparkfun.com/products/18084) |
| U2,     | 1    | ATtiny414-SS                 | ATtiny414-SS             | Package_SO:SOIC-14_3.9x8.7mm_P1.27mm                       | 20MHz, 4kB Flash, 256B SRAM, 128B EEPROM, SOIC-14                                                                                                                             | Digikey                                             |
| U3,     | 1    | RFM69HCW                     | RFM69HCW                 | RF_Module:HOPERF_RFM69HW                                   | Low power ISM Radio Transceiver Module, SPI interface, AES encryption, 434 or 915 MHz, up to 100mW, up to 300 kb/s, SMD-16, DIP-16                                            | [SparkFun](https://www.sparkfun.com/products/13909) |
| U4,     | 1    | SL871L                       | SL871L                   | Parts:SL871                                                |                                                                                                                                                                               | Digikey                                             |
| U5,     | 1    | MCP23008                     | MCP23008                 | Package_DIP:DIP-18_W7.62mm_LongPads                        |                                                                                                                                                                               | Digikey                                             |
| U6,     | 1    | LD1117V33                    | LD1117V33                | Package_TO_SOT_SMD:TO-252-2                                | 1.5A 35V Adjustable Linear Regulator, TO-220                                                                                                                                  | Digikey                                             |

## Features:

- **ATtiny414 PRU**:
  
  - ****Programmed with Pi's UART0
  
  - All IO pins broken out
  
  - I2C connected to Pi's I2C1
  
  - Jumper for selecting UPDI (program mode) and VCC pullup for reset pin

- **RFM6X and RFM9X radio connections**:
  
  - Seperate CS and DIO0 connections to Pi allowing simultaneous operation
    
    - 9X: CS = GPIO18, DIO0 = GPIO27
    
    - 6X: CS = GPIO17, DIO0 = GPIO22
  
  - Shared reset pin
  
  - Individual antenna connections
    
    - edge-mount SMA/RP-SMA footprints

- **Telit SL871(L) GPS**:
  
  - NMEA Serial connected to UART3
  
  - I2C connected to I2C0
  
  - PulseLarsen chip antenna
  
  - Coin cell RTC battery
  
  - All important IO connected:
    
    - FORCEON: GPIO25
    
    - ANT_SC: GPIO23
    
    - ANT_OC: GPIO24
    
    - RESET: GPIO16
      
      - Note: must be pulled high or floating to enable GPS

- **MCP23008 GPIO Expander**:
  
  - All IO and power rail broken out to an easily accesible location on the board
  
  - Interfaced with I2C1 and interrupt pin connected to Pi's GPIO6

- **Screw terminals and voltage regulator for powering the Pi**:
  
  - Connected directly to the Pi's 5v rail and the 3.3v voltage regulator
  
  - **WARNING**: No reverse polarity or overvoltage protection provided, use at your own risk, and double check before connecting!

## Assembly:

It's reccomended to use solder paste and a hot air gun to solder the SMD components 

1. GPS (if applicable)

2. GPS antenna

3. Voltage regulator

4. ATtiny PRU

5. All SMD passives (pullups, caps)

6. Qwiic (I2C) connectors

7. Radio modules

8. (RP)SMA connectors (if applicable)

9. GPIO expander

10. Pin headers

11. HAT socket

12. Screw terminals
