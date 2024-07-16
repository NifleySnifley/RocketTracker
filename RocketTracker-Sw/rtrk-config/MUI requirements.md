# MUI (Minimal Useable Interface) requirements for `basestation-v2`

## "needs to be":

- Functional

- NO bells and whistles

- NOT fancy

- Spartan

- Too much information given to the user, and a means to filter it down to what's neccesary

- ALL the controls and parameters

- Easy to maintain

- Easy to add functionality/debug stuff with it and in it

## How:

- Python
  
  - Utilize the SerDes and protocol implementation already used by the `cli`

- Some GL and windowing libraries (pyglet, whatever, maybe that thing I used for AoC)

- **pyimgui**
  
  - Dear imgui docking/windowing
  
  - (use the feature where imgui windows can detach from the windowing system)
  
  - compose the software as a main toolbar, and a bunch of little windowlets/menus/displaywidgets that do certain specific things
    
    - Map display (**might need to be custom, oh well!!**)
    
    - Live graph display
      
      - Tree view of values to be graphed? ability to break a value(s) out into another graph? etc.
    
    - Configuration menu
    
    - Logging controls & status
    
    - Debug `monitor`
    
    - Console for running commands from the cli
    
    - Alerts! somehow make them visible (maybe a specific alerts terminal with flashy colors to alert the user)
    
    - Device window
      
      - Receiver control window
      
      - Tracker USB window (idk why, maybe just status etc)
  
  - Potential GL displays for log data, or live 3D maps!
  
  - Orientation display with GL in a imgui window
  
  - Ability to connect a GPS on the PC

- Asyncio? need some IO/events management in the background

## Configuration system:

- Define configuration with JSON format that i've started to work on

- Make a class that owns an imgui widget specifically for editing this sort of configuration specified by JSON (or even better, a recursive function that can recursively parse the format with global/local defs, etc!!!)

- **Make the parser a library that can easily be used by GUI and codegen**

- Config easily saved as JSON

- Config sendable to the rocket in raw format with each value as a datum

- Codegen: (generate a way to get/set/list the config values on an embedded target!)
  
  - Enums & custom types to `#define`s, bounds/validity check functions, and set/get functions for *each* parameter!
  
  - Have a global config struct that only needs functions to set/get `bool`, one int type, `float`, `string`, (enum is int), etc.
    
    - Implement this for the ESP32 as NVS!
  
  - One global const structure (generated) that specifies all of the configuration values, their types, etc. 
    
    - Used to query the configuration state of a device & supported config values
    
    - Generic function that returns a void pointer and a type for ALL values
      
      - Need to `free` after use tho
    
    - Provide all values as a void pointer, and all types as an enum
    
    - All raw, use the types and make a function that converts config entry -> datum so can iterate over and send all of the current config
  
  - Struct for each value type, list of these structs with value names, default settings, etc for every value!


