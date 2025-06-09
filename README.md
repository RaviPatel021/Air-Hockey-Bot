# Autonomous Air Hockey Table

This repository contains all the code and hardware files for an autonomous air hockey table system. The system uses a 2-axis gantry controlled by an ESP32, with real-time vision and inference running on either a laptop or a Jetson Nano.

## Folder Overview

- **embedded/**  
  Contains the embedded C++ code for the ESP32 that controls the 2-axis gantry using stepper motors and drivers.

- **laptop/**  
  Python code that runs on a laptop to handle camera input, detect the puck, run inference using a trained model, and send paddle commands to the ESP32 over serial.

- **Jetson_Nano/**  
  Similar to the `laptop/` folder, but modified and optimized to run entirely on a Jetson Nano for a fully embedded setup.

- **KiCAD_Schematics/**  
  Electrical schematics and PCB layout files for the gantry control system, designed using KiCAD.

---

More detailed documentation and setup instructions will be added soon.
