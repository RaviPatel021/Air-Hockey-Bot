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


## 🚀 Getting Started

These instructions will help you get the system running on your machine.

### 1. Clone the Repository

```bash
git clone https://github.com/RaviPatel021/Air-Hockey-Bot.git
cd Air-Hockey-Bot
````

### 2. Install Python Dependencies

Use Python 3.8–3.10. It's strongly recommended to use a virtual environment:

```bash
python -m venv venv
venv\Scripts\activate  # Mac/Linux:: source venv/bin/activate
pip install -r laptop/requirements.txt
```

> ⚠️ On Jetson Nano, some packages (e.g., OpenCV, torch) may need to be installed with Jetson-specific wheels

---

## 🔧 Upload Firmware to ESP32

The ESP32 controls the stepper motors through UART and must be flashed with the code in `embedded/`.

You can upload the code using:

* **Arduino IDE** (select ESP32 board, e.g. “ESP32 Dev Module”)


Make sure you have the required libraries installed:

* TMCStepper
* FastAccelStepper

---

## ⚙️ Connect the System

1. Connect the camera and ESP32 to your computer or Jetson Nano.
2. Verify the serial port device (e.g., `/dev/ttyUSB0` on Linux or `COM3` on Windows).
3. Adjust serial port settings in the Python code if necessary.

---

## ▶️ Running the Control Script

From the `laptop/` or `Jetson_Nano/` directory, run:

```bash
python main.py
```

This will start the vision processing pipeline, detect the puck position, run the PPO model inference, and send velocity commands to the gantry controller.

---
