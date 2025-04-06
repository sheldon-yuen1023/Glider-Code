# UCL Underwater Glider – Arduino-Based Autonomous Test Vehicle

This repository contains the **early-stage control and diagnostic software** for the **UCL Underwater Glider project**, written using the **Arduino framework in C++**. The codebase is designed to support **tethered testing** of onboard subsystems prior to full autonomous integration.

## 🌊 Project Overview

The UCL Underwater Glider is a student-led initiative focused on building a **modular, autonomous underwater vehicle (AUV)** for oceanographic research. The glider features:
- A **blended-wing body** for efficient underwater motion
- A **variable buoyancy propulsion system**
- Distributed microcontroller-based subsystem management
- Sensor-rich navigation and control

This repository supports **tethered bench and field testing** of core components and subsystems, ensuring robust performance and calibration before open-water trials.

## 📁 Repository Structure

### `compass-calibration/`
Arduino sketch for calibrating the **magnetometer** once the IMU is installed inside the glider. It helps compute static magnetic offsets caused by onboard electronics or the structure itself, improving heading accuracy.

### `demo-code-snippets/`
Standalone test sketches for **individual components** (e.g., single motor, sensor). Useful for **hardware troubleshooting**, verification, or functional testing during assembly.

### `glider-bms/`
Full code for the **custom battery management system**, running on a **Seeed Studio XIAO ESP32-C3** board. It handles:
- Temperature monitoring via DS18B20 sensors
- Voltage and current sensing via an INA228
- Relay-based power cutoff on fault detection
- Emergency condition logging and reporting

### `glider-flight-controller/`
Core control code for the glider’s **main flight computer**. This module will coordinate:
- Buoyancy control
- Pitch and roll actuation
- IMU-based attitude sensing
- Mission logic (eventually autonomous)

## 🧰 Development Notes

- All code is written in **C++ using the Arduino framework**
- Target boards include **ESP32-C3**, **ESP32S3-devkit**, and other supported microcontrollers
- Testing is conducted in a **tethered configuration** for debugging, logging, and manual control during development

---

