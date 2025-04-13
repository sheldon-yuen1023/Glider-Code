# UCL Underwater Glider – Arduino-Based Autonomous Test Vehicle

This repository contains the **early-stage control and diagnostic software** for the **UCL Underwater Glider project**, written using the **Arduino framework in C++**. The codebase is designed to support **tethered testing** of onboard subsystems prior to full autonomous integration.

## Project Overview

The UCL Underwater Glider is a student-led initiative focused on building a **modular, autonomous underwater vehicle (AUV)** for oceanographic research. The glider features:
- A **blended-wing body** for efficient underwater motion
- A **variable buoyancy propulsion system**
- Distributed microcontroller-based subsystem management
- Sensor-rich navigation and control

This repository supports **tethered bench and field testing** of core components and subsystems, ensuring robust performance and calibration before open-water trials.

## Repository Structure

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
- **CAN bus-based telemetry and fault state broadcasting**

See the section below for a detailed overview of the CAN integration.

### `glider-flight-controller/`
Core code for the glider’s **main flight computer**, currently focused on reading IMU data and estimating 3D orientation in real time. This module will eventually coordinate:
- Buoyancy control
- Pitch and roll actuation
- IMU-based attitude estimation
- Mission and navigation logic

#### Key files:
- **`main.ino`** – The main runtime loop. Handles timing and schedules orientation updates and serial output. Acts as the entry point for the program.
- **`Orientation.h / Orientation.cpp`** – Interface and implementation for initializing the IMU (accelerometer, gyroscope, magnetometer), reading sensor data, and passing it to the orientation filter.
- **`KalmanOrientation.h / KalmanOrientation.cpp`** – A lightweight, custom 9DOF orientation filter using a complementary Kalman-style approach. Fuses accelerometer, gyroscope, and magnetometer data to estimate **pitch**, **roll**, and **yaw**.
  - Tunable for responsiveness via a single `beta` parameter.
  - Designed to be modular and easy to replace or expand later (e.g. full Kalman, quaternion-based filters, etc.)

All orientation angles returned by the filter are in **degrees** and updated at a consistent rate defined by `FILTER_UPDATE_RATE_HZ`.

## CAN Bus Integration (Battery Management System)

The `glider-bms/` module implements a fault-monitoring and communication layer using the **CAN bus protocol**, enabling the glider's flight controller to receive real-time battery telemetry and fault states from distributed battery management units (BMS).

### Features

- **CAN protocol** using the ESP32-C3’s native TWAI (Two-Wire Automotive Interface) driver at **500 kbps**
- Fault detection covering overcurrent, undervoltage, overtemperature, and sensor failures
- Latched emergency state disables power and broadcasts error until reset
- Compact 8-byte CAN messages broadcast every 5 seconds

### Message Format

Each BMS periodically transmits a `BMSMessage` structure over the CAN bus:

| Field       | Type      | Description                                  |
|-------------|-----------|----------------------------------------------|
| `bms_id`    | `uint8_t` | Unique identifier for this BMS node          |
| `status`    | `uint8_t` | System status code (see below)               |
| `voltage`   | `uint8_t` | Voltage in tenths of volts (e.g., 124 = 12.4V)|
| `current`   | `uint8_t` | Current in tenths of amps (e.g., 38 = 3.8A)   |
| `temp1_raw` | `uint16_t`| Temperature sensor 1, multiplied by 100      |
| `temp2_raw` | `uint16_t`| Temperature sensor 2, multiplied by 100      |

##### Data Scaling and Resolution

| Field       | Resolution    | Range                    | Notes                                               |
|-------------|---------------|--------------------------|-----------------------------------------------------|
| `voltage`   | 0.1 V         | 0.0 – 25.5 V             | 8-bit field scaled as `raw / 10.0`                  |
| `current`   | 0.1 A         | 0.0 – 25.5 A             | 8-bit field scaled as `raw / 10.0`                  |
| `temp*_raw` | 0.01 °C       | 0.00 – 655.35 °C         | 16-bit field scaled as `raw / 100.0` (sufficient for most use cases) |

> All values are compactly encoded to minimize bandwidth while preserving practical resolution for diagnostics and safety decisions.

### Status Codes

The `status` field in the CAN message reflects the operational state of the BMS:

| Code | Name                          | Description                                     |
|------|-------------------------------|-------------------------------------------------|
| 1    | `STATUS_OK`                   | Normal operation                               |
| 2    | `STATUS_INIT_FAIL`            | Sensor(s) failed to initialize on boot         |
| 3    | `STATUS_OVERCURRENT`          | Current exceeds safe threshold                 |
| 4    | `STATUS_DISCHARGED`           | Voltage below minimum threshold                |
| 5    | `STATUS_OVERTEMP`             | Temperature exceeds safe threshold             |
| 6    | `STATUS_TEMPERATURE_SENSOR_FAIL` | One or more DS18B20 sensors lost               |
| 7    | `STATUS_CURRENT_SENSOR_FAIL`     | INA228 sensor lost                             |

## RS485 Tether and Telemetry JSON Format

### Tether Communication (RS485)

During tethered testing, the glider communicates with a surface laptop over a **long-range RS485 serial connection**. RS485 was chosen over standard UART due to its:

- **Long-distance capability** (tens of meters)
- **Noise immunity** in marine and lab environments
- **Differential signaling**, which reduces error over twisted pair cables

The RS485 interface runs over UART2 of the ESP32 flight computer at **X baud**, using GPIO X (TX) and GPIO X (RX).

Every X, the ESP32 sends a complete telemetry snapshot as a **JSON-formatted string** over the tether, which can be decoded and displayed by the surface UI.

---

### JSON Telemetry Packet Format

Below is a full example of a telemetry packet transmitted via RS485:

```json
{
  "timestamp": 12345678,
  "bms": {
    "bms1": {
      "id": 1,
      "status": "OK",
      "voltage": 12.3,
      "current": 1.4,
      "temp1": 25.6,
      "temp2": 26.0,
      "timestamp": 12345555
    },
    "bms2": {
      "id": 2,
      "status": "OVERTEMP",
      "voltage": 11.9,
      "current": 1.7,
      "temp1": 45.3,
      "temp2": 44.8,
      "timestamp": 12345556
    }
  },
  "vehicle": {
    "stateCode": 3,
    "pitch": 5.8,
    "roll": -2.4,
    "yaw": 182.0
  },
  "sensors": {
    "pressure": 98.6,
    "distanceToBottom": 3.4,
    "verticalVelocity": -0.12,
    "horizontalVelocity": 0.75,
    "leakSensors": {
      "sensor1": false,
      "sensor2": false,
      "sensor3": true
    }
  },
  "actuators": {
    "vbd1Position": 42.1,
    "vbd2Position": 38.6,
    "pitchPosition": 12.5,
    "rollPosition": 3.7
  }
}
```

This packet includes:

- **BMS status** from each active battery monitoring unit
- **Vehicle state**, including glider orientation
- **Sensor data**, such as pressure, depth, leak detection, and movement
- **Actuator positions**, including both VBD and pitch/roll mechanisms

These packets are parsed by the surface-side Python application for real-time visualization and logging during all tethered tests.

