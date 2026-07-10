# ESP32-C3 Flight Controller for Fixed-Wing UAV

> An open-source Flight Controller firmware for Fixed-Wing UAV based on ESP32-C3, written in C using ESP-IDF and FreeRTOS.

![Language](https://img.shields.io/badge/Language-C-blue)
![Platform](https://img.shields.io/badge/Platform-ESP32--C3-green)
![Framework](https://img.shields.io/badge/Framework-ESP--IDF-red)
![RTOS](https://img.shields.io/badge/RTOS-FreeRTOS-orange)

---

# Overview

This project aims to build a lightweight, modular and fully customizable Flight Controller (FC) for fixed-wing UAVs without relying on Betaflight, INAV or ArduPilot.

The firmware is designed specifically for ESP32-C3 and focuses on:

- Stable attitude estimation
- Cascaded PID flight control
- SBUS receiver support
- Modular software architecture
- Easy expansion for GPS navigation
- Real-time execution using FreeRTOS

Current target hardware:

- ESP32-C3
- BMI160 IMU
- FrSky XM+ SBUS Receiver
- Standard PWM Servo / ESC
- GPS (planned)
- LoRa Telemetry (planned)

---

# Architecture

```
                 +----------------------+
                 |      XM+ Receiver    |
                 |      SBUS UART       |
                 +----------+-----------+
                            |
                            |
                     FreeRTOS Queue
                            |
                            v
+-------------------------------------------------------------+
|                    Algorithm Module                         |
|-------------------------------------------------------------|
|                                                             |
| Outer Loop (100Hz)                                          |
|     Angle Controller                                        |
|          ↓                                                  |
|     Rate Setpoint                                           |
|                                                             |
| Inner Loop (500Hz)                                          |
|     Rate PID                                                |
|          ↓                                                  |
|     Servo Commands                                          |
+------------------------+------------------------------------+
                         |
                         |
                  FreeRTOS Queue
                         |
                         v
                Control Output Module
                         |
         PWM / ESC / Servo / GPIO Output

                ↑
                |
          IMU Module (1000Hz)

BMI160
      ↓
 Madgwick AHRS
      ↓
 Roll Pitch Yaw
```

---

# Software Modules

## IMU

Features

- BMI160 Driver
- I2C @ 400kHz
- Gyroscope Calibration
- Calibration saved in NVS
- Madgwick AHRS
- 1000Hz update rate

Outputs

- Roll
- Pitch
- Yaw
- Raw Gyroscope
- Raw Accelerometer

---

## Receiver

Supports

- FrSky XM+
- SBUS
- 100000 baud
- 8E2
- Hardware UART inversion

Features

- Frame parser
- Frame validation
- Signal Lost Detection
- Thread-safe access

---

## Flight Algorithm

Current control modes

- Angle Mode
- Horizon Mode
- Acro Mode
- Return-To-Home (Framework ready)
- Waypoint (Framework ready)

Controller

```
Angle PID
        ↓
Rate PID
        ↓
Servo Output
```

Current frequencies

| Task | Frequency |
|-------|----------|
| IMU | 1000 Hz |
| Inner PID | 500 Hz |
| Outer PID | 100 Hz |

---

## Control Output

Supports

- PWM Servo
- ESC PWM
- Digital Outputs

Features

- Arm / Disarm
- Throttle Cut
- Failsafe
- Output Mapping

---

# FreeRTOS Tasks

| Task | Frequency | Responsibility |
|-------|----------|----------------|
| imu_task | 1000 Hz | Read BMI160 + Madgwick |
| alg_inner_task | 500 Hz | Rate PID |
| alg_outer_task | 100 Hz | Angle PID + Flight Mode |
| xm_plus_task | SBUS | Receiver |
| control_task | Event Driven | PWM Output |

---

# Hardware

Current hardware

- ESP32-C3
- BMI160
- FrSky XM+
- Servo
- ESC

Future hardware

- GPS (AI Thinker GP-02)
- LoRa RA-02
- Magnetometer
- Barometer
- Airspeed Sensor

---

# Planned Features

## Navigation

- GPS Position Hold
- Return To Home
- Waypoint Mission
- Dubins Path Planner

---

## Sensors

- Magnetometer
- Barometer
- Airspeed
- Battery Monitoring

---

## Telemetry

- LoRa Telemetry
- Ground Station
- Real-time Position
- Flight Logging

---

## Flight Modes

- Manual
- Stabilize
- Angle
- Horizon
- Acro
- Return To Home
- Auto Mission

---

# Project Structure

```
FC_xm_plus/
├── include/
│   ├── algorithm.h
│   ├── config.h
│   ├── control.h
│   ├── imu.h
│   └── xm_plus.h
│
├── lib/
│   ├── BMI160_SensorAPI/
│   │   ├── bmi160.c
│   │   ├── bmi160.h
│   │   └── bmi160_defs.h
│   ├── MadgwickAHRS/
│   │   ├── MadgwickAHRS.c
│   │   └── MadgwickAHRS.h
│   └── sbus/
│       └── src/
│           ├── sbus.c
│           └── sbus.h
│
├── src/
│   ├── main.c
│   ├── algorithm.c
│   ├── control.c
│   ├── imu.c
│   ├── uart_gps.c
│   └── xm_plus.c
│
├── test/
│
└── tools/
```

---

# Development Roadmap

- [x] ESP-IDF project
- [x] FreeRTOS architecture
- [x] BMI160 driver
- [x] Madgwick AHRS
- [x] SBUS Receiver
- [x] Cascaded PID
- [x] Servo Output
- [x] Arm / Disarm
- [ ] GPS Driver
- [ ] Navigation Controller
- [ ] Return-To-Home
- [ ] Waypoint Mission
- [ ] Dubins Path
- [ ] LoRa Telemetry
- [ ] Ground Control Station
- [ ] Data Logger
- [ ] PID Auto Tuning

---

# Design Philosophy

This project is built around four principles:

- Modular architecture
- Deterministic real-time execution
- Easy hardware customization
- Full source code transparency

The long-term objective is to develop a complete fixed-wing autopilot capable of autonomous GPS navigation while remaining lightweight enough to run on an ESP32-C3.

---

# License

MIT License

---

# Author

**Nguyen Tan Trinh**

Control Engineering and Automation

Ho Chi Minh City University of Technology (HCMUT)

Embedded Systems • UAV • Flight Control • Autonomous Navigation