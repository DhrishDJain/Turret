# Turret

> Autonomous pan-tilt aerial target detection and tracking system integrating embedded control, onboard sensing, machine vision, and custom mechanical design.

![GitHub repo size](https://img.shields.io/github/repo-size/DhrishDJain/Turret?style=for-the-badge)
![GitHub last commit](https://img.shields.io/github/last-commit/DhrishDJain/Turret?style=for-the-badge)
![GitHub top language](https://img.shields.io/github/languages/top/DhrishDJain/Turret?style=for-the-badge)

## Overview

**Turret** is a mechatronics project built around the STM32F446RE Nucleo platform for autonomous aerial target detection, tracking, and actuation.

The system uses an ESP-CAM for drone detection in air, a VL53L0X sensor for distance measurement, and an MPU6050 for pan-tilt feedback. Motion is handled through a 2-axis mechanism driven by 2 × 180-degree MG90S servos, while a 5V boost module powered by an 18650 cell supplies the platform electronics.

The repository is structured with dedicated `CAD` and `CODE` directories, and the CAD tree includes design and fabrication assets such as SolidWorks files, 3MF exports, and G-code outputs.[web:20]

## Problem Statement

Low-cost drones have become a major challenge in modern warfare because they can perform surveillance, spotting, and attack roles at very low cost compared with traditional defense systems.[web:114][web:116]

This creates a need for compact, responsive, and locally deployable counter-drone systems that can detect, track, and respond quickly to low-altitude aerial threats.[web:114][web:116]

## Solution

**Turret** addresses this problem by combining onboard detection, range sensing, orientation feedback, and real-time actuation into a compact tracking platform.

The ESP-CAM detects aerial targets, the VL53L0X measures distance, the MPU6050 provides pan-tilt feedback, and the STM32F446RE runs the control logic that aligns the mechanism using two MG90S servos.

This creates a closed sensing and control loop for detecting a drone, estimating its relative position, correcting turret orientation, and maintaining alignment through continuous feedback.

## Highlights

- STM32F446RE-based embedded control architecture
- ESP-CAM-assisted aerial target detection
- VL53L0X ranging for distance awareness
- MPU6050 feedback for pan-tilt stabilization and positioning
- 2-axis pan-tilt mechanism using 2 × MG90S 180-degree servos
- Portable power design using a 5V boost module and 18650 cell
- Repository split into firmware and mechanical design domains[web:20]

## System Architecture

### Perception Layer
- **ESP-CAM** detects drones in air and sends status information to the controller
- **VL53L0X** provides distance measurement
- **MPU6050** provides pan-tilt feedback for motion awareness

### Motion Layer
- **2 × MG90S 180° servos** implement the pan-tilt mechanism
- The mechanism is designed for directional aiming and target alignment

### Control Layer
- **STM32F446RE Nucleo** runs the real-time control system
- **RTOS** is used to separate sensing and control into dedicated tasks

### Power Layer
- **5V boost module + 18650 cell** powers the embedded system and actuators

## Software Flow

The firmware is organized around RTOS-based task separation on the STM32F446RE.

### RTOS Task 1: Sensor Task
- Reads **VL53L0X** distance data
- Reads **MPU6050** feedback data
- Receives **ESP-CAM UART messages** indicating whether a drone is detected or not
- Aggregates sensor status for the control layer

### RTOS Task 2: Control Task
- Uses servo control, camera status, and MPU feedback for targeting
- Controls the pan-tilt mechanism for alignment
- Manages the fire-control mechanism through the main control loop

## Repository Layout

```text
Turret/
├── CAD/
│   ├── 3MFS/
│   ├── Gcodes/
│   ├── Assem1.SLDASM
│   ├── BASE.SLDPRT
│   └── additional CAD and fabrication assets
├── CODE/
└── README.md
```

The repository root currently contains `.gitattributes`, `CAD`, `CODE`, and `README.md`, and the CAD folder includes multiple editable and fabrication-ready files.[web:20]

## Hardware Stack

| Subsystem | Component |
|---|---|
| MCU | STM32F446RE Nucleo |
| Vision | ESP-CAM |
| Distance Sensor | VL53L0X |
| IMU Feedback | MPU6050 |
| Actuation | 2 × MG90S 180° servo |
| Mechanism | Pan-tilt turret assembly |
| Power | 5V boost module + 18650 cell |

## Future Scope

Future versions of **Turret** can replace the current prototype-grade sensing and actuation hardware with defence-grade alternatives to improve accuracy, environmental robustness, and reliability in harsh operating conditions.[web:134][web:130]

Possible upgrade paths include:
- Replacing the **MPU6050** with a tactical-grade IMU for better stabilization and pointing performance.[web:134][web:144]
- Moving from the **VL53L0X** to a more capable long-range ranging or LiDAR-based sensing system for improved aerial target tracking.[web:129]
- Replacing **MG90S servos** with industrial or defence-oriented pan-tilt actuators designed for higher torque, lower latency, and better survivability.[web:130]
- Upgrading the motion-control chain with defence-grade servo drives and ruggedized control electronics.[web:130]

## CAD Package

The CAD directory includes named components such as `BASE`, `LEG`, `pan stand`, `tilt stand`, `cannon`, and `Limit Switch Stand`, along with assembly and fabrication files. This indicates a complete physical platform rather than isolated concept geometry.[web:20]

## Author

Created by [DhrishDJain](https://github.com/DhrishDJain). The public repository is named `Turret` and is hosted at [github.com/DhrishDJain/Turret](https://github.com/DhrishDJain/Turret).[web:20]
