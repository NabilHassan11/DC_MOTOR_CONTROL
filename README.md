# DC Motor Control and Drive Systems

## Overview

This repository documents the development of multiple DC motor control systems, ranging from mathematical simulations in MATLAB/Simulink to real-time embedded implementation on Arduino hardware.

The project was carried out to study DC motor drives, power electronic converters, encoder-based feedback systems, and classical PID control techniques before transitioning to hardware implementation.

The repository is divided into three major sections:

* MATLAB/Simulink drive simulations
* Arduino-based hardware control
* Simulink-to-Arduino hardware implementation

---

# Repository Structure

```
├── Simulink Simulation files/
│
├── Arduino hardware files/
│   ├── Position_Control/
│   ├── Speed_Control/
│   └── Hardware_Image/
│
└── Simulink Arduino hardware files/
    ├── DC_Motor_Openloop/
    └── DC_Motor_Openloop_Speed_Monitoring/
```

---

# Hardware Specifications

The hardware implementation uses the following components:

| Component                    | Specification        |
| ---------------------------- | -------------------- |
| DC Motor                     | JGA25-370            |
| Rated Voltage                | 12 V                 |
| Rated Speed                  | 230 RPM              |
| Encoder Resolution           | 11 PPR (Motor Shaft) |
| Gearbox Ratio                | 34:1                 |
| Effective Encoder Resolution | 1496 Pulses/Rev      |
| Motor Driver                 | L298 Dual H-Bridge   |
| Controller                   | Arduino Uno          |

The effective encoder resolution is calculated as

```
11 × 34 × 4 = 1496 counts/revolution
```

where quadrature decoding is used to count all encoder edges.

---

# 1. Simulink Simulation Files

This folder contains simulation models developed to study DC motor dynamics and drive systems before implementing controllers on hardware.

These simulations were created primarily for learning and understanding DC machine control principles rather than direct deployment to the Arduino hardware.

The following approaches are included:

### Mathematical Motor Models

* State-space modeling
* Transfer-function based modeling
* Direct PID controller implementation

### Converter-Based Drive Models

Several power electronic converters were modeled to understand their influence on motor dynamics, including:

* Full-wave controlled rectifier
* DC-DC Buck (Step-Down) Converter
* Class C DC-DC Converter
* H-Bridge Drive

These simulations demonstrate the relationship between converter topology, motor voltage, current, speed, and control performance.

---

# 2. Arduino Hardware Files

This section contains the embedded software developed for closed-loop control of the physical DC motor.

The code is written entirely in Arduino C++ and implements encoder-based feedback for real-time control.

The folder contains:

```
Arduino hardware files/
│
├── Position_Control/
├── Speed_Control/
└── Hardware Image
```

## Features

### Speed Control

* Encoder speed measurement
* Quadrature encoder decoding using interrupts
* PID speed controller
* Back-calculation anti-windup
* Low-pass filtered speed estimation
* Bidirectional motor control
* Serial reference input
* Real-time data monitoring

### Position Control

* Cascaded Position-Speed control
* Outer Position PID controller
* Inner Speed PID controller
* Angle wrapping (Shortest Path)
* Position error normalization
* Anti-windup compensation
* Independent controller sampling rates
* Encoder-based position estimation

The outer position controller generates the desired motor speed, while the inner controller regulates motor speed through the motor driver.

---

# Encoder Processing

Motor position and speed are calculated directly from the quadrature encoder.

The implementation includes:

* Quadrature interrupt decoding
* Position estimation
* Angular position calculation
* Speed estimation from encoder count differences
* RPM calculation
* Angle normalization
* Shortest-path position error computation

The implementation details and mathematical calculations are documented throughout the source code.

---

# Controller Design

Both speed and position controllers are implemented using PID control.

Controller tuning was initially performed using the **Ziegler–Nichols tuning method**, followed by manual refinement to improve transient response and stability.

Since an exact mathematical model of the physical motor was unavailable, final gains were obtained experimentally.

The implementation also includes:

* Output saturation
* Integral back-calculation anti-windup
* Low-pass filtering of measured speed
* Cascaded control architecture

---

# Demonstration Videos

Demonstration videos of the hardware implementation are included within the corresponding folders.

These videos show the operation of:

* Speed control
* Position control
* Closed-loop motor response

A hardware image is also included for reference.

---

# 3. Simulink Arduino Hardware Files

This folder contains Simulink models used to control the Arduino hardware directly.

Unlike the simulation models, these projects execute on the physical motor through Simulink's Arduino support package.

Current implementations include:

```
DC_Motor_Openloop

DC_Motor_Openloop_Speed_Monitoring
```

At the current stage, these models operate in **open-loop control**.

The intention was to extend this work toward closed-loop control directly from Simulink; however, the Arduino Uno's limited memory prevented the implementation of more advanced real-time control algorithms.

Future development will continue using a higher-performance microcontroller with greater computational and memory resources.

---

# Software Used

* MATLAB
* Simulink
* Arduino IDE

---

# Topics Covered

This repository demonstrates practical implementation of:

* DC Motor Modeling
* State-Space Control
* PID Control
* Cascaded Position-Speed Control
* Quadrature Encoder Processing
* Anti-Windup Compensation
* Low-Pass Filtering
* Power Electronics
* H-Bridge Motor Driving
* DC-DC Converter Drives
* Controlled Rectifiers
* Embedded Systems
* Real-Time Control
* MATLAB/Simulink
* Arduino Programming

---

# Future Improvements

Planned extensions of this project include:

* Migration to STM32/ESP32 hardware
* Simulink closed-loop hardware implementation
* Current control loop (This will need to change the driver to one that has current measurement)
* State observer implementation
* Automatic controller tuning
* Motor parameter identification
* Advanced digital control techniques

---

# References

1. Katsuhiko Ogata, *Modern Control Engineering*, Fifth Edition.

2. MATLAB & Simulink Documentation.

3. Arduino Documentation.

4. Motor driver and component datasheets used throughout the project.
