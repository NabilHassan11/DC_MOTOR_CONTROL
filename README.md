# DC Motor Modeling and Control (Simulink + State-Space + PID)

## Overview

This project presents a complete workflow for modeling, analyzing, and controlling a DC motor using:

* Block-based modeling in Simulink
* Analytical state-space formulation
* PID controller design and tuning

The objective is to establish consistency between physical modeling, mathematical representation, and control implementation.

---

## Contents

* Simulink model of DC motor (electrical + mechanical subsystems)
* State-space model derivation and validation
* PID controller design for speed control
* Simulation results and performance comparison

---

## DC Motor Model

### Electrical Dynamics

The armature circuit is governed by:

[
V_a(t) = L_a \frac{di(t)}{dt} + R_a i(t) + e_b(t)
]

Where:

* (V_a): armature voltage
* (L_a): armature inductance
* (R_a): armature resistance
* (i): armature current
* (e_b = K_b \omega): back EMF

---

### Mechanical Dynamics

[
J \frac{d\omega(t)}{dt} + B \omega(t) = T_m(t)
]

Where:

* (J): rotor inertia
* (B): viscous friction coefficient
* (\omega): angular speed
* (T_m = K_t i): motor torque

---

## Simulink Block Modeling

The Simulink implementation consists of:

* Electrical subsystem (R-L circuit + back EMF feedback)
* Mechanical subsystem (inertia + damping)
* Gain blocks for torque and back EMF constants
* Integrators for current and angular velocity

### Key Structure

* Input: Armature voltage
* Output: Angular speed
* Feedback: Back EMF proportional to speed

---

## State-Space Representation

### State Variables

[
x_1 = i(t), \quad x_2 = \omega(t) \quad x_3 = \theta(t)
]



### Tuning Approach

* Start with proportional control to stabilize response
* Add integral action to eliminate steady-state error
* Introduce derivative action to improve damping and reduce overshoot

---

## Simulation Results

Typical observations:

* Open-loop system shows slow response and steady-state error
* Closed-loop PID improves rise time and tracking accuracy
* Proper tuning reduces overshoot and settling time

Performance metrics:

* Rise time
* Settling time
* Overshoot
* Steady-state error

---

## Requirements

* MATLAB / Simulink
* Control System Toolbox (optional for analysis)

---

## How to Run

1. Open the `.slx` Simulink model
2. Set motor parameters (R, L, J, B, (K_t), (K_b))
3. Configure PID gains
4. Run simulation
5. Observe speed response and system behavior

---

## Notes

* Parameter sensitivity is significant; inaccurate values lead to poor control
* Discretization effects should be considered for real-time implementation
* Model assumes linear operation (no saturation or nonlinear friction)

---

## Future Work

* Implement state feedback control (LQR)
* Add observer (Kalman filter) for state estimation
* Extend model to include nonlinear effects and load disturbances

---
