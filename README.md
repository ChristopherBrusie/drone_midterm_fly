# drone_midterm_fly

# Autonomous Quadcopter Flight Control System (Crazyflie 2.1)

## Overview

This repository contains a complete modeling, simulation, and control architecture for an autonomous micro-unmanned aerial vehicle (UAV). Developed as part of the ESE 4481 Autonomous Aerial Vehicle Control Laboratory under Dr. Sankalp Bhan, the project bridges the gap between high-fidelity MATLAB simulation and physical hardware implementation on the Crazyflie 2.1 platform.

The software stack currently features nonlinear dynamics modeling, aerodynamic system identification, a custom control allocation mixer, and Linear Quadratic Regulator (LQR) control schemes. Future iterations will integrate state estimation and trajectory tracking directly onto the physical drone hardware.

## Core Capabilities

* **Nonlinear System Modeling:** Fully derived 12-state rigid body dynamics mapping position, velocity, Euler angles, and angular rates between the East-North-Up (ENU) inertial frame and the body-fixed frame.


* **Actuator Dynamics:** Augmented 16-state model incorporating motor transfer functions to accurately simulate realistic step responses and motor lag.


* **Aerodynamic Drag Mapping:** Integration of empirical aerodynamic effects and motor speed (RPM) mappings based on system identification data from Förster, Hamer, & D'Andrea.


* **3D Simulation & Visualization:** Custom MATLAB animation engine for real-time visualization of the quadcopter's 3D spatial trajectory, orientation, and frame transformations.

## Control Architecture

The flight control system relies on a decoupled Linear Quadratic Regulator (LQR) approach with integral action (LQR-PI), ensuring zero steady-state error.

* **Altitude Control:** Regulates vertical position via total thrust commands.


* **Translational Control:** Regulates forward and lateral positioning via pitching and rolling moments.


* **Yaw Control:** Damps yaw motion via yawing moment commands.


* **Control Allocation (Mixer):** A custom physical mixer translates the LQR commanded thrust and moments (roll, pitch, yaw) into individual motor PWM percentages, explicitly handling actuator saturation and infeasible force commands.



## Vehicle Specifications

The model parameters are tuned specifically for the Crazyflie 2.1 nano quadcopter:

* **Mass:** 0.033 kg 


* **Arm Length (Center to Motor):** 0.046 m 


* **Inertia Matrix ($J$):** Derived in the body frame $10^{-6} \text{ kg m}^2$ 


* **Operating Environment:** Standard day, sea-level conditions with constant gravity

<img width="600" height="450" alt="image" src="https://github.com/user-attachments/assets/5eb071c0-232c-4cc8-9fa5-d8f9f852053c" />
Source:  https://www.bitcraze.io/documentation/system/platform/cf2-coordinate-system/




## Project Roadmap

* [x] Derivation of 12-state Kinematics and Dynamics
* [x] Motor System Identification and 16-state Augmentation
* [x] Control Mixer Design with Saturation Handling
* [ ] LQR-PI Controller Design for Decoupled Axes
* [ ] Extended Kalman Filter (EKF) State Estimation
* [ ] Path Planning and Trajectory Tracking
* [ ] Hardware-in-the-loop (HITL) Testing
* [ ] Deployment to Physical Crazyflie Hardware

## Getting Started

### Prerequisites

* MATLAB (Tested on R202Xx)
* Control System Toolbox

### Running the Simulation

1. Clone the repository to your local machine.
2. Navigate to the `sim/` directory.
3. Run `main_sim.m` to execute the full nonlinear step response and launch the 3D trajectory animation.





  
crazy fly. 

<img width="650" height="450" alt="image" src="https://github.com/user-attachments/assets/dba2955a-6ffd-4dec-83f9-82fa618b9137" />
<img width="500" height="890" alt="image" src="https://github.com/user-attachments/assets/25e4a3ff-32e5-4954-8021-3ca7754c9c14" />

