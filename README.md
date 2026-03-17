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

<img width="700" height="809" alt="image" src="https://github.com/user-attachments/assets/7ac54dae-9820-40fa-8b5c-a7955847fae2" />

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

* MATLAB (Tested on R2025b)

### Running the Simulation

1. Clone the repository to your local machine.
2. TBD





  
crazy fly. 

## Dedicated to Ham and his Predecesors, Albert I - VI (monkeys)
Ham (July 1957 – January 19, 1983), a chimpanzee also known as Ham the Chimp and Ham the Astrochimp, was the first great ape launched into space. On January 31, 1961, Ham flew a suborbital flight on the Mercury-Redstone 2 mission, part of the U.S. space program's Project Mercury.[1][2]

Ham was known as "No. 65" before he safely returned to Earth, when he was named after an acronym for the laboratory that prepared him for his historic mission—the Holloman Aerospace Medical Center, located at Holloman Air Force Base in New Mexico, southwest of Alamogordo. His name was also in honor of the commander of Holloman Aeromedical Laboratory, Lieutenant Colonel Hamilton "Ham" Blackshear.

Albert I was a rhesus macaque monkey and the first mammal launched on a rocket (V-2 Rocket "Blossom No. 3") on June 11, 1948.[1][2] The launch was staged at White Sands Proving Ground, Las Cruces, New Mexico. Albert I, a nine-pound monkey, was anesthetized and placed inside the rocket's crew capsule in the nose of the V-2 rocket.[2] The flight did not reach outer space.

Albert II was a male rhesus macaque monkey who was the first primate and first mammal to travel to outer space. He flew from Holloman Air Force Base in New Mexico, United States, to an altitude of 83 miles (134 km) aboard Blossom No. 4B, a U.S. V-2 sounding rocket on June 14, 1949. Albert died upon landing after a parachute failure caused his capsule to strike the ground at high speed. Albert's respiratory and cardiological data were recorded up to the moment of impact. 

After Albert II, several other monkeys named Albert followed. Albert III was killed before he reached space when the V-2 rocket exploded during the ascent. Albert IV met a similar fate as Albert II - he reached space (though several miles lower than II), and died on impact with the ground due to a parachute release malfunction. Following this, NASA switched to using Aerobee rockets, and on April 18, 1951, a monkey, possibly called Albert V, died once again due to parachute failure during descent. Two months after Soviet space dogs Dezik and Tsygan survived a July 1951 spaceflight, Yorick, also called Albert VI, attained a non-space height of 72 km (44.7 mi) but did become the first primate to survive a landing. He died two hours later.

  
<img width="500" height="890" alt="image" src="https://github.com/user-attachments/assets/25e4a3ff-32e5-4954-8021-3ca7754c9c14" />

  
Depicted: Ham the Chimp. 

