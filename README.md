# magest
# High-Precision Position Estimation System using Actively Controlled Magnetic Field

## Overview
This repository contains the implementation of a high-precision position and orientation estimation system for robots operating in environments where traditional line-of-sight-based methods (such as vision-based tracking) are ineffective. The system uses an actively controlled magnetic field, leveraging a permanent magnet, Anisotropic Magnetoresistive (AMR) sensors, and an Unscented Kalman Filter (UKF) for state estimation.

The system is designed for applications where the robot's position and orientation need to be estimated with high accuracy, even when there is no line of sight to external reference points (e.g., navigating inside the human body).

## Key Features
- **Cost-Effective:** Utilizes a permanent magnet and two low-cost AMR sensors.
- **Active Magnet Control:** The magnet's orientation is actively controlled to track the robot's movement.
- **2D Position & Orientation Estimation:** The system currently estimates 2D position and orientation, with plans for future 3D extension.
- **Magnetic Field Mapping:** Magnetic field maps are generated using deep learning, enabling precise state estimation.
- **Real-Time Operation:** The system operates in real-time, leveraging deep learning for model inference and using NVIDIA's TensorRT framework for optimization.
- **Unscented Kalman Filter (UKF):** The UKF is used to estimate the robot’s radial distance, orientation, and pointing error from the magnet, ensuring precise tracking.

## System Setup

1. **Magnet:** A permanent magnet is mounted on a motor, which allows active control of its orientation to point towards the robot.
2. **Sensors:** The robot is equipped with two low-cost AMR magnetic sensors that provide feedback on the magnetic field strength and direction.
3. **Magnetic Field Map:** A magnetic field map is generated in a narrow ±10° range along the dipole direction of the magnet. This map is used as the measurement model for the UKF.
4. **Deep Learning Model:** The magnetic field map is generated using a deep learning model, trained in Python, and then implemented on an embedded system using ONXXRuntime-GPU for real-time inference.
5. **Control Loop:** The UKF estimates the robot’s position and orientation, while a proportional controller adjusts the magnet’s orientation to continuously track the robot.

[![Watch the video](https://img.youtube.com/vi/7kWq72qKpBU/0.jpg)](https://youtu.be/7kWq72qKpBU)

