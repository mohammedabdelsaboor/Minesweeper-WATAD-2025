# Minesweeper WATAD Tracked Robot for Landmine Detection and Mapping

## Project Description

This repository presents the development of a **Minesweeper Tracked Robot** designed to detect and map landmines in a field, both surface and underground, while providing live localization. The robot integrates advanced sensors, including ground-penetrating radar (GPR) and metal detectors, for detecting mines. Additionally, it features a robotic arm for collecting detected surface mines.

Built on **ROS 2**, the robot provides real-time processing of sensor data, autonomous navigation through the minefield, and a robust localization system.

## Objectives

- Detect surface and underground mines using metal detectors and GPR.
- Build a real-time mine detection map with live localization.
- Integrate a robotic arm to collect and remove surface mines.
- Use ROS 2 for navigation, mapping, and sensor data processing.
- Ensure safe, autonomous movement through the minefield with obstacle avoidance.

## Hardware Setup

- **Tracked Robot Base**: Provides mobility in challenging terrains and minefields.
- **Sensors**: 
  - Metal detectors for underground mine detection.
  - Cameras  sensors for surface mine detection.
  - Encoders  IMU for robot localization and mapping.
- **Arm Collector**: Robotic arm for collecting surface mines.
- **Onboard Computer**: Raspberry Pi 4 8GB
- **Power Supply**: LiPo Battery 

## Software Stack

- **ROS 2**: Robot control, sensor integration, and communication.
- **Nav2**: Navigation stack for autonomous movement and localization.
- **SLAM Toolbox**: For generating maps using onboard sensors in real-time.
- **MapServer**: For live map creation and visualization.
- **Custom ROS 2 Nodes**: For metal detection, arm control, and localization.

## Getting Started
![IMG_6963](https://github.com/user-attachments/assets/fefb2aa7-6ecb-41a8-910a-b22197e9611b)
![IMG_8251](https://github.com/user-attachments/assets/ca91f3c4-6ffc-4bcd-9d53-442c39a9ee41)
