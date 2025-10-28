# Autobot
> ROS2, Nav2 기반 실내 자율주행 로봇

## 1. Overview

Ros2 기반의 Wheel odom, IMU, Lidar 등의 센서를 활용해 Slam을 수행하고, Nav2 패키지를 활용해 경로 계획 주행을 기구 설계부터, Hardware, Software 까지 직접 구현하는데 목표를 두었습니다.

## 2.Demo

## 3. Design

### 3.1 Software Architecture

<img width="361" height="241" alt="Image" src="https://github.com/user-attachments/assets/d9fa8f88-2b13-4de9-804d-e269cc7c9a32" />

### 3.2 Autobot Apparence

<img width="1280" height="1706" alt="Image" src="https://github.com/user-attachments/assets/f3135d2d-c3e1-4713-9bcb-8e04e03187a2" />

### 3.3 Hardware

| Category | Name | Note |
|   ---    | ---  |  --- |
| Board | Rasberry pi 5 |   |
| Motor | JGA25-370 / RPM128, 12V | 2pcs |
| Motor Driver | L298N |   |
| Lidar | Rplidar c1 |   |
| IMU | ICM-20948 |   |
| Step-up transformer | MT-3608 |   |



