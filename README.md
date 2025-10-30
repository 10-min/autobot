# Autobot
> ROS2, Nav2 기반 실내 자율주행 로봇

## 1. Overview

Ros2 기반의 Wheel odom, IMU, Lidar 등의 센서를 활용해 Slam을 수행하고, Nav2 패키지를 활용해 경로 계획 주행을 기구 설계부터, Hardware, Software 까지 직접 구현하는데 목표를 두었습니다.

- Ros2 기반 자율주행 AMR
- Lidar, Wheel odom, IMU 센서 데이터 활용
- Nav2 기반 경로 계획 및 장애물 회피

## 2.Demo

[![Autobot](http://img.youtube.com/vi/'d7foac82g7I'/0.jpg)](https://youtu.be/'d7foac82g7I')

## 3. Design

### 3.1 Software Architecture

<img width="561" height="422" alt="Image" src="https://github.com/user-attachments/assets/c9386aa1-dc80-4f6d-be5e-32c4da5d5728" />

### 3.2 Autobot Apparence

[Design description](https://10-min-e.tistory.com/80)
---
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


## 4. Environment

OS : Ubuntu 24.04

ROS2 : Jazzy

**Clone**

```
git clone https://github.com/10-min/autobot.git
```

**build**

```
source /opt/ros/jazzy/setup.bash
cd autobot
colcon build
source install/local_setup.bash
```

In Robot
---

**bringup**

```
ros2 launch autobot_bringup autobot_bringup.launch.py
```

**navgiation**

```
ros2 launch autobot_navigation bringup_launch.py
```

In Other Computer
---

```
source /opt/ros/jazzy/setup.bash
ros2 launch nav2_bringup rviz_launch.py
```


