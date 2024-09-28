# AtlasPi: SLAM with Raspberry Pi 4 Model-B and RP LiDAR A2M12

## Overview
AtlasPi is an autonomous robot project utilizing SLAM techniques with an Extended Kalman Filter (EKF) on a Raspberry Pi 4 Model-B. The project integrates various sensors for real-time mapping and navigation.

## Table of Contents
1. [Hardware Components](#hardware-components)
2. [Software Setup](#software-setup)
3. [Installation Instructions](#installation-instructions)
4. [Running the Project](#running-the-project)
5. [File Structure](#file-structure)
6. [Contributing](#contributing)
7. [License](#license)

## Hardware Components
- **Raspberry Pi 4 Model-B**: Main processing unit for running the SLAM algorithm.
- **RP LiDAR A2M12**: Provides 360-degree laser scanning for mapping.
- **4WD Smart Robot Car Chassis Kit**: The robot's mechanical base, including motors and wheels.
- **L293N Motor Driver**: Controls the motors.
- **IMU Sensor MPU9250**: Tracks the robot's orientation and motion.
- **Power Supply (Battery Pack)**: Provides power to the Raspberry Pi and motors.
- **Camera Module(Optional) **: Captures visual data for future enhancements.

## Software Setup
### Prerequisites
- **Ubuntu 20.04** on Raspberry Pi 4
- **ROS Noetic**: Required for SLAM and sensor packages.
- **Python 3.x**: For custom scripts.
- **Git**: For version control.

### Installation Instructions
1. **Install Ubuntu 20.04 on Raspberry Pi 5**:
   - Flash Ubuntu using official instructions.
2. **Set Up ROS Noetic**:
   - `sudo apt install ros-noetic-desktop-full`
3. **Clone ProjectQ Repository**:
   - `git clone https://github.com/Veera53/AtlasPi.git`
   - `cd ProjectQ`
4. **Install Python Dependencies**:
   - `pip install -r requirements.txt`

## Running the Project
1. **Launch the LIDAR Node**:
   - `roslaunch rplidar_ros view_rplidar.launch`
2. **Run SLAM Algorithm**:
   - `roslaunch slam_gmapping gmapping_demo.launch`
3. **Control the Robot**:
   - `python scripts/motion_control.py`
4. **Visualize the Map**:
   - `rosrun rviz rviz`

## File Structure
```plaintext
AtlasPi/
│
├── hardware/
│   ├── wiring_diagram.png
│   ├── lidar_mounting_instructions.md
│   └── motor_driver_specs.pdf
│
├── software/
│   ├── src/
│   │   ├── slam_node.py
│   │   ├── motor_control.py
│   │   └── sensor_integration.py
│   ├── launch/
│   │   ├── atlaspi.launch
│   │   └── slam.launch
│   ├── config/
│   │   ├── lidar.yaml
│   │   └── motor_config.yaml
│   ├── CMakeLists.txt
│   └── package.xml
│
├── maps/
│   ├── home_map.pgm
│   ├── home_map.yaml
│   └── map_saving_instructions.md
│
├── images/
│   ├── lidar_setup.jpg
│   └── robot_complete.jpg
│
├── requirements.txt
├── README.md
└── LICENSE
