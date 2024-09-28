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
- **Encoders**
- **IMU Sensor MPU9250**: Tracks the robot's orientation and motion.
- **Power Supply (Battery Pack)**: Provides power to the Raspberry Pi and motors.
- **Camera Module(Optional)**: Captures visual data for future enhancements.

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
AtlasPi/
├── README.md               # Overview of the project
├── LICENSE                 # License for the project (optional)
├── docs/                   # Documentation folder
│   ├── hardware.md         # Detailed documentation for hardware components
│   ├── chassis.md          # Documentation for chassis kit
│   ├── sensors.md          # Documentation for sensors (e.g., LIDAR, encoders)
│   ├── software.md         # Documentation for software setup (ROS, algorithms, etc.)
│   ├── wiring_diagram.png  # Visual wiring diagram for hardware connections
│   └── block_diagram.png   # System architecture block diagram
├── src/                    # Source code directory
│   ├── arduino/            # Arduino-related code
│   ├── raspberry_pi/       # Raspberry Pi scripts (SLAM, sensor control)
│   └── python/             # Python scripts (e.g., sensor data processing)
├── data/                   # Data directory
│   ├── lidar_scans/        # Store LIDAR scan data (CSV, logs)
│   └── odometry/           # Store odometry data
├── images/                 # Images for documentation or results
│   ├── chassis.png         # Image of the chassis assembly
│   └── lidar_setup.png     # Image of the LIDAR setup
├── tests/                  # Testing scripts
│   ├── motor_test.py       # Script to test motor functionality
│   └── lidar_test.py       # Script to test LIDAR functionality
└── scripts/                # Helper scripts (e.g., installation, setup)
    └── install_ros.sh      # Script to install ROS Noetic on Raspberry Pi
