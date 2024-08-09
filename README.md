# SLAMify
SLAM with Extended Kalman Filter (EKF) using LiDAR on Raspberry Pi

# SLAM with Extended Kalman Filter (EKF) using LiDAR on Raspberry Pi

## Overview

This project implements Simultaneous Localization and Mapping (SLAM) using an Extended Kalman Filter (EKF) with LiDAR sensors on a Raspberry Pi. The aim is to enable a mobile robot to navigate an unknown environment by building a map and localizing itself within it in real-time.

### Key Features

- **Real-Time Mapping:** Utilizes LiDAR data for creating an accurate map of the environment.
- **Localization:** Employs the Extended Kalman Filter (EKF) for precise localization.
- **Raspberry Pi Integration:** Designed to run on Raspberry Pi, providing a cost-effective and portable solution.
- **Python Implementation:** Leveraging Python libraries for processing LiDAR data and implementing EKF.

## Getting Started

Follow these instructions to set up and run the project on your Raspberry Pi.

### Prerequisites

Before you begin, ensure you have the following installed:

- Raspberry Pi with Raspbian OS
- Python 3.x
- LiDAR sensor (compatible with Raspberry Pi)
- Git

### Installation

1. **Clone the Repository**

   Open a terminal on your Raspberry Pi and clone the repository:

   ```bash
   git clone https://github.com/yourusername/ekf-slam-lidar-raspberry-pi.git
   cd ekf-slam-lidar-raspberry-pi
