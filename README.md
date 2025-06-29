# 🚁 IITISoC-25-IVR002: Virtual Water Level Monitor for Autonomous Drone Missions

## 📌 Project Overview  
This project implements a **Virtual Water Level Monitor** integrated into an autonomous drone mission system using **ROS 2**, **PX4**, **Gazebo**, and **QGroundControl**. The drone begins its mission with a simulated water tank (10L capacity) and decreases by 0.5L at each waypoint — enabling realistic simulation of drones for use cases like agricultural spraying, firefighting, or water delivery.

---

## 👥 Team Members

- 🧑‍💻 [@Gopesh Srinivasan](https://github.com/Gopesh223)
- 🧑‍💻 [@Vimal Pranav S](https://github.com/VimalPranav)
- 🧑‍💻 [@Aadarsh](https://github.com/Aadarsh1406)

## 🧑‍🏫 Mentors

- 👨‍🏫 [@mentor1](https://github.com/mentor1)
- 👨‍🏫 [@mentor2](https://github.com/mentor2)

---

## 🛠️ Frameworks & Tools Used

- **ROS 2** – Humble / Jazzy  
- **Gazebo** – Harmonic  
- **PX4 Firmware**  
- **QGroundControl**  
- **MAVSDK-Python**

---

## ⚙️ Setup Instructions

### 1. Prerequisites

Make sure you have the following installed:

- [ROS 2 Humble or Jazzy](https://docs.ros.org/en/humble/index.html)
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic)
- [PX4 Firmware](https://github.com/PX4/PX4-Autopilot)
- [QGroundControl](https://docs.qgroundcontrol.com/en/getting_started/download_and_install.html)
- [MAVSDK-Python](https://mavsdk.mavlink.io/main/en/python/)

Install MAVSDK with:

```bash
pip install mavsdk
