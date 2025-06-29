#  IITISoC-25-IVR002: Dual Irrigation Drone

## Problem Statement  
Design and implement a dual-drone system for autonomous
coverage and irrigation in a simulated environment using ROS
2 (Humble or Jazzy), PX4 Firmware, and Ground Control
Station. The drones will collaborate to cover a defined region,
generate geotags for areas needing irrigation, and irrigate those
regions based on a predefined logic. The drones must monitor
its battery levels and recharge as needed to complete the
mission within a predefined timeframe.

---

##  Team Members

-  [@Gopesh Srinivasan](https://github.com/Gopesh223)
-  [@Vimal Pranav S](https://github.com/VimalPranav)
-  [@Aadarsh](https://github.com/Aadarsh1406)

## Mentors

-  [@mentor1](https://github.com/mentor1)
-  [@mentor2](https://github.com/mentor2)

---

## 🛠️ Frameworks & Tools Used

- **ROS 2** – Humble / Jazzy  
- **Gazebo** – Harmonic  
- **PX4 Firmware**  
- **QGroundControl**  
- **MAVROS**

---

## ⚙️ Setup Instructions

### 1. Prerequisites

Make sure you have the following installed:

- [ROS 2 Humble or Jazzy](https://docs.ros.org/en/humble/index.html)
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic)
- [PX4 Firmware](https://github.com/PX4/PX4-Autopilot)
- [QGroundControl](https://docs.qgroundcontrol.com/en/getting_started/download_and_install.html)

### 2. Spawning the drone in PX4

Open terminal and run:
```
cd PX4-Autopilot
```

