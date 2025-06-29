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
- Ubuntu 22.04 or Ubuntu 24.04
- [ROS 2 Humble](https://docs.ros.org/en/humble/index.html) (For Ubuntu 22.04)
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/index.html) (For Ubuntu 24.04)
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic)
- [PX4 Firmware](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html)
- [QGroundControl](https://docs.qgroundcontrol.com/en/getting_started/download_and_install.html)

### 2. Spawning the drone in PX4

Open terminal and run:
```
cd PX4-Autopilot
```
Run the given command to naviagate to the starting place of the mission:
```
 PX4_HOME_LAT=12.9716 PX4_HOME_LON=77.5946 PX4_HOME_ALT=100 make px4_sitl gz_x500
```

### 3. Uploading the mission in QGC
In another terminal, run:
```
./QGroundControl
```
The QGC UI will open. 

 - Go to the Fly view on the left panel.

 - Switch to the Plan tab.

 - Click File → Open, select the provided .plan file.

 - Click Upload Required to send the plan to the drone.

### 4. Start the mission:

Either slide the launch arrow on the interface or hold the spacebar to initiate the mission. The x500 drone will begin to navigate via the waypoints.

### 5. Creating package for irrigation drone
Create a python package using this:
```
ros2 pkg create --build-type ament_python dual_drone_1
```
Then create a python file in the package:
```
cd dual_drone_1
touch dual_drone_1/file_name.py
```
Make the file executable using:
```
chmod +x file_name.py
```
Similar to this, create two more python files in the given package and copy the contents of single_drone.py, dual_drone_cov.py, dual_drone_irr.py in the files you've created

### 6. Running the nodes

After launching
Open ROS2 workspace and run:
```
colcon build --packages-select /dual_drone_1
```

Then, run:
```
source install/setup.bash
```
The node names for single_drone.py, dual_drone_cov.py and dual_drone_irr.py are waypoint_water_tracker, geotag_publisher, geotag_subscriber as given in the setup.py in the package.

Then run the node:
```
ros2 run dual_drone_1 <node_name>
```
replace the node names as mentioned above in place of <node_name> to run the respective nodes.
