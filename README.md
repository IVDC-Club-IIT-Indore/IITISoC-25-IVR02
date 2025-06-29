# IITISoC-25-IVR0xx PS Title

Team Members

_**Team Member 1**:  [@Gopesh Srinivasan](https://github.com/Gopesh223)_

_**Team Member 2**:  [@Vimal Pranav S](https://github.com/VimalPranav)_

_**Team Member 3**:  [@Aadarsh](https://github.com/Aadarsh1406)_


Mentors

_**Mentor 1**:  [@mentor1](https://github.com/mentor1)_

_**Mentor 2**:  [@mentor2](https://github.com/mentor2)_

Framework used
ROS2-Humble/Jazzy
Gazebo-Harmonic
PX4 Firmware
QGroundControl

Setup
1) Make sure you have the above frameworks installed
2) In your command prompt window, run:
   cd PX4-Autopilot
3) Run
   PX4_HOME_LAT=12.9716 PX4_HOME_LON=77.5946 PX4_HOME_ALT=100 make px4_sitl gz_x500
4) You should see a gazebo window open with your drone
5) In another terminal, run
   ./QGroundControl.AppImage
6) The QGroundControl UI should appear. Under fly option on the left side, choose the option plan.
7) Select file and upload the given file and then click on upload required to upload the plan to the drone
8) Then slide the arrow or hold spacebar to start the mission
