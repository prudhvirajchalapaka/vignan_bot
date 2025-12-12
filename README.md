# Vignan Bot — Autonomous Mobile Robot for Indoor Mapping & Navigation

DEVELOPMENT OF AUTONOMOUS MOBILE ROBOT FOR INDOOR MAPPING AND NAVIGATION.  
Final Year Project R&A by: 211FA21001, 211FA21004, 211FA21007

---

Table of Contents
- Project overview
- Features
- Repository structure
- Hardware (recommended)
- Software (recommended)
- Quick start (clone, build, run)
- Mapping (create a map)
- Localization & Navigation (use existing map)
- Teleoperation & manual testing
- Data logging & playback
- Calibration & tuning
- Troubleshooting
- Contributing
- License & contact

Project overview
----------------
Vignan Bot is an academic project that implements an autonomous mobile robot capable of performing indoor mapping and navigation. The system integrates sensors (e.g., LIDAR, wheel encoders, IMU), embedded controllers, and higher-level autonomy software (SLAM, localization, path planning). The repository contains the code and resources used to collect data, run mapping sessions, and perform autonomous navigation.

Features
--------
- Real-time 2D mapping (SLAM)
- Laser (LIDAR)-based localization (AMCL / particle filter)
- Path planning and obstacle avoidance
- Teleoperation for manual testing and recoveries
- Data logging and map saving/loading
- Modular codebase with C++/Python components and web/HTML UI elements

Repository language composition
-------------------------------
- HTML: ~87.4%
- C++: ~8%
- JavaScript: ~1.5%
- Python: ~1%
- CSS: ~0.9%
- C: ~0.9%
- Other: ~0.3%

Repository structure (high-level)
---------------------------------
- /software or /src — main ROS/workspace packages (C++ & Python nodes)
- /firmware — microcontroller (Arduino / STM) code for motor controllers, encoders
- /launch — ROS launch files for mapping, navigation, simulation
- /maps — saved map images and YAML metadata
- /docs — design notes, wiring diagrams, datasheets
- /web — HTML/JS-based UI (remote control, status)

Hardware (recommended)
----------------------
- Differential drive mobile base (motors, encoders)
- LIDAR (2D, e.g., RPLIDAR, Hokuyo)
- IMU (optional but recommended)
- Onboard computer (Raspberry Pi, Jetson, or Intel NUC)
- Motor controllers (compatible with your motors)
- Battery, power distribution, connectors
- USB / serial interfaces for sensors

Software (recommended)
----------------------
- Ubuntu 20.04 LTS (recommended for ROS Noetic) or the OS matching your ROS distribution
- ROS 1 (Noetic recommended) with common packages:
  - rospy / roscpp
  - tf / tf2
  - sensor_msgs, nav_msgs, geometry_msgs
  - gmapping / cartographer (for SLAM)
  - amcl (for localization)
  - move_base (navigation stack)
  - rviz (visualization)
- catkin build tools (catkin_make or catkin build)
- Python 3.x, pip packages as needed
- Arduino IDE / platformio for microcontroller firmware

Quick start — clone, build, run
-------------------------------
1. Clone the repo:
```bash
git clone https://github.com/prudhvirajchalapaka/vignan_bot.git
cd vignan_bot
```

2. If this repo contains ROS packages, create or use a catkin workspace and copy packages into src:
```bash
# from your workspace folder
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/prudhvirajchalapaka/vignan_bot.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

3. Install system dependencies (example for ROS Noetic):
```bash
sudo apt update
sudo apt install ros-noetic-navigation ros-noetic-gmapping ros-noetic-amcl ros-noetic-robot-state-publisher ros-noetic-rosbridge-server
```
Adjust package names for your ROS distro.

4. Run the robot stack (example launch commands). Replace launch file names with the ones present in this repo:
```bash
# Start sensor and base drivers
roslaunch vignan_bot base_and_sensors.launch

# In a new terminal, start mapping (SLAM)
roslaunch vignan_bot mapping.launch

# Or start navigation using an existing map
roslaunch vignan_bot navigation.launch map:=/path/to/maps/my_map.yaml
```

Mapping (create a map)
----------------------
1. Make sure sensors are publishing expected topics:
   - LIDAR -> /scan
   - Odometry -> /odom
   - tf transforms between base_link and odom
2. Start SLAM (gmapping or cartographer). Example:
```bash
roslaunch vignan_bot mapping.launch
```
3. Drive the robot (teleop) around the environment to cover all areas. Use rviz to watch map building in realtime.
4. Save the map after mapping completes:
```bash
rosrun map_server map_saver -f ~/maps/my_map
```
This creates my_map.pgm and my_map.yaml.

Localization & Navigation (use existing map)
--------------------------------------------
1. Start the base drivers and bring up the map server + localization:
```bash
roslaunch vignan_bot bringup.launch
rosrun map_server map_server ~/maps/my_map.yaml
roslaunch vignan_bot localization.launch
```
2. Start the navigation stack (move_base) with parameters tuned for your robot:
```bash
roslaunch vignan_bot navigation.launch map:=~/maps/my_map.yaml
```
3. Use RViz or a teleop GUI to send goal poses. Monitor topics:
   - /move_base_simple/goal (for 2D Nav Goal)
   - /move_base/goal (action)

Teleoperation & manual testing
------------------------------
- Teleop with keyboard:
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
- Or use a joystick/gamepad with joy_node and teleop_twist_joy.

Data logging & playback
-----------------------
- Record a rosbag:
```bash
rosbag record -O run1.bag /tf /odom /scan /cmd_vel /tf_static
```
- Play back:
```bash
rosbag play run1.bag --clock
```

Calibration & tuning
--------------------
- Wheel encoders: verify correct odometry parameters (wheel radius, wheel separation).
- IMU: run calibration if available; check orientation topics.
- LIDAR: ensure mount is stable and data stream is correct.
- Navigation params: costmaps, inflation radius, local/global planners — tune according to robot size and speed.

Troubleshooting
---------------
- No LIDAR data: check USB/serial permissions, run roslaunch drivers manually and use rostopic echo /scan.
- Bad odometry: check encoder counts and conversion factors; verify tf tree is correct (use rosrun tf tf_monitor and rosrun rqt_tf_tree rqt_tf_tree).
- Robot doesn't move: confirm motor controller topics and verify safety interlocks are not enabled; check /cmd_vel is received.
- SLAM produces warped maps: ensure odometry and tf are reasonably accurate; reduce robot speed while mapping.

Contributing
------------
Contributions, bug reports, and improvements are welcome. To contribute:
1. Fork the repository.
2. Create a feature branch: git checkout -b feature/my-change
3. Commit and push, then open a pull request describing your change.

Please follow the coding conventions in the repo and add README updates if you add new modules or launch files.

License & contact
-----------------
Include license information here (e.g., MIT, BSD, GPL). If not present, add a LICENSE file before using the code in production.

For questions, contact the authors listed in the project or open an issue on GitHub:
https://github.com/prudhvirajchalapaka/vignan_bot/issues

What I did and what's next
--------------------------
I created this README to describe the project, summarize the repository layout, list hardware/software requirements, and provide step-by-step instructions for building, mapping, and navigation. Next, you can run the quick start steps with your robot hardware (or in simulation if available), try creating a map, and then test localization + navigation. If you want, I can adapt the README to include exact launch file names and commands found in the repository—point me to the package/launch filenames or allow me to inspect the repo and I'll update the README to be fully specific.
