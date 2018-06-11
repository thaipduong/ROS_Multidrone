# ROS_Sim-Ubuntu 16.04 Xenial
_________________________________________________________________

# PX4
_________________________________________________________________
1. run ubuntu_sim_ros_gazebo.sh from https://dev.px4.io/en/setup/dev_env_linux_ubuntu.html
     - it will install all the tools (gazebo, mavros, px4 sitl) and set up ROS
     - if ran into low graphics problem, drop to the root shell by pressing Ctrl + Alt + F1 and then try:
       sudo update
       - In my case, an older nvidia graphics card also caused issues, so reinstalling the drivers on Ubuntu VM could help.
2. clone this repo to your machine and build packages
     - after opening terminal, ensure ROS env variables are set up
     - if error please refer to (http://wiki.ros.org/ROS/Tutorials/catkin/CreatingPackage)
3. launch gazebo simulation (https://dev.px4.io/en/simulation/ros_interface.html)
     - after opening terminal, ensure ROS env variables are set up
     ```roslaunch px4 multi_uav_mavros_sitl.launch```
     
4. run drones
     - set up environment variables
     ```
     source ROS_Sim/ros_px4_multi/testnavi/devel/setup.bash
     ```
     
     - spawn model file and modify launch file
     ```
     generate_model.pl <number of drone> <starting port>
     ```
     - starting position of the drones can be changed in the launch file
     ```
     /src/Firmware/launch/multi_uav_mavros_sitl.launch
     ```
     
     - spawns ROS nodes that send mavlink messages
     ```
     rosrun drone DroneRun.py <drone count>
     ```
    
     - have nodes start communicating with gazebo simulator
     ```
     rosrun navi <drone count>
     ...
     ```
     
Modifying drones are taken care of by the new script and code
the following are for reference only.

Modifying drones:
- Everything in src is gazebo code
```
cd ~/src/Firmware/posix-configs/SITL/init/ekf2
```

- Create new drone
```
cp iris_2 iris_3
```

- Change the following params in iris_3:
```
MAV_SYS_ID //increment by 1
SITL_UDP_PORT //increment by 2
mavlink start -x -u # //increment both of these lines by 2
-m onboard -o # //increment by 2
anything that starts with mavlink, inc by 2
```
https://dev.px4.io/en/simulation/multi-vehicle-simulation.html

- Add new UAV to launch file and create new bindings for UDP ports
edit ~/src/Firmware/launch/multi_uav_mavros_sitl.launch
```
cp -r navi_2 navi_3
```
edit all files in ROS_Sim/rox_px4_multi/testnavi/src/navi_3
change all references of 'uav2' to 'uav3', 'navi_2' -> 'navi_3'

# Ardupilot [Stopped developing due to lack of support on multi drones]
_________________________________________________________________
Temp Environment setup (no gazebo but can do testing via SITL)

install ROS kinetic from ROS site 

follow : http://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html  
to setup SITL 

install mavros from : https://dev.px4.io/en/ros/mavros_installation.html 
#change indigo to kinetic 

follow : http://ardupilot.org/dev/docs/ros-sitl.html  
to start mvros + SITL (note need to install mavros)

______________________________________________________________________________
# ROS MSG Parameters [Based on PX4 ver]

Topics
1. mavros/state - tells current flight mode of the drone [https://dev.px4.io/en/concept/flight_modes.html]
2. droneObj - custom topic that tells drone the next objective in LLA format
3. mavros/global_position/global - GPS information
4. mavros/setpoint_position/global - tells drone where to go in LLA
     - *PX4 does not have LLA setpoint function. this topic will take in LLA format and 
    translate it into x,y,z format and send to FCU. However I encountered a problem
    with the time stamp safty feature, I managed to get it working after removing
    such safty feature in the source code then rebuild mavros.*
     - *In "\~/catkin_ws/src/mavros/mavros/src/plugins/setpoint_position.cpp" at line 225
    remove the if-else statement and just let it call "send_position_target(req->header.stamp, sp);"
    rebuild mavros by running "catkin build" in "\~/catkin_ws"*

Services
1. mavros/cmd/arming - arm the drone
2. mavros/set_mode - set the current flight mode
______________________________________________________________________________
# UPDATE
 multi drone control code working, need work on target determine code
