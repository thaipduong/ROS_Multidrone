# ROS_Sim-Ubuntu 16.04 Xenial
This is a ROS simulation framework for use in developing drone swarm applications. It is capable of generating multiple copies of drone models, which can be controlled separately and visualized flying in Gazebo. Below are instructions for setting this up as well as running the modules.
_________________________________________________________________

# PX4
Always ensure terminal has ROS env variables set up when working with ROS commands!
```
source ROS_Sim/ros_px4_multi/testnavi/devel/setup.bash
```

First time setup:
1. Run ubuntu_sim_ros_gazebo.sh from https://dev.px4.io/en/setup/dev_env_linux_ubuntu.html
     - It will install all the tools (gazebo, mavros, px4 sitl) and set up ROS
     - If ran into low graphics problem, drop to the root shell by pressing Ctrl + Alt + F1 and then try:
       sudo update
       - In my case, an older nvidia graphics card also caused issues, so reinstalling the drivers on Ubuntu VM could help.
2. Clone this repo to your machine and build packages
     - Open terminal, but DO NOT SOURCE devel/setup.bash
     ```
     cd ROS_Sim/ros_px4_multi/testnavi
     catkin_make
     ```
     - If error please refer to (http://wiki.ros.org/ROS/Tutorials/catkin/CreatingPackage)

Running simulation:

Generate the appropriate number of drone models for use in the simulation and then start the simulation using bash script.
```
cd ros_px4_multi
./generate_model.pl <number of drone> <starting port>
cd ../scripts
chmod +x start_sim.sh
./start_sim.sh [num_drones]
```

start_sim.sh takes in number of drones as cmdline parameter and start the individual components of the simulation (launch gazebo, start ros nodes, and have drones communicate between ROS and Gazebo).

If you do not want to use the start_sim.sh script, there are also individual scripts for starting gazebo (start_gazebo.sh) and running the drones separately (start_ros_nodes.sh and start_drones.sh).
1. Launch gazebo simulation (https://dev.px4.io/en/simulation/ros_interface.html)
     - Run script under scripts/start_gazebo.sh [firmware_location]
     
     This script does the following:
     - After opening terminal, ensure ROS env variables are set up. this includes the px4 Firmware directory
     - Source the environment (make sure you are in the correct directory)
     ```
     source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/posix_sitl_default
     export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/sitl_gazebo
     ```
     - Launch the simulation
     ```roslaunch px4 multi_uav_mavros_sitl.launch```
     
     - Starting position of the drones can be changed in the launch file (generate_model.pl also controls this)
     ```
     /src/Firmware/launch/multi_uav_mavros_sitl.launch
     ```     
2. Run drones
     In a new terminal:
     - Run scripts (ros_sim_dir is where ROS_Sim is cloned, including ROS_Sim directory name. ie: ~/projects/ROS_Sim)
     ```
     cd ROS_Sim/scripts
     ./start_ros_nodes.sh [ros_sim_dir]
     
     in new terminal:
     cd ROS_Sim/scripts
     ./start_drones.sh [ros_sim_dir]
     ```
     
     These scripts do the following:
     - Set up environment variables
     ```
     source ROS_Sim/ros_px4_multi/testnavi/devel/setup.bash
     ```
     
     - Spawns ROS nodes that send mavlink messages
     ```
     cd ROS_Sim/ros_px4_multi/testnavi/src/drone/scripts
     rosrun drone DroneRun.py <drone count>
     ```
    
     - In a different terminal, set up env variables
     ```
     source ROS_Sim/ros_px4_multi/testnavi/devel/setup.bash
     ```
     
     - Have nodes start communicating with gazebo simulator
     ```
     rosrun navi navi <drone count>
     ```

Changing drone parameters:
Adding drone models is taken care of by generate_model.pl, and the following are for reference for future changes to drone parameters.

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
Edit ~/src/Firmware/launch/multi_uav_mavros_sitl.launch
```
cp -r navi_2 navi_3
```
Edit all files in ROS_Sim/rox_px4_multi/testnavi/src/navi_3
Change all references of 'uav2' to 'uav3', 'navi_2' -> 'navi_3'

# Ardupilot [Legacy branch - Deprecated due to lack of support on multi drones]
_________________________________________________________________
Temp Environment setup (no gazebo but can do testing via SITL)

Install ROS kinetic from ROS site 

Follow : http://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html  
to setup SITL 

Install mavros from : https://dev.px4.io/en/ros/mavros_installation.html 
#Change indigo to kinetic 

Follow : http://ardupilot.org/dev/docs/ros-sitl.html  
To start mvros + SITL (note need to install mavros)

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
