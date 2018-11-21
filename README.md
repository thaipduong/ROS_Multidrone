# ROS_Sim-Ubuntu 16.04 Xenial
This is a ROS simulation framework for use in developing drone swarm applications. It is capable of generating multiple copies of drone models, which can be controlled separately and visualized flying in Gazebo. Below are instructions for setting this up as well as running the modules.
_________________________________________________________________

# ROS PX4 Multi-Drone Simulation
After building packages, always ensure terminal has env variables set up so ROS can find this package!
```
source ROS_Sim/ros_px4_multi/testnavi/devel/setup.bash
```

First time setup:
1. Run ubuntu_sim_ros_gazebo.sh from https://dev.px4.io/en/setup/dev_env_linux_ubuntu.html
     - This installs ROS along with the tools (gazebo, mavros, px4 sitl) necessary for running/visualizing drones.
     ```
     wget https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_ros_gazebo.sh
     chmod +x ubuntu_sim_ros_gazebo.sh
     ./ubuntu_sim_ros_gazebo.sh
     ```
2. Clone this repo to your machine
     ```
     git clone https://github.com/UCSD-SEELab/ROS_Sim.git
     ```
3. Build packages
     - Open terminal, but DO NOT SOURCE devel/setup.bash if it already exists!
     - Rebuilding packages is necessary any time the source is changed.
     ```
     cd ROS_Sim/ros_px4_multi/testnavi
     catkin_make
     ```
4. Install PX4 firmware
    - Check that under ~/src/Firmware, the PX4 firmware repository exists, which can be found here: https://github.com/PX4/Firmware
    - This is referenced in generate_model.pl (can change location here if PX4 is installed somewhere else) which is used later to generate launch file with n number of drones.
     
Running simulation:
Generate the appropriate number of drone models for use in the simulation and then start the simulation using bash script. Port number indicates starting port in the range [starting_port, starting_port + 4 * number_of_drones] which must be a free block of ports that can be allocated to mavros->ROS/Gazebo interactions. Something like 9000 or 10000 usually works well here.
```
cd ROS_Sim/scripts
./generate_model.pl <number of drone> <starting port>
chmod +x start_sim.sh
./launch_sim.sh [num_drones]
```
More about these scripts can be found under scripts/README.md

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
