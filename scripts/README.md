For convenience, launch_sim.sh handles all aspect of starting simulation.
Usage:
```
launch_sim.sh [num_drones]
```
- Num drones is the number of drones that will run in the simulation. Each currently have identical flight controllers, but the models are instantiated in different locations.

launch_sim.sh does the following:
1. Builds the project using catkin_make under root dir:
     ```
     source /opt/ros/kinetic/setup.bash
     cd ROS_Multidrone
     catkin_make
     ```
1. Generates model files and flight controllers
     - Associated script under ROS_Multidrone/scripts/gen_models.py.
     Usage:
     ```
     python3 gen_models.py [num_drones] [firmware_dir] [starting port]
     ```
     This script does the following:
     - Generate a set of launch files for ROS so that all drones run under a separate name space and their topics are published properly.
     - Create a set of individual flight controllers for each drone in simulation, with unique port bindings

2. Launch gazebo simulation (https://dev.px4.io/en/simulation/ros_interface.html)
     - Associated script under ROS_Multidrone/scripts/start_gazebo.sh
     Usage:
     ```
     ./start_gazebo.sh [firmware_location]
     ```
     This script does the following:
     - After opening terminal, ensure ROS env variables are set up. This includes the px4 Firmware directory
     ```
     source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/posix_sitl_default
     export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/sitl_gazebo
     ```
     - Launch the simulation
     ```roslaunch px4 multi_uav_mavros_sitl.launch```
     
     - Starting position of the drones can be changed in the launch file (generate_model.pl also controls this)
     ```
     .../Firmware/launch/multi_uav_mavros_sitl.launch
     ```     
2. Run drones
     In a new terminal, start the cpp controllers:
     ```
     ROS_Multidrone/scripts/start_drones_cpp.sh [ros_sim_dir] [num_drones]
     ```
     
     In a new terminal, start the python controllers:
     ```
     ROS_Multidrone/scripts/start_drones_py.sh [ros_sim_dir] [num_drones]
     ```
     
     These scripts do the following:
     - Set up environment variables so ROS can find this repo's packages. devel/setup.bash is generated at compile time using catkin_make
     ```
     source ROS_Multidrone/devel/setup.bash
     ```
     
     - Spawns ROS nodes that send mavlink messages
     - drones_cpp handles lower-level flight controller logic like managing takeoff/landing/state and can choose whether the python module delegates waypoints to it.
     - the python file can be easier to work with, but is slower with publishing at high rates (required for flight controller to stay active)

Changing drone parameters:
- Associated script: python3 gen_drones.py <num_drones> [firmware_dir] [starting_port]


OLDER NOTES ABOUT FLIGHT CONTROLLER PARAM GENERATION:
Modifying drones:
- Everything in src is gazebo code
```
cd ~/src/Firmware/posix-configs/SITL/init/ekf2
```

- Create new drone, ie:
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
