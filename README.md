# ROS_Sim-Ubuntu 16.04 Xenial
_________________________________________________________________

# PX4
_________________________________________________________________
1. run ubuntu_sim_ros_gazebo.sh from https://dev.px4.io/en/setup/dev_env_linux_ubuntu.html
     - it will install all the tools (gazebo, mavros, px4 sitl)
     - if ran into low graphics problem, login by press Ctrl + Alt + F1 and then
       sudo update
2. catkin_make the package 
     - if error please refer to (http://wiki.ros.org/ROS/Tutorials/catkin/CreatingPackage)
      create package and copy over the source codes
3. launch simulation (https://dev.px4.io/en/simulation/ros_interface.html)
     - need both mavros and gazebo launched
4. execute the code by running "source devel/setup.bash" then "rosrun navi navi"
     - *modification of mavros source code needed*


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
mavros/state - tells current flight mode of the drone [https://dev.px4.io/en/concept/flight_modes.html]
droneObj - custom topic that tells drone the next objective in LLA format
mavros/global_position/global - GPS information
mavros/setpoint_position/global - tells drone where to go 
  *PX4 does not have LLA setpoint function. this topic will take in LLA format and 
  translate it into x,y,z format and send to FCU. However I encountered a problem
  with the time stamp safty feature, I managed to get it working after removing
  such safty feature in the source code then rebuilt mavros.
  in "~/catkin_ws/src/mavros/mavros/src/plugins/setpoint_position.cpp" at line 225
  remove the if-else statement and just let it call "send_position_target(req->header.stamp, sp);"
  rebuild mavros by running "catkin build" in "~/catkin_ws"*

Services
mavros/cmd/arming - arm the drone
mavros/set_mode - set the current flight mode
______________________________________________________________________________
# UPDATE
 only single drone version is ready
