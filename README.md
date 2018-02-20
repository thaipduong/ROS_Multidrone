# ROS_Sim-Ubuntu 16.04 Xenial
_________________________________________________________________

PX4
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
     - *modification of mavros source code needed


Ardupilot [Stopped developing due to lack of support on multi drones]
_________________________________________________________________
Temp Environment setup (no gazebo but can do testing via SITL)

install ROS kinetic from ROS site 

follow : http://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html  
to setup SITL 

install mavros from : https://dev.px4.io/en/ros/mavros_installation.html 
#change indigo to kinetic 

follow : http://ardupilot.org/dev/docs/ros-sitl.html  
to start mvros + SITL (note need to install mavros)


# UPDATE
 only single drone version is ready
