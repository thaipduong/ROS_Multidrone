#!/bin/bash

ROS_SIM_DIR=$1
NUM_DRONES=$2

if [ -z "$ROS_SIM_DIR" ];
then
	echo "usage: ./start_drones.sh [ROS_SIM_DIR] [NUM_DRONES]"
	exit 1
fi

if [ -z "$NUM_DRONES" ];
then
	echo "usage: ./start_drones.sh [ROS_SIM_DIR] [NUM_DRONES]"
	exit 1
fi

source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash

source ${ROS_SIM_DIR}/devel/setup.bash
rosrun state_control state_control ${NUM_DRONES}
