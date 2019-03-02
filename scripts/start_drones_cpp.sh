#!/bin/bash

ROS_SIM_DIR=$1
MODE=$2
NUM_DRONES=$3

if [ -z "$ROS_SIM_DIR" ];
then
	echo "usage: ./start_drones.sh [ROS_SIM_DIR] [MODE] [NUM_DRONES]"
	exit 1
fi

if [ -z "$MODE" ];
then
	echo "usage: ./start_drones.sh [ROS_SIM_DIR] [MODE] [NUM_DRONES]"
	exit 1
fi

if [ -z "$NUM_DRONES" ];
then
	echo "usage: ./start_drones.sh [ROS_SIM_DIR] [MODE] [NUM_DRONES]"
	exit 1
fi

source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash

source ${ROS_SIM_DIR}/devel/setup.bash
rosrun dronecpp dronecpp ${MODE} ${NUM_DRONES}
