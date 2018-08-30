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

source ${ROS_SIM_DIR}/ros_px4_multi/testnavi/devel/setup.bash
python ${ROS_SIM_DIR}/ros_px4_multi/testnavi/src/drone/scripts/DroneRun.py ${NUM_DRONES}
