#!/bin/bash

export NUM_DRONES=$1
export ROS_SIM_DIR=~/projects/ROS_Multidrone
export FIRMWARE_DIR=~/src/Firmware

#tests for input
if [ -z "$NUM_DRONES" ];
then
	echo "No number of drones specified"
	echo "usage: ./launch_sim.sh [NUM_DRONES]"
	exit 1
fi

#tests for path correctness
if [ ! -d "$FIRMWARE_DIR" ];
then
	echo "PX4 Firmware path does not exist. Correct in launch_sim.sh:"
	echo "Path: ${FIRMWARE_DIR}"
	exit 1
fi

if [ ! -d "$ROS_SIM_DIR" ];
then
	echo "ROS Sim path does not exist. Correct in launch_sim.sh:"
	echo "Path: ${ROS_SIM_DIR}"
	exit 1
fi

echo "[launch] Building project"
source /opt/ros/kinetic/setup.bash
cd $ROS_SIM_DIR
catkin_make

echo "[launch] Num drones: ${NUM_DRONES}"
echo "[launch] Firmware: ${FIRMWARE_DIR}"
echo "[launch] ROS_Multi: ${ROS_SIM_DIR}"

#generate instances of models and flight controllers for each drone
python3 ${ROS_SIM_DIR}/scripts/gen_models.py ${NUM_DRONES} ${FIRMWARE_DIR}

#start simulation components
gnome-terminal -e 'bash -c "${ROS_SIM_DIR}/scripts/start_gazebo.sh ${ROS_SIM_DIR} ${FIRMWARE_DIR}; exec bash"' --geometry 80x24+800+0
gnome-terminal -e 'bash -c "${ROS_SIM_DIR}/scripts/start_drones_cpp.sh ${ROS_SIM_DIR} sim ${NUM_DRONES}; exec bash"' --geometry 80x24+0+470
gnome-terminal -e 'bash -c "${ROS_SIM_DIR}/scripts/start_drones_py.sh ${ROS_SIM_DIR} ${NUM_DRONES}; exec bash"' --geometry 80x24+800+470

