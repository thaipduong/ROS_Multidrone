#!/bin/bash

export NUM_DRONES=$1
export ROS_SIM_DIR=~/catkin_ws/src/ROS_Multidrone
export FIRMWARE_DIR=~/PX4/Firmware

#tests for input
if [ -z "$NUM_DRONES" ];
then
	echo "[launch] No number of drones specified"
	echo "[launch] usage: ./launch_sim.sh [NUM_DRONES]"
	exit 1
fi

#tests for path correctness
if [ ! -d "$FIRMWARE_DIR" ];
then
	echo "[launch] PX4 Firmware path does not exist. Correct in launch_sim.sh:"
	echo "[launch] Path: ${FIRMWARE_DIR}"
	exit 1
fi

if [ ! -d "$ROS_SIM_DIR" ];
then
	echo "[launch] ROS Sim path does not exist. Correct in launch_sim.sh:"
	echo "[launch] Path: ${ROS_SIM_DIR}"
	exit 1
fi

echo "[launch] Building project"
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
cd $ROS_SIM_DIR
catkin_make

if [ ! $? -eq 0 ];
then
	echo "[launch] Build failed, exiting."
	exit 1
fi


echo "[launch] Num drones: ${NUM_DRONES}"
echo "[launch] Firmware: ${FIRMWARE_DIR}"
echo "[launch] ROS_Multi: ${ROS_SIM_DIR}"

#generate instances of models and flight controllers for each drone
python3 ${ROS_SIM_DIR}/scripts/gen_models.py ${NUM_DRONES} ${FIRMWARE_DIR}

#start simulation components
gnome-terminal --geometry 80x24+800+0 -- bash -c "${ROS_SIM_DIR}/scripts/start_gazebo.sh ${ROS_SIM_DIR} ${FIRMWARE_DIR}; exec bash"
gnome-terminal --geometry 80x24+0+470 -- bash -c "${ROS_SIM_DIR}/scripts/start_drones_cpp.sh ${ROS_SIM_DIR} sim ${NUM_DRONES}; exec bash"
#gnome-terminal --geometry 80x24+800+470 -- bash -c "${ROS_SIM_DIR}/scripts/start_drones_py.sh ${ROS_SIM_DIR} ${NUM_DRONES}; exec bash"

