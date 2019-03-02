export ROS_SIM_DIR=~/projects/ROS_Multidrone
export FIRMWARE_DIR=~/src/Firmware
export NUM_DRONES=$1

if [ -z "$FIRMWARE_DIR" ];
then
	echo "No PX4 firmware directory specified"
	echo "usage: ./launch_sim.sh [NUM_DRONES]"
	exit 1
fi

if [ -z "$ROS_SIM_DIR" ];
then
	echo "No ROS_Sim base dir specified"
	echo "usage: ./launch_sim.sh [NUM_DRONES]"
	exit 1
fi

if [ -z "$NUM_DRONES" ];
then
	echo "No number of drones specified"
	echo "usage: ./launch_sim.sh [NUM_DRONES]"
	exit 1
fi

echo "Num drones: ${NUM_DRONES}"

gnome-terminal -e 'bash -c "${ROS_SIM_DIR}/scripts/start_gazebo.sh ${ROS_SIM_DIR} ${FIRMWARE_DIR}; exec bash"' --geometry 80x24+800+0
gnome-terminal -e 'bash -c "${ROS_SIM_DIR}/scripts/start_drones_cpp.sh ${ROS_SIM_DIR} sim ${NUM_DRONES}; exec bash"' --geometry 80x24+0+470
gnome-terminal -e 'bash -c "${ROS_SIM_DIR}/scripts/start_drones_py.sh ${ROS_SIM_DIR} ${NUM_DRONES}; exec bash"' --geometry 80x24+800+470

