export ROS_SIM_DIR=~/projects/ROS_Sim
export FIRMWARE_DIR=~/src/Firmware
export NUM_DRONES=$1

if [ -z "$FIRMWARE_DIR" ];
then
	echo "usage: ./start_sim.sh [NUM_DRONES]"
	exit 1
fi

echo "Num drones: ${NUM_DRONES}"

gnome-terminal -e 'bash -c "${ROS_SIM_DIR}/scripts/start_gazebo.sh ${FIRMWARE_DIR}; exec bash"' --geometry 80x24+800+0
gnome-terminal -e 'bash -c "${ROS_SIM_DIR}/scripts/start_ros_nodes.sh ${ROS_SIM_DIR} ${NUM_DRONES}; exec bash"' --geometry 80x24+0+470
gnome-terminal -e 'bash -c "${ROS_SIM_DIR}/scripts/start_drones.sh ${ROS_SIM_DIR} ${NUM_DRONES}; exec bash"' --geometry 80x24+800+470

