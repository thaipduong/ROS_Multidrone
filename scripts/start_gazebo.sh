ROS_SIM_DIR=$1
FIRMWARE_DIR=$2

if [ -z "$ROS_SIM_DIR" ];
then
	echo "usage: /start_gazebo.sh [ROS_SIM_DIR] [FIRMWARE_DIR]"
	exit 1
fi

if [ -z "$FIRMWARE_DIR" ];
then
	echo "usage: ./start_gazebo.sh [ROS_SIM_DIR] [FIRMWARE_DIR]"
	exit 1
fi

source ${ROS_SIM_DIR}/multi_uav_quad/devel/setup.bash
source ${FIRMWARE_DIR}/Tools/setup_gazebo.bash ${FIRMWARE_DIR} ${FIRMWARE_DIR}/build/posix_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${FIRMWARE_DIR}
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${FIRMWARE_DIR}/Tools/sitl_gazebo
roslaunch px4 multi_uav_mavros_sitl.launch
