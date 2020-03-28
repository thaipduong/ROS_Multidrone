'''*-----------------------------------------------------------------------*---
                                                         Author: Jason Ma
                                                         Date  : Jan 10 2019
                                  ROS_Multidrone

  File Name  : gen_models.py
  Description: Generates model files for Gazebo simulation.
---*-----------------------------------------------------------------------*'''

import sys
import os

'''[Global vars]------------------------------------------------------------'''
START_PORT_DEFAULT = 9000

MODEL_FILE_PREFIX = "iris_"

#flight controller params
IRIS_PARAMS_STR = """
uorb start
param load
dataman start
param set MAV_SYS_ID %d
param set BAT_N_CELLS 3
param set CAL_ACC0_ID 1376264
param set CAL_ACC0_XOFF 0.01
param set CAL_ACC0_XSCALE 1.01
param set CAL_ACC0_YOFF -0.01
param set CAL_ACC0_YSCALE 1.01
param set CAL_ACC0_ZOFF 0.01
param set CAL_ACC0_ZSCALE 1.01
param set CAL_ACC1_ID 1310728
param set CAL_ACC1_XOFF 0.01
param set CAL_GYRO0_ID 2293768
param set CAL_GYRO0_XOFF 0.01
param set CAL_MAG0_ID 196616
param set CAL_MAG0_XOFF 0.01
param set COM_DISARM_LAND 3
param set COM_OBL_ACT 2
param set COM_OBL_RC_ACT 0
param set COM_OF_LOSS_T 5
param set COM_RC_IN_MODE 1
param set EKF2_AID_MASK 1
param set EKF2_ANGERR_INIT 0.01
param set EKF2_GBIAS_INIT 0.01
param set EKF2_HGT_MODE 0
param set EKF2_MAG_TYPE 1
param set MAV_TYPE 2
param set MC_PITCH_P 6
param set MC_PITCHRATE_P 0.2
param set MC_ROLL_P 6
param set MC_ROLLRATE_P 0.2
param set MIS_TAKEOFF_ALT 2.5
param set MPC_HOLD_MAX_Z 2.0
param set MPC_Z_VEL_I 0.15
param set MPC_Z_VEL_P 0.6
param set NAV_ACC_RAD 2.0
param set NAV_DLL_ACT 2
param set RTL_DESCEND_ALT 5.0
param set RTL_LAND_DELAY 5
param set RTL_RETURN_ALT 30.0
param set SDLOG_DIRS_MAX 7
param set SENS_BOARD_ROT 0
param set SENS_BOARD_X_OFF 0.000001
param set SITL_UDP_PRT %d
param set SYS_AUTOSTART 4010
param set SYS_MC_EST_GROUP 2
param set SYS_RESTART_TYPE 2
replay tryapplyparams
simulator start -s
tone_alarm start
gyrosim start
accelsim start
barosim start
adcsim start
gpssim start
pwm_out_sim mode_pwm
sensors start
commander start
land_detector start multicopter
navigator start
ekf2 start
mc_pos_control start
mc_att_control start
mixer load /dev/pwm_output0 ROMFS/px4fmu_common/mixers/quad_w.main.mix
mavlink start -x -u %d -r 4000000
mavlink start -x -u %d -r 4000000 -m onboard -o %d
mavlink stream -r 50 -s POSITION_TARGET_LOCAL_NED -u %d
mavlink stream -r 50 -s LOCAL_POSITION_NED -u %d
mavlink stream -r 50 -s GLOBAL_POSITION_INT -u %d
mavlink stream -r 50 -s ATTITUDE -u %d
mavlink stream -r 50 -s ATTITUDE_QUATERNION -u %d
mavlink stream -r 50 -s ATTITUDE_TARGET -u %d
mavlink stream -r 50 -s SERVO_OUTPUT_RAW_0 -u %d
mavlink stream -r 20 -s RC_CHANNELS -u %d
mavlink stream -r 250 -s HIGHRES_IMU -u %d
mavlink stream -r 10 -s OPTICAL_FLOW_RAD -u %d
logger start -e -t
mavlink boot_complete
replay trystart""" # % (num_drones, port, port+1, port+2, port+3, port+1, port+1, port+1, port+1, port+1, port+1, port+1, port+1, port+1, port+1)

#from px4's firmware launch files
LAUNCH_BASE_STR = """<?xml version="1.0"?>
<launch>

    <!-- MAVROS posix SITL environment launch script -->

    <arg name="debug" default="false"/>
    <arg name="verbose" default="false"/>
    <arg name="paused" default="false"/>

    <arg name="est" default="ekf2"/>
    <arg name="vehicle" default="iris"/>
    <arg name="world" default="$(find mavlink_sitl_gazebo)/worlds/empty.world"/>

    <arg name="headless" default="false"/>
    <arg name="gui" default="true"/>
    <arg name="ns" default="/"/>

    <!-- Load world -->
    <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <arg name="headless" value="$(arg headless)"/>
        <arg name="gui" value="$(arg gui)"/>
        <arg name="world_name" value="$(arg world)" />
        <arg name="debug" value="$(arg debug)" />
        <arg name="verbose" value="$(arg verbose)" />
        <arg name="paused" value="$(arg paused)" />
    </include>
"""

#individual drone entries
LAUNCH_GROUP_STR = """    <group ns="uav%d">
        <!-- MAVROS and vehicle configs -->
        <arg name="ID" value="%d"/>
        <arg name="fcu_url" default="udp://:%d@localhost:%d"/>

        <!-- PX4 SITL and vehicle spawn -->
        <include file="$(find px4)/launch/single_vehicle_spawn.launch">
            <arg name="x" value="%f"/>
            <arg name="y" value="%f"/>
            <arg name="z" value="0"/>
            <arg name="R" value="0"/>
            <arg name="P" value="0"/>
            <arg name="Y" value="0"/>
            <arg name="vehicle" value="$(arg vehicle)"/>
            <arg name="mavlink_udp_port" value="%d"/>
            <arg name="mavlink_tcp_port" value="%d"/>
            <arg name="ID" value="$(arg ID)"/>
        </include>

        <!-- MAVROS -->
        <include file="$(find mavros)/launch/px4.launch">
            <arg name="fcu_url" value="$(arg fcu_url)" />
            <arg name="gcs_url" value="" />
            <arg name="tgt_system" value="$(eval 1 + arg('ID'))"/>
            <arg name="tgt_component" value="1" />
        </include>
    </group>
"""

'''[main]----------------------------------------------------------------------
  Parses cmd line input, fills in params
----------------------------------------------------------------------------'''
def main():

  if len(sys.argv) < 3:
    print("usage: python3 gen_models.py [num_drones] [firmware_dir] [starting_port=9000]")
    return

  #set num drones
  num_drones = int(sys.argv[1])
  
  #set firmware dir
  firmware_dir = os.path.expanduser("~") + "/projects/Firmware" #TODO fix hardcode
  if len(sys.argv) > 2:
    firmware_dir = sys.argv[2]

  #set starting port
  starting_port = START_PORT_DEFAULT
  if len(sys.argv) > 3:
    starting_port = int(sys.argv[3])
  
  #write launch file
  with open(firmware_dir + "/launch/multi_uav_mavros_sitl.launch", "w+") as f:
    f.write(LAUNCH_BASE_STR + "\n")

    port = starting_port
    for i in range(1, num_drones + 1):
      group_str = LAUNCH_GROUP_STR % (
          i, #UAV namespace ID
          i, #UAV ID arg
          port, #fcu_url udp src?
          port+1, #fcu_url udp target?
          (i-1)/4, #x starting pos
          (i-1)%4, #y starting pos
          port+2, #mavlink_udp_port
          port+3) #mavlink_tcp_port
      f.write(group_str + "\n")

      port += 10
    f.write("</launch>\n")
  
  print("[gen_models] model generation complete")

if __name__ == '__main__':
  main()
