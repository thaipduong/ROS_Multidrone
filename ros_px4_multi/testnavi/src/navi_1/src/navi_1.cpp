#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/SetMode.h> 
#include <mavros_msgs/State.h> 
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>


/*get state of drone*/
mavros_msgs::State current_state;
void state_callback(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

/*get next target msg from decision code*/
sensor_msgs::NavSatFix nextT_;
void nextT_callback(const sensor_msgs::NavSatFix::ConstPtr& msg){
    nextT_ = *msg;
}

/*get current lcoation*/
sensor_msgs::NavSatFix gpsLoc_;
void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg){
    gpsLoc_ = *msg;
}

/*get landing signal*/
std_msgs::Bool landSig_;
void signal(const std_msgs::Bool::ConstPtr& msg){
    landSig_ = *msg;
}

/*get distance*/
double range_calc(float lat,float lon,float alt,float t_lat,float t_lon,float t_alt){
    return pow(pow((lat-t_lat)*111111.11,2)+pow((lon-t_lon)*111111.11,2)+pow(alt-t_alt,2),0.5);
}

int main(int argc, char **argv)
{
  /*start script*/
  //const char command []= "rosrun drone drone_node_simulation_external-sensing.py &";
  //system(command);
  //havn't tested this yet ^^ the & makes it unblocking apparently

  /*create ros node*/
  ros::init(argc, argv, "px4_navi_1");
  
  /*initialize handlers*/
  ros::NodeHandle nh;
  ros::Subscriber state_sub_ = nh.subscribe<mavros_msgs::State>("uav1/mavros/state", 10, state_callback);
  ros::Subscriber nextT_sub_ = nh.subscribe<sensor_msgs::NavSatFix>("FakeDroneObj_1", 10, nextT_callback);//geo msg
  ros::Subscriber gps_sub_ = nh.subscribe<sensor_msgs::NavSatFix>("uav1/mavros/global_position/global", 10, gps_callback);//geo msg
  ros::Subscriber signal_sub_ =nh.subscribe<std_msgs::Bool>("land_sig", 10, signal);

  ros::Publisher target_publisher_ = nh.advertise<mavros_msgs::GlobalPositionTarget>("uav1/mavros/setpoint_position/global", 10);

  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("uav1/mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("uav1/mavros/set_mode");
  //ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("uav1/mavros/cmd/takeoff");
  ros::ServiceClient landing_client = nh.serviceClient<mavros_msgs::CommandTOL>("uav1/mavros/cmd/land") ;
  /*declare msg*/
  mavros_msgs::GlobalPositionTarget target_;
  mavros_msgs::SetMode guided_mode;
  mavros_msgs::CommandBool arm_cmd;
  mavros_msgs::CommandTOL takeoff_cmd;
  mavros_msgs::CommandTOL land_cmd;
  
  /*check sensors NOT IMPLEMENTED*/
  
  /*check battery NOT IMPLEMENTED*/
  
  //battery state msg in mavros
  
  /*operating rate in Hz?*/
  ros::Rate rate(20.0);
  while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
  
  
  /*assign value to msg*/

  /*get starting location, mask n frame*/
  
  //target_.coordinate_frame=6;
  //target_.type_mask=4088;
  target_.latitude=gpsLoc_.latitude;
  target_.longitude=gpsLoc_.longitude;
  target_.altitude=gpsLoc_.altitude+30;
  /*mode switch request-MODE NAME DEPENDS ON FLIGHT STACK*/
  //px4 uses offboard mode
  guided_mode.request.custom_mode = "OFFBOARD";
   /* 
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 10;
*/
  /*offboard mode require stream setpoints to be started, 100 is arbituary*/
  for(int i = 100; ros::ok() && i > 0; --i){
        target_publisher_.publish(target_);
        ros::spinOnce();
        rate.sleep();
    }

  /*arming request*/
  arm_cmd.request.value = true;

  /*take off request*/
  takeoff_cmd.request.altitude= 30;

  /*mode switch*/
  if( set_mode_client.call(guided_mode) && guided_mode.response.mode_sent){
    ROS_INFO("OFFBOARD enabled");
  }else{
    ROS_INFO("OFFBOARD Mode switch FAILED");
    exit(EXIT_FAILURE);
  }
  
  /*arming*/
  if( arming_client.call(arm_cmd) && arm_cmd.response.success){
    ROS_INFO("Vehicle armed");
  }else{
    ROS_INFO("Arming FAILED");
    exit(EXIT_FAILURE);
  }
  /*wait for arming to finish*/
  //sleep(2);
  double home_alt=gpsLoc_.altitude;
  /*take off-px4 take off is taken care of by */
  /*//try other take off methods
  if( takeoff_client.call(takeoff_cmd) && takeoff_cmd.response.success){
    ROS_INFO("drone took off");
  }else{
    ROS_INFO("take off FAILED");
    exit(EXIT_FAILURE);
  }*/
  /*wait for take off*/
  
  //sleep(5);
  bool target_set=false;
  int debugger=0;
  /*Loop*/
  while(ros::ok()){

    
    if(
    range_calc(gpsLoc_.latitude,gpsLoc_.longitude,gpsLoc_.altitude,target_.latitude,target_.longitude,target_.altitude)
    <5
    ){
      target_set=false;
    
    }
    /*TODO:Landing not implemented, though px4 auto returns home if setpoint stream is cut*/
    if(landSig_.data){
    
      /*can use landing state in topic extended state?*/
      /*perform service call to land*/
      if( landing_client.call(land_cmd) && land_cmd.response.success){
        ROS_INFO("drone is landing");
        sleep(100);
      }else{
        ROS_INFO("landing FAILED");
        
        /*not sure what to do here*/
        //exit(EXIT_FAILURE);
      }
    
      exit(0);
    }

    /*condition check for msg-px4 does not have guided mode thus the check for guided is gone*/
    if(
    /*not in guided mode || (drone moving to target or gps!=target)*/
    //!current_state.guided || 
    target_set){
      /*do nothing*/
      ROS_INFO("Drone Moving to Target lat %f, long  %f,alt %f, home %f, gpsl: %f, %f, %f",
      target_.latitude,target_.longitude,target_.altitude, home_alt,
      gpsLoc_.latitude,gpsLoc_.longitude, gpsLoc_.altitude);
      //target_publisher_.publish(target_);
      
    }else if(
    /*guided mode && waypoint msg*/
    //current_state.guided && 
    !target_set
    //&& not sure about this implementation
    ){
      ROS_INFO("Target Acquired");
      /*get target location*/
      //target_.header.stamp = ros::Time::now();
      target_.latitude=nextT_.latitude;
      target_.longitude=nextT_.longitude;
      target_.altitude=nextT_.altitude;
      
      /*publish waypoint and fly there*/
      
      ROS_INFO("%f, %f ,%f", target_.latitude, target_.longitude,target_.altitude);
      target_set=true;
      debugger++;
      //if(debugger>1)
        //exit(0);
    }
    
    target_publisher_.publish(target_);
    ros::spinOnce();
    rate.sleep();
  
  }
  return 0;
}
