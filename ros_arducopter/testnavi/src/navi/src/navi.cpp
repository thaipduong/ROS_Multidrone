#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/SetMode.h> 
#include <mavros_msgs/State.h> 
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <sensor_msgs/NavSatFix.h>



mavros_msgs::State current_state;
void state_callback(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
sensor_msgs::NavSatFix nextT_;
void nextT_callback(const sensor_msgs::NavSatFix::ConstPtr& msg){
    nextT_ = *msg;
}

int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "arducopter_navi");
  
  ros::NodeHandle nh;
  ros::Subscriber state_sub_ = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_callback);
  ros::Subscriber nextT_sub_ = nh.subscribe<sensor_msgs::NavSatFix>("droneObj", 10, nextT_callback);//geo msg

  ros::Publisher target_publisher_ = nh.advertise<mavros_msgs::GlobalPositionTarget>("mavros/setpoint_raw/global", 10);

  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");

  
  ros::Rate rate(20.0);
  mavros_msgs::GlobalPositionTarget target_;
  
  target_.coordinate_frame=6;
  target_.type_mask=4088;
  target_.latitude=-35;
  target_.longitude=100;
  target_.altitude=20;
  //send a few setpoints before starting
  /*for(int i = 100; ros::ok() && i > 0; --i){
   location_publisher_.publish(location_);
   ros::spinOnce();
   rate.sleep();
  }*/

  mavros_msgs::SetMode guided_mode;
  guided_mode.request.custom_mode = "GUIDED";

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  mavros_msgs::CommandTOL takeoff_cmd;
  takeoff_cmd.request.altitude= 20;

  target_.latitude=nextT_.latitude;
  target_.longitude=nextT_.longitude;
  //target_.altitude=nextT_.altitude;
  
  if( set_mode_client.call(guided_mode) && guided_mode.response.mode_sent){
    ROS_INFO("GUIDED enabled");
  }

  if( arming_client.call(arm_cmd) && arm_cmd.response.success){
    ROS_INFO("Vehicle armed");
  }
  rate.sleep();
  if( takeoff_client.call(takeoff_cmd) && takeoff_cmd.response.success){
    ROS_INFO("drone took off");
  }
  sleep(20);

  while(ros::ok()){
    target_.latitude=nextT_.latitude;
    target_.longitude=nextT_.longitude;
    //target_.altitude=nextT_.altitude;
    target_publisher_.publish(target_);
    ros::spinOnce();
    rate.sleep();
  
  }
  return 0;
}
