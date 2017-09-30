//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================


#include <ros/ros.h>
#include <geodesy/utm.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Twist.h>
#include <hector_uav_msgs/YawrateCommand.h>
#include <hector_uav_msgs/ThrustCommand.h>
#include <hector_uav_msgs/AttitudeCommand.h>
#include <iostream>
#include <fstream>
#include <queue>

namespace hector_quadrotor
{

class Navi
{
private:
  ros::NodeHandle node_handle_;
  ros::Subscriber fix_subscriber_;

  ros::Publisher velocity_publisher_, attitude_publisher_, yawrate_publisher_, thrust_publisher_;
  geometry_msgs::Twist velocity_;
  hector_uav_msgs::AttitudeCommand attitude_;
  hector_uav_msgs::ThrustCommand thrust_;
  hector_uav_msgs::YawrateCommand yawrate_;
  bool flag;
  double targetLat;
  double targetLong;

  struct Axis
  {
    int axis;
    double max;
  };

  struct Button
  {
    int button;
  };

  struct
  {
    Axis x;
    Axis y;
    Axis z;
    Axis yaw;
  } axes_;

  struct
  {
    Button slow;
  } buttons_;

  double slow_factor_;


  std::queue<double> lat_que;
  double lat_val;
  std::queue<double> long_que;
  double long_val;
  std::queue<double> alt_que;
  double alt_val;
  
  std::ifstream latFile; 
  std::ifstream longFile; 
  std::ifstream altFile; 

public:
  Navi()
  {
    ros::NodeHandle params("~");
    latFile.open("/home/hello/hector_quadrotor_tutorial/src/hector_quadrotor/hector_quadrotor_navi/src/lat_val.txt", std::ios::app);
    if (latFile.is_open()){
      while(latFile >> lat_val){
        
        lat_que.push(lat_val);
        std::cout << "lat value is " <<lat_val<< std::endl;
      }
      latFile.close();
    }
    else ROS_INFO("Unable to open the file"); 

    longFile.open("/home/hello/hector_quadrotor_tutorial/src/hector_quadrotor/hector_quadrotor_navi/src/long_val.txt", std::ios::app);
    if (longFile.is_open()){
      while(longFile >> long_val){
        
        long_que.push(long_val);
        std::cout << "long value is " <<long_val<< std::endl;
      }
      longFile.close();
    }
    else ROS_INFO("Unable to open the file"); 
    
    altFile.open("/home/hello/hector_quadrotor_tutorial/src/hector_quadrotor/hector_quadrotor_navi/src/alt_val.txt", std::ios::app);
    if (altFile.is_open()){
      while(altFile >> alt_val){
        
        lat_que.push(alt_val);
        std::cout << "alt value is " <<alt_val<< std::endl;
      }
      altFile.close();
    }
    else ROS_INFO("Unable to open the file"); 

    params.param<int>("x_axis", axes_.x.axis, 4);
    params.param<int>("y_axis", axes_.y.axis, 3);
    params.param<int>("z_axis", axes_.z.axis, 2);
    params.param<int>("yaw_axis", axes_.yaw.axis, 1);

    params.param<double>("yaw_velocity_max", axes_.yaw.max, 90.0 * M_PI / 180.0);
    params.param<int>("slow_button", buttons_.slow.button, 1);
    params.param<double>("slow_factor", slow_factor_, 0.2);

    std::string control_mode_str;
    params.param<std::string>("control_mode", control_mode_str, "twist");
    targetLat=20;
    targetLong=20;

    //if (control_mode_str == "twist")
    //{
      params.param<double>("x_velocity_max", axes_.x.max, 2.0);
      params.param<double>("y_velocity_max", axes_.y.max, 2.0);
      params.param<double>("z_velocity_max", axes_.z.max, 2.0);

      fix_subscriber_ = node_handle_.subscribe<sensor_msgs::NavSatFix>("fix", 1, boost::bind(&Navi::fixTwistCallback, this, _1));
      //pos_subscriber_=node_handle_.subscribe<geometry_msgs::Pose>("pose", 1, boost::bind(&Teleop::callback_u1, this, _1));
      velocity_publisher_ = node_handle_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    /*}
    else if (control_mode_str == "attitude")
    {
      params.param<double>("x_roll_max", axes_.x.max, 0.35);
      params.param<double>("y_pitch_max", axes_.y.max, 0.35);
      params.param<double>("z_thrust_max", axes_.z.max, 25.0);
      joy_subscriber_ = node_handle_.subscribe<sensor_msgs::Joy>("joy", 1, boost::bind(&Teleop::joyAttitudeCallback, this, _1));
      attitude_publisher_ = node_handle_.advertise<hector_uav_msgs::AttitudeCommand>("command/attitude", 10);
      yawrate_publisher_ = node_handle_.advertise<hector_uav_msgs::YawrateCommand>("command/yawrate", 10);
      thrust_publisher_ = node_handle_.advertise<hector_uav_msgs::ThrustCommand>("command/thrust", 10);
    }*/

  }

  ~Navi()
  {
    stop();
  }

  /*void callback_u1(const geometry_msgs::PoseStamped& ps) { //ROS_INFO_STREAM(ps.position.x); 
//ROS_INFO_STREAM(ps.position.y); 
//ROS_INFO_STREAM(ps.position.z); 
}*/

  void fixTwistCallback(const sensor_msgs::NavSatFixConstPtr &fix)
  {
    
    if(!flag){
      flag=true;
      velocity_.linear.x = 0;
      velocity_.linear.y = 0;
      velocity_.linear.z = 0.3;
      velocity_publisher_.publish(velocity_);
      sleep(5);
      velocity_.linear.x = 0;
      velocity_.linear.y = 0;
      velocity_.linear.z = 0;
      velocity_publisher_.publish(velocity_);
    }
    
    /*
    ROS_INFO_STREAM(fix->latitude); //when x is positive latitude increase
    ROS_INFO_STREAM(fix->longitude);//when y is positive longitude decrease
    ROS_INFO_STREAM(fix->altitude);
    ROS_INFO_STREAM(que.front());*/

    if(targetLat-0.00002 > fix->latitude)
      velocity_.linear.x = 1;
    else if (targetLat+0.00002 < fix->latitude)
      velocity_.linear.x = -1;
    else
      velocity_.linear.x = 0;


    if(targetLong-0.00002 > fix->longitude)
      velocity_.linear.y = -1;
    else if (targetLong+0.00002 < fix->longitude)
      velocity_.linear.y = 1;
    else
      velocity_.linear.y = 0;

    if(velocity_.linear.y ==0 && velocity_.linear.x == 0){
      ROS_INFO("target lat: [%f]", targetLat);
      ROS_INFO("curr lat: [%f]", fix->latitude);
      targetLat=lat_que.front();
      lat_que.pop();
      ROS_INFO("target long: [%f]", targetLong);
      ROS_INFO("curr long: [%f]", fix->longitude);
      targetLong=long_que.front();
      long_que.pop();
    }
    
    //velocity_.linear.x = 0;
    //velocity_.linear.y = 0;
    //velocity_.linear.z = 0.3;
    /*geographic_msgs::GeoPoint geo_pt;
    geo_pt.latitude = fix->latitude;
    geo_pt.longitude = fix->longitude;
    geo_pt.altitude = fix->altitude;
    geodesy::UTMPoint utm_pt(geo_pt);
    ROS_INFO_STREAM(utm_pt.northing);
    ROS_INFO_STREAM(utm_pt.easting);*/
    velocity_publisher_.publish(velocity_);
  }

  /*void joyAttitudeCallback(const sensor_msgs::JoyConstPtr &joy)
  {
    attitude_.roll = getAxis(joy, axes_.x);
    attitude_.pitch = getAxis(joy, axes_.y);
    attitude_publisher_.publish(attitude_);

    thrust_.thrust = getAxis(joy, axes_.z);
    thrust_publisher_.publish(thrust_);

    yawrate_.turnrate = getAxis(joy, axes_.yaw);
    if (getButton(joy, buttons_.slow.button))
    {
      yawrate_.turnrate *= slow_factor_;
    }
    yawrate_publisher_.publish(yawrate_);
  }

  sensor_msgs::Joy::_axes_type::value_type getAxis(const sensor_msgs::JoyConstPtr &joy, Axis axis)
  {
    if (axis.axis == 0)
    {return 0;}
    sensor_msgs::Joy::_axes_type::value_type sign = 1.0;
    if (axis.axis < 0)
    {
      sign = -1.0;
      axis.axis = -axis.axis;
    }
    if ((size_t) axis.axis > joy->axes.size())
    {return 0;}
    return sign * joy->axes[axis.axis - 1] * axis.max;
  }

  sensor_msgs::Joy::_buttons_type::value_type getButton(const sensor_msgs::JoyConstPtr &joy, int button)
  {
    if (button <= 0)
    {return 0;}
    if ((size_t) button > joy->buttons.size())
    {return 0;}
    return joy->buttons[button - 1];
  }*/

  void stop()
  {
    if(velocity_publisher_.getNumSubscribers() > 0)
    {
      velocity_ = geometry_msgs::Twist();
      velocity_publisher_.publish(velocity_);
    }
    if(attitude_publisher_.getNumSubscribers() > 0)
    {
      attitude_ = hector_uav_msgs::AttitudeCommand();
      attitude_publisher_.publish(attitude_);
    }
    if(thrust_publisher_.getNumSubscribers() > 0)
    {
      thrust_ = hector_uav_msgs::ThrustCommand();
      thrust_publisher_.publish(thrust_);
    }
    if(yawrate_publisher_.getNumSubscribers() > 0)
    {
      yawrate_ = hector_uav_msgs::YawrateCommand();
      yawrate_publisher_.publish(yawrate_);
    }

  }
};

} // namespace hector_quadrotor

int main(int argc, char **argv)
{
  ros::init(argc, argv, "quadrotor_navi");

  hector_quadrotor::Navi navi;
  ros::spin();

  return 0;
}
