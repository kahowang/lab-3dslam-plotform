#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"
#include "axis_ptz_cmd.h"
#include "axis_ptz_msg.h"

class IQRJoy
{
public:
  IQRJoy();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void ptzCallback(const axis_camera::axis_ptz_msg msg);
  void publish();

  ros::NodeHandle ph_, nh_;

  int pan_, tilt_, safe_button_, home_button_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
  ros::Subscriber ptz_sub_;

  axis_camera::axis_ptz_cmd last_published_;
  axis_camera::axis_ptz_msg last_ptz_state_;
  boost::mutex publish_mutex_, subscr_mutex_;
  bool safe_pressed_, home_pos_;
  bool zero_twist_published_;
  ros::Timer timer_;
};

IQRJoy::IQRJoy():
  ph_("~"),
  pan_(3),
  tilt_(4),
  safe_button_(4),
  home_button_(0)
{
  ph_.param("axis_pan", pan_, pan_);
  ph_.param("axis_tilt", tilt_, tilt_);
  ph_.param("safe_nutton", safe_button_, safe_button_);
  ph_.param("home_nutton", home_button_, home_button_);

  safe_pressed_ = false;
  home_pos_ = false;
  zero_twist_published_ = false;

  vel_pub_ = ph_.advertise<axis_camera::axis_ptz_cmd>("axis_ptz_cmd", 1, true);
  joy_sub_ = ph_.subscribe<sensor_msgs::Joy>("joy", 10, &IQRJoy::joyCallback, this);
  ptz_sub_ = nh_.subscribe<axis_camera::axis_ptz_msg>("axis_ptz_msg", 10, &IQRJoy::ptzCallback, this);

  timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&IQRJoy::publish, this));
}

void IQRJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{ 
  boost::mutex::scoped_lock lock(subscr_mutex_);

  axis_camera::axis_ptz_cmd vel;
  
  if(abs(joy->axes[pan_]) > 0.2)
  {
    vel.pan = -20 * joy->axes[pan_] + last_ptz_state_.pan;
  }
  else
  {
    vel.pan = last_ptz_state_.pan;
  }
  if(abs(joy->axes[tilt_]) > 0.2)
  {
    vel.tilt = 20 * joy->axes[tilt_] + last_ptz_state_.tilt;
  }
  else
  {
    vel.tilt = last_ptz_state_.tilt;
  }
  vel.zoom = 1;
  vel.speed = 50;
  last_published_ = vel;
  safe_pressed_ = joy->buttons[safe_button_];
  home_pos_ = joy->buttons[home_button_];
}

void IQRJoy::ptzCallback(const axis_camera::axis_ptz_msg msg)
{
  last_ptz_state_ = msg;
}

void IQRJoy::publish()
{
  boost::mutex::scoped_lock lock(publish_mutex_);

  if (safe_pressed_)
  {
    vel_pub_.publish(last_published_);
    zero_twist_published_=false;
  }
  else if(!safe_pressed_ && !zero_twist_published_)
  {
    vel_pub_.publish(*new axis_camera::axis_ptz_cmd());
    zero_twist_published_=true;
  }
  else if(home_pos_)
  {
    axis_camera::axis_ptz_cmd msg;
    msg.zoom = 1;
    msg.speed = 100; 
    vel_pub_.publish(msg);
    home_pos_=false;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "iqr_teleop");
  IQRJoy turtlebot_teleop;

  ros::spin();
}
