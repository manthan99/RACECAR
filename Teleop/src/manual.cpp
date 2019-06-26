/*
Node to publish data for velocity and steering control
Teleop Control
published topics -  /cmd_vel1  type -twist
published topic -   /mode      type -Int64 (mode=1->manual mode=0->auto)
subscribed topics - /joy      type -snesor_msgs/Joy
x button - activation
RB - manual mode
LB - auto mode
RT - forward throttle
LT - reverse throttle
Left Joystick - steering control

Author - Manthan Patel
*/


#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int64.h>
#include <geometry_msgs/Twist.h>

#define wheelbase 0.35

geometry_msgs::Twist manual_control ;
geometry_msgs::Twist auto_control ;
ros::Publisher control;
ros::Publisher mode;
ros::Subscriber joy_sub; 
ros::Subscriber auto_vel;
int flag = 1;
std_msgs::Int64 mode_status ;
float omega = 0;
float v;
float radius;
float steering_angle;

//mapping function

double fmap (double toMap, double in_min, double in_max, double out_min, double out_max) {
  return (toMap - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//function for converting the differntial velocity into ackermann velocity

void velCallback(const geometry_msgs::Twist::ConstPtr& data)
{
  if (data->linear.x >= 0)
  {
    auto_control.linear.x = fmap(data->linear.x,0.5,1,100,105);
  }
  omega = (-1)*data->angular.z;
  v = 0.5;
  radius= v/omega; //at the C.M
  steering_angle = (180/3.146)*wheelbase/(std::sqrt(radius*radius-wheelbase*wheelbase/4));
  if(omega<=0)
    steering_angle = -steering_angle;
  steering_angle = steering_angle + 95 ;
  if(steering_angle>135)
    steering_angle = 135;
  if(steering_angle<55)
    steering_angle = 55;
  auto_control.angular.z = steering_angle;

}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{   

if(joy->buttons[5]==1)
{
  flag=1; //manual mode
}

if(joy->buttons[4]==1)
{
  flag=0; //auto mode
}

if(flag==1)
{
//activation button  
if(joy->buttons[2]==1)
{

//for steering control output between 55 to 135
// 55 -> extreme left , 135 -> extreme right, 95 ->straight
manual_control.angular.z = fmap((-1)*joy->axes[0],-1,1,55,135);

//esc speed control
//70-> max reverse 90->stall 110->max forward
//speed has been restricted for easy control otherwise value go from 0-180
if(joy->axes[5]<0.99)
manual_control.linear.x = fmap((-1)*joy->axes[5],-1,1,90,110); 

else if(joy->axes[2]<0.99)
manual_control.linear.x = fmap((1)*joy->axes[2],-1,1,70,90);  

if((joy->axes[5]==1) && (joy->axes[2]==1)) 
manual_control.linear.x = 90;

}

else
//when activation button is not pressed, reset everything  
{
manual_control.angular.z = 95;
manual_control.linear.x = 90;
control.publish(manual_control);
}

control.publish(manual_control);
}

mode_status.data = flag;
mode.publish(mode_status);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  ros::NodeHandle nh;
  control = nh.advertise<geometry_msgs::Twist>("cmd_vel1", 1);
  joy_sub= nh.subscribe<sensor_msgs::Joy>("joy", 10, &joyCallback);
  auto_vel = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 5, &velCallback);
  mode = nh.advertise<std_msgs::Int64>("mode",1);

  //default values
  manual_control.angular.z = 95;
  manual_control.linear.x = 90;
  ros::Rate loop_rate(10);

  
  while (ros::ok())
  {
    if(flag==1)
    control.publish(manual_control);
    else if(flag==0)
    control.publish(auto_control);
    mode.publish(mode_status);
    ros::spinOnce();
    loop_rate.sleep();  
  }

  return 0;  
}
