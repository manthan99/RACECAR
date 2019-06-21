/*
Node to publish data for velocity and steering control
Teleop Control
published topics -  /cmd_vel  type -twist
subscribed topics - /joy      type -snesor_msgs/Joy
x button - activation
RT - forward throttle
LT - reverse throttle
Left Joystick - steering control

Author - Manthan Patel
*/


#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Twist.h>

geometry_msgs::Twist manual_control ;
ros::Publisher control;
ros::Subscriber joy_sub; 

//mapping function

double fmap (double toMap, double in_min, double in_max, double out_min, double out_max) {
  return (toMap - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  ros::NodeHandle nh;
  control = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  joy_sub= nh.subscribe<sensor_msgs::Joy>("joy", 10, &joyCallback);
  //default values
  manual_control.angular.z = 95;
  manual_control.linear.x = 90;
  ros::Rate loop_rate(10);

  
  while (ros::ok())
  {
    control.publish(manual_control);
    ros::spinOnce();
    loop_rate.sleep();  
  }

  return 0;  
}
