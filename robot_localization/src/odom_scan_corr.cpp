#include <ros/ros.h>
#include <boost/assign.hpp>
#include <nav_msgs/Odometry.h>
#include "ros/time.h"
nav_msgs::Odometry msg;
ros::Publisher odom_pub;

  
void odomCallback(nav_msgs::Odometry msg)
{
	if (ros::ok())
{

    //add covariance
    msg.pose.covariance=boost::assign::list_of(0.01)(0.0)(0.0)(0.0)(0.0)(0.0)
                                               (0.0)(0.01)(0.0)(0.0)(0.0)(0.0)
                                               (0.0)(0.0)(1.0e+9)(0.0)(0.0)(0.0)
                                               (0.0)(0.0)(0.0)(1.0e+9)(0.0)(0.0)
                                               (0.0)(0.0)(0.0)(0.0)(1.0e+9)(0.0)
                                               (0.0)(0.0)(0.0)(0.0)(0.0)(0.01);

    msg.twist.covariance=boost::assign::list_of(0.01)(0.0)(0.0)(0.0)(0.0)(0.0)
                                               (0.0)(0.01)(0.0)(0.0)(0.0)(0.0)
                                               (0.0)(0.0)(1.0e+9)(0.0)(0.0)(0.0)
                                               (0.0)(0.0)(0.0)(1.0e+9)(0.0)(0.0)
                                               (0.0)(0.0)(0.0)(0.0)(1.0e+9)(0.0)
                                               (0.0)(0.0)(0.0)(0.0)(0.0)(0.01);

    msg.header.frame_id="base_link";

    msg.twist.twist.linear.x = (-1)*msg.twist.twist.linear.x;
    msg.twist.twist.linear.y = 0;
    //publish the message
    odom_pub.publish(msg);
    //last_time = current_time;

    
  }
}


int main(int argc, char** argv){
  ros::init(argc, argv, "odom_scan_corr");

  ros::NodeHandle n;
  //odometry - covariance + tf
  ros::Subscriber odom_sub=n.subscribe<nav_msgs::Odometry>("odom_scan",5,odomCallback);
  odom_pub = n.advertise<nav_msgs::Odometry>("odom_scan_c", 5);
  ros::spin();
  return 0;
}
