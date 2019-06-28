//Node converts NED output of IMU into ENU and adds covariance with timestamps Outputs on imu
#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <boost/assign.hpp>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64.h>

#define PI 3.14159265359
ros::Publisher imu_pub,imu_pub2;
sensor_msgs::Imu imu;
using namespace std_msgs;

void imuCallback(const geometry_msgs::Twist msg)
{
    imu.linear_acceleration.x = 0;
    imu.linear_acceleration.y = 0;
    imu.linear_acceleration.z = 0;

    imu.angular_velocity.x = msg.angular.x;
    imu.angular_velocity.y = msg.angular.y;
    imu.angular_velocity.z = msg.angular.z;

    imu.orientation_covariance=boost::assign::list_of(0.001)(0.0)(0.0)
    	                                                (0.0)(0.001)(0.0)
    	                                                (0.0)(0.0)(0.0001);

    imu.angular_velocity_covariance=boost::assign::list_of(0.001)(0.0)(0.0)
                                                  (0.0)(0.001)(0.0)
                                                  (0.0)(0.0)(0.0001);
    imu.linear_acceleration_covariance=boost::assign::list_of(10000)(0.0)(0.0)
                                                      (0.0)(0.001)(0.0)
                                                      (0.0)(0.0)(0.0001);
    imu.header.frame_id="imu";

    tf::Quaternion quat;

    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    

    yaw=msg.linear.z/(180.0/3.1415926);
    pitch=msg.linear.y/(180.0/3.1415926);
    roll=msg.linear.x/(180.0/3.1415926);
    Float64 yawf;
    yawf.data=yaw*(180.0/3.1415926);
    
    
    imu_pub2.publish(yawf);
    tf::Quaternion q;
    q.setRPY(tfScalar(roll), tfScalar(pitch), tfScalar(yaw));
    
    geometry_msgs::Quaternion odom_quat;
    tf::quaternionTFToMsg(q, odom_quat);
    imu.orientation = odom_quat;
    imu.header.stamp=ros::Time::now();
    imu_pub.publish(imu);
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_node");
    ros::NodeHandle n;
    imu_pub2=n.advertise<Float64>("/imu/orientation",1);
    ros::Subscriber imu_sub=n.subscribe<geometry_msgs::Twist>("arduino_nano/raw_orientation",5,imuCallback);
    imu_pub = n.advertise<sensor_msgs::Imu>("/imu", 5);
    ros::spin();
    return 0;
}

