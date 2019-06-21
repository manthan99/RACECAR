/*
  Arduino ROS node for JetsonCar project
  The Arduino controls a TRAXXAS Rally Car
  MIT License
  JetsonHacks (2016)
*/

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <Servo.h> 
#define USB_USBCON
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle  nodeHandle;
// These are general bounds for the steering servo and the
// TRAXXAS Electronic Speed Controller (ESC)
const int minSteering = 30 ;
const int maxSteering = 150 ;
const int minThrottle = 0 ;
const int maxThrottle = 150 ;

Servo steeringServo;
Servo electronicSpeedController ;  // The ESC on the TRAXXAS works like a Servo

std_msgs::Int32 str_msg;
ros::Publisher chatter("chatter", &str_msg); 

// Arduino 'map' funtion for floating point
double fmap (double toMap, double in_min, double in_max, double out_min, double out_max) {
  return (toMap - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void driveCallback ( const geometry_msgs::Twist&  data )
{
  
//  int steeringAngle = fmap(twistMsg.angular.z, -1.0, 1.0, minSteering, maxSteering) ;
//  // The following could be useful for debugging
//  // str_msg.data= steeringAngle ;
chatter.publish(&str_msg);
//  // Check to make sure steeringAngle is within car range
//  if (steeringAngle < minSteering) { 
//    steeringAngle = minSteering;
//  }
//  if (steeringAngle > maxSteering) {
//    steeringAngle = maxSteering ;
//  }
  steeringServo.write(data.angular.z) ;
  delay(10);
  str_msg.data = data.linear.x;
  if(data.linear.x<85 || data.linear.x>95)
  electronicSpeedController.write(data.linear.x) ;
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
  chatter.publish(&str_msg);
 
}
 
ros::Subscriber<geometry_msgs::Twist> driveSubscriber("/cmd_vel", &driveCallback) ;

void setup(){
  pinMode(13, OUTPUT);
  Serial.begin(57600) ;
  nodeHandle.initNode();
  // This can be useful for debugging purposes
  nodeHandle.advertise(chatter);
  // Subscribe to the steering and throttle messages
  nodeHandle.subscribe(driveSubscriber) ;
  // Attach the servos to actual pins
  steeringServo.attach(9); // Steering servo is attached to pin 9
  electronicSpeedController.attach(11); // ESC is on pin 10
  // Initialize Steering and ESC setting
  // Steering centered is 90, throttle at neutral is 90
  steeringServo.write(95) ;
  electronicSpeedController.write(90) ;
  delay(1000) ;
  
}

void loop(){
  nodeHandle.spinOnce();
  delay(10);
}
