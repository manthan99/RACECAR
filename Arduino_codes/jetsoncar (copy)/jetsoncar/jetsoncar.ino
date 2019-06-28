

#include <Servo.h> 

Servo steeringServo;
Servo electronicSpeedController ;  // The ESC on the TRAXXAS works like a Servo
 

// Arduino 'map' funtion for floating point
double fmap (double toMap, double in_min, double in_max, double out_min, double out_max) {
  return (toMap - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void setup(){
  pinMode(13, OUTPUT);

  steeringServo.attach(9); // Steering servo is attached to pin 9
  electronicSpeedController.attach(10); // ESC is on pin 11
  steeringServo.write(95) ;
  electronicSpeedController.write(90) ;
  delay(1000) ;
  
}

void loop(){
   electronicSpeedController.write(105);
  delay(10);
}
