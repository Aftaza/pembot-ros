#include <ros.h>
#include <std_msgs/Int64.h>

const int trigPin = 23;
const int echoPin = 22;

//define sound speed in cm/uS
#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701

long duration;
float distanceCm;
float distanceInch;

ros::NodeHandle nh;
std_msgs::Int64 ultrasonic_msg;
ros::Publisher ultrasonic_node( "ultrasonic_data", &ultrasonic_msg );

void setup() {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input

  nh.initNode();
  nh.advertise(ultrasonic_node);

}

void loop() {
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  
  // Calculate the distance
  distanceCm = duration * SOUND_SPEED/2;
  
  // Convert to inches
  distanceInch = distanceCm * CM_TO_INCH;

  ultrasonic_msg.data = (int) distanceCm ;

  ultrasonic_node.publish(&ultrasonic_msg);
  nh.spinOnce();

  delay(10);
}