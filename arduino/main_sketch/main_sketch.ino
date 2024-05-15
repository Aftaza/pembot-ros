#include <ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/String.h>

#define leftMotor1 26
#define leftMotor2 27
#define rightMotor1 14
#define rightMotor2 12

#define enaLeft 25
#define enaRight 13


ros::NodeHandle  nh;

void mundur(int pwm){
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, HIGH);
  digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, LOW);
  analogWrite(enaLeft, pwm);
  analogWrite(enaRight, pwm);
}

void maju(int pwm){
  digitalWrite(leftMotor1, HIGH);
  digitalWrite(leftMotor2, LOW);
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, HIGH);
  analogWrite(enaLeft, pwm);
  analogWrite(enaRight, pwm);
}

void stop(){
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, LOW);
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, LOW);
  analogWrite(enaLeft, 0);
  analogWrite(enaRight, 0);
}

void messageCb( const std_msgs::Int64& cmd_vel){
  if(cmd_vel.data == 1){
    mundur(127);
  }else if(cmd_vel.data == 2){
    maju(127);
  }else{
    stop();
  }
}

ros::Subscriber<std_msgs::Int64> sub("cmd_vel", &messageCb );

void setup()
{ 
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);

  pinMode(enaLeft, OUTPUT);
  pinMode(enaRight, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{  
  nh.spinOnce();
  delay(1);
}