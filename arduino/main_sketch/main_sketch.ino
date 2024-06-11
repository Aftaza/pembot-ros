#include <WiFi.h>
#define ROSSERIAL_ARDUINO_TCP
#include <timer.h>
#include <ros.h>
#include <ESP32_Servo.h>
#include <sensor_pembot/vel_motor.h>
#include <sensor_pembot/opt_rotary.h>
#include <std_msgs/Int64.h>

// Motor pin
#define leftMotor1 26
#define leftMotor2 27
#define rightMotor1 14
#define rightMotor2 12

#define enaLeft 25
#define enaRight 13

// ultrasonik pin
const int trigPin = 23;
const int echoPin = 22;

//define sound speed in cm/uS
#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701

long duration;
float distanceCm;

// servo pin
int servoPin = 18;

// optocoupler pin
const int rotaryLeft = 32;
const int rotaryRight = 33;

int counterLeft = 0;
int counterRight = 0;

float leftRPM;
float rightRPM;

//server settings
IPAddress server(192,168,43,221);
uint16_t serverPort = 11411;
const char* ssid = "meizu";
const char* password = "jeksentiago";


ros::NodeHandle nh;

//servo init
Servo myservo;

// timer init
Timer timer;

void setupWiFi(){
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

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

void turnLeft(int pwm){
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, HIGH);
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, HIGH);
  analogWrite(enaLeft, pwm);
  analogWrite(enaRight, pwm);
}

void turnRight(int pwm){
  digitalWrite(leftMotor1, HIGH);
  digitalWrite(leftMotor2, LOW);
  digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, LOW);
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

void messageCbMotor( const sensor_pembot::vel_motor& cmd_vel){
  if(cmd_vel.turn == 1){
    mundur(cmd_vel.vel);
  }else if(cmd_vel.turn == 2){
    maju(cmd_vel.vel);
  }else if(cmd_vel.turn == 3){
    turnLeft(cmd_vel.vel);
  }else if(cmd_vel.turn == 4){
    turnRight(cmd_vel.vel);
  }
  else{
    stop();
  }
}

void ultrasonikRange(){
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
}

void messageCbServo(const std_msgs::Int64& servo_pos){
  myservo.write(servo_pos.data);
}

ros::Subscriber<sensor_pembot::vel_motor> subMotor("cmd_vel", &messageCbMotor );
ros::Subscriber<std_msgs::Int64> subServo("servo_pos", &messageCbServo);

std_msgs::Int64 ultrasonic_msg;
ros::Publisher ultrasonic_node( "ultrasonic_data", &ultrasonic_msg );

sensor_pembot::opt_rotary opt_msg;
ros::Publisher optocoupler_node("optocoupler_data", &opt_msg);

void countLeft(){
  counterLeft++;
}

void countRight(){
  counterRight++;
}

void RPM(){
  leftRPM = counterLeft * 60;
  rightRPM = counterRight * 60;
  counterLeft = 0;
  counterRight = 0;
}

void setup()
{ 
  Serial.begin(115200);
  setupWiFi();
  delay(1000);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);

  pinMode(enaLeft, OUTPUT);
  pinMode(enaRight, OUTPUT);

  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input

  myservo.attach(servoPin);

  attachInterrupt(digitalPinToInterrupt(rotaryLeft), countLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(rotaryRight), countRight, RISING);

  timer.setInterval(1000);
  timer.setCallback(RPM);
  timer.start();

  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  nh.subscribe(subMotor);
  nh.subscribe(subServo);
  nh.advertise(ultrasonic_node);
  nh.advertise(optocoupler_node);

  Serial.println("Get Ready");
  delay(1000);
}

void loop()
{  
  ultrasonikRange();
  opt_msg.left_opt = (int) leftRPM;
  opt_msg.right_opt = (int) rightRPM;
  optocoupler_node.publish(&opt_msg);
  ultrasonic_msg.data = (int) distanceCm ;
  ultrasonic_node.publish(&ultrasonic_msg);
  nh.spinOnce();
  timer.update();
  delay(10);
}