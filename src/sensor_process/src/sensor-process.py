#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32, Int64
from math import pi
from sensor_pembot.msg import sensor_data, vel_motor, opt_rotary

class SensorProcess:
    def __init__(self):
        rospy.init_node('sensor_process', anonymous=True)
        rospy.Subscriber('Servo_Pos', Int64, self.servocallback)
        rospy.Subscriber('Ultrasonic_data', Int32, self.Ultrasoniccallback)
        rospy.Subscriber('Cmd_vel', vel_motor, self.Cmdcallback)
        rospy.Subscriber('Optocoupler_data', opt_rotary, self.Optocouplercallback)
        self.pub = rospy.Publisher('Sensor_data', sensor_data, queue_size=10)
        self.rate = rospy.Rate(10) # 10hz
        rospy.loginfo("Node has been started")
        
        self.prev_time = None
        self.rpm_right = 0
        self.rpm_left = 0
        self.v_right = 0
        self.v_left = 0
        self.data_ultrasonic = 0
        self.data_servo = 0
        self.data_cmd_vel = vel_motor()
        self.avg_v = 0
        
        rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        
        rospy.spin()

    def Ultrasoniccallback(self, data):
        self.data_ultrasonic = data.data

    def servocallback(self, data):  
        self.data_servo = data.data

    def Cmdcallback(self, data):
        self.data_cmd_vel = data

    def Optocouplercallback(self, data):
        self.rpm_right = data.right_opt
        self.rpm_left = data.left_opt
        self.v_right = self.rpm_right * ((2 * pi * 2.6) / 60)
        self.v_left = self.rpm_left * ((2 * pi * 2.6) / 60)
        self.avg_v = (self.v_right + self.v_left) / 2  # Correctly calculate the average velocity

    def timer_callback(self, event):
        current_time = rospy.get_time()
        if self.prev_time is None:
            self.prev_time = current_time
            return

        delta_t = current_time - self.prev_time
        self.prev_time = current_time

        delta_theta = ((self.v_right - self.v_left) * delta_t) / 10.8

        draw_data_msg = sensor_data()
        draw_data_msg.Optocoupler.right_opt = self.rpm_right
        draw_data_msg.Optocoupler.left_opt = self.rpm_left
        draw_data_msg.cmd_vel.turn = self.data_cmd_vel.turn
        draw_data_msg.cmd_vel.vel = self.data_cmd_vel.vel
        draw_data_msg.servo = self.data_servo
        draw_data_msg.ultrasonic = self.data_ultrasonic
        draw_data_msg.sudut = delta_theta
        draw_data_msg.kecepatan = self.avg_v
        
        rospy.loginfo(f'Draw data: {draw_data_msg}')
        self.pub.publish(draw_data_msg)
        self.rate.sleep()

if __name__ == '__main__':
    try:
        SensorProcess()
    except rospy.ROSInterruptException:
        pass
