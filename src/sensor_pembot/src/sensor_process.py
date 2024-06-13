import rospy
from math import pi, degrees
from std_msgs.msg import Int64
from sensor_pembot.msg import vel_motor, opt_rotary, sensor_data # type: ignore

class SensorProcess:
    def __init__(self):
        rospy.init_node('sensor_process', anonymous=True)
        rospy.Subscriber('/servo_pos', Int64, self.servocallback)
        rospy.Subscriber('/ultrasonic_data', Int64, self.Ultrasoniccallback)
        rospy.Subscriber('/cmd_vel', vel_motor, self.Cmdcallback)
        rospy.Subscriber('/optocoupler_data', opt_rotary, self.Optocouplercallback)
        self.pub = rospy.Publisher('/sensor_data', sensor_data, queue_size=10)
        self.rate = rospy.Rate(10) # 10hz
        rospy.loginfo("Sensor Node has been started")
        
        self.start_time = 1
        self.end_time = 0
        self.rpm_right = 0
        self.rpm_left = 0
        self.v_right = 0
        self.v_left = 0
        self.data_ultrasonic = 0
        self.data_servo = 0
        self.data_cmd_vel = vel_motor()
        self.avg_v = 0
        
        rospy.Timer(rospy.Duration(0, 100000), self.timer_callback)
        
        rospy.spin()

    def Ultrasoniccallback(self, data):
        self.data_ultrasonic = data.data

    def servocallback(self, data):  
        self.data_servo = data.data

    def Cmdcallback(self, data):
        self.data_cmd_vel.turn = data.turn
        self.data_cmd_vel.vel = data.vel
        if self.data_cmd_vel.turn == 3 or self.data_cmd_vel.turn == 4:
            self.start_time = rospy.get_time()
            self.end_time = 0
        elif self.data_cmd_vel.turn == 0:
            self.end_time = rospy.get_time()
            self.start_time = 0
        else:
            self.end_time = 0
            self.start_time = 1

    def Optocouplercallback(self, data):
        self.rpm_right = data.right_opt
        self.rpm_left = data.left_opt
        self.v_right = (self.rpm_right/60) * (pi * 0.06)
        self.v_left = (self.rpm_left/60) * (pi * 0.06)
        self.avg_v = (self.v_right + self.v_left) / 2  # Correctly calculate the average velocity
    
    def degreeTurn(self):
        delta_theta = 0.0
        if self.start_time == 0:
            delta_t = self.end_time - self.start_time
            omega = (self.v_right - self.v_left) / 0.12
            delta_theta = degrees(omega * delta_t)

        return delta_theta


    def timer_callback(self, event):

        delta_theta = self.degreeTurn()

        sensor_data_msg = sensor_data()
        sensor_data_msg.rpm.right_opt = self.rpm_right
        sensor_data_msg.rpm.left_opt = self.rpm_left
        sensor_data_msg.cmd_vel.turn = self.data_cmd_vel.turn
        sensor_data_msg.cmd_vel.vel = self.data_cmd_vel.vel
        sensor_data_msg.servo = self.data_servo
        sensor_data_msg.ultrasonic = self.data_ultrasonic
        sensor_data_msg.degree = round(float(delta_theta), 2)
        sensor_data_msg.velocity = round(float(self.avg_v), 2)

        self.pub.publish(sensor_data_msg)
        self.rate.sleep()

if __name__ == '__main__':
    try:
        SensorProcess()
    except rospy.ROSInterruptException:
        pass