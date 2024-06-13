import rospy
from sensor_pembot.msg import vel_motor  # type: ignore
from std_msgs.msg import Int64, Bool
import random

class RobotControl:
    def __init__(self):
        rospy.init_node("robot_control", anonymous=True)
        rospy.Subscriber("/ultrasonic_data", Int64, self.ultrasonicCb)
        rospy.Subscriber("/mission", Bool, self.missionCb)
        self.servo_pub = rospy.Publisher("/servo_pos", Int64, queue_size=10)
        self.cmd_pub = rospy.Publisher("/cmd_vel", vel_motor, queue_size=10)
        self.rate = rospy.Rate(10)  # Hz
        rospy.loginfo("Control Node has been started")

        self.mission_status = False
        self.distance = 0
        self.turn = 0
        self.vel = 0
        self.servo_pos = 90
        self.turning = False
        self.turning_timer = None
        self.checking_distance = False

        rospy.Timer(rospy.Duration(0, 100000), self.timerCb)
        rospy.spin()

    def servo_proc(self):
        # Choose a new direction to turn
        directions = [0, 180]  # Left and right
        self.servo_pos = random.choice(directions)

    def cmd_proc(self):
        if self.distance > 60 and not self.turning:
            self.servo_pos = 90
            self.turning = False
            self.checking_distance = False
            self.turn = 2  # Move forward
            self.vel = 127
        elif 35 < self.distance <= 60 and not self.turning:
            self.servo_pos = 90
            self.turning = False
            self.checking_distance = False
            self.turn = 2  # Move forward
            self.vel = 90
        elif 20 < self.distance <= 35 and not self.turning:
            self.servo_pos = 90
            self.turning = False
            self.checking_distance = False
            self.turn = 2  # Move forward
            self.vel = 85
        elif self.distance <= 10:
            self.servo_pos = 90
            self.turning = False
            self.checking_distance = False
            self.turn = 1  # Move backward
            self.vel = 90
        elif self.distance <= 20 and not self.turning:
            self.servo_pos = 90
            self.turning = True
            self.servo_proc()
            self.turning_timer = rospy.Time.now()
        elif self.turning:
            if rospy.Time.now() - self.turning_timer >= rospy.Duration(1, 300000):
                # Check distance after turning
                self.checking_distance = True
                self.turning = False
            else:
                if self.servo_pos == 0:
                    self.turn = 4  # Turn right
                    self.vel = 85
                elif self.servo_pos == 180:
                    self.turn = 3  # Turn left
                    self.vel = 85
        elif self.checking_distance:
            # Recheck the distance after turning
            if self.distance <= 20:
                self.servo_proc()
                self.turning = True
                self.turning_timer = rospy.Time.now()
            else:
                self.servo_pos = 90
                self.checking_distance = False
                self.turn = 2  # Move forward
                self.vel = 80

    def timerCb(self, event):
        servo = Int64()
        cmd = vel_motor()

        self.cmd_proc()

        servo.data = self.servo_pos
        cmd.turn = self.turn
        cmd.vel = self.vel

        if self.mission_status:
            self.servo_pub.publish(servo)
            self.cmd_pub.publish(cmd)
        else:
            servo.data = 90
            cmd.turn = 0
            cmd.vel = 0
            self.servo_pub.publish(servo)
            self.cmd_pub.publish(cmd)

        self.rate.sleep()

    def missionCb(self, msg):
        self.mission_status = msg.data

    def ultrasonicCb(self, msg):
        self.distance = msg.data

if __name__ == "__main__":
    try:
        RobotControl()
    except rospy.ROSInterruptException:
        pass
