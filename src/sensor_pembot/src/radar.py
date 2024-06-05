import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import rospy
from std_msgs.msg import Int64

class RadarInterface:
    def __init__(self):
        self.start_angle = 83
        self.end_angle = 97
        self.current_distance = 0
        self.servo_angle = 90  # Awal sudut servo
        
        # Inisialisasi node ROS
        rospy.init_node('radar_interface', anonymous=True)
        rospy.Subscriber('/ultrasonic_data', Int64, self.ultrasonic_callback)
        rospy.Subscriber('/servo_pos', Int64, self.servo_angle_cb)
        
        # Inisialisasi plot
        self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})
        self.ax.set_ylim(0, 100)  # Mengatur ulang batas y untuk menampilkan data dari 0 hingga maksimum
        self.lims = [-10, 190]
        self.ax.set_thetalim(np.deg2rad(self.lims))
        
        # Inisialisasi teks
        self.text = self.ax.text(0.5, 0.9, '', transform=self.ax.transAxes, ha='center')
        
        # Inisialisasi bar chart
        angles = np.linspace(np.deg2rad(self.start_angle), np.deg2rad(self.end_angle), num=100)
        self.bars = self.ax.bar(angles, [0] * len(angles), width=np.deg2rad(1))
        
        # Membuat animasi
        self.ani = FuncAnimation(self.fig, self.update, init_func=self.init, interval=100, repeat=True)
        
    def ultrasonic_callback(self, msg):
        self.current_distance = msg.data
    
    def servo_angle_cb(self, msg):
        self.servo_angle = int(msg.data)
        self.start_angle = self.servo_angle - 7
        self.end_angle = self.servo_angle + 7
    
    def init(self):
        for bar in self.bars:
            bar.set_height(0)
        self.text.set_text('')
        return self.bars + (self.text,)
    
    def update(self, frame):
        start_angle_rad = np.deg2rad(self.start_angle)
        end_angle_rad = np.deg2rad(self.end_angle)
        
        angles = np.linspace(start_angle_rad, end_angle_rad, num=len(self.bars))
        distances = [self.current_distance for _ in angles]
        
        for bar, angle, distance in zip(self.bars, angles, distances):
            bar.set_height(distance)
            bar.set_x(angle)
            if self.current_distance <= 30:
                bar.set_color('red')
            else:
                bar.set_color('blue')
        
        # Memperbarui teks
        self.text.set_text(f'Start Angle: {self.start_angle}°\nEnd Angle: {self.end_angle}°')
        
        return self.bars + (self.text,)
    
    def run(self):
        plt.show()

if __name__ == '__main__':
    radar = RadarInterface()
    radar.run()
