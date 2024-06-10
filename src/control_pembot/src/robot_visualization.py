#!/usr/bin/env python

import rospy
from sensor_pembot.msg import vel_motor
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyArrow
from matplotlib.animation import FuncAnimation

class RobotVisualizer:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.robot = Rectangle((0.4, 0.4), 0.2, 0.1, fill=True, color='blue')
        self.ax.add_patch(self.robot)
        self.arrow = None
        self.status_text = self.ax.text(0.5, 0.9, '', horizontalalignment='center', transform=self.ax.transAxes, fontsize=12, color='black')

        self.ax.set_xlim(0, 1)
        self.ax.set_ylim(0, 1)
        self.ax.set_aspect('equal')
        self.command = None

        # Sembunyikan label pada sumbu x dan y
        self.ax.set_xticks([])
        self.ax.set_yticks([])
        self.ax.set_xticklabels([])
        self.ax.set_yticklabels([])

        rospy.init_node('robot_visualizer', anonymous=True)
        rospy.Subscriber('/cmd_vel', vel_motor, self.callback)

        self.ani = FuncAnimation(self.fig, self.update, interval=100)
        plt.show()

    def callback(self, msg):
        self.command = msg.turn

    def update(self, frame):
        if self.arrow:
            self.arrow.remove()

        if self.command is not None:
            if self.command == 2:  # Maju
                self.arrow = FancyArrow(0.5, 0.55, 0, 0.1, width=0.05, color='green')
                self.status_text.set_text('Maju')
            elif self.command == 1:  # Mundur
                self.arrow = FancyArrow(0.5, 0.35, 0, -0.1, width=0.05, color='red')
                self.status_text.set_text('Mundur')
            elif self.command == 4:  # Putar kanan
                self.arrow = FancyArrow(0.5, 0.55, 0.1, 0.1, width=0.05, color='orange')
                self.status_text.set_text('Putar Kanan')
            elif self.command == 3:  # Putar kiri
                self.arrow = FancyArrow(0.5, 0.55, -0.1, 0.1, width=0.05, color='purple')
                self.status_text.set_text('Putar Kiri')
            else:
                self.arrow = None
                self.status_text.set_text('Stop')

            if self.arrow:
                self.ax.add_patch(self.arrow)

        plt.draw()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    visualizer = RobotVisualizer()
    visualizer.run()
