#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32, Float32
import tkinter as tk
from tkinter import ttk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

class RobotStatusInterface:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Status Interface")

        self.status_var = tk.StringVar()
        self.rpm_var = tk.StringVar()
        self.speed_var = tk.StringVar()
        self.angle_var = tk.StringVar()

        self.create_widgets()
        self.setup_ros()

    def create_widgets(self):
        frame = ttk.Frame(self.root, padding="10")
        frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        ttk.Label(frame, text="Status:").grid(row=0, column=0, sticky=tk.W)
        ttk.Label(frame, textvariable=self.status_var).grid(row=0, column=1, sticky=tk.W)

        ttk.Label(frame, text="Roda RPM:").grid(row=1, column=0, sticky=tk.W)
        ttk.Label(frame, textvariable=self.rpm_var).grid(row=1, column=1, sticky=tk.W)

        ttk.Label(frame, text="Kecepatan (m/s):").grid(row=2, column=0, sticky=tk.W)
        ttk.Label(frame, textvariable=self.speed_var).grid(row=2, column=1, sticky=tk.W)

        ttk.Label(frame, text="Derajat Putar:").grid(row=3, column=0, sticky=tk.W)
        ttk.Label(frame, textvariable=self.angle_var).grid(row=3, column=1, sticky=tk.W)

        # Create a matplotlib figure
        self.figure = Figure(figsize=(5, 5), dpi=100)
        self.ax = self.figure.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.figure, master=frame)
        self.canvas.get_tk_widget().grid(row=4, column=0, columnspan=2)

    def setup_ros(self):
        rospy.init_node('robot_status_interface', anonymous=True)
        rospy.Subscriber('/cmd_vel', Int32, self.cmd_vel_callback)
        rospy.Subscriber('/wheel_rpm', Float32, self.rpm_callback)
        rospy.Subscriber('/robot_speed', Float32, self.speed_callback)
        rospy.Subscriber('/turn_angle', Float32, self.angle_callback)

    def cmd_vel_callback(self, data):
        if data.data == 0:
            self.status_var.set("Maju")
        elif data.data == 1:
            self.status_var.set("Mundur")
        elif data.data == 2:
            self.status_var.set("Putar Kanan")
        elif data.data == 3:
            self.status_var.set("Putar Kiri")

    def rpm_callback(self, data):
        self.rpm_var.set(f"{data.data:.2f}")

    def speed_callback(self, data):
        self.speed_var.set(f"{data.data:.2f}")

    def angle_callback(self, data):
        self.angle_var.set(f"{data.data:.2f}")

    def update_plot(self):
        self.ax.clear()
        self.ax.set_xlim(0, 1)
        self.ax.set_ylim(0, 1)
        self.ax.set_aspect('equal')

        status = self.status_var.get()
        if status == "Maju":
            self.ax.arrow(0.5, 0.5, 0, 0.1, head_width=0.05, head_length=0.05, fc='green', ec='green')
        elif status == "Mundur":
            self.ax.arrow(0.5, 0.5, 0, -0.1, head_width=0.05, head_length=0.05, fc='red', ec='red')
        elif status == "Putar Kanan":
            self.ax.arrow(0.5, 0.5, 0.1, 0.1, head_width=0.05, head_length=0.05, fc='orange', ec='orange')
        elif status == "Putar Kiri":
            self.ax.arrow(0.5, 0.5, -0.1, 0.1, head_width=0.05, head_length=0.05, fc='purple', ec='purple')

        self.canvas.draw()

    def run(self):
        self.root.after(100, self.update_plot)
        self.root.mainloop()
        rospy.spin()

if __name__ == '__main__':
    root = tk.Tk()
    app = RobotStatusInterface(root)
    app.run()
