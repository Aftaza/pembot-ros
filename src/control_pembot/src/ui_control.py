import tkinter as tk
from tkinter import messagebox
import rospy
from sensor_pembot.msg import vel_motor # type: ignore

class ControlConsole:
    def __init__(self, root):
        self.root = root
        self.root.title("Control Console")

        # Create direction buttons
        self.btn_forward = tk.Button(root, text="Maju", command=lambda: self.set_command(self.move_forward))
        self.btn_backward = tk.Button(root, text="Mundur", command=lambda: self.set_command(self.move_backward))
        self.btn_left = tk.Button(root, text="Belok Kiri", command=lambda: self.set_command(self.turn_left))
        self.btn_right = tk.Button(root, text="Belok Kanan", command=lambda: self.set_command(self.turn_right))
        self.btn_stop = tk.Button(root, text="Stop", command=self.stop)

        # Create speed buttons
        self.btn_fast = tk.Button(root, text="Cepat", command=self.set_fast)
        self.btn_slow = tk.Button(root, text="Lambat", command=self.set_slow)

        # Place direction buttons
        self.btn_forward.grid(row=0, column=1, pady=5)
        self.btn_left.grid(row=1, column=0, padx=5)
        self.btn_stop.grid(row=1, column=1, pady=5)
        self.btn_backward.grid(row=2, column=1, pady=5)
        self.btn_right.grid(row=1, column=2, padx=5)

        # Place speed buttons
        self.btn_fast.grid(row=3, column=0, columnspan=3, sticky="ew", pady=5)
        self.btn_slow.grid(row=4, column=0, columnspan=3, sticky="ew", pady=5)

        # Speed state
        self.speed = 85  # Default speed is slow

        # ROS node initialization
        rospy.init_node('control_console', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', vel_motor, queue_size=10)

        # Timer
        self.current_command = None
        self.publish_rate = 10  # Publish rate in Hz
        self.run_publisher()

    def set_command(self, command):
        self.current_command = command

    def run_publisher(self):
        if self.current_command:
            self.current_command()
        self.root.after(1000 // self.publish_rate, self.run_publisher)

    def send_command(self, linear=0):
        msg = vel_motor()
        msg.turn = int(linear)
        msg.vel = self.speed
        self.pub.publish(msg)

    def move_backward(self):
        self.send_command(linear=1)
    
    def move_forward(self):
        self.send_command(linear=2)

    def turn_left(self):
        self.send_command(linear=3)

    def turn_right(self):
        self.send_command(linear=4)

    def stop(self):
        self.set_command(0)
        self.send_command()

    def set_fast(self):
        self.speed = 100
        messagebox.showinfo("Speed", "Kecepatan diatur ke Cepat")

    def set_slow(self):
        self.speed = 85
        messagebox.showinfo("Speed", "Kecepatan diatur ke Lambat")

if __name__ == "__main__":
    root = tk.Tk()
    console = ControlConsole(root)
    root.mainloop()
