import tkinter as tk
from tkinter import messagebox
import rospy
from costum_msg_srv.msg import vel # type: ignore

class ControlConsole:
    def __init__(self, root):
        rospy.init_node('ui_control', anonymous=False)
        rospy.Rate(10)
        self.pub = rospy.Publisher('cmd_vel', vel, queue_size=10)
        self.root = root
        self.root.title("Pembot Control Console")

        # Create direction buttons
        self.btn_forward = tk.Button(root, text="Maju", command=self.move_forward)
        self.btn_backward = tk.Button(root, text="Mundur", command=self.move_backward)
        self.btn_left = tk.Button(root, text="Belok Kiri", command=self.turn_left)
        self.btn_right = tk.Button(root, text="Belok Kanan", command=self.turn_right)
        self.btn_stop = tk.Button(root, text="Stop", command=self.stop)

        # Create speed buttons
        self.btn_fast = tk.Button(root, text="Cepat", command=self.set_fast)
        self.btn_slow = tk.Button(root, text="Lambat", command=self.set_slow)

        # Place direction buttons
        self.btn_forward.grid(row=0, column=1, pady=5)
        self.btn_left.grid(row=1, column=0, padx=5)
        self.btn_backward.grid(row=2, column=1, pady=5)
        self.btn_right.grid(row=1, column=2, padx=5)
        self.btn_stop.grid(row=1, column=1, padx=5)

        # Place speed buttons
        self.btn_fast.grid(row=3, column=0, columnspan=3, sticky="ew", pady=5)
        self.btn_slow.grid(row=4, column=0, columnspan=3, sticky="ew", pady=5)

        # Speed state
        self.speed = 127
        self.vel = vel()
    
    def stop(self):
        self.vel.pwm = 0
        self.vel.turn = "stop"
        self.pub.publish(self.vel)
        messagebox.showinfo("Action", f"Stop")

    def move_forward(self):
        self.vel.pwm = self.speed
        self.vel.turn = "maju"
        self.pub.publish(self.vel)
        messagebox.showinfo("Action", f"Maju dengan kecepatan {self.speed} pwm")

    def move_backward(self):
        self.vel.pwm = self.speed
        self.vel.turn = "mundur"
        self.pub.publish(self.vel)
        messagebox.showinfo("Action", f"Mundur dengan kecepatan {self.speed} pwm")

    def turn_left(self):
        messagebox.showinfo("Action", f"Belok kiri dengan kecepatan {self.speed} pwm")

    def turn_right(self):
        messagebox.showinfo("Action", f"Belok kanan dengan kecepatan {self.speed} pwm")

    def set_fast(self):
        self.speed = 255
        messagebox.showinfo("Speed", "Kecepatan diatur ke Cepat")

    def set_slow(self):
        self.speed = 127
        messagebox.showinfo("Speed", "Kecepatan diatur ke Lambat")

if __name__ == "__main__":
    try:
        root = tk.Tk()
        console = ControlConsole(root)
        root.mainloop()
    except rospy.ROSInterruptException:
        pass
