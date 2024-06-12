import tkinter as tk
from tkinter import ttk
from datetime import datetime
import rospy
from sensor_pembot.msg import sensor_data, vel_motor, opt_rotary

class MainConsole:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Status Interface")
        self.status_var = tk.StringVar()
        self.rpm_var = tk.StringVar()
        self.speed_var = tk.StringVar()
        self.angle_var = tk.StringVar()
        self.history_filename = "history.txt"

        self.create_widgets()
        self.setup_ros()
        self.is_running = False 

    def create_widgets(self):
        self.frame = ttk.Frame(self.root, padding="10")
        self.frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        ttk.Label(self.frame, text=f"Status: {self.status_var.get()}").grid(row=0, column=0, sticky=tk.W, padx=(5, 10), pady=5)
        ttk.Label(self.frame, text=f"RPM: {self.rpm_var.get()}").grid(row=0, column=3, sticky=tk.W, padx=(5, 10), pady=5)
        ttk.Label(self.frame, text=f"Kecepatan: {self.speed_var.get()}").grid(row=0, column=5, sticky=tk.W, padx=(5, 10), pady=5)
        ttk.Label(self.frame, text=f"Sudut: {self.angle_var.get()}").grid(row=0, column=7, sticky=tk.W, padx=(5, 10), pady=5)
        ttk.Label(self.frame, text="").grid(row=1, column=0, columnspan=8)

        self.text_widget = tk.Text(self.frame, height=6, width=50)
        self.text_widget.grid(row=1, column=0, columnspan=7, rowspan=3, padx=5, pady=5)

        ttk.Button(self.frame, text="Start", command=self.start).grid(row=1, column=7, padx=5, pady=5, sticky=tk.E)
        ttk.Button(self.frame, text="End", command=self.end).grid(row=2, column=7, padx=5, pady=5, sticky=tk.E)
        ttk.Button(self.frame, text="Exit", command=self.exit).grid(row=3, column=7, padx=5, pady=5, sticky=tk.E)

    def update_status_text(self):
        if self.is_running:
            status_text = f"Robot telah {self.status_var.get()}Maju dengan {self.rpm_var.get()} RPM serta kecepatan {self.speed_var.get()}m/s ke arah sudut {self.angle_var.get()}°"
            self.text_widget.delete('1.0', tk.END)
            self.text_widget.insert(tk.END, status_text)
            ttk.Label(self.frame, text=f"Status: {self.status_var.get()}").grid(row=0, column=0, sticky=tk.W, padx=(5, 10), pady=5)
            ttk.Label(self.frame, text=f"RPM: {self.rpm_var.get()}").grid(row=0, column=3, sticky=tk.W, padx=(5, 10), pady=5)
            ttk.Label(self.frame, text=f"Kecepatan: {self.speed_var.get()}m/s").grid(row=0, column=5, sticky=tk.W, padx=(5, 10), pady=5)
            ttk.Label(self.frame, text=f"Sudut: {self.angle_var.get()}").grid(row=0, column=7, sticky=tk.W, padx=(5, 10), pady=5)
            ttk.Label(self.frame, text="").grid(row=1, column=0, columnspan=8)
            self.save_to_history(status_text)

    def start(self):
        self.is_running = True

    def end(self):
        self.text_widget.insert(tk.END, "\n--- Stopped ---\n")
        self.is_running = False 

    def exit(self):
        self.root.quit()

    def save_to_history(self, status_text):
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        with open(self.history_filename, "a") as f:
            f.write(f"{timestamp}: {status_text}\n")

    def run(self):
        self.update_status_text()
        self.root.mainloop()

    def setup_ros(self):
        try:
            rospy.init_node('robot_status_interface', anonymous=True)
            rospy.Publisher('/cmd_vel', vel_motor, queue_size=10)
            rospy.Subscriber('Sensor_data', sensor_data, self.sensor_data_callback)
        except rospy.ROSException as e:
            print("Failed to initialize ROS:", e)

    def sensor_data_callback(self, data):
        self.angle_var.set(f"{data.sudut:.2f}")
        self.speed_var.set(f"{data.kecepatan:.2f}")
        self.rpm_var.set(f"{data.Optocoupler}")
        self.cmd_vel_callback(data.cmd_vel)
        self.update_status_text()

    def cmd_vel_callback(self, data):
        try:
            if data == 0:
                self.status_var.set("Maju")
            elif data == 1:
                self.status_var.set("Mundur")
            elif data == 2:
                self.status_var.set("Putar Kanan")
            elif data == 3:
                self.status_var.set("Putar Kiri")
        except Exception as e:
            print("Error in cmd_vel_callback:", e)

if __name__ == '__main__':
    root = tk.Tk()
    app = MainConsole(root)
    app.run()
    rospy.spin()
