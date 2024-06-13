import tkinter as tk
from tkinter import ttk
import rospy
from datetime import datetime
from sensor_pembot.msg import sensor_data # type: ignore
from std_msgs.msg import Int64, Bool

class MainConsole:
    def __init__(self, root):
        self.root = root
        self.root.title("Main Console Robot")

        self.status_var = tk.StringVar()
        self.rpmL_var = tk.StringVar()
        self.rpmR_var = tk.StringVar()
        self.speed_var = tk.StringVar()
        self.angle_var = tk.StringVar()

        self.history_filename = str(rospy.get_param('/logs_path', f'/home/aftaza/pembot/logs/{datetime.now().strftime("%Y-%m-%d %H:%M:%S")}_logs.txt'))

        self.create_widgets()

        self.mission_status = False
        self.cur_status = 0

        self.rpmL = 0
        self.rpmR = 0
        self.velocity = 0
        self.degree = 0
        self.distance = 0
        self.status = 0
        self.servo = 90

        # init node
        rospy.init_node("main_console", anonymous=True)
        rospy.Subscriber("/sensor_data", sensor_data, self.update_var)
        self.cmd_pub = rospy.Publisher("/mission", Bool, queue_size=10)
        self.update_status_text(f"Robot sedang berhenti v:{self.velocity} m/s wL:{self.rpmL} rpm wR:{self.rpmR} rpm deg:{self.degree}°")
        rospy.Timer(rospy.Duration(0, 100000), self.publish_mission)

        # Schedule the update function to be called periodically
        # self.update_ui()

    def create_widgets(self):
        style = ttk.Style()
        style.configure("Custom.TLabel", foreground="black")

        frame = ttk.Frame(self.root, padding="10", style="Custom.TFrame")
        frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        ttk.Label(frame, text="Status:", style="Custom.TLabel").grid(row=0, column=0, sticky=tk.W, padx=(5, 10), pady=5)
        ttk.Label(frame, textvariable=self.status_var, style="Custom.TLabel").grid(row=0, column=1, sticky=tk.W, padx=(5, 10), pady=5)
        ttk.Label(frame, text="RPM Left:", style="Custom.TLabel").grid(row=0, column=2, sticky=tk.W, padx=(5, 10), pady=5)
        ttk.Label(frame, textvariable=self.rpmL_var, style="Custom.TLabel").grid(row=0, column=3, sticky=tk.W, padx=(5, 10), pady=5)
        ttk.Label(frame, text="RPM Right:", style="Custom.TLabel").grid(row=0, column=4, sticky=tk.W, padx=(5, 10), pady=5)
        ttk.Label(frame, textvariable=self.rpmR_var, style="Custom.TLabel").grid(row=0, column=5, sticky=tk.W, padx=(5, 10), pady=5)
        ttk.Label(frame, text="Kecepatan (m/s):", style="Custom.TLabel").grid(row=0, column=6, sticky=tk.W, padx=(5, 10), pady=5)
        ttk.Label(frame, textvariable=self.speed_var, style="Custom.TLabel").grid(row=0, column=7, sticky=tk.W, padx=(5, 10), pady=5)
        ttk.Label(frame, text="Derajat Putar:", style="Custom.TLabel").grid(row=0, column=8, sticky=tk.W, padx=(5, 10), pady=5)
        ttk.Label(frame, textvariable=self.angle_var, style="Custom.TLabel").grid(row=0, column=9, sticky=tk.W, padx=(5, 10), pady=5)

        # Add an empty row as separator
        ttk.Label(frame, text="").grid(row=1, column=0, columnspan=8)

        # Create a Text widget to display robot status
        self.text_widget = tk.Text(frame, height=6, width=80)
        self.text_widget.grid(row=1, column=0, columnspan=8, rowspan=4, padx=5, pady=5)
        # self.text_widget.config(state=tk.DISABLED)

        # Add buttons
        ttk.Button(frame, text="Start", command=self.start).grid(row=1, column=8, padx=5, pady=5, sticky=tk.E, columnspan=2)
        ttk.Button(frame, text="End", command=self.end).grid(row=2, column=8, padx=5, pady=5, sticky=tk.E, columnspan=2)
        ttk.Button(frame, text="Exit", command=self.exit).grid(row=3, column=8, padx=5, pady=5, sticky=tk.E, columnspan=2)

    def update_var(self, msg):
        self.status = msg.cmd_vel.turn
        self.velocity = msg.velocity
        self.degree = msg.degree
        self.distance = msg.ultrasonic
        self.servo = msg.servo
        self.rpmL = msg.rpm.left_opt
        self.rpmR = msg.rpm.right_opt

        self.rpmL_var.set(f"{self.rpmL} rpm")
        self.rpmR_var.set(f"{self.rpmR} rpm")
        self.speed_var.set(f"{self.velocity} m/s")
        self.angle_var.set(f"{self.degree}°")

        if self.status == 1:
            cmd = f"Robot sedang mundur v:{self.velocity} m/s wL:{self.rpmL} rpm wR:{self.rpmR} rpm deg:{self.degree}°"
            self.status_var.set("Mundur")
        elif self.status == 2:
            cmd = f"Robot sedang maju v:{self.velocity} m/s wL:{self.rpmL} rpm wR:{self.rpmR} rpm deg:{self.degree}°"
            self.status_var.set("Maju")
        elif self.status == 3:
            cmd = f"Robot sedang berputar kekiri v:{self.velocity} m/s wL:{self.rpmL} rpm wR:{self.rpmR} rpm deg:{self.degree}°"
            self.status_var.set("Kiri")
        elif self.status == 4:
            cmd = f"Robot sedang berputar kekanan v:{self.velocity} m/s wL:{self.rpmL} rpm wR:{self.rpmR} rpm deg:{self.degree}°"
            self.status_var.set("Kanan")
        else:
            cmd = f"Robot sedang berhenti v:{self.velocity} m/s wL:{self.rpmL} rpm wR:{self.rpmR} rpm deg:{self.degree}°"
            self.status_var.set("Stop")

        if self.cur_status != self.status:
            self.update_status_text(cmd)
            self.cur_status = self.status

    def update_status_text(self, cmd):
        self.save_to_history(cmd)
        status_text = self.read_logs()
        self.rpmL_var.set(f"{self.rpmL} rpm")
        self.rpmR_var.set(f"{self.rpmR} rpm")
        self.speed_var.set(f"{self.velocity} m/s")
        self.angle_var.set(f"{self.degree}°")
        self.text_widget.delete('1.0', tk.END)
        self.text_widget.insert(tk.END, status_text)
        self.text_widget.see(tk.END)

    def publish_mission(self, event):
        msg = Bool()
        msg.data = self.mission_status
        self.cmd_pub.publish(msg)

    def start(self):
        self.mission_status = True
        self.text_widget.insert(tk.END, "\nProgram mulai!\n")

    def end(self):
        self.mission_status = False
        self.text_widget.insert(tk.END, "Program berhenti!\n")

    def exit(self):
        self.root.quit()

    def read_logs(self):
        stringny = ""
        with open(self.history_filename, "r") as f:
            cmds = f.readlines()
        for cmd in cmds:
            stringny += cmd
        return stringny
    
    def save_to_history(self, status_text):
        with open(self.history_filename, "a") as f:
            f.write(f"{status_text}\n")

    def run(self):
        self.root.mainloop()
        rospy.spin()

if __name__ == '__main__':
    root = tk.Tk()
    app = MainConsole(root)
    app.run()
