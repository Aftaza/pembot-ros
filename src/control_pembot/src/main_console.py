import tkinter as tk
from tkinter import ttk
from datetime import datetime

class MainConsole:
    def _init_(self, root):
        self.root = root
        self.root.title("Robot Status Interface")

        self.status_var = tk.StringVar()
        self.rpm_var = tk.StringVar()
        self.speed_var = tk.StringVar()
        self.angle_var = tk.StringVar()

        self.history_filename = "history.txt"

        self.create_widgets()

    def create_widgets(self):
        style = ttk.Style()
        style.configure("Custom.TLabel", foreground="black")

        frame = ttk.Frame(self.root, padding="10", style="Custom.TFrame")
        frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        ttk.Label(frame, text="Status:", style="Custom.TLabel").grid(row=0, column=0, sticky=tk.W, padx=(5, 10), pady=5)
        ttk.Label(frame, text="RPM Roda:", style="Custom.TLabel").grid(row=0, column=3, sticky=tk.W, padx=(5, 10), pady=5)
        ttk.Label(frame, text="Kecepatan (m/s):", style="Custom.TLabel").grid(row=0, column=5, sticky=tk.W, padx=(5, 10), pady=5)
        ttk.Label(frame, text="Derajat Putar:", style="Custom.TLabel").grid(row=0, column=7, sticky=tk.W, padx=(5, 10), pady=5)

        # Add an empty row as separator
        ttk.Label(frame, text="").grid(row=1, column=0, columnspan=8)

        # Create a Text widget to display robot status
        self.text_widget = tk.Text(frame, height=6, width=50)
        self.text_widget.grid(row=1, column=0, columnspan=7, rowspan=3, padx=5, pady=5)

        # Add buttons
        ttk.Button(frame, text="Start", command=self.start).grid(row=1, column=7, padx=5, pady=5, sticky=tk.E)
        ttk.Button(frame, text="End", command=self.end).grid(row=2, column=7, padx=5, pady=5, sticky=tk.E)
        ttk.Button(frame, text="Exit", command=self.exit).grid(row=3, column=7, padx=5, pady=5, sticky=tk.E)

    def update_status_text(self):
        status_text = f"Robot telah {self.status_var.get()} MAJU dengan {self.rpm_var.get()}10 RPM serta kecepatan {self.speed_var.get()}100m/s yang berputar sebesar 30 derajat {self.angle_var.get()}"
        self.text_widget.delete('1.0', tk.END)
        self.text_widget.insert(tk.END, status_text)
        self.save_to_history(status_text)

    def start(self):
        self.text_widget.insert(tk.END, "\nProgram mulai!\n")

    def end(self):
        self.text_widget.insert(tk.END, "Program berhenti!\n")

    def exit(self):
        self.root.quit()

    def save_to_history(self, status_text):
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        with open(self.history_filename, "a") as f:
            f.write(f"{timestamp}: {status_text}\n")

    def run(self):
        self.update_status_text()
        self.root.mainloop()

if __name__ == '__main__':
    root = tk.Tk()
    app = MainConsole(root)
    app.run()