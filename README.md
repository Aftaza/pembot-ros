# Pembot: ROS Obstacle-Avoiding Robot

Course project at Universitas Brawijaya (2024). A two-wheel ESP32 robot talks to ROS 1 over Wi-Fi
(rosserial TCP). It sweeps an ultrasonic sensor on a servo, turns away from obstacles, and streams wheel RPM and
speed back to a desktop console with a live radar view.

## Nodes

```
ESP32 (arduino/main_sketch)  ◄── rosserial TCP :11411 ──►  ROS master
  motors · servo · ultrasonic · 2× optocoupler encoders

/ultrasonic_data, /optocoupler_data ──► sensor_process ──► /sensor_data
/ultrasonic_data, /mission          ──► robot_control  ──► /cmd_vel, /servo_pos
/sensor_data                        ──► main_console (Tkinter: start/stop mission, status, log file)
/ultrasonic_data, /servo_pos        ──► radar_interface (polar plot of the sweep)
/sensor_data                        ──► robot_visualization (heading and motion)
```

| Package | Contents |
| --- | --- |
| `sensor_pembot` | Custom messages (`vel_motor`, `opt_rotary`, `sensor_data`), RPM and speed from the encoders, radar plot |
| `control_pembot` | Obstacle avoidance, Tkinter console, robot visualization |
| `launch_pembot` | `start_program.launch` starts rosserial and every node |

## Running it

1. Set your Wi-Fi credentials and the ROS host IP in `arduino/main_sketch/main_sketch.ino`, then flash the ESP32
   (needs `rosserial_arduino` and `ESP32_Servo`).
2. Build the workspace and start everything:

```bash
catkin_make
source devel/setup.bash
roslaunch launch_pembot start_program.launch
```

3. Press start in the console to begin the mission.

## Stack

ROS 1 (rospy, catkin) · rosserial · ESP32 (Arduino) · Python · Tkinter · Matplotlib
