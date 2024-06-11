#!/usr/bin/env python3

import rospy
import turtle
from sensor_pembot.msg import sensor_data

# Initialize Turtle
screen = turtle.Screen()
pen = turtle.Turtle()

# Global variable to store the latest drawing data
latest_data = None
prev_time = None

# Function callback to receive data from the Sensor_data topic
def callback(data):
    global latest_data
    latest_data = data

# Function to process the drawing data and draw with Turtle
def process_draw_queue():
    global latest_data, prev_time
    if latest_data is not None:
        # Get the current time
        current_time = rospy.get_time()

        # Calculate the time difference
        if prev_time is None:
            t = 0
        else:
            t = current_time - prev_time
        prev_time = current_time

        # Extract data
        status = latest_data.cmd_vel.turn
        jarak = latest_data.kecepatan * t
        sudut_belok = latest_data.sudut

        # Process drawing commands
        if status == 2:
            pen.forward(jarak)
        elif status == 1:
            pen.backward(jarak)
        elif status == 4:
            pen.right(sudut_belok)
            pen.forward(jarak)
        elif status == 3:
            pen.left(sudut_belok)
            pen.forward(jarak)
        
        screen.update()

# Function to handle ROS node initialization and subscription
def draw_track_node():
    rospy.init_node('Draw_Track', anonymous=True)
    rospy.Subscriber('Sensor_data', sensor_data, callback)
    
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        process_draw_queue()
        rate.sleep()

if __name__ == '__main__':
    # Set up the turtle screen and tracer
    screen.tracer(0)
    
    # Start the ROS node and Turtle drawing
    try:
        draw_track_node()
    except rospy.ROSInterruptException:
        pass
    
    # Start the turtle main loop
    turtle.mainloop()
