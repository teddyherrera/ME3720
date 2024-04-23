#!/usr/bin/env python
# Every Python ROS Node will have the line 1 declaration at the top. The first line makes sure your script is executed
# as a Python script.
# From ROS wiki
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, TwistStamped
import numpy as np
import matplotlib.pyplot as plt
import odometry_callback
import update_plot




# Documentation on how to initialize ROS Node in Python:
# http://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown#Initializing_your_ROS_Node
rospy.init_node('Lab1_main', anonymous=True)

# create the publisher object
# documentatiion for rospy.Publisher: http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers
bowPortThrusterPub = rospy.Publisher('/bow_port_thruster', Float64, queue_size=10)
bowStbdThrusterPub = rospy.Publisher('/bow_stbd_thruster', Float64, queue_size=10)
vertPortThrusterPub = rospy.Publisher('/vert_port_thruster', Float64, queue_size=10)
vertStbdThrusterPub = rospy.Publisher('/vert_stbd_thruster', Float64, queue_size=10)
aftPortThrusterPub = rospy.Publisher('/aft_port_thruster', Float64, queue_size=10)
aftStbdThrusterPub = rospy.Publisher('/aft_stbd_thruster', Float64, queue_size=10)
aftVertThrusterPub = rospy.Publisher('/aft_vert_thruster', Float64, queue_size=10)

# Assign callback function
fusion_state_sub = rospy.Subscriber('/fusion/pose_gt', Float64, odometry_callback)

# Matplotlib setup
plt.ion()
fig, axs = plt.subplots(4, 1)
depth_line, = axs[0].plot([], [], 'b-', linewidth=2)
depth_target_line, = axs[0].plot([], [], 'r--', linewidth=2)
error_line, = axs[1].plot([], [], 'k-', linewidth=2)
control_signal_line, = axs[2].plot([], [], 'm-', linewidth=2)
port_thruster_line, = axs[3].plot([], [], 'b-', linewidth=2)
stbd_thruster_line, = axs[3].plot([], [], 'g-', linewidth=2)

# Initialize the desired headings
desiredDepth = -10 # meters
desiredHeading = 90 # degrees
MAX_PROP = 1200 # max propeller rpm

desiredRate = 6 # Hertz
sampleRate = rospy.Rate(desiredRate)
dt = 1/desiredRate # change in time

# Create the publish message
bowPortThrusterMsg = Float64()
bowStbdThrusterMsg = Float64()
vertPortThrusterMsg = Float64()
vertStbdThrusterMsg = Float64()
aftPortThrusterMsg = Float64()
aftStbdThrusterMsg = Float64()
aftVertThrusterMsg = Float64()

# Waiting for odometry
received_odometry = False
while not received_odometry and not rospy.is_shutdown():
    rospy.loginfo("Waiting for odometry")
    sampleRate.sleep()

# Main control loop
start_time = rospy.get_time()
elapsed_time = 0
while not rospy.is_shutdown() and elapsed_time < 1200:  # 20 minutes
    if received_odometry:
        # PID Control logic here
        depth_error = desired_depth - current_depth
        depth_control_signal = compute_pid(depth_error)  # Define your PID computation logic

        # Update thrusters
        vert_port_thruster_msg = Float64()
        vert_stbd_thruster_msg = Float64()
        vert_port_thruster_msg.data = depth_control_signal
        vert_stbd_thruster_msg.data = -depth_control_signal
        vert_port_thruster_pub.publish(vert_port_thruster_msg)
        vert_stbd_thruster_pub.publish(vert_stbd_thruster_msg)

        # Update plots
        current_time = rospy.get_time() - start_time
        update_plot(depth_line, [current_time, current_depth])
        update_plot(depth_target_line, [current_time, desired_depth])
        update_plot(error_line, [current_time, depth_error])
        update_plot(control_signal_line, [current_time, depth_control_signal])
        update_plot(port_thruster_line, [current_time, vert_port_thruster_msg.data])
        update_plot(stbd_thruster_line, [current_time, vert_stbd_thruster_msg.data])
        plt.draw()
        plt.pause(0.1)

    rate.sleep()
    elapsed_time = rospy.get_time() - start_time
