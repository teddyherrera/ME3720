#!/usr/bin/env python
# Every Python ROS Node will have the line 1 declaration at the top. The first line makes sure your script is executed
# as a Python script.
# From ROS wiki
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np
import json
from plot_data import plot_data
from PID_Controller import PIDController
from update_thrusters import update_thrusters

# Initialize ROS node
# Documentation on how to initialize ROS Node in Python:
# http://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown#Initializing_your_ROS_Node
rospy.init_node('Lab1_main', anonymous=True)

# create the publisher object
# documentation for rospy.Publisher: http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers
bowPortThrusterPub = rospy.Publisher('/bow_port_thruster', Float64, queue_size=10)
bowStbdThrusterPub = rospy.Publisher('/bow_stbd_thruster', Float64, queue_size=10)
vertPortThrusterPub = rospy.Publisher('/vert_port_thruster', Float64, queue_size=10)
vertStbdThrusterPub = rospy.Publisher('/vert_stbd_thruster', Float64, queue_size=10)
aftPortThrusterPub = rospy.Publisher('/aft_port_thruster', Float64, queue_size=10)
aftStbdThrusterPub = rospy.Publisher('/aft_stbd_thruster', Float64, queue_size=10)
aftVertThrusterPub = rospy.Publisher('/aft_vert_thruster', Float64, queue_size=10)

# Global variables
current_depth = None
current_heading = None
desired_rate = 6  # Hz
kp_depth, ki_depth, kd_depth = 300.0, 10, 5
kp_heading, ki_heading, kd_heading = 20.0, 1, 2

# Initialize PID Controllers
depth_PID = PIDController(kp_depth, ki_depth, kd_depth)
heading_PID = PIDController(kp_heading, ki_heading, kd_heading)


# build callback function
def fusion_state_callback(msg):
    global current_depth, current_heading
    # Convert quaternion to Euler angles to get heading...
    quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    angles = euler_from_quaternion(quat)
    current_heading = np.rad2deg(angles[2])
    current_depth = msg.pose.pose.position.z


# subscriber function
fusion_state_sub = rospy.Subscriber('/fusion/pose_gt', PoseStamped, fusion_state_callback)


# PID control setup
desired_depth = -10.0
desired_heading = 90
rate = rospy.Rate(desired_rate)  # 6 Hz
dt = 1 / desired_rate
# Data for plotting
plot_time = []
actual_depth_data = []
actual_heading_data = []
depth_error_data = []
heading_error_data = []
control_signal_data = []
heading_control_signal_data = []

# Main control loop
start_time = rospy.get_time()
elapsed_time = 0
try:
    while not rospy.is_shutdown() and elapsed_time < 430:
        if 'current_depth' in globals() and 'current_heading' in globals():
            depth_control_signal, depth_error = depth_PID.update(current_depth, desired_depth, dt)
            heading_control_signal, heading_error = heading_PID.update(current_heading, desired_heading, dt)

            # Update thrusters
            thruster_msgs = update_thrusters(depth_control_signal, heading_control_signal)
            bowPortThrusterPub.publish(thruster_msgs['bow_port'])
            bowStbdThrusterPub.publish(thruster_msgs['bow_stbd'])
            vertPortThrusterPub.publish(thruster_msgs['vert_port'])
            vertStbdThrusterPub.publish(thruster_msgs['vert_stbd'])
            # aftPortThrusterPub.publish(thruster_msgs['aft_port'])
            # aftStbdThrusterPub.publish(thruster_msgs['aft_stbd'])
            # aftVertThrusterPub.publish(thruster_msgs['aft_vert'])

            # Collect data for plotting
            current_time = rospy.get_time() - start_time
            plot_time.append(current_time)
            actual_depth_data.append(current_depth)
            actual_heading_data.append(current_heading)
            depth_error_data.append(depth_error)
            heading_error_data.append(heading_error)
            control_signal_data.append(depth_control_signal)
            heading_control_signal_data.append(heading_control_signal)

            rate.sleep()
            elapsed_time = rospy.get_time() - start_time
except rospy.ROSInterruptException:  # handles ROS shutdown requests Ctrl+C or rosnode kill etc.
    rospy.loginfo("ROS shutdown request received.")
except Exception as e:  # handles and displays unexpected errors that aren't keyboard interrupts or system exits
    rospy.logerr("Unhandled exception: {}".format(e))
finally:
    # Save and plot final data
    rospy.loginfo("Shutting down, saving data...")
    # Plotting function call
    plot_data(plot_time, actual_depth_data, [desired_depth] * len(plot_time), actual_heading_data,
              [desired_heading] * len(plot_time), depth_error_data, heading_error_data, control_signal_data,
              heading_control_signal_data)

    # Create a filename with ros_rate and PID gains
    filename = "results__ROSrate{}_KpD{}_KiD{}_KdD{}_KpH{}_KiH{}_KdH{}.json".format(
        desired_rate,
        kp_depth, ki_depth, kd_depth,
        kp_heading, ki_heading, kd_heading
    )
    # Save data to JSON for later analysis
    data = {
        "plot_time": plot_time,
        "actual_depth_data": actual_depth_data,
        "desired_depth_data": [desired_depth] * len(plot_time),
        "actual_heading_data": actual_heading_data,
        "desired_heading_data": [desired_heading] * len(plot_time),
        "depth_error_data": depth_error_data,
        "heading_error_data": heading_error_data
    }
    with open(filename, 'w') as f:
        json.dump(data, f)
