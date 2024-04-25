from std_msgs.msg import Float64
# update_thrusters.py
def update_thrusters(depth_control_signal, heading_control_signal):
    # Set Max/Min RPMs
    maxThrusterCMD = 1200
    minThrusterCMD = -1200
    # Calculate thruster commands and clamp them
    bow_port_command = max(min(depth_control_signal + heading_control_signal, maxThrusterCMD),
                           minThrusterCMD)
    bow_stbd_command = max(min(depth_control_signal - heading_control_signal, maxThrusterCMD),
                           minThrusterCMD)
    vert_port_command = max(min(depth_control_signal, maxThrusterCMD), minThrusterCMD)
    vert_stbd_command = max(min(depth_control_signal, maxThrusterCMD), minThrusterCMD)

    # Returning a dictionary of ROS messages for each thruster
    return {
        'bow_port': Float64(data=bow_port_command),
        'bow_stbd': Float64(data=bow_stbd_command),
        'vert_port': Float64(data=vert_port_command),
        'vert_stbd': Float64(data=vert_stbd_command)
    }