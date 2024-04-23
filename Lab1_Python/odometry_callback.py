# Callback for odometry
def odometry_callback(data):
    global received_odometry, current_depth
    received_odometry = True
    current_depth = data.pose.pose.position.z
