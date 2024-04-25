# update_thrusters.py
def update_thrusters(depth_control_signal, heading_control_signal):
    # Calculate thruster commands and clamp them
    # Return a dictionary of thruster messages
    return {
        'bow_port': Float64(data=bow_port_command),
        'bow_stbd': Float64(data=bow_stbd_command),
        # Include other thrusters...
    }