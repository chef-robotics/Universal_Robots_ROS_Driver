# Similar to sensor_msgs/JointState, but with more fields

# Metadata
# `stamp` corresponds to the time when the state was recorded.
Header header
# NB: The order in all other lists follows the order of `names`.
string[] names

# Position data [rad]
# RTDE 'target_q'
# float64[] target_positions
# RTDE 'actual_q'
# float64[] actual_positions

# Velocity data [rad/s]
# RTDE 'target_qd'
# float64[] target_velocities
# RTDE 'actual_qd'
# float64[] actual_velocities

# Acceleration data [rad/s^2]
# RTDE 'target_qdd' (only target available)
# float64[] target_accelerations

# Current data [A]
# RTDE 'target_current'
float64[] target_currents
# RTDE 'actual_current'
float64[] actual_currents
# RTDE 'actual_current_window'
float64[] actual_current_windows

# Voltage data [V]
# RTDE 'actual_joint_voltage' (no target voltage)
float64[] actual_voltages

# Torque data [Nm]
# RTDE 'target_moment' (only target available)
float64[] target_torques

# Contextual data
# RTDE 'joint_mode'
int32[] control_modes
# RTDE 'joint_control_output' (typically the same as 'actual_current')
float64[] joint_control_outputs
