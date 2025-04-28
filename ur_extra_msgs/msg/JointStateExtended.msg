# Similar to sensor_msgs/JointState, but with more fields.
# TODO(cj): Should probably add `has_*` flags / sentinel values.

# Meta data
#
# `stamp` corresponds to the time when the state was recorded.
Header header
# NB: The order in all other lists follows the order of `names`.
string[] names

# Position data [rad]
#
# RTDE `target_q`
float64[] target_positions
# RTDE `actual_q`
float64[] actual_positions

# Velocity data [rad/s]
#
# RTDE `target_qd`
float64[] target_velocities
# RTDE `actual_qd`
float64[] actual_velocities

# Acceleration data [rad/s^2]
#
# RTDE `target_qdd`
float64[] target_accelerations
# RTDE `actual_qdd` -- possibly unavailable?
float64[] actual_accelerations

# Current data [mA]
#
# RTDE `target_current`
float64[] target_currents
# RTDE `actual_current`
float64[] actual_currents
# RTDE `actual_current_window`
float64[] actual_current_windows
# RTDE `joint_control_output` -- possibly same as `actual_current`?
float64[] joint_control_outputs

# Voltage data [V]: RTDE `actual_joint_voltage`
float64[] actual_voltages

# Torque data [Nm]
#
# RTDE `target_moment`
float64[] target_torques
# RTDE `actual_moment` -- possibly unavailable?
float64[] actual_torques

# Contextual data
#
# Temperature [C]: RTDE `joint_temperatures`
float64[] temperatures
# Mode: RTDE `joint_mode`
int32[] control_modes
