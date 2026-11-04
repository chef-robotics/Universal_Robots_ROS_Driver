# How compliant the arm is during a ComplianceWindow:
# a tool-frame spring-damper around the planned motion.
# Tool-frame order: x, y, z, rx, ry, rz.

# [N/m, N/m, N/m, N·m/rad, N·m/rad, N·m/rad]
float64[6] stiffness
# [N·s/m, N·s/m, N·s/m, N·m·s/rad, N·m·s/rad, N·m·s/rad]
float64[6] damping
# Saturation of the spring-damper force and torque.
float64 max_force   # [N]
float64 max_torque  # [N·m]
