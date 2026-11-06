# How compliant the arm is during a ComplianceWindow:
# a tool-frame spring-damper around the planned motion.
# Tool-frame order: x, y, z, rx, ry, rz.

# [N/m, N/m, N/m, N·m/rad, N·m/rad, N·m/rad]
float64[6] stiffness
# [N·s/m, N·s/m, N·s/m, N·m·s/rad, N·m·s/rad, N·m·s/rad]
float64[6] damping
# Saturation of the spring-damper's linear force.
float64 max_force  # [N]

# Hand back to servoj on the first cycle the tool is within both limits of
# the plan; servoj then tracks the plan until compliance ends or the profile
# changes. Both 0 (the default) never hands back early.
float64 handback_max_deflection  # Tool-frame linear position error [m].
float64 handback_max_speed       # Tool-frame linear relative speed [m/s].
