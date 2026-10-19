# Outcome of one armed grasp compliance window (GraspComplianceWindow),
# published once the window ends.

# Stamped with the window end.
Header header
time window_begin
time window_end

# Values match the compliant robot script's status register.
uint8 GUARD_NONE=0
uint8 GUARD_DEFLECTION_Z=2
uint8 GUARD_DEFLECTION_XY=3
uint8 GUARD_DEFLECTION_ROTATION=4
uint8 GUARD_RELATIVE_SPEED=5
uint8 GUARD_JOINT_SPEED=6
uint8 GUARD_DURATION=7
uint8 GUARD_EXTRAPOLATION=8

# Whether the deviation ratio reached the trigger, so the driver sent
# compliant mode, and when.
bool is_triggered
time trigger_time
# Whether the robot script reported running compliant at least once.
bool was_compliant
# First guard the robot script reported tripping.
uint8 guard

float64 max_joint_position_deviation_ratio
# Peak TCP force magnitude, N.
float64 max_force
# Planned minus actual TCP position in the tool frame while compliant, m;
# positive z means the tool was held short along its axis.
float64 max_deflection_z
float64 max_deflection_xy
# At the last compliant cycle.
float64 exit_deflection_z
