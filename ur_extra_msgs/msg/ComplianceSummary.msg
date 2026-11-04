# Outcome of one ComplianceWindow,
# published by the driver once compliance for that window has ended.

# Stamped when published.
Header header
time window_begin
time window_end
# The window's index into its schedule's profiles.
uint8 profile

# Whether the robot script reported compliant at least once.
bool was_compliant
# If not, why. NOT_COMPLIANT_NONE when `was_compliant`.
uint8 NOT_COMPLIANT_NONE=0
# The robot script refused the profile values.
uint8 NOT_COMPLIANT_INVALID=1
# The schedule was replaced or cleared before the window began.
uint8 NOT_COMPLIANT_CANCELLED=2
# The window passed without the robot script reporting compliant
# (e.g. the arm controller or the robot program was not running).
uint8 NOT_COMPLIANT_NOT_REPORTED=3
uint8 not_compliant_reason
# First and last compliant cycle (zero if never compliant).
time compliance_begin
time compliance_end

# How compliance ended; only meaningful when `was_compliant`.
# Values the robot script reports match its end-reason register.
uint8 END_WINDOW=0                     # The window ended (after the unload).
uint8 END_RELEASE=1                    # Lift release: back on the plan early.
uint8 END_CANCELLED=2                  # The schedule was replaced or cleared mid-window.
uint8 END_GUARD_DEFLECTION_Z=3
uint8 END_GUARD_DEFLECTION_XY=4
uint8 END_GUARD_DEFLECTION_ROTATION=5
uint8 END_GUARD_RELATIVE_SPEED=6
uint8 END_GUARD_JOINT_SPEED=7
uint8 END_GUARD_EXTRAPOLATION=8
# The robot script still reported compliant well after the window ended
# (e.g. a protective stop ended the script).
uint8 END_NOT_REPORTED=9
uint8 end_reason

# Peak TCP force magnitude from the window's begin until the summary [N].
float64 max_force
# Planned minus actual TCP position in the tool frame while compliant [m];
# positive z means the tool was held short along its axis.
float64 max_deflection_z
float64 max_deflection_xy
# At the last compliant cycle.
float64 exit_deflection_z
