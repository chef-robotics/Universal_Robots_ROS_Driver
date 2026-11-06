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
# The schedule was replaced or cleared before the window began.
uint8 NOT_COMPLIANT_CANCELLED=1
# The window passed without the robot script reporting compliant
# (e.g. the arm controller or the robot program was not running).
uint8 NOT_COMPLIANT_NOT_REPORTED=2
uint8 not_compliant_reason
# First and last compliant cycle (zero if never compliant).
time compliance_begin
time compliance_end

# How compliance ended; only meaningful when `was_compliant`.
uint8 END_WINDOW=0        # The window ended.
uint8 END_CANCELLED=1     # The schedule was replaced or cleared mid-window.
# The robot script stopped reporting compliant before the window ended.
uint8 END_EARLY=2
# The robot script still reported compliant well after the window ended
# (e.g. a protective stop ended the script).
uint8 END_NOT_REPORTED=3
# The tool came within the profile's hand-back limits, so servoj took over.
uint8 END_HANDED_BACK=4
uint8 end_reason

# Peak TCP force magnitude from the window's begin until the summary [N].
float64 max_force
# Planned minus actual TCP position in the tool frame while compliant [m];
# positive z means the tool was held short along its axis.
float64 max_deflection_z
float64 max_deflection_xy
# At the last compliant cycle.
float64 exit_deflection_z
