# Time window in which grasp compliance is armed. While armed, the driver
# streams plain servoj until UR's joint position deviation ratio reaches the
# trigger, then compliant servo mode (MODE_SERVOJ_COMPLIANT) until the window
# ends. A newer window replaces the current one.
time begin
time end
# If nonzero, switch to compliant mode at this time even if the deviation
# ratio has not reached the trigger (e.g. just before the gripper closes).
time switch_time
