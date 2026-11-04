# Compliance for one trajectory, sent to the driver.
# A new schedule replaces the previous one; an empty one clears it.
# The driver rejects a schedule with invalid windows or profiles
# (and clears the previous one).

# Window times are relative to this stamp.
time stamp
ComplianceProfile[] profiles
# Ordered and non-overlapping.
ComplianceWindow[] windows
