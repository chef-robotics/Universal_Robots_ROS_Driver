# Compliance for one trajectory, sent to the driver.
# Applied from `stamp`, as a joint trajectory controller applies a new goal
# from its start time: held windows that end by then are kept, one running
# across it is cut there, later ones are dropped, then these windows are
# added. So an empty schedule clears from its stamp. Stamp 0 means on
# receipt, for the cut-off and the windows alike. The driver rejects an
# invalid schedule (and clears on receipt).

# Window times are relative to this stamp.
time stamp
ComplianceProfile[] profiles
# Ordered and non-overlapping.
ComplianceWindow[] windows
