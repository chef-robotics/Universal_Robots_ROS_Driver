# Span of a trajectory during which the arm is compliant.
# Timing only; the profile says how compliant.
# Times are relative to the owning stamp, like `time_from_start`.
duration begin
duration end
# Index into the owning message's compliance profiles.
uint8 profile
