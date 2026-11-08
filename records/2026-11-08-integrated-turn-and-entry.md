# Integrated turn and ordered entry correction

Baseline f2092a1. Cruise 0.65 m/s, maximum yaw 2 rad/s and marker pass
distance 0.12 m remain unchanged. Do not infer a 0.30 m requirement without
matching the detection time to physical location.

Measured near/mid heading supplies a bounded steering feedforward at gain
0.30 times steering_gain. Excludes rescue, handoff, task-owned routes and
opposite-side S-bend geometry. Existing acceleration and turn bounds apply.

Normal and degraded search now share the same 0.05 rad/s direction deadband.
This fixes a tiny negative remembered turn changing a default-left search to
right at degradation. Genuine recent opposite observations remain usable.

First fork cannot directly jump SEEK to ENTRY. It starts the existing
APPROACH pass; later fresh entry evidence and existing time/distance gates
authorize ENTRY. Missing first junction may now cause missed entry rather
than a speculative immediate turn: verify this on the physical course.

Related regression suite: 569 passed, one checksum test deferred until refresh.
Offline videos do not include IMU, odometry, task motion overrides or vehicle
response. Neither turn performance nor physical gate identity is established
by unit tests. First-lap right offset remains a diagnostic question; no fixed
centre bias is added. Existing full-project failures are outside this change.
