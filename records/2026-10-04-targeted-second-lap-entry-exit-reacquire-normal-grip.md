# 2026-10-04 targeted control fixes

Only the three user-requested areas are changed from the previous package:

1. Marker-confirmed 2.00 s main-line pass keeps a 0.38 m/s loss floor and bypasses the normal-patrol slow re-acceleration cap during that window. FAST_ENTRY keeps its existing state machine, with a 0.45 s minimum +0.60 rad/s left commit so a lost handoff frame on lap 2 cannot look like "no forced left".
2. The existing 200/220-class exit-zone lost-line left search remains unchanged until it reacquires a trustworthy line. After that first reacquisition, forced-left is latched off for the rest of the ring FOLLOW; later dropouts use normal trend recovery, and visible left-branch components are filtered from the main-path candidate when available.
3. Normal patrol gains a visible-line centering assist based on the existing weighted N/M/F trend. It acts before full line loss and only in normal patrol phases, leaving ROUNDABOUT_FOLLOW unchanged.

No changes to adaptive_tuning.yaml, adaptive_runtime.py, icn.py, ring marker thresholds, gyro exit thresholds, post-exit main filtering, crosswalk, finish, or step logic.
