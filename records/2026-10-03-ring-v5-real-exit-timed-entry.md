# Ring V5: real exit, timed straight, direct left entry

- Keep first-ring arming delay at 10 s after real race motion start.
- Replace permissive raw-left marker1 with a real-exit shape gate:
  - center raw components >= 2 and span >= 0.06;
  - left raw components >= 2 and span >= 0.08;
  - confirm for 2 consecutive frames.
- After exit confirmation, command `v=0.25, w=0` for exactly 2.0 s.
- The hard-straight command bypasses angular slew so residual turn cannot remain.
- At timer expiry, enter `ROUNDABOUT_FAST_ENTRY` immediately; do not wait for branch clear, center corridor, or a new left branch.
- Preserve normal/ring-follow smoothing outside this intentional straight bridge.
- Preserve gyro-z exit state logic and visual-exit gyro arm gate.
- Preserve 25 s post-exit roundabout ignore window before the next ring can be considered.
