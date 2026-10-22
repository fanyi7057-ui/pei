# Ring V5: real exit -> 2s straight -> direct left entry

- Marker1 no longer arms from a raw left branch alone. It requires center dashed corridor and a raw left branch in the same frame.
- After marker1, force straight for 2.0 s at the configured low speed.
- When the 2.0 s hold ends, enter ROUNDABOUT_FAST_ENTRY directly. Do not wait for left-branch clear, center-corridor re-arm, or a second visual marker.
- Keep the proven FAST_ENTRY left commit (0.30 m/s, +0.20 bias, +0.45 rad/s minimum left turn).
- Keep normal/roundabout smoothing, gyro exit logic, 10 s initial ring arm delay, and 25 s post-exit ring ignore window.
