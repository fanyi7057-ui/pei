# 2026-10-03 · Ring V4 timing + smoothing + gyro gate

- First roundabout detection is disabled for the first 10.0 s after the first valid race frame.
- Marker #1 is treated as the pre-ring/exit cue: command 0.25 m/s straight with zero target yaw rate for 1.50 s; marker #2 cannot trigger during that window.
- Marker #2 no longer uses an odometry-distance gate; it still requires marker #1 clear, center-corridor evidence, and valid entrance morphology.
- After a real roundabout exit, roundabout detection remains disabled for 25.0 s before the second roundabout can be considered.
- Adaptive command smoothing uses slower acceleration, faster deceleration, 3.0 rad/s^2 normal angular slew and 2.4 rad/s^2 ring-follow angular slew.
- race_tuning defaults cap normal steering at 0.80 rad/s, reduce minimum curve speed to 0.26 m/s, and use stronger curve slowdown.
- gyro_z integration is retained. With gyro enabled, visual fork exit is blocked until gyro accumulated turn has armed at 270 deg; gyro still confirms near 360 deg.
- Full wheel_robot test suite: 147 passed. Python compileall: passed.
