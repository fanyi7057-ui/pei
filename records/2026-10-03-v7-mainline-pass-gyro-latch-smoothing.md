# V7: main-line pass window, gyro exit latch, smoother steering

- Base: user-tested `Two(4).zip` / good2 smooth V6.
- Roundabout pre-entry 4.0 s window no longer forces `v=0.25,w=0`.
  It keeps normal main-line following, ignores roundabout branch events, caps large left steering while the left branch is visible, and bridges short dashed-line gaps with recent trend/speed.
- Gyro zero/reference is still taken exactly when the 4.0 s window ends and `ROUNDABOUT_FAST_ENTRY` begins.
- `roundabout_gyro_require_line_valid=false`: the 360deg exit window is no longer blocked by a momentary line loss at the junction.
- Added a target-crossing latch: if the near-360 window is somehow missed, crossing target+5deg for 2 fresh frames forces gyro exit instead of allowing 500+deg circulation.
- Added per-lap completion latch so a completed roundabout cannot be re-armed again in the same lap.
- Steering smoothing now filters target angular velocity and uses a slower slew only when reversing steering direction; same-direction turn build-up remains fast.
- Normal hard-turn speed cap raised slightly from 0.18 to 0.20m/s, linear accel/decel retuned to 0.48/0.72m/s^2.
- Added `CTRLDBG` 1Hz diagnostics showing real processed vision FPS vs configured 30Hz.
