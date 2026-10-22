# V8 ring handoff and exit-direction fix

- Base: V7 main-line pass + gyro latch.
- User pass window retained at 2.00 s.
- FAST_ENTRY -> FOLLOW now clears line-trend history.
- After entry handoff, short line loss keeps forward motion and a mild LEFT arc; stale/right history cannot reverse the chassis.
- Inner-line reacquisition is 3 frames instead of 5.
- Post-exit protection waits for stable main-line reacquisition instead of ending only by a fixed timer; line loss keeps a mild LEFT exit direction.
- Motion smoothing uses real processed-frame spacing up to 0.12 s because field CTRLDBG showed about 9-12 vision Hz despite a 30 Hz timer.
- Debug-video encoding is disabled by default; RAW recording and HDMI debug remain enabled.
