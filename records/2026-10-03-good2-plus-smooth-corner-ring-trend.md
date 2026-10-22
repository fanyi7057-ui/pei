# good2+ smooth corner / ring trend update

Base: `good2`.

Changes:
- Keep all V5 roundabout entry timing, 4.0 s hard-straight, gyro exit, and 25 s re-arm cooldown unchanged.
- Raise camera control loop from 15 Hz to 30 Hz to align with the 30 fps camera.
- Raise hard-turn speed cap from 0.105 m/s to 0.18 m/s so tight bends no longer feel like a stop.
- Raise adaptive acceleration-up from 0.34 to 0.42 m/s^2 while keeping decel at 0.90 m/s^2.
- Raise normal/ring angular slew to 3.5 / 3.0 rad/s^2 for denser, more responsive steering at 30 Hz.
- ROUNDABOUT_FOLLOW line loss now follows recent weighted trend bidirectionally first; fixed left is only a final fallback after the predictive window and only when no reliable direction remains.
