# Restore baseline steering after early-turn regression

Remove only the heading feedforward introduced in e062434. AST comparison
confirms FixedSpeedController._steer matches b3c879e exactly. Retain search
deadband correction, ordered entry and diagnostics. No speed or colour
threshold changes. This is not a full rollback to the stable branch.

Three input videos were replayed through vision and ordinary controller.
Invalid frames (stable/current): 607/624, 64/64, 65/63. Removing feedforward
does not change vision validity on fixed input. Current single-point/history
behaviour still differs from baseline, especially in the old robot video.
No conclusion of improved vehicle performance follows from these counts.
No IMU/odom, full task overrides or closed-loop dynamics were simulated.
Latest external-view video cannot serve as the onboard camera input.

Regression checks preserve baseline steering and retain the independent
search-direction and ordered-entry fixes. Start-up vision instability still
requires matching onboard frames; do not mask it by forced cruise.
