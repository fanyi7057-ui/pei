# Patrol vision evidence review

Baseline: 735c5c6. Preserve tuning, task ordering and motion authorization.

Only finally selected, observed, non-rescue/non-handoff original black paths
renew black evidence. Rescue restores extractor history and never renews it.
Black evidence retains at most 12 observations over 0.35 seconds. Small motion
and bounded return towards centre require fresh geometric support.

Ordinary road jump exceptions use full-image-width displacement, at most 0.35,
time-scaled motion agreement, four common observed rows and no known blue
pixels between paths. They are disabled outside SEEK and during ring tracking.

Vision summaries are limited to 2 Hz and include interval source switches and
invalid-path counts. Candidate rejection counters do not change thresholds.
blue_removed_pixels counts pixels, not objects or candidates. Final published
commands remain in node PATROL/CONTROL_CHANGE logs and screen overlay.

Optional anomaly capture: set PATROL_SAVE_ANOMALIES=1 before starting the menu.
At most one background write at a time, once per five seconds, twelve rotating
PNG/text slots in records/vision_anomalies. Panel: original/selected above,
black/blue masks below. Off by default. Captures invalid selected paths only;
a valid-looking false path still requires operator/video evidence.

Validation: core contour suite 499 passed. Whole baseline suite had 27 failures,
825 passed, 11 skipped. Compare final full-suite results in the handoff; old
menu/adaptive/step contracts and loopback permission failures are not evidence
of new visual regressions. Real-camera latency, reflections, S-bends and robot
completion remain unverified. Current thresholds need controlled vehicle tests.
