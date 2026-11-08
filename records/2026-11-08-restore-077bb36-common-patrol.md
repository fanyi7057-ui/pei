# Common patrol restored to 077bb36

Restore PatrolVision baseline processing and FixedSpeedController from
077bb36. Black segmentation thresholds, road fallback, PathSelector and
ordinary error/control/loss semantics are baseline. Diagnostic-only counters
in icn.py remain. Preview is fixed to black mask. V29 search compatibility
fields remain readable but are unused by restored control; maintain_cruise
defaults to zero in code and shipped configuration.

Current RoundaboutRouter, dash detection, route ownership, entry/exit task
overrides, task receipts, authority, zebra and stance are retained. Shared
vision and loss behaviour now follow baseline also within task execution;
this intentionally changes shared behaviour, not ring-specific rules.
Existing task candidate speed/turn overrides still apply, not another
tracking algorithm. No new mask or colour threshold is introduced.

Removed later single-point/history restrictions and road-jump exception.
This restores the baseline possibility of false one-point rescue. Blue
misidentification is not claimed fixed. Startup dropouts may remain.

PatrolVision regression tests restored from 077bb36. Later tests referring
to removed trusted-history internals were replaced by baseline tests, rather
than retaining assertions for the intentionally removed behaviour.
