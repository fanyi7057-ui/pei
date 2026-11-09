# Local preprocessing fix, base 2cc454b
Blue guard now requires connected spatial regions with credible bright/chromatic blue seeds. Dark tape hue alone cannot establish a blue guard region. Existing guard dilation and weak blue extension retained.
Normal-line extraction uses corrected mask. Dash detector tries corrected events first; only if no valid chain, it tries legacy events with all existing shape/gap checks. mask_source identifies compatibility fallback.
Camera capture includes events_legacy and mask-source metadata.
No speed, steering, entry-distance, probe-duration or dash-chain threshold changes.
Dataset: user supplied 18 stationary captures. Old valid 8/18; corrected-only 12/18 (2 formerly valid regressions); compatible final 14/18, no formerly-valid regressions. Four failures remain column/gap rejections.
Tests: 536 related tests passed. On-car latency, all-dark blue regions with insufficient color evidence, false triggers and moving acquisition remain unvalidated. Static positive samples do not establish false-positive rate.
