# Partial regression fix on 41188da
- Withdraw added black near-gap override and strict observed-route-only reacquisition.
- All bounded recovery paths, including explicit ring left search, target <=0.30m/s.
- Hold recovery speed/turn target limits for 0.35s after reacquisition; weak rescue cannot accelerate to cruise.
- FOLLOW requires actual observed dash evidence during ENTRY; no five/four classification added.
- Validate gaps in source-resolution events/white mask, require chain reaching near ROI.
- Trigger snapshots and raw/sided candidate counts explain rejected or accidental cues.
- Preserve one-frame four-block latch, 0.12m advance and <=0.3s forced left probe.
- Retired five/four topology tests replaced; 533 selected regression tests pass.
- Not physically validated. Risks: glare may erase gap proof; 0.35s settling may affect cadence; follow proof may delay ring acquisition.
