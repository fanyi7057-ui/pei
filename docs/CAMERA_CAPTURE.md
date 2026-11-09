# Stationary camera capture (menu 7)
Uses existing stand_only publisher and wheel bridge; no race controller starts.
Requested posture: height 0.15m, pitch +25deg. Publishes zero wheel speed continuously.
Complete existing calibration/enable and retain remote emergency stop.
After startup, automatic capture at 5s; menu input 1 captures again, 0 or Ctrl+C exits.
Output: ~/Two_patrol/log/camera_capture/YYYYMMDD_HHMMSS_xxxxxx/
raw.png is original full-size RGB; other images show process-resolution black/events before and after blue guard, white road, independent color proposal, labels and comparison.
The independent proposal and candidates.csv are diagnostics only; they do not replace live detection.
RGB camera is owned only by this process. Do not run other preview/race programs concurrently.
No C++ rebuild required when current bridge environment is already compiled.
Validation: Python syntax, bash -n, 503 selected offline tests passed. Physical posture/camera/serial integration requires Orange Pi verification.
