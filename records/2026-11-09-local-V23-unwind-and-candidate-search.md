# Local V23 unwind and candidate search
Base: ff51398. Partial port only, not a whole-project restore.
- Port V23 CornerExitGuard: 3 observed centered frames, error <=0.12; preserve issued motion.
- Cancel stale opposite search turn on strong current geometry before starting new direction.
- Permit consistent measured near two-point routes to reacquire; rescue/handoff cannot confirm.
- Measure black near coverage rather than assume a far path reaches the near row.
- SEEK left-search permission requires a measured valid diverging entry branch and minimum blocks; max 0.6s after last cue. Reliable ordinary path/phase change releases it.
- No unconditional left search when blocks=0; no ring receipt is awarded by this helper.
- Ordinary 0.65m/s, max turn 2rad/s, entry 1 frame + 1s and 0.20m unchanged.
- Snapshots: ~/Two_patrol/log/vision_snapshots, asynchronous, max one per 5s, 24 rolling slots.
Validation: 501 tests passed; physical course validation still required.
Risks: false current geometry can cancel a real turn; candidate false positive can cause a short left search; image-copy/thread overhead requires on-car timing verification.
