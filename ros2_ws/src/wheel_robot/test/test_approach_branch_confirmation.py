from dataclasses import replace

import numpy as np

from patrol_contour_runtime import PathEstimate, PatrolTuning
from patrol_dash_entry import ApproachBranchConfirmation, DashObservation


def _left_cue(mid=-.20, far=-.40):
    return DashObservation(valid=True, branch=PathEstimate(
        True, points=np.array([[20., 35.], [30., 60.]]), mid_error=mid, far_error=far))


def test_approach_confirms_current_left_branch_after_one_missed_frame():
    check = ApproachBranchConfirmation()
    tuning = PatrolTuning()
    assert check.observe(_left_cue(), 1., 100, tuning) == "left_branch_first_frame"
    assert check.observe(DashObservation(), 1.04, 100, tuning) != "left_branch_confirmed"
    assert check.observe(_left_cue(), 1.08, 100, tuning) == "left_branch_confirmed"


def test_approach_resets_branch_on_conflicting_fork_or_two_misses():
    tuning = PatrolTuning()
    for break_frames in (
        [replace(_left_cue(), right_fork=True)],
        [DashObservation(valid=True, exit_branch=PathEstimate(True))],
        [DashObservation(), DashObservation()],
    ):
        check = ApproachBranchConfirmation()
        check.observe(_left_cue(), 1., 100, tuning)
        for index, cue in enumerate(break_frames, 1):
            assert check.observe(cue, 1.+index*.04, 100, tuning) != "left_branch_confirmed"
        assert check.observe(_left_cue(), 1.12, 100, tuning) == "left_branch_first_frame"


def test_approach_rejects_spatial_jump_and_expired_frame():
    check = ApproachBranchConfirmation()
    tuning = PatrolTuning()
    check.observe(_left_cue(), 1., 100, tuning)
    assert check.observe(_left_cue(mid=-.65), 1.04, 100, tuning) == "left_branch_discontinuous"
    assert check.observe(_left_cue(mid=-.65), 1.21, 100, tuning) == "left_branch_first_frame"
