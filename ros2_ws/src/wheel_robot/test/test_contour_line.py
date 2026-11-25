import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from patrol_contour_runtime import PatrolTuning, replace
from patrol_contour_line import (CornerObservation, CornerRingState,
                                 _corner_candidates, observe_corners)


def _tuning(**changes):
    return replace(PatrolTuning(), contour_corner_enabled=1, **changes)


def _lane_hsv(width=320, height=240, left=80, right=240):
    image = np.zeros((height, width, 3), np.uint8)
    image[:] = (40, 40, 40)                         # dark outside the lane
    image[:, left:right] = (220, 220, 220)          # white lane
    image[110:170, left + 20:right - 30] = (110, 110, 110)  # gray uncertain
    return cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


def test_connected_road_mask_and_corridor():
    hsv = _lane_hsv()
    obs = observe_corners(hsv, _tuning())
    assert obs.valid and obs.corridor and obs.reason == 'ok'
    assert obs.width_ratio == pytest.approx((240 - 80) / 320, abs=.05)
    assert obs.dual_fraction > .8 and obs.branch_ratio < .05
    left = obs.left[np.isfinite(obs.left)]
    right = obs.right[np.isfinite(obs.right)]
    assert len(left) and left.max() <= 100
    assert len(right) and right.min() >= 220


def test_open_area_is_not_a_corridor():
    # A whole white hall with a few blue edges must NOT be drivable as a lane:
    # this is exactly what made the car drive straight into the scenery.
    image = np.full((240, 320, 3), 220, np.uint8)
    image[:, :8] = (180, 80, 30)
    image[:, -8:] = (180, 80, 30)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    obs = observe_corners(hsv, _tuning())
    assert obs.valid and not obs.corridor
    assert obs.reason in ('open_area', 'too_wide')


def test_single_edge_has_no_corridor():
    image = np.zeros((240, 320, 3), np.uint8)
    image[:, 160:] = (220, 220, 220)               # road touches the right border
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    obs = observe_corners(hsv, _tuning())
    assert not obs.corridor


def test_blue_tape_boundaries_form_a_corridor():
    from patrol_contour_line import observe_blue_road
    image = np.full((240, 320, 3), 220, np.uint8)
    image[:, 80:96] = (180, 80, 30)     # left blue tape
    image[:, 225:241] = (180, 80, 30)   # right blue tape
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    obs = observe_blue_road(hsv, _tuning())
    assert obs.valid and obs.corridor
    assert obs.reason.startswith('blue_')
    assert obs.width_ratio == pytest.approx(129 / 320, abs=.06)
    assert abs(obs.near_error) < .15 and abs(obs.mid_error) < .15


def test_blue_single_edge_uses_width_prior():
    from patrol_contour_line import observe_blue_road
    image = np.full((240, 320, 3), 220, np.uint8)
    image[:, 225:241] = (180, 80, 30)   # only the right tape
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    assert not observe_blue_road(hsv, _tuning()).corridor
    obs = observe_blue_road(hsv, _tuning(), prior_width=130.)
    assert obs.corridor and obs.reason == 'blue_single_edge'
    assert abs(obs.near_error) < .20


def test_blue_pairing_follows_reference_centre():
    from patrol_contour_line import observe_blue_road
    image = np.full((240, 320, 3), 220, np.uint8)
    image[:, 20:30] = (180, 80, 30)     # outer left tape
    image[:, 80:96] = (180, 80, 30)     # inner left tape
    image[:, 225:241] = (180, 80, 30)   # right tape
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    obs = observe_blue_road(hsv, _tuning())
    assert obs.corridor and obs.width_ratio == pytest.approx(129 / 320, abs=.06)
    # A reference far to the left pairs the OUTER left tape with the inner one,
    # which is a different (narrower) corridor: the pairing really follows ref.
    obs2 = observe_blue_road(hsv, _tuning(), ref_points=np.array([[40., 200.], [40., 140.]]))
    assert obs2.corridor and obs2.width_ratio < .25

    width, height = 320, 240
    left = np.full(height, np.nan)
    for y in range(20, 200):
        left[y] = 40. if y < 70 else 40. + (y - 70) * 2.0
    right = np.full(height, np.nan)
    for y in range(20, 200):
        right[y] = 300. if y < 50 else 250.
    entry, exit_ = _corner_candidates(left, right, width, height, _tuning())
    assert entry is not None and entry[0] == 40 and 68 <= entry[1] <= 90
    assert exit_ is not None and 40 <= exit_[1] <= 70


def test_corner_state_entry_then_exit_pull():
    t = _tuning(contour_corner_entry_seconds=.8, contour_corner_exit_seconds=.9,
                contour_corner_entry_pull=.28, contour_corner_exit_pull=.22,
                contour_corner_cooldown_seconds=2.0,
                contour_corner_min_ring_seconds=1.5)
    s = CornerRingState()
    entry = CornerObservation(True, 'ok', corridor=True, entry_corner=(40, 60))
    s.update(entry, 1.0, t, has_black=True)
    assert s.mode == 'entry_pull' and s.pull(1.1, t) == pytest.approx(.28)
    assert s.pull(1.9, t) == 0. and s.in_ring
    exit_ = CornerObservation(True, 'ok', corridor=True, exit_corner=(280, 60))
    s.update(exit_, 2.5, t, has_black=True)      # black present: no exit
    assert s.pull(2.6, t) == pytest.approx(0.)
    s.update(exit_, 2.7, t, has_black=False)     # no black: exit pull left
    assert s.mode == 'exit_pull' and s.pull(2.8, t) == pytest.approx(.22)
    assert s.pull(3.7, t) == 0.


def test_corner_state_ignores_unreliable_corridor():
    t = _tuning()
    s = CornerRingState()
    obs = CornerObservation(True, 'open_area', corridor=False, entry_corner=(40, 60))
    s.update(obs, 1.0, t, has_black=False)
    assert s.mode == 'idle' and s.events == 0


def test_corner_state_requires_black_route_agreement():
    t = _tuning()
    s = CornerRingState()
    obs = CornerObservation(True, 'ok', corridor=True, entry_corner=(40, 60))
    s.update(obs, 1.0, t, has_black=False, agree=False)
    assert s.mode == 'idle' and s.events == 0


def test_corner_state_cooldown_prevents_repeat():
    t = _tuning(contour_corner_cooldown_seconds=2.0)
    s = CornerRingState()
    obs = CornerObservation(True, 'ok', corridor=True, entry_corner=(40, 60))
    s.update(obs, 1.0, t, has_black=True)
    s.update(obs, 1.1, t, has_black=True)
    assert s.events == 1


def test_corner_tuning_validation():
    for changes in ({"contour_corner_enabled": 2},
                    {"contour_takeover_enabled": 2},
                    {"contour_takeover_max_seconds": 0},
                    {"contour_takeover_max_gap_seconds": 2},
                    {"contour_corner_min_ring_seconds": 0},
                    {"contour_corner_gray_v_min": -1},
                    {"contour_corner_min_area_ratio": 0},
                    {"contour_corner_min_angle_deg": 10},
                    {"contour_corner_entry_pull": 0},
                    {"contour_corner_entry_seconds": 0},
                    {"contour_corner_cooldown_seconds": -1}):
        with pytest.raises(ValueError):
            replace(PatrolTuning(), **changes).validate()
