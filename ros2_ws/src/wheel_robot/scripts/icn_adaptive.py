#!/usr/bin/env python3
"""Adaptive race entrypoint layered on top of the proven icn.py baseline.

Changes kept deliberately local:
1) line loss follows the weighted frame-to-frame line trend left OR right;
2) roundabout counts the first/second LEFT BRANCH while center dashes may persist;
3) entry uses frame-by-frame trend with a left-entry bias, not a fixed steering lock;
4) the photographed fork-shaped dashed exit returns directly to normal tracking + cooldown;
5) crosswalk adds a farther ROI and shorter post-ring arming/approach distance.

The original icn.py remains untouched as a rollback entrypoint.
"""

from __future__ import annotations

import copy
import hashlib
import math
import os
import time
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np

import icn as base
from adaptive_runtime import (
    AdaptiveLogicTuning,
    BranchMarkerGate,
    RoundaboutGyroExitGate,
    WeightedLineTrend,
    detect_forked_exit,
    fit_dashed_main_line,
    load_adaptive_tuning_file,
    roundabout_exit_heading_turn,
    post_exit_search_turn,
    post_exit_reacquire_turn,
)


DEFAULT_ADAPTIVE_TUNING = Path(
    "/home/orangepi/robot/ros2_ws/src/wheel_robot/config/adaptive_tuning.yaml"
)


# Normal patrol only: keep deceleration responsive, but rebuild speed more gently
# after a bend so 15Hz vision updates do not feel like repeated throttle steps.
NORMAL_PATROL_REACCEL_RATE = 0.22

# Latest real-car fixes are deliberately local.  Do not change the proven marker,
# ring-follow, gyro-exit, or post-exit state thresholds.
MARKER_PASS_LOSS_SPEED_FLOOR = 0.38
FAST_ENTRY_COMMIT_SECONDS = 0.45
FAST_ENTRY_COMMIT_MIN_LEFT = 0.60
# Forward-road preview never changes the race phase.  It only prepares speed and
# a small same-direction steering feed-forward while 5.26 remains authoritative
# for roundabout timing and committed manoeuvres.
ROAD_PREVIEW_ALPHA = 0.38
ROAD_PREVIEW_CONFIRM_FRAMES = 3
ROAD_PREVIEW_CURVE_THRESHOLD = 0.11
ROAD_PREVIEW_MAX_FEEDFORWARD = 0.12
ROAD_PREVIEW_CURVE_SPEED_CAP = 0.22
ROAD_PREVIEW_RING_CONFIRM_FRAMES = 2
ROAD_PREVIEW_RING_SPEED_CAP = 0.21
NORMAL_GRIP_START_ERROR = 0.16
NORMAL_GRIP_OUTWARD_TREND = 0.015
NORMAL_GRIP_GAIN = 1.12
NORMAL_GRIP_MAX_TURN = 0.72
NORMAL_GRIP_SOFT_EDGE = 0.45
NORMAL_GRIP_HARD_EDGE = 0.72
NORMAL_GRIP_SOFT_SPEED_CAP = 0.24
NORMAL_GRIP_HARD_SPEED_CAP = 0.16


class AdaptiveCourseVision(base.CourseVision):
    """Reuse the current masks/path tracker, adding dual-dash and far-crosswalk cues."""

    def __init__(self, tuning: AdaptiveLogicTuning) -> None:
        super().__init__()
        self.adaptive_tuning = tuning
        self.dual_dash_visible = False
        self.center_dash_visible = False
        self.left_branch_visible = False
        self.left_branch_raw_visible = False
        self.dual_center_count = 0
        self.dual_left_count = 0
        self.dual_center_span = 0.0
        self.dual_left_span = 0.0
        self.roundabout_horizontal_gap = 0.0
        self.roundabout_chain_independent = False
        self.roundabout_same_scene = False
        self.exit_fork_visible = False
        self.exit_fork_main_count = 0
        self.exit_fork_branch_count = 0
        self.exit_fork_main_x_span = 0.0
        self.exit_fork_main_y_span = 0.0
        self.exit_fork_angle_deg = 0.0
        self.exit_fork_branch_span = 0.0
        self.exit_fork_junction_margin = 1.0
        self.roundabout_single_block_visible = False
        self.roundabout_single_block_error = 0.0
        self.roundabout_single_block_y_ratio = 0.0
        self.roundabout_marker_count = 0
        # Post-exit only: virtual main road made from separated black blocks.
        # It is never used for roundabout entry or in-ring control.
        self.post_exit_main_valid = False
        self.post_exit_main_error = 0.0
        self.post_exit_main_near_error = 0.0
        self.post_exit_main_mid_error = 0.0
        self.post_exit_main_far_error = 0.0
        self.post_exit_main_confidence = 0.0
        self.post_exit_main_count = 0
        self.post_exit_main_span = 0.0
        self.post_exit_left_branch_filtered = False
        self.crosswalk_far_visible = False
        self.crosswalk_far_score = 0.0

    def update_adaptive_tuning(self, tuning: AdaptiveLogicTuning) -> None:
        self.adaptive_tuning = tuning

    @staticmethod
    def _best_zone_chain(
        components: List[Tuple[float, float, int, int, int]],
        *,
        width: int,
        height: int,
        x_min_ratio: float,
        x_max_ratio: float,
        max_dx_ratio: float = 0.13,
    ) -> Tuple[
        int, float, Optional[float], Optional[float],
        Optional[float], Optional[float], Tuple[int, ...]
    ]:
        # Keep original component indices. They are used to guarantee that a
        # black block cannot simultaneously serve as both the center corridor
        # and the left branch.
        zone = [
            (idx, item) for idx, item in enumerate(components)
            if x_min_ratio * width <= item[0] < x_max_ratio * width
        ]
        if not zone:
            return 0, 0.0, None, None, None, None, ()

        order = sorted(range(len(zone)), key=lambda i: zone[i][1][1], reverse=True)
        best_len = [1] * len(zone)
        pred: List[Optional[int]] = [None] * len(zone)
        min_dy = 0.018 * height
        max_dy = 0.20 * height
        max_dx = max(0.01, float(max_dx_ratio)) * width
        for pos_i, i in enumerate(order):
            _orig_i, item_i = zone[i]
            xi, yi, *_ = item_i
            for j in order[:pos_i]:
                _orig_j, item_j = zone[j]
                xj, yj, *_ = item_j
                dy = yj - yi
                if not min_dy <= dy <= max_dy:
                    continue
                if abs(xi - xj) > max_dx:
                    continue
                proposal = best_len[j] + 1
                if proposal > best_len[i]:
                    best_len[i] = proposal
                    pred[i] = j

        end_i = int(np.argmax(best_len))
        local_indices: List[int] = []
        cursor: Optional[int] = end_i
        while cursor is not None:
            local_indices.append(cursor)
            cursor = pred[cursor]

        if not local_indices:
            return 0, 0.0, None, None, None, None, ()
        ys = [float(zone[i][1][1]) for i in local_indices]
        xs = [float(zone[i][1][0]) for i in local_indices]
        original_indices = tuple(int(zone[i][0]) for i in local_indices)
        span = (max(ys) - min(ys)) / max(1.0, float(height))
        return (
            len(local_indices), float(span), min(ys), max(ys),
            min(xs), max(xs), original_indices,
        )

    def _detect_roundabout_fast_entry(
        self, dark: np.ndarray, hsv: np.ndarray
    ) -> Tuple[bool, int, int, List[Tuple[float, float, int, int, int]]]:
        """Detect simultaneous center + left dashed chains.

        Compatibility note: the base process still calls this old hook name.
        The adaptive node does not use the old raw 3+2 component rule.
        """
        h, w = dark.shape
        tuning = self.adaptive_tuning
        components = self._small_neutral_components(dark, hsv, 0.18, 0.82)

        # V10 single-block ring rescue. A dashed ring line can momentarily leave
        # only one isolated square in view. That square may not intersect a
        # horizontal scan band, so base line extraction reports line_valid=false.
        # Preserve its center as a temporary local target for ROUNDABOUT_FOLLOW.
        self.roundabout_single_block_visible = False
        self.roundabout_single_block_error = 0.0
        self.roundabout_single_block_y_ratio = 0.0
        if len(components) == 1:
            cx, cy, _cw, _ch, _area = components[0]
            y_ratio = float(cy) / max(1.0, float(h))
            if 0.20 <= y_ratio <= 0.86:
                self.roundabout_single_block_visible = True
                self.roundabout_single_block_error = base.clamp(
                    (float(cx) - 0.5 * float(w)) / max(1.0, 0.5 * float(w)),
                    -1.0,
                    1.0,
                )
                self.roundabout_single_block_y_ratio = y_ratio

        (
            center_len, center_span, center_y0, center_y1,
            center_x0, center_x1, center_ids,
        ) = self._best_zone_chain(
            components,
            width=w,
            height=h,
            x_min_ratio=tuning.roundabout_dual_center_x_min,
            x_max_ratio=tuning.roundabout_dual_center_x_max,
            max_dx_ratio=0.13,
        )
        (
            left_len, left_span, left_y0, left_y1,
            left_x0, left_x1, left_ids,
        ) = self._best_zone_chain(
            components,
            width=w,
            height=h,
            x_min_ratio=tuning.roundabout_dual_left_x_min,
            x_max_ratio=tuning.roundabout_dual_left_x_max,
            max_dx_ratio=0.28,
        )

        center_visible = (
            center_len >= tuning.roundabout_dual_center_min_components
            and center_span >= tuning.roundabout_dual_center_min_span_ratio
        )
        left_visible = (
            left_len >= tuning.roundabout_dual_left_min_components
            and left_span >= tuning.roundabout_dual_left_min_span_ratio
        )

        same_scene = False
        horizontal_gap_ratio = -1.0
        chain_independent = not bool(set(center_ids).intersection(left_ids))
        if (
            center_y0 is not None
            and left_y0 is not None
            and center_x0 is not None
            and left_x1 is not None
        ):
            overlap = min(float(center_y1), float(left_y1)) - max(
                float(center_y0), float(left_y0)
            )
            vertical_gap = max(0.0, -overlap)
            horizontal_gap_ratio = (float(center_x0) - float(left_x1)) / max(1.0, float(w))
            same_scene = (
                chain_independent
                and vertical_gap <= tuning.roundabout_dual_max_vertical_gap_ratio * h
                and horizontal_gap_ratio >= tuning.roundabout_dual_min_horizontal_gap_ratio
            )

        self.center_dash_visible = bool(center_visible)
        # Raw left branch is useful for marker #1: on the real course the left
        # branch entered the camera about one second before the center dashed
        # corridor became visible.  Marker #2 still uses the strict geometry.
        self.left_branch_raw_visible = bool(left_visible)
        self.roundabout_horizontal_gap = float(horizontal_gap_ratio)
        self.roundabout_chain_independent = bool(chain_independent)
        self.roundabout_same_scene = bool(same_scene)
        # The left branch is the event edge.  It is only associated with the
        # ring marker when it occupies the same vertical scene as the center
        # dashed corridor.  The center chain itself may remain visible between
        # marker #1 and marker #2.
        self.left_branch_visible = bool(left_visible and same_scene)
        self.dual_dash_visible = bool(
            self.center_dash_visible and self.left_branch_visible
        )
        self.dual_center_count = int(center_len)
        self.dual_left_count = int(left_len)
        self.dual_center_span = float(center_span)
        self.dual_left_span = float(left_span)

        # Post-exit main-road candidate.  Build a broad main chain, but when a
        # strict independent LEFT branch exists remove its components first.
        # This is only exposed as metadata here; control consumes it solely in
        # SEEK_CROSSWALK after ROUNDABOUT_EXIT.
        filtered_components = list(components)
        filtered_left = bool(left_visible and same_scene and chain_independent)
        if filtered_left:
            left_id_set = {int(i) for i in left_ids}
            filtered_components = [
                comp for idx, comp in enumerate(components) if idx not in left_id_set
            ]
        (
            main_len, main_span, _my0, _my1, _mx0, _mx1, main_ids
        ) = self._best_zone_chain(
            filtered_components,
            width=w,
            height=h,
            x_min_ratio=0.12,
            x_max_ratio=0.90,
            max_dx_ratio=0.20,
        )
        main_points = [
            (float(filtered_components[idx][0]), float(filtered_components[idx][1]))
            for idx in main_ids
            if 0 <= int(idx) < len(filtered_components)
        ]
        main_estimate = fit_dashed_main_line(
            main_points, w, h, min_components=2, min_span_ratio=0.055
        )
        self.post_exit_main_valid = bool(
            main_len >= 2
            and main_span >= 0.055
            and main_estimate.valid
            and main_estimate.confidence >= 0.30
        )
        self.post_exit_main_error = float(main_estimate.error)
        self.post_exit_main_near_error = float(main_estimate.near_error)
        self.post_exit_main_mid_error = float(main_estimate.mid_error)
        self.post_exit_main_far_error = float(main_estimate.far_error)
        self.post_exit_main_confidence = float(main_estimate.confidence)
        self.post_exit_main_count = int(main_estimate.count)
        self.post_exit_main_span = float(main_estimate.span_ratio)
        self.post_exit_left_branch_filtered = filtered_left

        forked = detect_forked_exit(components, w, h, tuning)
        self.exit_fork_visible = bool(forked.visible)
        self.exit_fork_main_count = int(forked.main_count)
        self.exit_fork_branch_count = int(forked.branch_count)
        self.exit_fork_main_x_span = float(forked.main_x_span_ratio)
        self.exit_fork_main_y_span = float(forked.main_y_span_ratio)
        self.exit_fork_angle_deg = float(forked.angle_deg)
        self.exit_fork_branch_span = float(forked.branch_y_span_ratio)
        self.exit_fork_junction_margin = float(forked.junction_margin_ratio)

        left_raw = sum(
            1 for cx, _cy, *_rest in components
            if tuning.roundabout_dual_left_x_min * w
            <= cx
            < tuning.roundabout_dual_left_x_max * w
        )
        return self.dual_dash_visible, len(components), left_raw, components

    def _detect_crosswalk(self, dark: np.ndarray) -> Tuple[bool, float]:
        """Keep the proven near detector and add a farther perspective-tolerant ROI."""
        near_visible, near_score = super()._detect_crosswalk(dark)
        tuning = self.adaptive_tuning
        h, w = dark.shape
        x0 = int(0.07 * w)
        x1 = int(0.93 * w)
        y0 = int(tuning.crosswalk_far_y_top_ratio * h)
        y1 = int(tuning.crosswalk_far_y_bottom_ratio * h)
        max_runs = 0
        good_rows = 0
        for y in range(y0, y1, 3):
            strip_y0 = max(0, y - 2)
            strip_y1 = min(h, y + 3)
            occupancy = np.mean(dark[strip_y0:strip_y1, x0:x1] > 0, axis=0)
            binary = (occupancy >= tuning.crosswalk_row_occupancy).astype(np.uint8)
            runs = []
            for start, end in self._runs(binary):
                run_width = end - start
                if (
                    tuning.crosswalk_far_min_run_width_px
                    <= run_width
                    <= int(tuning.crosswalk_far_max_run_width_ratio * w)
                ):
                    runs.append((start, end))
            if len(runs) >= tuning.crosswalk_far_min_runs:
                centers = [0.5 * (a + b) for a, b in runs]
                span_ratio = (
                    (max(centers) - min(centers))
                    / max(1.0, float(x1 - x0))
                )
                if span_ratio >= tuning.crosswalk_far_min_span_ratio:
                    good_rows += 1
                    max_runs = max(max_runs, len(runs))

        far_score = base.clamp(
            0.55 * (
                max_runs / max(1.0, tuning.crosswalk_far_min_runs + 3.0)
            )
            + 0.45 * (
                good_rows / max(1.0, tuning.crosswalk_far_min_good_rows + 3.0)
            ),
            0.0,
            1.0,
        )
        far_visible = (
            max_runs >= tuning.crosswalk_far_min_runs
            and good_rows >= tuning.crosswalk_far_min_good_rows
        )
        self.crosswalk_far_visible = bool(far_visible)
        self.crosswalk_far_score = float(far_score)
        return bool(near_visible or far_visible), max(
            float(near_score), float(far_score)
        )

    def process(self, *args, **kwargs):
        result = super().process(*args, **kwargs)
        result.roundabout_dual_dash = self.dual_dash_visible
        result.roundabout_center_dash = self.center_dash_visible
        result.roundabout_left_branch = self.left_branch_visible
        result.roundabout_left_branch_raw = self.left_branch_raw_visible
        result.roundabout_dual_center_count = self.dual_center_count
        result.roundabout_dual_left_count = self.dual_left_count
        result.roundabout_dual_center_span = self.dual_center_span
        result.roundabout_dual_left_span = self.dual_left_span
        result.roundabout_horizontal_gap = self.roundabout_horizontal_gap
        result.roundabout_chain_independent = self.roundabout_chain_independent
        result.roundabout_same_scene = self.roundabout_same_scene
        result.roundabout_exit_fork = self.exit_fork_visible
        result.roundabout_exit_fork_main_count = self.exit_fork_main_count
        result.roundabout_exit_fork_branch_count = self.exit_fork_branch_count
        result.roundabout_exit_fork_main_x_span = self.exit_fork_main_x_span
        result.roundabout_exit_fork_main_y_span = self.exit_fork_main_y_span
        result.roundabout_exit_fork_angle_deg = self.exit_fork_angle_deg
        result.roundabout_exit_fork_branch_span = self.exit_fork_branch_span
        result.roundabout_exit_fork_junction_margin = self.exit_fork_junction_margin
        result.roundabout_single_block_visible = self.roundabout_single_block_visible
        result.roundabout_single_block_error = self.roundabout_single_block_error
        result.roundabout_single_block_y_ratio = self.roundabout_single_block_y_ratio
        result.post_exit_main_valid = self.post_exit_main_valid
        result.post_exit_main_error = self.post_exit_main_error
        result.post_exit_main_near_error = self.post_exit_main_near_error
        result.post_exit_main_mid_error = self.post_exit_main_mid_error
        result.post_exit_main_far_error = self.post_exit_main_far_error
        result.post_exit_main_confidence = self.post_exit_main_confidence
        result.post_exit_main_count = self.post_exit_main_count
        result.post_exit_main_span = self.post_exit_main_span
        result.post_exit_left_branch_filtered = self.post_exit_left_branch_filtered
        result.crosswalk_far_visible = self.crosswalk_far_visible
        result.crosswalk_far_score = self.crosswalk_far_score

        debug = result.debug_frame
        if self.post_exit_main_valid:
            h_dbg, w_dbg = debug.shape[:2]
            pts = []
            for ratio, err in (
                (0.84, self.post_exit_main_near_error),
                (0.60, self.post_exit_main_mid_error),
                (0.38, self.post_exit_main_far_error),
            ):
                x = int((0.5 + 0.5 * float(err)) * w_dbg)
                y = int(float(ratio) * h_dbg)
                pts.append((x, y))
                cv2.circle(debug, (x, y), 4, (255, 0, 255), -1)
            for a, b in zip(pts, pts[1:]):
                cv2.line(debug, a, b, (255, 0, 255), 2)
            cv2.putText(
                debug,
                (
                    f"POST MAIN n={self.post_exit_main_count} "
                    f"e={self.post_exit_main_error:+.2f} "
                    f"L-filter={int(self.post_exit_left_branch_filtered)}"
                ),
                (8, 136),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (255, 0, 255),
                1,
                cv2.LINE_AA,
            )
        cv2.putText(
            debug,
            (
                f"RING C={int(self.center_dash_visible)} "
                f"L={int(self.left_branch_visible)} "
                f"LR={int(self.left_branch_raw_visible)} "
                f"mark={min(2, int(self.roundabout_marker_count))}/2 "
                f"c={self.dual_center_count} l={self.dual_left_count} "
                f"s={int(self.roundabout_same_scene)} "
                f"gap={self.roundabout_horizontal_gap:.2f}"
            ),
            (8, 62),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (0, 255, 255) if self.left_branch_visible else (180, 180, 180),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            debug,
            (
                f"RING EXIT F={int(self.exit_fork_visible)} "
                f"main={self.exit_fork_main_count} "
                f"branch={self.exit_fork_branch_count} "
                f"ang={self.exit_fork_angle_deg:.0f} "
                f"drop={self.exit_fork_branch_span:.2f}"
            ),
            (8, 80),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (0, 165, 255) if self.exit_fork_visible else (180, 180, 180),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            debug,
            (
                f"RING ONE={int(self.roundabout_single_block_visible)} "
                f"e={self.roundabout_single_block_error:+.2f} "
                f"y={self.roundabout_single_block_y_ratio:.2f}"
            ),
            (8, 98),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (0, 255, 0) if self.roundabout_single_block_visible else (180, 180, 180),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            debug,
            (
                f"FAR ZEBRA={int(self.crosswalk_far_visible)} "
                f"score={self.crosswalk_far_score:.2f}"
            ),
            (8, 116),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (0, 255, 0) if self.crosswalk_far_visible else (180, 180, 180),
            1,
            cv2.LINE_AA,
        )
        return result


class AdaptiveWheelLegRaceNode(base.WheelLegRaceNode):
    """Use the existing ROS/hardware chain, overriding only the requested logic."""

    def __init__(self) -> None:
        tuning_path = Path(
            os.environ.get(
                "WHEEL_ROBOT_ADAPTIVE_TUNING_FILE",
                str(DEFAULT_ADAPTIVE_TUNING),
            )
        )
        initial, reason = load_adaptive_tuning_file(tuning_path)
        self._adaptive_tuning = initial
        self._adaptive_tuning_path = tuning_path
        self._adaptive_tuning_digest = ""

        super().__init__()

        self.vision = AdaptiveCourseVision(self._adaptive_tuning)
        self._line_trend = WeightedLineTrend(
            self._adaptive_tuning.line_history_frames
        )
        self._ring_branch_gate = BranchMarkerGate()
        self._roundabout_gyro_exit_gate = RoundaboutGyroExitGate()
        self._adaptive_roundabout_exit_frames = 0
        self._roundabout_exit_source = "none"
        self._roundabout_ignore_until = 0.0
        self._roundabout_marker1_straight_until = 0.0
        self._road_preview_near = 0.0
        self._road_preview_mid = 0.0
        self._road_preview_far = 0.0
        self._road_preview_direction = 0
        self._road_preview_confirm_frames = 0
        self._road_preview_ring_frames = 0
        self._road_preview_mode = "clear"
        self._road_preview_curve = 0.0
        self._roundabout_post_exit_guard_until = 0.0
        self._roundabout_post_exit_guard_started_at = 0.0
        self._roundabout_post_exit_stable_frames = 0
        self._roundabout_follow_entry_guard_until = 0.0
        self._roundabout_gyro_fresh = False
        self._roundabout_gyro_turn_deg = 0.0
        self._roundabout_gyro_peak_turn_deg = 0.0
        self._roundabout_gyro_error_deg = float("nan")
        self._roundabout_gyro_armed = False
        self._roundabout_gyro_confirm_frames = 0
        self._roundabout_gyro_rate_deg_s = 0.0
        self._roundabout_gyro_force_exit_frames = 0
        self._roundabout_gyro_exit_reason = "none"
        # V11 exit-heading lock.  The gyro reference is still created only at
        # the real FAST_ENTRY; from ~330deg onward it may also steer the chassis
        # back toward that saved entry heading instead of letting the left branch
        # physically pull the car into the ring again.
        self._roundabout_exit_heading_lock_armed = False
        self._roundabout_exit_left_suppress_until = 0.0
        self._roundabout_exit_heading_suppress_used = False
        self._roundabout_post_exit_heading_suppress_used = False
        self._roundabout_exit_heading_post_until = 0.0
        # V11.4: smooth post-exit no-line LEFT search and smooth handoff to vision.
        self._roundabout_post_exit_search_started_at = 0.0
        self._roundabout_post_exit_reacquire_started_at = 0.0
        self._roundabout_post_exit_reacquire_from_angular = 0.0
        self._roundabout_post_exit_last_search_angular = 0.0
        # Once the post-exit main road has really been seen, any later dropout
        # must search along that freshly observed main-road trend.  Do not fall
        # back to the initial fixed-left exit search after reacquisition.
        self._post_exit_main_reacquired = False
        self._roundabout_exit_left_search_active = False
        self._roundabout_exit_reacquire_frames = 0
        # 220deg+ exit-zone forced-left is only a one-shot reacquisition aid.
        # Once a real line is found again, later losses use normal trend recovery
        # and LEFT branch metadata is ignored instead of re-entering forced-left.
        self._roundabout_exit_zone_force_left_started = False
        self._roundabout_exit_zone_line_reacquired = False
        self._roundabout_completed_lap_index = -1
        self._first_crosswalk_confirmed_at = 0.0
        self._second_roundabout_enable_after = 0.0
        self._smoothed_angular_target = 0.0
        self._adaptive_perf_frames = 0
        self._adaptive_perf_window_start = time.monotonic()
        self._adaptive_perf_last_error = 0.0
        self._adaptive_perf_last_reason = "init"
        # This monitor is read-only: it never changes the race state machine,
        # line following, step locking, or jump execution.
        self._status_last_vision: Optional[base.VisionResult] = None
        self._status_wheel_velocity = {"left": None, "right": None}
        self._status_previous_wheel_velocity = {"left": None, "right": None}
        self._status_previous_time = time.monotonic()
        self.create_subscription(
            base.JointState,
            "/robot_joint_states",
            self._on_status_joint_state,
            10,
        )
        self.create_timer(2.5, self._log_robot_status)

        if reason:
            self.get_logger().warn(f"自适应参数使用默认值：{reason}")
        else:
            self.get_logger().info(
                f"已加载自适应参数: {self._adaptive_tuning_path}"
            )
        self._poll_adaptive_tuning_file(force=True)
        self.create_timer(0.5, self._poll_adaptive_tuning_file)
        self.get_logger().warn(
            "自适应比赛逻辑已启用：趋势左右找线；比赛开始先延迟环岛判断；"
            "真实出口确认后进入定时主线巡线窗口；窗口内忽略左分叉事件，计时结束直接左入环；"
            "普通巡线/环内巡线使用方向反转抑制平滑；gyro_z从真正入环角度开始积分并带越界锁存退出；"
            f"首次环岛延迟={self._adaptive_tuning.roundabout_initial_arm_delay_seconds:.1f}s；"
            f"出环后忽略环岛={self._adaptive_tuning.roundabout_ignore_after_exit_seconds:.1f}s；"
            f"第一次人行横道确认后第二环岛再延迟={self._adaptive_tuning.second_roundabout_arm_delay_after_first_crosswalk_seconds:.1f}s；"
            "出环后立即恢复逐帧巡线；220deg后丢线提速左找；330deg后按入环起始航向收敛并抑制出口左支；"
            "人行横道使用远距 ROI"
        )

    def _on_status_joint_state(self, msg: base.JointState) -> None:
        """Keep measured wheel angular velocities for operator telemetry."""
        positions = {name: index for index, name in enumerate(msg.name)}
        for side, joint_name in (
            ("left", "left_wheel_joint_link"),
            ("right", "right_wheel_joint_link"),
        ):
            index = positions.get(joint_name)
            if index is None or index >= len(msg.velocity):
                continue
            self._status_wheel_velocity[side] = float(msg.velocity[index])

    @staticmethod
    def _wheel_direction(speed: float) -> str:
        if speed > 0.03:
            return "正转"
        if speed < -0.03:
            return "反转"
        return "停止"

    def _log_robot_status(self) -> None:
        """Print a readable snapshot every 2.5 seconds for track-side checks."""
        now = time.monotonic()
        interval = max(1e-3, now - self._status_previous_time)
        wheel_parts = []
        for side, label in (("left", "左轮"), ("right", "右轮")):
            speed = self._status_wheel_velocity[side]
            previous = self._status_previous_wheel_velocity[side]
            if speed is None:
                wheel_parts.append(f"{label}=无反馈")
                continue
            acceleration = 0.0 if previous is None else (speed - previous) / interval
            wheel_parts.append(
                f"{label}={speed:+.2f}rad/s {acceleration:+.2f}rad/s2 "
                f"{self._wheel_direction(speed)}"
            )
            self._status_previous_wheel_velocity[side] = speed
        self._status_previous_time = now

        command_linear, command_angular = getattr(self, "_last_command", (0.0, 0.0))
        steering = "左转" if command_angular > 0.04 else (
            "右转" if command_angular < -0.04 else "直行"
        )
        result = self._status_last_vision
        if result is None:
            camera_healthy = bool(getattr(self, "_camera_healthy", False))
            frame_age = max(
                0.0,
                now - float(getattr(self, "_latest_frame_time", 0.0)),
            )
            line_text = (
                f"黑线=等待首帧 相机={'有画面' if camera_healthy else '无画面'} "
                f"帧龄={frame_age:.1f}s"
            )
        else:
            line_text = (
                f"黑线={'识别' if result.line_valid else '未识别'} "
                f"置信={result.confidence:.2f} 带={result.valid_bands} "
                f"误差N/M/F={result.near_error:+.2f}/"
                f"{result.mid_error:+.2f}/{result.far_error:+.2f} "
                f"原因={result.reason}"
            )

        detection = self._latest_step_detection
        if detection is None:
            step_text = "台阶=未收到深度检测"
        else:
            fresh = self._step_detection_fresh(now)
            jump_heading_source = str(
                getattr(self, "_step_jump_yaw_source", "none")
            )
            jump_heading_error_deg = math.degrees(
                float(getattr(self, "_step_yaw_error", 0.0))
            )
            step_text = (
                f"版本={base.STEP_EXPERIMENT_VERSION} "
                f"台阶={'识别' if detection.detected and fresh else '未识别'} "
                f"新鲜={int(fresh)} 距离={float(detection.distance):.2f}m "
                f"高度={float(detection.height):.3f}m "
                f"置信={float(detection.confidence):.2f} "
                f"确认={int(detection.confirm_frames)}/{int(detection.required_confirm_frames)} "
                f"辅助={getattr(self, '_step_aux_confirm_reason', 'waiting_depth')} "
                f"状态={base.step_text(self._step_state)} "
                f"航向={jump_heading_source}/{jump_heading_error_deg:+.1f}deg"
            )

        self.get_logger().info(
            "状态[2.5s] "
            f"阶段={base.phase_text(self.phase)} 圈={min(self.lap_count + 1, base.TOTAL_LAPS)}/{base.TOTAL_LAPS} "
            f"指令v={command_linear:+.2f}m/s w={command_angular:+.2f}rad/s {steering}; "
            + "；".join(wheel_parts)
        )
        self.get_logger().info(f"状态[2.5s] {line_text}")
        self.get_logger().info(f"状态[2.5s] {step_text}")

    def _log_control_perf(self) -> None:
        now = time.monotonic()
        dt = max(1e-6, now - self._adaptive_perf_window_start)
        fps = self._adaptive_perf_frames / dt
        self._adaptive_perf_frames = 0
        self._adaptive_perf_window_start = now
        cmd_linear, cmd_angular = getattr(self, "_last_command", (0.0, 0.0))
        self.get_logger().info(
            "CTRLDBG "
            f"vision={fps:.1f}/{base.CONTROL_RATE:.1f}Hz "
            f"e={self._adaptive_perf_last_error:+.3f} "
            f"reason={self._adaptive_perf_last_reason} "
            f"cmd=({cmd_linear:.2f},{cmd_angular:+.2f})"
        )

    def _log_roundabout_debug(self) -> None:
        phase = getattr(self, "phase", None)
        vision = getattr(self, "vision", None)
        if not isinstance(vision, AdaptiveCourseVision):
            return
        tuning = self._adaptive_tuning

        if phase == "SEEK_ROUNDABOUT":
            now = time.monotonic()
            race_age = (
                max(0.0, now - self._race_motion_start_time)
                if self._race_motion_start_time > 0.0
                else 0.0
            )
            hold_left = max(0.0, self._roundabout_marker1_straight_until - now)
            second_ring_wait = 0.0
            if int(getattr(self, 'lap_count', 0)) >= 1 and self._second_roundabout_enable_after > 0.0:
                second_ring_wait = max(0.0, self._second_roundabout_enable_after - now)
            self.get_logger().info(
                "RINGDBG "
                f"arm={int(race_age >= tuning.roundabout_initial_arm_delay_seconds)} "
                f"age={race_age:.1f}/{tuning.roundabout_initial_arm_delay_seconds:.1f} "
                f"pass={hold_left:.2f}/{tuning.roundabout_marker1_straight_seconds:.2f} "
                f"cw2={second_ring_wait:.1f}/{tuning.second_roundabout_arm_delay_after_first_crosswalk_seconds:.1f} "
                f"C={int(vision.center_dash_visible)} "
                f"c={vision.dual_center_count} "
                f"cs={vision.dual_center_span:.3f}/{tuning.roundabout_dual_center_min_span_ratio:.3f} "
                f"L={int(vision.left_branch_visible)} "
                f"LR={int(vision.left_branch_raw_visible)} "
                f"l={vision.dual_left_count} "
                f"ls={vision.dual_left_span:.3f}/{tuning.roundabout_dual_left_min_span_ratio:.3f} "
                f"scene={int(vision.roundabout_same_scene)} "
                f"road={self._road_preview_mode} "
                f"curve={self._road_preview_curve:+.2f} "
                f"gap={vision.roundabout_horizontal_gap:.3f}/{tuning.roundabout_dual_min_horizontal_gap_ratio:.3f} "
                f"ind={int(vision.roundabout_chain_independent)} "
                f"EXITcand={int(vision.dual_center_count >= tuning.roundabout_exit_marker_center_min_components and vision.dual_center_span >= tuning.roundabout_exit_marker_center_min_span_ratio and vision.dual_left_count >= tuning.roundabout_exit_marker_left_min_components and vision.dual_left_span >= tuning.roundabout_exit_marker_left_min_span_ratio)} "
                f"excf={self._ring_branch_gate.candidate_frames}/{tuning.roundabout_exit_marker_confirm_frames} "
                f"mark={min(1, int(vision.roundabout_marker_count))}/1"
            )
            return

        if phase == "ROUNDABOUT_FOLLOW":
            age = max(0.0, time.monotonic() - self.phase_enter_time)
            self.get_logger().info(
                "EXITDBG "
                f"fork_arm={int(age >= tuning.roundabout_exit_arm_seconds)} "
                f"gyro_arm={int(self._roundabout_gyro_armed)} "
                f"F={int(vision.exit_fork_visible)} "
                f"main={vision.exit_fork_main_count}/{tuning.roundabout_exit_fork_min_main_components} "
                f"branch={vision.exit_fork_branch_count}/{tuning.roundabout_exit_fork_min_branch_components} "
                f"mainspan={vision.exit_fork_main_x_span:.3f}/{tuning.roundabout_exit_fork_min_main_x_span_ratio:.3f} "
                f"flat={vision.exit_fork_main_y_span:.3f}<={tuning.roundabout_exit_fork_max_main_y_span_ratio:.3f} "
                f"angle={vision.exit_fork_angle_deg:.1f}/{tuning.roundabout_exit_fork_min_angle_deg:.1f} "
                f"drop={vision.exit_fork_branch_span:.3f}/{tuning.roundabout_exit_fork_min_branch_y_span_ratio:.3f} "
                f"junction={vision.exit_fork_junction_margin:.3f}<={tuning.roundabout_exit_fork_junction_margin_ratio:.3f} "
                f"ONE={int(vision.roundabout_single_block_visible)} "
                f"onee={vision.roundabout_single_block_error:+.2f} "
                f"gyro={'OK' if self._roundabout_gyro_fresh else 'NA'} "
                f"turn={self._roundabout_gyro_turn_deg:.0f}/{tuning.roundabout_gyro_target_turn_deg:.0f}deg "
                f"peak={self._roundabout_gyro_peak_turn_deg:.0f}deg "
                f"hlock={int(self._roundabout_exit_heading_lock_armed)} "
                f"herr={self._roundabout_exit_heading_error_deg():+.1f}deg "
                f"lhold={max(0.0, self._roundabout_exit_left_suppress_until - time.monotonic()):.1f}s "
                f"err={self._roundabout_gyro_error_deg:+.1f}/{tuning.roundabout_gyro_return_tolerance_deg:.1f}deg "
                f"gcf={self._roundabout_gyro_confirm_frames}/{tuning.roundabout_gyro_return_confirm_frames} "
                f"gx={self._roundabout_gyro_force_exit_frames}/{tuning.roundabout_gyro_force_exit_confirm_frames} "
                f"gwhy={self._roundabout_gyro_exit_reason} "
                f"gz={self._roundabout_gyro_rate_deg_s:+.1f}deg/s "
                f"imu_to={tuning.roundabout_gyro_imu_timeout_seconds:.2f}s "
                f"age={age:.1f}/{tuning.roundabout_follow_timeout_seconds:.1f}s"
            )

    def _start_roundabout_gyro_reference(self, now: float) -> None:
        """Zero gyro integration at the real marker #2 entry; never steer from gyro."""
        self._roundabout_gyro_exit_gate.reset()
        self._roundabout_gyro_fresh = False
        self._roundabout_gyro_turn_deg = 0.0
        self._roundabout_gyro_peak_turn_deg = 0.0
        self._roundabout_gyro_error_deg = float("nan")
        self._roundabout_gyro_armed = False
        self._roundabout_gyro_confirm_frames = 0
        self._roundabout_gyro_rate_deg_s = 0.0
        self._roundabout_gyro_force_exit_frames = 0
        self._roundabout_gyro_exit_reason = "none"
        self._roundabout_exit_heading_lock_armed = False
        self._roundabout_exit_left_suppress_until = 0.0
        self._roundabout_exit_heading_suppress_used = False
        self._roundabout_post_exit_heading_suppress_used = False
        self._roundabout_exit_heading_post_until = 0.0
        if not self._adaptive_tuning.roundabout_gyro_exit_enabled:
            return
        last_imu = float(getattr(self, "_last_imu_time", 0.0))
        if (
            last_imu <= 0.0
            or now - last_imu
            > self._adaptive_tuning.roundabout_gyro_imu_timeout_seconds
        ):
            return
        if self._roundabout_gyro_exit_gate.start(now):
            self._roundabout_gyro_fresh = True
            self._roundabout_gyro_error_deg = (
                -self._adaptive_tuning.roundabout_gyro_target_turn_deg
            )
            self.get_logger().info(
                "环岛gyro积分已在真正入口清零：0deg；"
                "仅积分 /imu/data angular_velocity.z 判断是否绕回入口朝向，"
                "绝不参与巡线转向"
            )

    def _update_roundabout_gyro_exit(
        self, now: float, line_valid: bool, allow_confirm: bool
    ) -> bool:
        tuning = self._adaptive_tuning
        if not tuning.roundabout_gyro_exit_enabled:
            self._roundabout_gyro_fresh = False
            return False

        last_imu = float(getattr(self, "_last_imu_time", 0.0))
        yaw_rate = float(getattr(self, "_imu_yaw_rate", 0.0))
        if (
            last_imu <= 0.0
            or now - last_imu > tuning.roundabout_gyro_imu_timeout_seconds
            or not np.isfinite(yaw_rate)
        ):
            self._roundabout_gyro_fresh = False
            self._roundabout_gyro_confirm_frames = 0
            self._roundabout_gyro_force_exit_frames = 0
            return False

        self._roundabout_gyro_fresh = True
        if not self._roundabout_gyro_exit_gate.active:
            self._roundabout_gyro_exit_gate.start(now)
            self.get_logger().info(
                "环岛gyro积分补记：首次有效 angular_velocity.z 到达，积分角从0deg开始"
            )

        (
            triggered,
            turn_deg,
            error_deg,
            armed,
            confirm_frames,
            rate_deg_s,
        ) = self._roundabout_gyro_exit_gate.update(
            now,
            yaw_rate,
            tuning.roundabout_gyro_arm_turn_deg,
            tuning.roundabout_gyro_target_turn_deg,
            tuning.roundabout_gyro_return_tolerance_deg,
            tuning.roundabout_gyro_return_confirm_frames,
            tuning.roundabout_gyro_deadband_deg_s,
            tuning.roundabout_gyro_scale,
            tuning.roundabout_gyro_max_rate_deg_s,
            tuning.roundabout_gyro_max_integration_dt_seconds,
            bool(
                allow_confirm
                and (
                    line_valid
                    or not tuning.roundabout_gyro_require_line_valid
                )
            ),
        )
        self._roundabout_gyro_turn_deg = turn_deg
        self._roundabout_gyro_peak_turn_deg = max(
            float(self._roundabout_gyro_peak_turn_deg),
            float(turn_deg),
        )
        self._roundabout_gyro_error_deg = error_deg
        self._roundabout_gyro_armed = armed
        self._roundabout_gyro_confirm_frames = confirm_frames
        self._roundabout_gyro_rate_deg_s = rate_deg_s

        if (
            tuning.roundabout_exit_heading_lock_enabled
            and not self._roundabout_exit_heading_lock_armed
            and self._roundabout_gyro_peak_turn_deg
            >= tuning.roundabout_exit_heading_lock_arm_deg
        ):
            self._roundabout_exit_heading_lock_armed = True
            self.get_logger().warn(
                "环岛出口航向锁已武装："
                f"peak={self._roundabout_gyro_peak_turn_deg:.1f}deg >= "
                f"{tuning.roundabout_exit_heading_lock_arm_deg:.1f}deg；"
                f"目标回到入环起始朝向 {tuning.roundabout_exit_heading_target_deg:.1f}deg；"
                "之后视觉大左转只作可疑左支处理"
            )

        if triggered:
            self._roundabout_gyro_exit_reason = "target_window"

        force_threshold = (
            tuning.roundabout_gyro_target_turn_deg
            + tuning.roundabout_gyro_force_exit_margin_deg
        )
        if (
            tuning.roundabout_gyro_force_exit_after_target_enabled
            and allow_confirm
            and armed
            and turn_deg >= force_threshold
        ):
            self._roundabout_gyro_force_exit_frames += 1
        elif turn_deg < tuning.roundabout_gyro_target_turn_deg:
            self._roundabout_gyro_force_exit_frames = 0

        force_exit = (
            tuning.roundabout_gyro_force_exit_after_target_enabled
            and self._roundabout_gyro_force_exit_frames
            >= tuning.roundabout_gyro_force_exit_confirm_frames
        )
        if force_exit:
            self._roundabout_gyro_exit_reason = "crossed_target_latch"

        return bool(triggered or force_exit)

    def _roundabout_exit_heading_error_deg(self) -> float:
        return (
            float(self._adaptive_tuning.roundabout_exit_heading_target_deg)
            - float(self._roundabout_gyro_turn_deg)
        )

    def _roundabout_exit_heading_lock_available(self, now: float) -> bool:
        tuning = self._adaptive_tuning
        if not tuning.roundabout_exit_heading_lock_enabled:
            return False
        if not self._roundabout_exit_heading_lock_armed:
            return False
        if not self._roundabout_gyro_fresh:
            return False
        if self.phase in ("ROUNDABOUT_FOLLOW", "ROUNDABOUT_EXIT"):
            return True
        return bool(
            self.phase == "SEEK_CROSSWALK"
            and now < self._roundabout_exit_heading_post_until
        )

    def _roundabout_exit_heading_command(
        self,
        result: base.VisionResult,
        now: float,
        *,
        post_exit: bool = False,
    ) -> Optional[Tuple[float, float, str]]:
        """Heading-primary control near the roundabout exit.

        330deg is only the *arming* point.  The command does not freeze the
        current chassis heading; instead it continues steering toward the
        saved FAST_ENTRY heading (360deg accumulated turn).  Vision remains a
        small correction.  A suspicious large LEFT visual command receives one
        1.5s heading-only hold, then is re-evaluated; post-exit large-left commands
        are still capped for the short protection window.
        """
        if not self._roundabout_exit_heading_lock_available(now):
            return None

        tuning = self._adaptive_tuning
        error_deg = self._roundabout_exit_heading_error_deg()
        heading_angular = roundabout_exit_heading_turn(
            self._roundabout_gyro_turn_deg,
            tuning.roundabout_exit_heading_target_deg,
            tuning.roundabout_exit_heading_tolerance_deg,
            tuning.roundabout_exit_heading_kp,
            tuning.roundabout_exit_heading_max_left_turn,
            tuning.roundabout_exit_heading_max_right_turn,
        )

        if not result.line_valid:
            # Before formal exit keep the original 330~360deg heading convergence.
            # After formal exit, if the main road has not yet been reacquired,
            # DO NOT invent a fixed LEFT search.  The real log showed that a
            # +0.35 -> +0.55 search could physically curl the chassis back into
            # the roundabout even though the state machine had already exited.
            # Keep the proven 0.24m/s exit motion, but steer only toward the saved
            # FAST_ENTRY heading.  Any real/virtual main line takes control on
            # the very next frame; later dropouts use the fresh post-exit trend.
            angular = heading_angular
            linear_no_line = float(tuning.roundabout_exit_heading_lock_speed)
            if post_exit:
                linear_no_line = float(tuning.roundabout_post_exit_search_speed)
                self._roundabout_post_exit_search_started_at = 0.0
                self._roundabout_post_exit_reacquire_started_at = 0.0
                self._roundabout_post_exit_reacquire_from_angular = 0.0
                self._roundabout_post_exit_last_search_angular = angular
            self._last_good_angular = angular
            self._smoothed_angular_target = angular
            self._have_previous_error = False
            return (
                linear_no_line,
                angular,
                (
                    "adaptive_roundabout_post_exit_no_line_heading_hold"
                    if post_exit
                    else "adaptive_roundabout_exit_heading_no_line"
                ),
            )

        linear, visual_angular, _mode = super()._planned_command(result, now)
        suspicious_left = bool(
            visual_angular > tuning.roundabout_exit_left_branch_trigger_turn
        )
        if suspicious_left:
            # Never let a suspected ring-left branch poison the recovery history.
            self._line_trend.reset()

        suppress_used = (
            self._roundabout_post_exit_heading_suppress_used
            if post_exit
            else self._roundabout_exit_heading_suppress_used
        )
        if suspicious_left and not suppress_used:
            self._roundabout_exit_left_suppress_until = (
                now + tuning.roundabout_exit_left_branch_hold_seconds
            )
            if post_exit:
                self._roundabout_post_exit_heading_suppress_used = True
            else:
                self._roundabout_exit_heading_suppress_used = True
            self.get_logger().warn(
                "出口附近检测到可疑大左转："
                f"vision_w={visual_angular:+.2f} > "
                f"{tuning.roundabout_exit_left_branch_trigger_turn:+.2f}；"
                f"未来 {tuning.roundabout_exit_left_branch_hold_seconds:.2f}s "
                f"不用w=0盲直，而是按gyro从 {self._roundabout_gyro_turn_deg:.1f}deg "
                f"继续收敛到 {tuning.roundabout_exit_heading_target_deg:.1f}deg"
            )

        if now < self._roundabout_exit_left_suppress_until:
            angular = heading_angular
            mode = "adaptive_roundabout_exit_heading_left_branch_hold"
        else:
            # After a post-exit 1.5-second hold, a still-left line may be the
            # actual course, so re-evaluate it but cap the left authority during
            # the short heading-reference window instead of accepting +0.6rad/s.
            if post_exit and suspicious_left:
                visual_angular = min(
                    visual_angular,
                    tuning.roundabout_exit_left_branch_trigger_turn,
                )
            if abs(error_deg) > tuning.roundabout_exit_heading_tolerance_deg:
                angular = (
                    heading_angular
                    + tuning.roundabout_exit_heading_visual_weight * visual_angular
                )
                angular = base.clamp(
                    angular,
                    -tuning.line_recovery_max_turn_speed,
                    tuning.roundabout_exit_heading_max_left_turn,
                )
                mode = "adaptive_roundabout_exit_heading_blend"
            else:
                angular = visual_angular
                mode = "adaptive_roundabout_exit_heading_aligned_line_follow"

        if post_exit and self._roundabout_post_exit_search_started_at > 0.0:
            # First valid main-line frame after an exit search: do not snap from
            # a strong LEFT search command directly to a potentially opposite
            # visual correction. Blend over a short window, then fully return
            # control to the ordinary frame-by-frame planner.
            if self._roundabout_post_exit_reacquire_started_at <= 0.0:
                self._roundabout_post_exit_reacquire_started_at = now
                self._roundabout_post_exit_reacquire_from_angular = float(
                    self._roundabout_post_exit_last_search_angular
                )
            blend_elapsed = max(
                0.0,
                now - self._roundabout_post_exit_reacquire_started_at,
            )
            angular = post_exit_reacquire_turn(
                self._roundabout_post_exit_reacquire_from_angular,
                angular,
                blend_elapsed,
                tuning.roundabout_post_exit_reacquire_blend_seconds,
            )
            mode = mode + "/post_exit_reacquire_blend"
            if blend_elapsed >= tuning.roundabout_post_exit_reacquire_blend_seconds:
                self._roundabout_post_exit_search_started_at = 0.0
                self._roundabout_post_exit_reacquire_started_at = 0.0
                self._roundabout_post_exit_reacquire_from_angular = 0.0
                self._roundabout_post_exit_last_search_angular = 0.0

        # With a valid main line, keep the normal patrol speed decision.  The
        # V11 0.24m/s value is only for no-line exit search; heading lock must
        # not turn a real hard bend into an artificial speed-up.
        linear = min(float(base.FORWARD_SPEED), max(0.0, float(linear)))
        self._last_valid_line_time = now
        self._last_good_angular = angular
        self._smoothed_angular_target = angular
        return linear, angular, mode

    def _post_exit_mainline_command(
        self, result: base.VisionResult, now: float
    ) -> Optional[Tuple[float, float, str]]:
        """Use the post-exit main road without changing good3_1 elsewhere.

        Priority after the ring:
        1) If an independent LEFT branch is visible, prefer the filtered main
           chain so that branch components cannot pull steering.
        2) Otherwise a normal continuous line uses the untouched good3_1 planner.
        3) If normal line extraction fails but separated main blocks form a chain,
           follow that chain as a virtual line.
        4) If neither exists, return None and keep the existing fast-exit search.
        """
        if self.phase != "SEEK_CROSSWALK":
            return None

        virtual_valid = bool(getattr(result, "post_exit_main_valid", False))
        left_filtered = bool(
            getattr(result, "post_exit_left_branch_filtered", False)
        )

        # Continuous line stays exactly on the good3_1 path unless a confirmed
        # independent left branch is present and a filtered main chain is ready.
        if result.line_valid and not (left_filtered and virtual_valid):
            linear, angular, mode = super()._planned_command(result, now)
            self._last_valid_line_time = now
            self._last_good_angular = angular
            self._post_exit_main_reacquired = True
            return linear, angular, "adaptive_post_exit_normal_main/" + mode

        if not virtual_valid:
            return None

        virtual = copy.copy(result)
        virtual.line_valid = True
        virtual.error = float(getattr(result, "post_exit_main_error", 0.0))
        virtual.near_error = float(
            getattr(result, "post_exit_main_near_error", virtual.error)
        )
        virtual.mid_error = float(
            getattr(result, "post_exit_main_mid_error", virtual.error)
        )
        virtual.far_error = float(
            getattr(result, "post_exit_main_far_error", virtual.error)
        )
        virtual.confidence = max(
            float(getattr(result, "post_exit_main_confidence", 0.0)), 0.35
        )
        virtual.valid_bands = max(
            2, int(getattr(result, "post_exit_main_count", 2))
        )
        virtual.reason = "post_exit_virtual_main"

        # Feed only the filtered/main chain into future good3_1 trend recovery.
        self._line_trend.observe(
            now, virtual.near_error, virtual.mid_error, virtual.far_error,
            self._adaptive_tuning,
        )
        linear, angular, mode = super()._planned_command(virtual, now)
        self._last_valid_line_time = now
        self._last_good_angular = angular
        self._post_exit_main_reacquired = True
        suffix = "/left_branch_filtered" if left_filtered else ""
        return linear, angular, "adaptive_post_exit_virtual_main/" + mode + suffix

    def _post_exit_reacquired_loss_command(
        self, now: float
    ) -> Tuple[float, float, str]:
        """Recover only along the freshly reacquired post-exit main-road trend.

        This path is intentionally local to SEEK_CROSSWALK after a real/virtual
        main road has already controlled the chassis at least once.  It keeps
        good3_1's trend direction (left/right/center), but caps speed and turn so
        a later camera dropout cannot surge forward or snap into a large circle.
        """
        linear, angular, mode = self._trend_recovery_command(now)
        linear = base.clamp(float(linear), 0.10, 0.16)
        angular = base.clamp(float(angular), -0.30, 0.30)
        self._last_good_angular = angular
        self._have_previous_error = False
        return (
            linear,
            angular,
            "adaptive_post_exit_reacquired_trend_loss/" + mode,
        )


    def _poll_adaptive_tuning_file(self, force: bool = False) -> None:
        try:
            text = self._adaptive_tuning_path.read_text(encoding="utf-8")
        except OSError as error:
            if force:
                self.get_logger().warn(
                    f"自适应参数文件读取失败，保留当前值: {error}"
                )
            return
        digest = hashlib.sha256(text.encode("utf-8")).hexdigest()
        if not force and digest == self._adaptive_tuning_digest:
            return
        updated, reason = load_adaptive_tuning_file(
            self._adaptive_tuning_path, self._adaptive_tuning
        )
        self._adaptive_tuning_digest = digest
        if reason:
            self.get_logger().error(f"自适应参数未生效: {reason}")
            return
        self._adaptive_tuning = updated
        self._line_trend.reconfigure(updated.line_history_frames)
        if isinstance(self.vision, AdaptiveCourseVision):
            self.vision.update_adaptive_tuning(updated)
        if not force:
            self.get_logger().info(
                "adaptive_tuning.yaml 已热加载："
                f"丢线预测段={updated.line_recovery_timeout_seconds:.2f}s，"
                f"持续找线={'ON' if updated.line_recovery_persistent_enabled else 'OFF'} "
                f"{updated.line_recovery_persistent_speed:.2f}m/s，"
                f"首次环岛延迟={updated.roundabout_initial_arm_delay_seconds:.1f}s，"
                f"marker1主线巡线窗口={updated.roundabout_marker1_straight_seconds:.2f}s，"
                f"窗口最大转向={updated.roundabout_marker1_follow_max_turn_speed:.2f}rad/s，"
                f"marker最小间隔={updated.roundabout_marker_min_separation_seconds:.2f}s，"
                f"marker确认={updated.roundabout_marker_confirm_frames}帧，"
                f"中间走廊={updated.roundabout_marker_between_center_frames}帧，"
                f"平滑={'ON' if updated.motion_smoothing_enabled else 'OFF'} "
                f"a+={updated.motion_linear_accel_up:.2f} a-={updated.motion_linear_decel:.2f} "
                f"w普通={updated.motion_angular_slew_normal:.2f} w环内={updated.motion_angular_slew_roundabout:.2f} "
                f"w反向={updated.motion_angular_reverse_slew:.2f} alpha={updated.motion_angular_target_alpha:.2f}，"
                f"入环v={updated.roundabout_entry_forward_speed:.2f}m/s，"
                f"入环最小左转={updated.roundabout_entry_min_left_turn_speed:.2f}rad/s，"
                f"环内丢线左转={updated.roundabout_blind_recovery_left_turn:.2f}~"
                f"{updated.roundabout_blind_recovery_max_turn_speed:.2f}rad/s，"
                f"gyro目标={updated.roundabout_gyro_target_turn_deg:.0f}deg，"
                f"出口丢线v={updated.roundabout_exit_loss_speed:.2f}m/s，"
                f"航向锁={updated.roundabout_exit_heading_lock_arm_deg:.0f}->"
                f"{updated.roundabout_exit_heading_target_deg:.0f}deg，"
                f"左支保持={updated.roundabout_exit_left_branch_hold_seconds:.1f}s，"
                f"环岛冷却={updated.roundabout_ignore_after_exit_seconds:.1f}s，"
                f"首横道后二环岛延迟={updated.second_roundabout_arm_delay_after_first_crosswalk_seconds:.1f}s，"
                f"出环最小左转={updated.roundabout_post_exit_min_left_turn_speed:.2f}rad/s，"
                f"横道远距ROI={updated.crosswalk_far_y_top_ratio:.2f}~"
                f"{updated.crosswalk_far_y_bottom_ratio:.2f}"
            )

    def _set_phase(self, phase: str, now: float) -> None:
        old = getattr(self, "phase", None)
        super()._set_phase(phase, now)
        if phase == "SEEK_ROUNDABOUT" and old != "SEEK_ROUNDABOUT":
            self._ring_branch_gate.reset()
            self._reset_forward_road_preview()
            self._adaptive_roundabout_exit_frames = 0
            self._roundabout_exit_source = "none"
            self._roundabout_marker1_straight_until = 0.0
            self._roundabout_gyro_exit_gate.reset()
            self._roundabout_exit_heading_lock_armed = False
            self._roundabout_exit_left_suppress_until = 0.0
            self._roundabout_exit_heading_suppress_used = False
            self._roundabout_post_exit_heading_suppress_used = False
            self._roundabout_exit_heading_post_until = 0.0
            if isinstance(self.vision, AdaptiveCourseVision):
                self.vision.roundabout_marker_count = 0
        if phase in ("ROUNDABOUT", "ROUNDABOUT_FAST_ENTRY"):
            self._line_trend.reset()
            self._roundabout_post_exit_search_started_at = 0.0
            self._roundabout_post_exit_reacquire_started_at = 0.0
            self._roundabout_post_exit_reacquire_from_angular = 0.0
            self._roundabout_post_exit_last_search_angular = 0.0
            self._smoothed_angular_target = float(getattr(self, "_last_command", (0.0, 0.0))[1])
            self._start_roundabout_gyro_reference(now)
            if phase == "ROUNDABOUT":
                self._reset_roundabout_progress()
        if phase == "ROUNDABOUT_FOLLOW" and old != "ROUNDABOUT_FOLLOW":
            self._roundabout_exit_zone_force_left_started = False
            self._roundabout_exit_zone_line_reacquired = False
            # State boundaries must never inherit a stale left/right trend.
            # The V7 bug came from FAST_ENTRY -> FOLLOW keeping the pre-entry
            # history; one missing frame could therefore command a right turn
            # immediately after a successful left entry.
            self._line_trend.reset()
            if old in ("ROUNDABOUT", "ROUNDABOUT_FAST_ENTRY"):
                self._roundabout_follow_entry_guard_until = (
                    now + self._adaptive_tuning.roundabout_follow_entry_guard_seconds
                )
                self._last_good_angular = max(
                    self._adaptive_tuning.roundabout_follow_entry_guard_min_left_turn,
                    float(getattr(self, "_last_command", (0.0, 0.0))[1]),
                )
        if phase == "ROUNDABOUT_EXIT" and old != "ROUNDABOUT_EXIT":
            self._roundabout_exit_left_search_active = True
            self._roundabout_exit_reacquire_frames = 0
        if phase == "SEEK_CROSSWALK" and old == "ROUNDABOUT_EXIT":
            # V11: detector/state gates still prevent software re-entry.  Keep the
            # saved gyro heading available for a few seconds only to stop a large
            # LEFT visual branch from physically curling the chassis back into
            # the ring. Normal/RIGHT main-line corrections remain available.
            self._line_trend.reset()
            self._roundabout_completed_lap_index = int(self.lap_count)
            self._roundabout_post_exit_guard_started_at = 0.0
            self._roundabout_post_exit_stable_frames = 0
            self._roundabout_post_exit_guard_until = 0.0
            self._roundabout_post_exit_heading_suppress_used = False
            self._roundabout_exit_left_suppress_until = 0.0
            self._roundabout_post_exit_search_started_at = 0.0
            self._roundabout_post_exit_reacquire_started_at = 0.0
            self._roundabout_post_exit_reacquire_from_angular = 0.0
            self._roundabout_post_exit_last_search_angular = 0.0
            self._post_exit_main_reacquired = False
            self._roundabout_exit_heading_post_until = (
                now + self._adaptive_tuning.roundabout_exit_heading_post_exit_seconds
                if self._roundabout_exit_heading_lock_armed
                else 0.0
            )
            self._last_good_angular = max(
                0.0,
                float(getattr(self, "_last_command", (0.0, 0.0))[1]),
            )
        if phase != "ROUNDABOUT_FOLLOW":
            self._adaptive_roundabout_exit_frames = 0

    def _sync_course_state(
        self, result: base.VisionResult, now: float
    ) -> None:
        tuning = self._adaptive_tuning
        phase_age = now - self.phase_enter_time

        if self.phase == "SEEK_ROUNDABOUT":
            self._update_seek_roundabout_progress()

            # 用户指定的新赛序：每圈从起步到台阶完整结束期间完全不判断环岛。
            # 深度台阶可以提前预判，但只有台阶结束并再冷却4秒后才重新允许环岛marker。
            step_cooldown_until = float(
                getattr(self, "_step_roundabout_enable_after", 0.0)
            )
            if (
                not bool(getattr(self, "_step_done_this_lap", False))
                or now < step_cooldown_until
            ):
                self._ring_branch_gate.reset()
                self._roundabout_marker1_straight_until = 0.0
                if isinstance(self.vision, AdaptiveCourseVision):
                    self.vision.roundabout_marker_count = 0
                return

            # V5: race start delay remains, and the first ring completion still
            # starts the 25s second-ring cooldown. During either gate, only
            # normal line following runs.
            if (
                self._race_motion_start_time > 0.0
                and now - self._race_motion_start_time
                < tuning.roundabout_initial_arm_delay_seconds
            ):
                self._ring_branch_gate.reset()
                self._roundabout_marker1_straight_until = 0.0
                if isinstance(self.vision, AdaptiveCourseVision):
                    self.vision.roundabout_marker_count = 0
                return

            if phase_age < tuning.roundabout_ignore_after_lap_start_seconds:
                self._ring_branch_gate.reset()
                if isinstance(self.vision, AdaptiveCourseVision):
                    self.vision.roundabout_marker_count = 0
                return

            if now < self._roundabout_ignore_until:
                self._ring_branch_gate.reset()
                if isinstance(self.vision, AdaptiveCourseVision):
                    self.vision.roundabout_marker_count = 0
                return

            # V9: the second roundabout must not arm immediately after lap 2 starts.
            # Start this gate at the FIRST crosswalk confirmation, not at the yellow
            # finish line. During the remaining cooldown only normal line following
            # runs and every roundabout marker accumulator stays cleared.
            if (
                int(self.lap_count) >= 1
                and self._second_roundabout_enable_after > 0.0
                and now < self._second_roundabout_enable_after
            ):
                self._ring_branch_gate.reset()
                self._roundabout_marker1_straight_until = 0.0
                if isinstance(self.vision, AdaptiveCourseVision):
                    self.vision.roundabout_marker_count = 0
                return

            # Hard state latch: once this lap has completed a roundabout, no later
            # state transition may re-arm that same lap's left branch. A new lap
            # increments lap_count, so its legitimate roundabout remains available.
            if self._roundabout_completed_lap_index == int(self.lap_count):
                self._ring_branch_gate.reset()
                if isinstance(self.vision, AdaptiveCourseVision):
                    self.vision.roundabout_marker_count = 0
                return

            # Once the real exit is confirmed, do not look at branch events again for
            # entrance timing. Keep following the main line, then switch to
            # Enter on the first frame after the configured timer expires.  Use
            # the proven 5.28 physical entry arc instead of the aggressive
            # FAST_ENTRY visual search, which could turn before reaching the gate.
            if self._ring_branch_gate.count == 1:
                if now < self._roundabout_marker1_straight_until:
                    return
                self._roundabout_gate_source = "adaptive_exit_then_mainline_pass_left_entry"
                self._set_phase("ROUNDABOUT", now)
                self.get_logger().warn(
                    "真实环岛出口后的主线巡线窗口完成："
                    f"{tuning.roundabout_marker1_straight_seconds:.2f}s 已到；"
                    "窗口内已忽略左分叉事件；现在记录gyro起始角并进入"
                    f"低速固定入环弧线(v={tuning.roundabout_entry_forward_speed:.2f}m/s, "
                    f"w={tuning.roundabout_entry_min_left_turn_speed:.2f}rad/s)"
                )
                return

            occurrence = self._ring_branch_gate.observe(
                bool(getattr(result, "roundabout_center_dash", False)),
                bool(getattr(result, "roundabout_left_branch", False)),
                now,
                tuning.roundabout_marker_clear_frames,
                tuning.roundabout_marker_min_separation_seconds,
                progress=self._seek_distance,
                min_separation_distance=tuning.roundabout_marker_min_separation_distance_m,
                confirm_frames_required=tuning.roundabout_marker_confirm_frames,
                between_center_frames_required=tuning.roundabout_marker_between_center_frames,
                left_branch_raw_visible=bool(
                    getattr(result, "roundabout_left_branch_raw", False)
                ),
                marker1_center_follow_seconds=tuning.roundabout_marker1_center_follow_seconds,
                marker1_raw_confirm_frames=tuning.roundabout_marker1_raw_confirm_frames,
                left_branch_raw_count=int(
                    getattr(result, "roundabout_dual_left_count", 0)
                ),
                left_branch_raw_span=float(
                    getattr(result, "roundabout_dual_left_span", 0.0)
                ),
                marker2_raw_min_components=tuning.roundabout_marker2_raw_min_components,
                marker2_raw_min_span_ratio=tuning.roundabout_marker2_raw_min_span_ratio,
                center_raw_count=int(
                    getattr(result, "roundabout_dual_center_count", 0)
                ),
                center_raw_span=float(
                    getattr(result, "roundabout_dual_center_span", 0.0)
                ),
                exit_center_min_components=tuning.roundabout_exit_marker_center_min_components,
                exit_center_min_span_ratio=tuning.roundabout_exit_marker_center_min_span_ratio,
                exit_left_min_components=tuning.roundabout_exit_marker_left_min_components,
                exit_left_min_span_ratio=tuning.roundabout_exit_marker_left_min_span_ratio,
                exit_confirm_frames=tuning.roundabout_exit_marker_confirm_frames,
            )
            if isinstance(self.vision, AdaptiveCourseVision):
                self.vision.roundabout_marker_count = min(
                    1, self._ring_branch_gate.count
                )
            if occurrence == 1:
                self._roundabout_marker1_straight_until = (
                    now + tuning.roundabout_marker1_straight_seconds
                )
                self.get_logger().warn(
                    "真实环岛出口形态确认："
                    f"c={int(getattr(result, 'roundabout_dual_center_count', 0))}/"
                    f"span={float(getattr(result, 'roundabout_dual_center_span', 0.0)):.3f}，"
                    f"l={int(getattr(result, 'roundabout_dual_left_count', 0))}/"
                    f"span={float(getattr(result, 'roundabout_dual_left_span', 0.0)):.3f}；"
                    f"当前位置={self._seek_distance:.2f}m；"
                    f"进入 {tuning.roundabout_marker1_straight_seconds:.2f}s 主线巡线窗口；"
                    "窗口内不再累计/执行左分叉事件，速度和转向继续按主线逐帧控制，"
                    "计时结束才记录gyro起始角并强制左入环"
                )
            return

        if self.phase == "ROUNDABOUT":
            self._update_roundabout_gyro_exit(
                now, result.line_valid, allow_confirm=False
            )
            self._update_roundabout_progress()
            odom_ready = (
                self._odom_snapshot() is not None
                and self._rb_entry_pose is not None
            )
            distance_ready = (
                odom_ready
                and self._rb_entry_displacement
                >= base.ROUNDABOUT_ENTRY_TURN_DISTANCE
            )
            fallback_ready = phase_age >= base.ROUNDABOUT_ENTRY_FALLBACK_SECONDS
            if distance_ready or fallback_ready:
                reason = "distance" if distance_ready else "timeout"
                displacement = self._rb_entry_displacement
                self._set_phase("ROUNDABOUT_FOLLOW", now)
                self.get_logger().warn(
                    "低速固定入环完成："
                    f"reason={reason}, distance={displacement:.2f}/"
                    f"{base.ROUNDABOUT_ENTRY_TURN_DISTANCE:.2f}m；"
                    "开始按内环黑色虚线逐帧循迹"
                )
            return

        if self.phase == "ROUNDABOUT_FAST_ENTRY":
            # From the real second marker onward, integrate gyro_z only as an
            # exit-state cue. It never changes the steering command.
            self._update_roundabout_gyro_exit(
                now, result.line_valid, allow_confirm=False
            )
            if phase_age < tuning.roundabout_entry_min_search_seconds:
                return
            # Entrance marker itself is not the inner line.  Do not release the
            # entry controller until the left branch has cleared and a stable
            # line is seen for several fresh frames.
            if (
                result.line_valid
                and not bool(getattr(result, "roundabout_left_branch", False))
            ):
                self._roundabout_fast_entry_reacquire_frames += 1
            else:
                self._roundabout_fast_entry_reacquire_frames = 0
            if (
                self._roundabout_fast_entry_reacquire_frames
                >= tuning.roundabout_entry_reacquire_frames
            ):
                frames = self._roundabout_fast_entry_reacquire_frames
                self._set_phase("ROUNDABOUT_FOLLOW", now)
                self.get_logger().warn(
                    f"左分支已离开且连续 {frames} 帧稳定看到内线："
                    "进入环内逐帧权重巡线"
                )
                return

            # 实车可能因为遮挡/透视导致 line_valid 断续，使连续帧计数长期清零。
            # FAST_ENTRY 使用较强固定左转下限，不能无限持续，否则会在环内绕多圈。
            if phase_age >= tuning.roundabout_entry_max_search_seconds:
                frames = self._roundabout_fast_entry_reacquire_frames
                self._set_phase("ROUNDABOUT_FOLLOW", now)
                self.get_logger().warn(
                    "FAST_ENTRY 达到最大保持时间 "
                    f"{tuning.roundabout_entry_max_search_seconds:.2f}s，"
                    f"当前连续线 {frames}/{tuning.roundabout_entry_reacquire_frames}；"
                    "停止强左提交；进入环内巡线，若刚交接就丢线则短暂保持轻微左弧，绝不反打右"
                )
            return

        if self.phase == "ROUNDABOUT_FOLLOW":
            gyro_exit = self._update_roundabout_gyro_exit(
                now, result.line_valid, allow_confirm=True
            )
            if phase_age >= tuning.roundabout_exit_arm_seconds:
                visual_angle_ok = bool(
                    (not tuning.roundabout_gyro_exit_enabled)
                    or (not tuning.roundabout_exit_require_gyro_arm)
                    or self._roundabout_gyro_armed
                )
                if (
                    bool(getattr(result, "roundabout_exit_fork", False))
                    and visual_angle_ok
                ):
                    self._adaptive_roundabout_exit_frames += 1
                else:
                    self._adaptive_roundabout_exit_frames = 0
                if (
                    self._adaptive_roundabout_exit_frames
                    >= tuning.roundabout_exit_fork_confirm_frames
                ):
                    self._roundabout_exit_source = "visual"
                    self._set_phase("ROUNDABOUT_EXIT", now)
                    self.get_logger().warn(
                        "环内连续确认横向虚线主链 + 斜向虚线分支，且gyro角度门已满足："
                        "确认真正出环；"
                        f"累计转角={self._roundabout_gyro_turn_deg:.1f}deg；"
                        f"先进入 {tuning.roundabout_exit_commit_seconds:.2f}s "
                        "左侧出口提交，禁止在分叉处跟右线"
                    )
                    return
                if gyro_exit:
                    self._roundabout_exit_source = "gyro"
                    self._set_phase("ROUNDABOUT_EXIT", now)
                    self.get_logger().warn(
                        "IMU gyro_z 出环兜底确认：从真正入口累计转角 "
                        f"{self._roundabout_gyro_turn_deg:.1f}deg，"
                        f"相对目标{tuning.roundabout_gyro_target_turn_deg:.1f}deg的误差="
                        f"{self._roundabout_gyro_error_deg:+.1f}deg，"
                        f"窗口确认={self._roundabout_gyro_confirm_frames}帧，"
                        f"越界锁存={self._roundabout_gyro_force_exit_frames}帧，"
                        f"reason={self._roundabout_gyro_exit_reason}；"
                        "gyro只判断状态，不控制方向"
                    )
                    return
            if phase_age >= tuning.roundabout_follow_timeout_seconds:
                self._roundabout_exit_source = "timeout"
                self._set_phase("ROUNDABOUT_EXIT", now)
                self.get_logger().warn(
                    "视觉分叉与IMU gyro_z积分均未及时确认：进入旧受限出口段作为最终安全兜底"
                )
            return

        if self.phase == "ROUNDABOUT_EXIT":
            # Continue gyro integration through the physical exit so heading
            # control can actually converge from ~350deg toward the saved 360deg
            # entry direction. Confirmation is disabled because exit is already latched.
            self._update_roundabout_gyro_exit(
                now, result.line_valid, allow_confirm=False
            )
            exit_commit_seconds = (
                tuning.roundabout_gyro_exit_commit_seconds
                if self._roundabout_exit_source == "gyro"
                else tuning.roundabout_exit_commit_seconds
            )
            if phase_age >= exit_commit_seconds:
                self._roundabout_ignore_until = (
                    now + tuning.roundabout_ignore_after_exit_seconds
                )
                exit_source = self._roundabout_exit_source
                self._set_phase("SEEK_CROSSWALK", now)
                self.get_logger().info(
                    "出环完成："
                    f"source={exit_source}；恢复普通巡线/找人行横道；"
                    f"未来 {tuning.roundabout_ignore_after_exit_seconds:.1f}s "
                    "忽略环岛标志"
                )
            return

        if self.phase == "SEEK_CROSSWALK":
            reliable_exit_line = bool(
                getattr(result, "post_exit_main_valid", False)
                or (
                    result.line_valid
                    and int(result.valid_bands)
                    >= int(tuning.roundabout_exit_reacquire_min_bands)
                    and float(result.confidence)
                    >= float(tuning.roundabout_exit_reacquire_min_confidence)
                )
            )
            if self._roundabout_exit_left_search_active:
                self._roundabout_exit_reacquire_frames = (
                    self._roundabout_exit_reacquire_frames + 1
                    if reliable_exit_line
                    else 0
                )
                if (
                    self._roundabout_exit_reacquire_frames
                    >= int(tuning.roundabout_exit_reacquire_confirm_frames)
                ):
                    self._roundabout_exit_left_search_active = False
                    self.get_logger().info(
                        "出环左找线完成：连续看到可信主线，恢复正常视觉巡线"
                    )
                else:
                    # Do not advance to crosswalk/finish logic while the exit
                    # road itself has not yet been reacquired.
                    self._crosswalk_frames = 0
                    self._yellow_frames = 0
                    return
            if now < self._roundabout_exit_heading_post_until:
                self._update_roundabout_gyro_exit(
                    now, result.line_valid, allow_confirm=False
                )
            if phase_age < tuning.crosswalk_detect_delay_after_roundabout:
                self._crosswalk_frames = 0
                self._yellow_frames = 0
                return
            if not self._live_tuning.crosswalk_enabled:
                self._crosswalk_frames = 0
                if result.yellow_finish:
                    self._yellow_frames += 1
                else:
                    self._yellow_frames = 0
                if self._yellow_frames >= base.YELLOW_FINISH_CONFIRM_FRAMES:
                    self._handle_yellow_finish(now, "crosswalk_disabled")
                return
            self._crosswalk_frames = (
                self._crosswalk_frames + 1 if result.crosswalk else 0
            )
            if self._crosswalk_frames >= tuning.crosswalk_confirm_frames:
                if int(self.lap_count) == 0 and self._first_crosswalk_confirmed_at <= 0.0:
                    self._first_crosswalk_confirmed_at = now
                    self._second_roundabout_enable_after = (
                        now
                        + tuning.second_roundabout_arm_delay_after_first_crosswalk_seconds
                    )
                    self.get_logger().warn(
                        "第一次人行横道确认：从现在开始锁住第二次环岛识别 "
                        f"{tuning.second_roundabout_arm_delay_after_first_crosswalk_seconds:.1f}s；"
                        "期间即使进入第二圈也只正常巡线，不累计环岛marker"
                    )
                self._set_phase("CROSSWALK_APPROACH", now)
                source = (
                    "far"
                    if bool(getattr(result, "crosswalk_far_visible", False))
                    else "near"
                )
                self.get_logger().info(
                    f"人行横道提前确认({source})："
                    f"score={result.crosswalk_score:.2f}；"
                    f"继续循迹 {tuning.crosswalk_approach_distance_m:.2f}m "
                    "后停车"
                )
                return
            if base.YELLOW_ACCEPT_WHILE_SEEK_CROSSWALK:
                self._yellow_frames = (
                    self._yellow_frames + 1 if result.yellow_finish else 0
                )
                if self._yellow_frames >= base.YELLOW_FINISH_CONFIRM_FRAMES:
                    self._handle_yellow_finish(
                        now, "crosswalk_missed_yellow"
                    )
            return

        if self.phase == "CROSSWALK_APPROACH":
            self._update_crosswalk_approach_progress()
            odom_ready = (
                self._odom_snapshot() is not None
                and self._crosswalk_approach_start_pose is not None
            )
            distance_ready = (
                phase_age >= tuning.crosswalk_approach_min_seconds
                and self._crosswalk_approach_distance
                >= tuning.crosswalk_approach_distance_m
            )
            fallback_ready = (
                (not odom_ready)
                and phase_age >= tuning.crosswalk_approach_fallback_seconds
            )
            if distance_ready or fallback_ready:
                traveled = self._crosswalk_approach_distance
                self._begin_crosswalk_brake_or_stop(
                    now, traveled, "提前停车点"
                )
            return

        super()._sync_course_state(result, now)

    def _trend_recovery_command(
        self, now: float, *, exit_bias: float = 0.0
    ) -> Tuple[float, float, str]:
        """Keep searching from the last trustworthy frame instead of freezing.

        `line_recovery_timeout_seconds` is now the boundary between predictive
        recovery and persistent low-speed search; it is no longer a stop timer.
        """
        tuning = self._adaptive_tuning
        has_previous_line = float(getattr(self, "_last_valid_line_time", 0.0)) > 0.0

        if self._line_trend.last_seen_time <= 0.0:
            if tuning.line_recovery_persistent_enabled and has_previous_line:
                angular = base.clamp(
                    float(self._last_good_angular),
                    -tuning.line_recovery_persistent_turn_max,
                    tuning.line_recovery_persistent_turn_max,
                )
                if abs(angular) >= tuning.line_recovery_direction_deadband:
                    if abs(angular) < tuning.line_recovery_persistent_turn_start:
                        angular = (1.0 if angular >= 0.0 else -1.0) * tuning.line_recovery_persistent_turn_start
                else:
                    # History may have been intentionally reset at a phase
                    # boundary. Crawl straight rather than freezing in place.
                    angular = 0.0
                self._have_previous_error = False
                return (
                    tuning.line_recovery_persistent_speed,
                    angular,
                    "adaptive_line_no_history_persistent_search",
                )
            return 0.0, 0.0, "adaptive_line_no_history_stop"

        loss_age = max(0.0, now - self._line_trend.last_seen_time)
        estimate = self._line_trend.estimate(loss_age, tuning)
        if estimate is None:
            return 0.0, 0.0, "adaptive_line_estimate_missing_stop"
        predicted, _position, trend = estimate
        angular = (
            base.STEERING_SIGN * tuning.line_recovery_kp * predicted
            + exit_bias
        )
        angular = base.clamp(
            angular,
            -tuning.line_recovery_max_turn_speed,
            tuning.line_recovery_max_turn_speed,
        )

        reference_speed = max(
            0.0,
            float(self._last_command[0]),
            float(self._last_trusted_linear),
        )
        age_ratio = base.clamp(
            loss_age / max(1e-6, tuning.line_recovery_timeout_seconds),
            0.0,
            1.0,
        )
        linear = (
            reference_speed
            * tuning.line_recovery_speed_ratio
            * (1.0 - 0.65 * age_ratio)
        )
        linear = max(tuning.line_recovery_min_speed, linear)

        # A one/two-frame visual dropout should feel invisible to the chassis.
        if loss_age <= tuning.line_recovery_bridge_seconds:
            linear = max(
                linear,
                reference_speed * tuning.line_recovery_bridge_speed_ratio,
            )
            phase_tag = "bridge"
        elif loss_age <= tuning.line_recovery_timeout_seconds:
            phase_tag = "predict"
        elif tuning.line_recovery_persistent_enabled:
            # Never collapse to (0,0) simply because the camera lost the line
            # for >1.5s. Preserve the last trustworthy direction and slowly
            # increase search curvature while moving at a safe crawl.
            direction_source = angular
            if abs(direction_source) < tuning.line_recovery_direction_deadband:
                direction_source = float(self._last_good_angular)
            if abs(direction_source) < tuning.line_recovery_direction_deadband:
                direction_source = base.STEERING_SIGN * float(trend)

            if abs(direction_source) >= tuning.line_recovery_direction_deadband:
                search_age = max(0.0, loss_age - tuning.line_recovery_timeout_seconds)
                ramp = base.clamp(
                    search_age / max(1e-6, tuning.line_recovery_persistent_turn_ramp_seconds),
                    0.0,
                    1.0,
                )
                search_mag = (
                    tuning.line_recovery_persistent_turn_start
                    + (
                        tuning.line_recovery_persistent_turn_max
                        - tuning.line_recovery_persistent_turn_start
                    ) * ramp
                )
                angular = (1.0 if direction_source > 0.0 else -1.0) * max(
                    search_mag,
                    min(abs(angular), tuning.line_recovery_persistent_turn_max),
                )
            else:
                # Last observed line was centered: crawl straight instead of
                # inventing a left/right direction. As soon as any line returns,
                # normal tracking takes control again.
                angular = 0.0
            angular = base.clamp(
                angular,
                -tuning.line_recovery_persistent_turn_max,
                tuning.line_recovery_persistent_turn_max,
            )
            linear = tuning.line_recovery_persistent_speed
            phase_tag = "persistent"
        else:
            return 0.0, 0.0, "adaptive_line_loss_timeout_stop"

        self._last_good_angular = angular
        self._have_previous_error = False
        if angular > 0.04:
            direction = "left"
        elif angular < -0.04:
            direction = "right"
        else:
            direction = "center"
        trend_tag = "L" if trend < -0.01 else ("R" if trend > 0.01 else "C")
        return (
            linear,
            angular,
            f"adaptive_trend_recovery_{phase_tag}_{direction}_{trend_tag}",
        )

    def _normal_patrol_grip_command(
        self, result: base.VisionResult, now: float
    ) -> Tuple[float, float, str]:
        """Strengthen centering only while a normal-patrol line is still visible.

        This does not replace good3_1 steering.  It only prevents the common
        visible->lost failure: when the line is already walking toward an edge,
        use the same weighted N/M/F history to bite toward it before segmentation
        disappears, and trim forward speed only near the edge.
        """
        linear, angular, mode = super()._planned_command(result, now)
        linear, angular, preview_mode = self._apply_forward_road_preview(
            result, linear, angular
        )
        if preview_mode != "clear":
            mode = preview_mode + "/" + mode
        tuning = self._adaptive_tuning
        estimate = self._line_trend.estimate(0.0, tuning)
        if estimate is None:
            return linear, angular, mode

        predicted, position, trend = estimate
        edge_pressure = max(
            abs(float(result.near_error)),
            abs(float(result.mid_error)),
            abs(float(result.far_error)),
            abs(float(predicted)),
        )
        moving_outward = (
            float(position) * float(trend) > 0.0
            and abs(float(trend)) >= NORMAL_GRIP_OUTWARD_TREND
        )
        if edge_pressure < NORMAL_GRIP_START_ERROR and not moving_outward:
            return linear, angular, mode

        grip_angular = (
            base.STEERING_SIGN
            * float(tuning.line_recovery_kp)
            * NORMAL_GRIP_GAIN
            * float(predicted)
        )
        grip_angular = base.clamp(
            grip_angular, -NORMAL_GRIP_MAX_TURN, NORMAL_GRIP_MAX_TURN
        )
        # Never reverse a valid good3_1 command.  Only add authority in the same
        # direction (or from nearly zero) when the line is visibly escaping.
        if angular * grip_angular >= 0.0 or abs(float(angular)) < 0.04:
            if abs(grip_angular) > abs(float(angular)):
                angular = grip_angular

        if edge_pressure >= NORMAL_GRIP_HARD_EDGE:
            linear = min(float(linear), NORMAL_GRIP_HARD_SPEED_CAP)
        elif edge_pressure >= NORMAL_GRIP_SOFT_EDGE:
            linear = min(float(linear), NORMAL_GRIP_SOFT_SPEED_CAP)

        self._last_good_angular = float(angular)
        return linear, angular, "adaptive_normal_center_grip/" + mode

    def _reset_forward_road_preview(self) -> None:
        self._road_preview_near = 0.0
        self._road_preview_mid = 0.0
        self._road_preview_far = 0.0
        self._road_preview_direction = 0
        self._road_preview_confirm_frames = 0
        self._road_preview_ring_frames = 0
        self._road_preview_mode = "clear"
        self._road_preview_curve = 0.0

    def _apply_forward_road_preview(
        self,
        result: base.VisionResult,
        linear: float,
        angular: float,
    ) -> Tuple[float, float, str]:
        """Prepare for visible road geometry without changing the 5.26 state flow."""
        if not result.line_valid or int(result.valid_bands) < 3:
            self._road_preview_confirm_frames = max(
                0, self._road_preview_confirm_frames - 1
            )
            self._road_preview_ring_frames = max(0, self._road_preview_ring_frames - 1)
            self._road_preview_mode = "clear"
            return linear, angular, self._road_preview_mode

        alpha = ROAD_PREVIEW_ALPHA
        if self._road_preview_confirm_frames == 0 and self._road_preview_ring_frames == 0:
            self._road_preview_near = float(result.near_error)
            self._road_preview_mid = float(result.mid_error)
            self._road_preview_far = float(result.far_error)
        else:
            self._road_preview_near = (
                alpha * float(result.near_error)
                + (1.0 - alpha) * self._road_preview_near
            )
            self._road_preview_mid = (
                alpha * float(result.mid_error)
                + (1.0 - alpha) * self._road_preview_mid
            )
            self._road_preview_far = (
                alpha * float(result.far_error)
                + (1.0 - alpha) * self._road_preview_far
            )

        # Far-minus-mid leads the vehicle; mid-minus-near confirms that the
        # geometry is a continuous bend rather than one isolated dark patch.
        far_curve = self._road_preview_far - self._road_preview_mid
        mid_curve = self._road_preview_mid - self._road_preview_near
        curve = 0.68 * far_curve + 0.32 * mid_curve
        self._road_preview_curve = float(curve)
        candidate_direction = (
            1 if curve > ROAD_PREVIEW_CURVE_THRESHOLD
            else (-1 if curve < -ROAD_PREVIEW_CURVE_THRESHOLD else 0)
        )
        continuous_shape = bool(
            candidate_direction != 0
            and (
                far_curve * mid_curve >= 0.0
                or abs(far_curve) >= 1.7 * ROAD_PREVIEW_CURVE_THRESHOLD
            )
        )
        if continuous_shape:
            if candidate_direction == self._road_preview_direction:
                self._road_preview_confirm_frames += 1
            else:
                self._road_preview_direction = candidate_direction
                self._road_preview_confirm_frames = 1
        else:
            self._road_preview_confirm_frames = max(
                0, self._road_preview_confirm_frames - 1
            )
            if self._road_preview_confirm_frames == 0:
                self._road_preview_direction = 0

        ring_shape = bool(
            getattr(result, "roundabout_left_branch_raw", False)
            and (
                getattr(result, "roundabout_center_dash", False)
                or getattr(result, "roundabout_same_scene", False)
            )
        )
        self._road_preview_ring_frames = (
            self._road_preview_ring_frames + 1
            if ring_shape
            else max(0, self._road_preview_ring_frames - 1)
        )
        if self._road_preview_ring_frames >= ROAD_PREVIEW_RING_CONFIRM_FRAMES:
            # Prepare speed only.  The proven 5.26 marker gate and timer still
            # decide when entry starts, so preview cannot cause an early turn.
            self._road_preview_mode = "road_preview_ring_speed"
            return (
                min(float(linear), ROAD_PREVIEW_RING_SPEED_CAP),
                float(angular),
                self._road_preview_mode,
            )

        if self._road_preview_confirm_frames < ROAD_PREVIEW_CONFIRM_FRAMES:
            self._road_preview_mode = "clear"
            return linear, angular, self._road_preview_mode

        preview_angular = base.clamp(
            base.STEERING_SIGN * float(curve),
            -ROAD_PREVIEW_MAX_FEEDFORWARD,
            ROAD_PREVIEW_MAX_FEEDFORWARD,
        )
        # Never reverse the current controller.  Preview may gently strengthen a
        # matching bend or prepare from near-zero, but cannot fight the near line.
        if float(angular) * preview_angular >= 0.0 or abs(float(angular)) < 0.04:
            if abs(preview_angular) > abs(float(angular)):
                angular = preview_angular
        self._road_preview_mode = (
            "road_preview_left"
            if preview_angular > 0.0
            else "road_preview_right"
        )
        return (
            min(float(linear), ROAD_PREVIEW_CURVE_SPEED_CAP),
            float(angular),
            self._road_preview_mode,
        )

    def _roundabout_entry_trend_command(
        self, result: base.VisionResult, now: float
    ) -> Tuple[float, float, str]:
        """Frame-by-frame left-biased trend controller for ring entry."""

        tuning = self._adaptive_tuning
        if result.line_valid:
            self._last_valid_line_time = now
            self._line_trend.observe(
                now,
                result.near_error,
                result.mid_error,
                result.far_error,
                tuning,
            )
            estimate = self._line_trend.estimate(0.0, tuning)
        elif self._line_trend.last_seen_time > 0.0:
            loss_age = max(0.0, now - self._line_trend.last_seen_time)
            estimate = self._line_trend.estimate(loss_age, tuning)
        else:
            estimate = None

        if estimate is None:
            predicted = 0.0
            trend = 0.0
        else:
            predicted, _position, trend = estimate

        trend_angular = (
            base.STEERING_SIGN * tuning.line_recovery_kp * predicted
        )
        angular = trend_angular + tuning.roundabout_entry_left_bias
        entry_age = max(0.0, now - float(self.phase_enter_time))
        entry_min_left = float(tuning.roundabout_entry_min_left_turn_speed)
        if entry_age <= FAST_ENTRY_COMMIT_SECONDS:
            # Marker->FAST_ENTRY must feel identical on both laps.  If the second
            # lap happens to lose the main line exactly at handoff, do not fall
            # back to the weak +0.45 minimum; keep a short, explicit left commit.
            entry_min_left = max(entry_min_left, FAST_ENTRY_COMMIT_MIN_LEFT)
        angular = base.clamp(
            angular,
            entry_min_left,
            tuning.roundabout_entry_max_turn_speed,
        )
        self._last_good_angular = angular
        self._have_previous_error = False

        if trend < -0.01:
            trend_tag = "L"
        elif trend > 0.01:
            trend_tag = "R"
        else:
            trend_tag = "C"
        source = "line" if result.line_valid else "loss"
        return (
            tuning.roundabout_entry_forward_speed,
            angular,
            f"adaptive_roundabout_entry_trend_{source}_{trend_tag}",
        )

    def _limit_command(
        self, linear: float, angular: float, now: float
    ) -> Tuple[float, float]:
        """Asymmetric output slew: brake quickly, accelerate/steer smoothly.

        Normal patrol and roundabout FOLLOW both pass through here.  This avoids
        frame-to-frame command steps while still letting the car slow down fast
        enough before a bend instead of carrying speed into it.
        """
        tuning = self._adaptive_tuning

        if not tuning.motion_smoothing_enabled:
            return super()._limit_command(linear, angular, now)

        # Use real processed-frame spacing. The camera is configured for 30Hz,
        # but field logs show vision processing around 9~12Hz; clamping dt to
        # 2/30s made steering acceleration artificially too weak and laggy.
        dt = base.clamp(
            now - self._last_motion_update_time,
            0.0,
            max(2.0 / base.CONTROL_RATE, 0.12),
        )
        current_linear, current_angular = self._last_command

        # Slowing down must be faster than speeding up to avoid geometric overshoot.
        accelerating = (
            current_linear * linear >= 0.0
            and abs(linear) > abs(current_linear)
        )
        linear_rate = (
            tuning.motion_linear_accel_up
            if accelerating
            else tuning.motion_linear_decel
        )
        # Keep good3_1 steering and curve-speed targets untouched.  Only normal
        # patrol re-acceleration is gentler, so leaving a bend does not produce
        # repeated 15Hz throttle steps.  Deceleration remains the original fast
        # value, and ROUNDABOUT_FOLLOW keeps its existing acceleration behavior.
        marker_pass_active = (
            self.phase == "SEEK_ROUNDABOUT"
            and self._ring_branch_gate.count == 1
            and now < self._roundabout_marker1_straight_until
        )
        if (
            accelerating
            and self.phase in ("SEEK_ROUNDABOUT", "SEEK_CROSSWALK", "SEEK_FINISH")
            and not marker_pass_active
        ):
            linear_rate = min(linear_rate, NORMAL_PATROL_REACCEL_RATE)

        # Build turn promptly in one direction, but slow down direction reversals.
        # This targets the observed left-right snapping without making a real bend lazy.
        if self.phase == "ROUNDABOUT_FOLLOW":
            angular_rate = tuning.motion_angular_slew_roundabout
        else:
            angular_rate = tuning.motion_angular_slew_normal

        if abs(angular) < tuning.motion_angular_deadband:
            angular = 0.0

        if self.phase not in ("ROUNDABOUT_FAST_ENTRY", "ROUNDABOUT_EXIT"):
            alpha = tuning.motion_angular_target_alpha
            self._smoothed_angular_target = (
                alpha * angular
                + (1.0 - alpha) * self._smoothed_angular_target
            )
            angular = self._smoothed_angular_target
        else:
            self._smoothed_angular_target = angular

        reversing = (
            abs(current_angular) > tuning.motion_angular_deadband
            and abs(angular) > tuning.motion_angular_deadband
            and current_angular * angular < 0.0
        )
        if reversing:
            angular_rate = min(angular_rate, tuning.motion_angular_reverse_slew)

        limited_linear = self._approach(
            current_linear,
            linear,
            linear_rate * dt,
        )
        limited_angular = self._approach(
            current_angular,
            angular,
            angular_rate * dt,
        )
        self._last_motion_update_time = now
        return limited_linear, limited_angular

    def _planned_command(
        self, result: base.VisionResult, now: float
    ) -> Tuple[float, float, str]:
        tuning = self._adaptive_tuning
        self._adaptive_perf_frames += 1
        self._status_last_vision = result
        self._adaptive_perf_last_error = float(getattr(result, "error", 0.0))
        self._adaptive_perf_last_reason = str(getattr(result, "reason", "unknown"))

        if result.line_valid and self.phase in (
            "SEEK_ROUNDABOUT",
            "SEEK_CROSSWALK",
            "SEEK_FINISH",
            "ROUNDABOUT_FOLLOW",
            "ROUNDABOUT_EXIT",
        ):
            self._line_trend.observe(
                now,
                result.near_error,
                result.mid_error,
                result.far_error,
                tuning,
            )

        # The final pass remains a sprint, but it is no longer a blind 1.0m/s
        # straight command. Follow the visible line at a bounded 0.35m/s; if the
        # line drops out, reuse the same directional recovery as normal patrol.
        if (
            int(getattr(self, "lap_count", 0)) + 1 >= int(base.TOTAL_LAPS)
            and self.phase == "FINISH_PASS"
        ):
            if result.line_valid:
                linear, angular, _mode = super()._planned_command(result, now)
                linear = max(
                    linear,
                    min(
                        0.35,
                        float(base.FORWARD_SPEED)
                        * float(base.FINAL_SPRINT_SPEED_MULTIPLIER),
                    ),
                )
                angular = base.clamp(angular, -0.40, 0.40)
                self._last_good_angular = angular
                return linear, angular, "final_straight_double_speed_limited_line_follow"
            linear, angular, _mode = self._trend_recovery_command(now)
            return (
                min(linear, 0.18),
                base.clamp(angular, -0.40, 0.40),
                "final_straight_double_speed_limited_line_recovery",
            )

        # V7: after the real exit shape is confirmed, ONLY roundabout branch
        # events are suppressed. The car continues to follow the main line at the
        # normal patrol speed for the configured pass window. Large LEFT commands
        # are capped when the left branch is visible so the branch cannot suck the
        # chassis into the ring early. Intermittent dashed gaps reuse recent trend.
        if (
            self.phase == "SEEK_ROUNDABOUT"
            and self._ring_branch_gate.count == 1
            and now < self._roundabout_marker1_straight_until
        ):
            branch_visible = bool(
                getattr(result, "roundabout_left_branch", False)
                or getattr(result, "roundabout_left_branch_raw", False)
            )
            if tuning.roundabout_marker1_follow_line_enabled and result.line_valid:
                linear, angular, mode = super()._planned_command(result, now)
                self._last_valid_line_time = now
                if branch_visible:
                    branch_speed = max(
                        float(base.MINIMUM_FORWARD_SPEED),
                        float(getattr(self, "_last_trusted_linear", base.FORWARD_SPEED))
                        * tuning.roundabout_marker1_gap_speed_ratio,
                    )
                    linear = max(linear, min(float(base.FORWARD_SPEED), branch_speed))
            else:
                linear, angular, mode = self._trend_recovery_command(now)
                loss_age = (
                    max(0.0, now - self._line_trend.last_seen_time)
                    if self._line_trend.last_seen_time > 0.0
                    else tuning.line_recovery_timeout_seconds
                )
                if loss_age <= tuning.roundabout_marker1_gap_bridge_seconds:
                    bridge_speed = max(
                        float(base.MINIMUM_FORWARD_SPEED),
                        float(getattr(self, "_last_trusted_linear", base.FORWARD_SPEED))
                        * tuning.roundabout_marker1_gap_speed_ratio,
                    )
                    linear = max(linear, min(float(base.FORWARD_SPEED), bridge_speed))
                # good3_1 原样：marker确认后的2秒主线通过窗口不因虚线闪断掉到慢速。
                linear = max(
                    linear,
                    min(
                        float(base.FORWARD_SPEED),
                        max(
                            float(tuning.roundabout_marker1_pass_loss_min_speed),
                            MARKER_PASS_LOSS_SPEED_FLOOR,
                        ),
                    ),
                )

            angular = base.clamp(
                angular,
                -tuning.roundabout_marker1_follow_max_turn_speed,
                tuning.roundabout_marker1_follow_max_turn_speed,
            )
            if branch_visible and angular > tuning.roundabout_marker1_branch_left_turn_cap:
                angular = tuning.roundabout_marker1_branch_left_turn_cap
                mode = mode + "/left_branch_suppressed"
            self._last_good_angular = angular
            return (
                linear,
                angular,
                "adaptive_marker1_mainline_pass_" + mode,
            )

        if self.phase == "ROUNDABOUT_EXIT":
            # Keep the first part of the exit committed to the selected branch.
            # Once the chassis has cleared the fork, let the independently fitted
            # forward main chain steer it out.  A small right correction is allowed
            # here so the car can align with the straight without turning back into
            # the ring.
            if (
                phase_age >= 0.55
                and bool(getattr(result, "post_exit_main_valid", False))
            ):
                exit_main = copy.copy(result)
                exit_main.line_valid = True
                exit_main.error = float(
                    getattr(result, "post_exit_main_error", 0.0)
                )
                exit_main.near_error = float(
                    getattr(result, "post_exit_main_near_error", exit_main.error)
                )
                exit_main.mid_error = float(
                    getattr(result, "post_exit_main_mid_error", exit_main.error)
                )
                exit_main.far_error = float(
                    getattr(result, "post_exit_main_far_error", exit_main.error)
                )
                exit_main.confidence = max(
                    0.35,
                    float(getattr(result, "post_exit_main_confidence", 0.0)),
                )
                exit_main.valid_bands = max(
                    2, int(getattr(result, "post_exit_main_count", 2))
                )
                exit_main.reason = "roundabout_exit_forward_main"
                linear, angular, _mode = super()._planned_command(exit_main, now)
                angular = base.clamp(
                    angular,
                    -0.12,
                    float(tuning.roundabout_exit_max_turn_speed),
                )
                self._last_valid_line_time = now
                self._last_good_angular = angular
                return (
                    min(linear, float(tuning.roundabout_exit_speed)),
                    angular,
                    "adaptive_roundabout_exit_mainline_takeover",
                )

            heading_cmd = self._roundabout_exit_heading_command(
                result, now, post_exit=False
            )
            if heading_cmd is not None:
                return heading_cmd

            if self._roundabout_exit_source == "gyro":
                # V10: gyro confirms STATE only. It must never create a blind
                # straight segment. If a real line is visible, follow it normally;
                # if the exit junction hides the line, keep turning LEFT.
                if result.line_valid:
                    linear, angular, _mode = super()._planned_command(result, now)
                    self._last_valid_line_time = now
                    self._last_good_angular = angular
                    return (
                        linear,
                        angular,
                        "adaptive_roundabout_gyro_exit_line_follow",
                    )
                angular = max(
                    float(tuning.roundabout_exit_loss_left_turn),
                    float(self._last_good_angular),
                )
                angular = base.clamp(
                    angular,
                    0.0,
                    max(
                        float(tuning.roundabout_exit_max_turn_speed),
                        float(tuning.roundabout_exit_loss_left_turn),
                    ),
                )
                self._last_good_angular = angular
                self._have_previous_error = False
                return (
                    float(tuning.roundabout_exit_loss_speed),
                    angular,
                    "adaptive_roundabout_gyro_exit_loss_keep_left",
                )

            # Visual fork / timeout fallback keeps the proven LEFT exit commit.
            # 分叉出口已确认后必须选左支。正角速度为左转；
            # 即使普通巡线瞬间更偏向右支，也不允许给出负角速度。
            min_left = max(
                0.0,
                float(self._live_tuning.roundabout_exit_min_turn_speed),
            )
            if result.line_valid:
                self._last_valid_line_time = now
                angular = (
                    self._normal_angular(result, now)
                    * tuning.roundabout_follow_steering_gain
                    + tuning.roundabout_exit_left_bias
                )
                angular = base.clamp(
                    angular,
                    min_left,
                    tuning.roundabout_exit_max_turn_speed,
                )
                self._last_good_angular = angular
                return (
                    tuning.roundabout_exit_speed,
                    angular,
                    "adaptive_roundabout_exit_line_left_bias",
                )
            linear, angular, _mode = self._trend_recovery_command(
                now,
                exit_bias=tuning.roundabout_exit_left_bias,
            )
            angular = max(
                min_left,
                min(tuning.roundabout_exit_max_turn_speed, angular),
            )
            return (
                min(linear, tuning.roundabout_exit_speed),
                angular,
                "adaptive_roundabout_exit_trend_left",
            )

        if self.phase == "ROUNDABOUT":
            # Configurable gentle physical entry.  Do not inherit the legacy
            # 0.98rad/s global entry turn that caused the chassis to pivot on the
            # dashed gate before its wheels had actually entered the roundabout.
            self._last_good_angular = float(
                tuning.roundabout_entry_min_left_turn_speed
            )
            self._have_previous_error = False
            return (
                float(tuning.roundabout_entry_forward_speed),
                float(tuning.roundabout_entry_min_left_turn_speed),
                "adaptive_roundabout_gentle_physical_entry",
            )

        if self.phase == "ROUNDABOUT_FAST_ENTRY":
            return self._roundabout_entry_trend_command(result, now)

        if self.phase == "SEEK_CROSSWALK":
            # Only post-exit selection is changed: a real/virtual main road takes
            # control immediately.  LEFT branch rejection happens in vision
            # candidate selection, never by clamping a legitimate main-line turn.
            mainline_cmd = self._post_exit_mainline_command(result, now)
            if mainline_cmd is not None:
                return mainline_cmd
            if self._roundabout_exit_left_search_active:
                angular = base.clamp(
                    max(
                        float(tuning.roundabout_exit_loss_left_turn),
                        float(self._last_good_angular),
                    ),
                    0.0,
                    max(
                        float(tuning.roundabout_exit_max_turn_speed),
                        float(tuning.roundabout_exit_loss_left_turn),
                    ),
                )
                self._last_good_angular = angular
                self._have_previous_error = False
                return (
                    min(
                        float(tuning.roundabout_exit_loss_speed),
                        float(tuning.roundabout_exit_speed),
                    ),
                    angular,
                    "adaptive_post_exit_forced_left_search",
                )
            # Initial physical exit search stays exactly as before.  Only after
            # a normal/virtual main road has actually been reacquired do later
            # dropouts switch to the freshly observed trend direction.
            if self._post_exit_main_reacquired:
                return self._post_exit_reacquired_loss_command(now)

        if (
            self.phase == "SEEK_CROSSWALK"
            and now < self._roundabout_exit_heading_post_until
        ):
            heading_cmd = self._roundabout_exit_heading_command(
                result, now, post_exit=True
            )
            if heading_cmd is not None:
                return heading_cmd

        # V11: after ROUNDABOUT_EXIT finishes, SEEK_CROSSWALK still performs
        # ordinary frame-by-frame line following. Software re-entry prevention is
        # state-only (25s cooldown + per-lap completion lock + first-crosswalk gate);
        # the short gyro window above only suppresses a physically dangerous big-left
        # branch command and never creates a blind fixed-forward segment.

        # 左环岛内单独控制速度与丢线恢复。
        # 有线：继续原逐帧巡线。
        # 丢线：先完全服从最近几帧趋势，不再因为“线=无”就立刻强制左打。
        # 只有持续丢线超过普通趋势预测窗口、并且确实没有可靠左右方向时，才给很小的左偏保底。
        if self.phase == "ROUNDABOUT_FOLLOW":
            if result.line_valid:
                if (
                    self._roundabout_exit_zone_force_left_started
                    and not self._roundabout_exit_zone_line_reacquired
                    and self._roundabout_gyro_peak_turn_deg
                    >= tuning.roundabout_exit_loss_left_arm_turn_deg
                ):
                    self._roundabout_exit_zone_line_reacquired = True
                    self._roundabout_exit_zone_force_left_started = False
                    self.get_logger().warn(
                        "出口区强左找线已完成：重新看到可信主线；"
                        "后续退出强制左找，恢复逐帧巡线/趋势恢复；左侧分叉仅作干扰忽略"
                    )

                heading_cmd = self._roundabout_exit_heading_command(
                    result, now, post_exit=False
                )
                if heading_cmd is not None:
                    return heading_cmd

                follow_result = result
                branch_visible = bool(
                    getattr(result, "roundabout_left_branch", False)
                    or getattr(result, "roundabout_left_branch_raw", False)
                )
                if (
                    self._roundabout_exit_zone_line_reacquired
                    and branch_visible
                    and bool(getattr(result, "post_exit_main_valid", False))
                ):
                    # Reuse the already-computed branch-filtered black-block main
                    # chain, but only here after the 220deg+ forced-left recovery
                    # has successfully reacquired a line.  This prevents the LEFT
                    # fork from dragging FOLLOW back into another forced-left arc.
                    virtual = copy.copy(result)
                    virtual.line_valid = True
                    virtual.error = float(getattr(result, "post_exit_main_error", 0.0))
                    virtual.near_error = float(
                        getattr(result, "post_exit_main_near_error", virtual.error)
                    )
                    virtual.mid_error = float(
                        getattr(result, "post_exit_main_mid_error", virtual.error)
                    )
                    virtual.far_error = float(
                        getattr(result, "post_exit_main_far_error", virtual.error)
                    )
                    virtual.valid_bands = max(
                        2, int(getattr(result, "post_exit_main_count", 2))
                    )
                    virtual.reason = "roundabout_exit_main_branch_filtered"
                    follow_result = virtual

                linear, angular, _mode = super()._planned_command(follow_result, now)
                angular = base.clamp(
                    angular * tuning.roundabout_follow_steering_gain,
                    -tuning.line_recovery_max_turn_speed,
                    tuning.line_recovery_max_turn_speed,
                )
                linear = min(
                    linear,
                    float(self._live_tuning.roundabout_forward_speed),
                )
                return (
                    linear,
                    angular,
                    "adaptive_roundabout_follow_speed_cap",
                )

            # V11: once ~330deg is reached, lost-line steering is heading-based
            # rather than a fixed-left command: keep converging to the saved
            # FAST_ENTRY heading (360deg).
            heading_cmd = self._roundabout_exit_heading_command(
                result, now, post_exit=False
            )
            if heading_cmd is not None:
                return heading_cmd

            # 220~330deg exit-zone rule: line loss is the exit junction. Do not
            # let stale trend point right; search LEFT at the faster V11 speed.
            if (
                self._roundabout_gyro_fresh
                and self._roundabout_gyro_peak_turn_deg
                >= tuning.roundabout_exit_loss_left_arm_turn_deg
                and not self._roundabout_exit_zone_line_reacquired
            ):
                self._roundabout_exit_zone_force_left_started = True
                angular = base.clamp(
                    max(
                        tuning.roundabout_exit_loss_left_turn,
                        float(self._last_good_angular),
                    ),
                    0.0,
                    max(
                        tuning.roundabout_exit_loss_left_turn,
                        tuning.roundabout_blind_recovery_max_turn_speed,
                    ),
                )
                self._last_good_angular = angular
                self._have_previous_error = False
                return (
                    max(
                        tuning.roundabout_exit_loss_speed,
                        tuning.roundabout_blind_recovery_speed,
                    ),
                    angular,
                    "adaptive_roundabout_exit_zone_loss_keep_left",
                )

            # V10 single-block rescue: one isolated dashed square is still a
            # local path target even if no scan-band chain can be formed.
            if (
                tuning.roundabout_single_block_rescue_enabled
                and bool(getattr(result, "roundabout_single_block_visible", False))
                and not (
                    self._roundabout_exit_zone_line_reacquired
                    and (
                        bool(getattr(result, "roundabout_left_branch", False))
                        or bool(getattr(result, "roundabout_left_branch_raw", False))
                    )
                )
            ):
                block_error = float(
                    getattr(result, "roundabout_single_block_error", 0.0)
                )
                angular = base.STEERING_SIGN * tuning.roundabout_single_block_kp * block_error
                angular = base.clamp(
                    angular,
                    -tuning.roundabout_single_block_max_turn_speed,
                    tuning.roundabout_single_block_max_turn_speed,
                )
                self._last_good_angular = angular
                self._have_previous_error = False
                return (
                    min(
                        tuning.roundabout_single_block_speed,
                        float(self._live_tuning.roundabout_forward_speed),
                    ),
                    angular,
                    "adaptive_roundabout_single_block_rescue",
                )

            # Immediately after FAST_ENTRY, a one-frame segmentation blink must
            # not reverse the chassis. Keep moving into the ring with a mild left
            # arc until fresh inner-line history has been rebuilt.
            if now < self._roundabout_follow_entry_guard_until:
                linear = max(
                    tuning.roundabout_follow_entry_guard_speed,
                    min(
                        float(self._live_tuning.roundabout_forward_speed),
                        max(0.0, float(self._last_command[0])),
                    ),
                )
                angular = max(
                    tuning.roundabout_follow_entry_guard_min_left_turn,
                    min(
                        tuning.roundabout_follow_entry_guard_max_left_turn,
                        float(self._last_good_angular),
                    ),
                )
                self._last_good_angular = angular
                self._have_previous_error = False
                return (
                    linear,
                    angular,
                    "adaptive_roundabout_entry_handoff_loss_keep_left",
                )

            # Reuse the same weighted history used by normal patrol.  This keeps
            # LEFT/RIGHT/CENTER continuity across a one- or multi-frame dropout
            # instead of inventing a left turn as soon as the segmentation blinks.
            linear, angular, recovery_mode = self._trend_recovery_command(now)
            loss_age = (
                max(0.0, now - self._line_trend.last_seen_time)
                if self._line_trend.last_seen_time > 0.0
                else tuning.line_recovery_timeout_seconds
            )
            max_turn = float(tuning.roundabout_blind_recovery_max_turn_speed)
            angular = base.clamp(angular, -max_turn, max_turn)

            # Final insurance only: after the predictive window is exhausted,
            # use a tiny left arc ONLY when history no longer carries a reliable
            # direction.  A real rightward trend is therefore never overwritten.
            if (
                loss_age >= tuning.line_recovery_timeout_seconds
                and abs(angular) < tuning.line_recovery_direction_deadband
            ):
                angular = float(tuning.roundabout_blind_recovery_left_turn)
                recovery_mode = "adaptive_roundabout_blind_fallback_left"

            linear = min(
                max(linear, float(tuning.roundabout_blind_recovery_speed)),
                float(self._live_tuning.roundabout_forward_speed),
            )
            self._last_good_angular = angular
            self._have_previous_error = False
            return (
                linear,
                angular,
                f"adaptive_roundabout_blind_trend_{recovery_mode}",
            )

        if (
            result.line_valid
            and self.phase in ("SEEK_ROUNDABOUT", "SEEK_FINISH")
        ):
            return self._normal_patrol_grip_command(result, now)

        if (
            not result.line_valid
            and self.phase in (
                "SEEK_ROUNDABOUT",
                "SEEK_CROSSWALK",
                "SEEK_FINISH",
            )
        ):
            return self._trend_recovery_command(now)

        return super()._planned_command(result, now)


def main(args=None) -> None:
    base.WheelLegRaceNode = AdaptiveWheelLegRaceNode
    base.main(args)


if __name__ == "__main__":
    main()
