#!/usr/bin/env python3
"""Single-owner controller for task-three line, step, and crosswalk actions."""
from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass, replace
from typing import Optional

import cv2
import numpy as np
import rclpy
import line_follower as line_follower_module
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from rcl_interfaces.msg import SetParametersResult

from line_follower import (
    CONTROL_RATE_HZ, IMAGE_HEIGHT, IMAGE_TIMEOUT_SECONDS, IMAGE_WIDTH,
    PROCESS_HEIGHT, PROCESS_WIDTH,
    STATUS_LOG_SECONDS, DetectionResult, OutdoorLineFollower,
    OutdoorWhiteLineDetector,
)
from wheel_robot.msg import StepDetection


@dataclass
class CurveProfile:
    """Current-frame near/far path geometry, filtered across recent frames."""
    valid: bool = False
    near_x: float = 0.0
    far_x: float = 0.0
    preview_x: Optional[float] = None
    curvature_raw: float = 0.0
    curvature_filtered: float = 0.0
    bend_sign: int = 0
    alternated: bool = False
    s_curve_active: bool = False
    sample_count: int = 0


@dataclass(frozen=True)
class RingMarker:
    """One close dashed-junction cluster in the robot-camera image."""
    side: str
    center_x: float
    center_y: float
    component_count: int


class BlackLineDetector(OutdoorWhiteLineDetector):
    """Track a black centre line; bridge the short gaps used on the roundabout."""

    def __init__(
        self, value_max, saturation_max, dash_kernel_height, memory_frames,
        center_corridor_px, max_offset_px, track_jump_px, lookahead_gain,
        target_alpha, min_curve_points, curvature_filter_alpha,
        s_curve_curvature_threshold, s_curve_hold_frames,
        s_curve_enter_confirm_frames,
        fit_max_extrapolation_px, min_fit_row_span_px,
        s_curve_lookahead_extra, s_curve_expand_px,
        s_curve_preview_top_ratio, s_curve_preview_bottom_ratio,
    ):
        super().__init__()
        self.value_max = int(value_max)
        self.saturation_max = int(saturation_max)
        height = max(3, int(dash_kernel_height))
        if height % 2 == 0:
            height += 1
        self._noise_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        self._dash_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, height))
        self.memory_frames = max(6, int(memory_frames))
        self.center_corridor_px = max(20.0, float(center_corridor_px))
        # The fixed central corridor is useful on straight road, but a correct
        # centre line legitimately moves toward either side in an S bend.
        # Keep a larger hard guard and choose points by path continuity.
        self.max_offset_px = max(self.center_corridor_px, float(max_offset_px))
        self.track_jump_px = max(25.0, float(track_jump_px))
        self.lookahead_gain = float(np.clip(lookahead_gain, 0.0, 0.85))
        self.target_alpha = float(np.clip(target_alpha, 0.35, 1.0))
        # A tight S bend may leave only three useful current-frame samples.
        # Three points are enough for a quadratic; accepting them is safer
        # than declaring the real centre line lost and searching toward an edge.
        self.min_curve_points = int(np.clip(min_curve_points, 3, 10))
        self.center_x = PROCESS_WIDTH / 2.0
        self._sample_target_x = self.center_x
        self._previous_x = {name: self.center_x for name in ("near", "mid", "far")}
        self._previous_path_x = {}
        self.curvature_filter_alpha = float(np.clip(curvature_filter_alpha, .05, .95))
        self.s_curve_curvature_threshold = max(.01, float(s_curve_curvature_threshold))
        self.s_curve_hold_frames = max(1, int(s_curve_hold_frames))
        self.s_curve_enter_confirm_frames = max(1, int(s_curve_enter_confirm_frames))
        self.fit_max_extrapolation_px = max(0.0, float(fit_max_extrapolation_px))
        self.min_fit_row_span_px = max(12.0, float(min_fit_row_span_px))
        self.s_curve_lookahead_extra = float(np.clip(s_curve_lookahead_extra, 0.0, 1.0))
        self.s_curve_expand_px = max(0.0, float(s_curve_expand_px))
        self.s_curve_preview_top_ratio = float(np.clip(s_curve_preview_top_ratio, .05, .70))
        self.s_curve_preview_bottom_ratio = float(np.clip(
            s_curve_preview_bottom_ratio, self.s_curve_preview_top_ratio + .05, .95
        ))
        self._curvature_filtered = 0.0
        self._last_high_bend_sign = 0
        self._pending_bend_sign = 0
        self._alternate_hits = 0
        self._s_curve_hold_remaining = 0
        self.s_curve_preview_enabled = False
        # The dashed ring is handled by the roundabout state machine and its
        # gap bridge.  It must not be fed through a solid-line polynomial.
        self.curve_fit_enabled = True
        self.curve_profile = CurveProfile()

    def _update_history(self, detections, accept):
        """Retain a confirmed centre-line history over several dash gaps."""
        for detection in detections:
            if accept and detection.valid and detection.x is not None:
                self._previous_x[detection.name] = detection.x
                self._misses[detection.name] = 0
            else:
                self._misses[detection.name] += 1
                if self._misses[detection.name] > self.memory_frames:
                    self._previous_x.pop(detection.name, None)

    def _reset_to_centre(self):
        self._previous_x = {name: self.center_x for name in ("near", "mid", "far")}
        self._misses = {name: 0 for name in ("near", "mid", "far")}
        self._sample_target_x = self.center_x
        self._previous_path_x = {}
        self._curvature_filtered = 0.0
        self._last_high_bend_sign = 0
        self._pending_bend_sign = 0
        self._alternate_hits = 0
        self._s_curve_hold_remaining = 0
        self.curve_profile = CurveProfile()

    @staticmethod
    def _run_centres(mask, rows, expected_x, max_jump, max_offset, centre_x):
        """Read narrow black-line runs at requested row positions."""
        points = []
        expected = expected_x
        for y in rows:
            row_y = min(mask.shape[0] - 1, max(0, int(y)))
            row = mask[row_y] > 0
            edges = np.flatnonzero(np.diff(np.pad(row.astype(np.int8), (1, 1))))
            runs = []
            for start, end in zip(edges[::2], edges[1::2]):
                width = int(end - start)
                if 2 <= width <= 65:
                    runs.append((0.5 * (start + end - 1), width))
            if not runs:
                continue
            x, _width = min(runs, key=lambda item: abs(item[0] - expected))
            if abs(x - expected) > max_jump or abs(x - centre_x) > max_offset:
                continue
            points.append((float(row_y), float(x)))
            expected = x
        return points

    def _expanded_far_preview(self, frame_640x480):
        """Read an additional current-frame far ROI only while in S-curve mode."""
        if not self.s_curve_preview_enabled:
            return None
        height, width = frame_640x480.shape[:2]
        y0 = int(round(height * self.s_curve_preview_top_ratio))
        y1 = int(round(height * self.s_curve_preview_bottom_ratio))
        if y1 <= y0 + 4:
            return None
        preview = cv2.resize(
            frame_640x480[y0:y1, :], (PROCESS_WIDTH, PROCESS_HEIGHT),
            interpolation=cv2.INTER_AREA,
        )
        mask, _ = self._make_mask(preview)
        expected = self.curve_profile.far_x if self.curve_profile.valid else self._sample_target_x
        points = self._run_centres(
            mask, (42, 64, 86, 108, 130), expected,
            self.track_jump_px + self.s_curve_expand_px,
            min(self.center_x - 2.0, self.max_offset_px + self.s_curve_expand_px),
            self.center_x,
        )
        if len(points) < 3:
            return None
        return float(np.median([point[1] for point in points]))

    def _five_point_black_target(self, mask):
        """Fit an eight-point black centre-line path and look ahead in bends."""
        # y grows toward the robot: high image rows are farther ahead and let
        # us turn before the near row has already drifted to a track edge.
        rows = (12, 26, 42, 60, 78, 96, 114, 132, 148, 162)
        expanded = self.s_curve_preview_enabled and self._s_curve_hold_remaining > 0
        max_jump = self.track_jump_px + (self.s_curve_expand_px if expanded else 0.0)
        max_offset = min(
            self.center_x - 2.0,
            self.max_offset_px + (self.s_curve_expand_px if expanded else 0.0),
        )
        expected = self._sample_target_x
        points = []
        for y in rows:
            expected = self._previous_path_x.get(y, expected)
            one = self._run_centres(mask, (y,), expected, max_jump, max_offset, self.center_x)
            if one:
                points.extend(one)
                expected = one[-1][1]
        if len(points) < self.min_curve_points:
            # Do not leave predictive mode latched forever after a genuine
            # loss.  The short hold still permits the expanded far ROI to
            # bridge a tight S-bend, then the normal safe loss logic resumes.
            if self._s_curve_hold_remaining > 0:
                self._s_curve_hold_remaining -= 1
                self.curve_profile = replace(
                    self.curve_profile, valid=False, alternated=False,
                    s_curve_active=self._s_curve_hold_remaining > 0,
                )
            else:
                self.curve_profile = CurveProfile()
            return None

        ys = np.array([point[0] for point in points], dtype=np.float32)
        xs = np.array([point[1] for point in points], dtype=np.float32)
        if float(np.ptp(ys)) < self.min_fit_row_span_px:
            self.curve_profile = CurveProfile()
            return None
        # A quadratic is sufficient for the continuous S bends, while fitting
        # all rows makes a single noisy row unable to pull the car to an edge.
        polynomial = np.polyfit(ys, xs, deg=2)
        near_x = float(np.polyval(polynomial, 148.0))
        lookahead_x = float(np.polyval(polynomial, 78.0))
        far_x = float(np.polyval(polynomial, 24.0))
        lower = -self.fit_max_extrapolation_px
        upper = (PROCESS_WIDTH - 1) + self.fit_max_extrapolation_px
        if not all(lower <= value <= upper for value in (near_x, lookahead_x, far_x)):
            # Three unrelated dash fragments can mathematically fit a very
            # steep parabola.  Reject it and let the current-frame base
            # detector / short dashed-gap bridge make the safe decision.
            self.curve_profile = CurveProfile()
            return None
        # Change of slope between the near and far ROIs is a scale-independent
        # curvature estimate.  Filter it before deciding that an S turn flipped.
        near_slope = float(2.0 * polynomial[0] * 148.0 + polynomial[1])
        far_slope = float(2.0 * polynomial[0] * 24.0 + polynomial[1])
        curvature_raw = near_slope - far_slope
        self._curvature_filtered = (
            self.curvature_filter_alpha * curvature_raw
            + (1.0 - self.curvature_filter_alpha) * self._curvature_filtered
        )
        bend_sign = int(np.sign(self._curvature_filtered)) if abs(self._curvature_filtered) >= self.s_curve_curvature_threshold else 0
        # A single noisy curvature-sign sample must never switch the vehicle
        # into S mode.  Require the filtered flip to persist for several
        # frames, then hold the branch briefly so it cannot flap each frame.
        alternated = False
        if not self.s_curve_preview_enabled:
            # S prediction belongs only to the known post-island solid S
            # section.  Keeping it disabled elsewhere prevents dash/marker
            # geometry from changing an ordinary-line target.
            self._last_high_bend_sign = 0
            self._pending_bend_sign = 0
            self._alternate_hits = 0
            self._s_curve_hold_remaining = 0
        elif bend_sign == 0:
            self._pending_bend_sign = 0
            self._alternate_hits = 0
            if self._s_curve_hold_remaining > 0:
                self._s_curve_hold_remaining -= 1
        elif self._last_high_bend_sign == 0:
            self._last_high_bend_sign = bend_sign
            self._pending_bend_sign = 0
            self._alternate_hits = 0
        elif bend_sign == self._last_high_bend_sign:
            self._pending_bend_sign = 0
            self._alternate_hits = 0
            if self._s_curve_hold_remaining > 0:
                self._s_curve_hold_remaining -= 1
        else:
            if self._pending_bend_sign == bend_sign:
                self._alternate_hits += 1
            else:
                self._pending_bend_sign = bend_sign
                self._alternate_hits = 1
            if self._alternate_hits >= self.s_curve_enter_confirm_frames:
                alternated = True
                self._last_high_bend_sign = bend_sign
                self._pending_bend_sign = 0
                self._alternate_hits = 0
                self._s_curve_hold_remaining = self.s_curve_hold_frames
            elif self._s_curve_hold_remaining > 0:
                self._s_curve_hold_remaining -= 1
        s_curve_active = self._s_curve_hold_remaining > 0
        preview_x = self._expanded_far_preview(self._last_frame) if hasattr(self, "_last_frame") else None
        raw_target = near_x + self.lookahead_gain * (lookahead_x - near_x)
        if s_curve_active:
            raw_target += self.s_curve_lookahead_extra * (far_x - lookahead_x)
            if preview_x is not None:
                raw_target += self.s_curve_lookahead_extra * (preview_x - far_x)
        raw_target = float(np.clip(
            raw_target,
            self.center_x - self.max_offset_px,
            self.center_x + self.max_offset_px,
        ))
        target = self.target_alpha * raw_target + (1.0 - self.target_alpha) * self._sample_target_x
        self._sample_target_x = float(target)
        self._previous_path_x = {
            y: float(np.polyval(polynomial, y)) for y in rows
        }
        self.curve_profile = CurveProfile(
            valid=True, near_x=near_x, far_x=far_x, preview_x=preview_x,
            curvature_raw=curvature_raw, curvature_filtered=self._curvature_filtered,
            bend_sign=bend_sign, alternated=alternated,
            s_curve_active=s_curve_active, sample_count=len(points),
        )
        return float(target), len(points)

    def detect(self, frame_640x480):
        self._last_frame = frame_640x480
        result = super().detect(frame_640x480)
        sampled = self._five_point_black_target(result.mask) if self.curve_fit_enabled else None
        if not self.curve_fit_enabled:
            self._s_curve_hold_remaining = 0
            self._pending_bend_sign = 0
            self._alternate_hits = 0
            self.curve_profile = CurveProfile()
        if sampled is not None:
            sampled_target, sample_count = sampled
            control_x = self.center_x + float(line_follower_module.LINE_TARGET_OFFSET_PX)
            result = replace(
                result,
                valid=True,
                target_x=sampled_target,
                error=float(np.clip((sampled_target - control_x) / self.center_x, -1.0, 1.0)),
                confidence=max(float(result.confidence), min(.96, .42 + .07 * sample_count)),
                reason="eight_point_black_curve_track",
            )
        elif self.s_curve_preview_enabled and self.curve_profile.s_curve_active:
            preview_x = self._expanded_far_preview(frame_640x480)
            if preview_x is not None:
                target = .75 * preview_x + .25 * self._sample_target_x
                self._sample_target_x = float(target)
                self.curve_profile.preview_x = preview_x
                return replace(
                    result, valid=True, target_x=float(target),
                    error=float(np.clip((target - self.center_x) / self.center_x, -1.0, 1.0)),
                    confidence=max(.35, float(result.confidence)), reason="s_curve_far_roi_track",
                )
        if result.target_x is None:
            return result
        if abs(float(result.target_x) - self.center_x) > self.max_offset_px:
            self._reset_to_centre()
            return replace(result, valid=False, confidence=0.0, reason="edge_candidate_reject")
        if result.reason == "white_ratio_reject" and result.confidence >= 0.24:
            self._update_history([result.near, result.mid, result.far], accept=True)
            return replace(result, valid=True, reason="tracking_dark_ratio")
        return result

    def _make_mask(self, roi_bgr):
        hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
        saturation, value = hsv[:, :, 1], hsv[:, :, 2]
        black = (value <= self.value_max) & (saturation <= self.saturation_max)
        mask = (black.astype(np.uint8) * 255)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._noise_kernel)
        # The circular section uses black dashes: close vertically so one dash
        # sequence remains a usable line candidate rather than a lost line.
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._dash_kernel)
        return mask, self.value_max


class RaceController(OutdoorLineFollower):
    """The only RGB-camera and /cmd_vel owner during a competition run."""
    (FOLLOW, STEP_APPROACH, JUMP_WAIT, JUMP_RETREAT, JUMP_RUNUP, JUMP, LAND,
     DASH_APPROACH, ROUNDABOUT_ENTRY, ROUNDABOUT, CROSSWALK_APPROACH,
     CROSSWALK_STOP, FINISHED, ROUNDABOUT_EXIT_SEARCH) = range(14)

    def __init__(self):
        super().__init__()
        defaults = {
            "enable_step_jump": True, "step_arm_delay_s": 1.0,
            "step_arm_timeout_s": 60.0, "step_min_height_m": 0.045,
            "step_max_height_m": 0.105, "step_min_confidence": 0.70,
            "step_min_confirm_frames": 5, "step_min_distance_m": 0.15,
            "step_max_distance_m": 1.30, "step_max_lateral_offset_m": 0.10,
            "step_max_edge_angle_rad": 0.18, "step_takeoff_distance_m": 0.20,
            "step_approach_speed_m_s": 0.20, "step_stop_distance_m": 0.30,
            "jump_confirm_topic": "/race/jump_confirm",
            "jump_retreat_speed_m_s": 0.10, "jump_retreat_seconds": 0.50,
            "jump_runup_speed_m_s": 0.30, "jump_runup_seconds": 1.00,
            "jump_pulse_seconds": 0.20, "jump_forward_speed_m_s": 0.50,
            "landing_forward_speed_ratio": 0.55, "landing_forward_seconds": 1.20,
            "first_turn_required": True, "first_turn_target_rad": 1.5708,
            "first_turn_tolerance_rad": 0.0, "first_turn_center_error": 0.12,
            "first_turn_center_frames": 5,
            "skip_step_after_turns_for_test": 0, "test_turn_angle_rad": 0.95,
            "roundabout_arm_after_turns": 2,
            # Once the two large turns have been observed, decelerate before
            # looking for the two left dashed markers.  This is a distinct
            # profile from the forced low-speed island-entry manoeuvre.
            "roundabout_prepare_speed_m_s": 0.20,
            "roundabout_prepare_min_speed_m_s": 0.14,
            # Before the island entrance is proven, a 0.70rad/s lost-line
            # command can make the robot turn into the forward-converging
            # exit dashes. Keep the preparation turn rate bounded; the
            # committed island manoeuvre has its own command.
            "roundabout_prepare_max_angular_rad_s": 0.40,
            "roundabout_debug_log_seconds": 1.0,
            "crosswalk_enabled": True, "crosswalk_arm_after_step_s": 8.0,
            # Course order is island first, crosswalk later.  Keep visual
            # island dashes from ever being considered zebra bars.
            "crosswalk_require_roundabout_complete": True,
            "crosswalk_confirm_frames": 8, "crosswalk_min_stripes": 5,
            "crosswalk_stop_seconds": 3.0, "crosswalk_black_value_max": 90,
            "crosswalk_min_band_height_px": 3, "crosswalk_roi_top_ratio": 0.64,
            "crosswalk_roi_bottom_ratio": 0.98, "crosswalk_min_dark_ratio": 0.055,
            "crosswalk_row_min_coverage": 0.35, "crosswalk_min_confidence": 0.72,
            # Zebra geometry: each near black bar must be broad, with similar
            # widths and regular gaps.  Ring dashes cannot meet this pattern.
            "crosswalk_min_bar_width_ratio": 0.32,
            "crosswalk_length_ratio_max": 1.65,
            "crosswalk_gap_ratio_max": 2.50,
            "crosswalk_approach_speed_m_s": 0.10,
            "crosswalk_approach_distance_m": 0.06,
            "crosswalk_debug_each_frame": True,
            "stand_height_m": 0.25,
            # Positive pitching is "head down" on this chassis.  Keep the
            # camera aimed toward the nearby centre line while racing.
            "track_height_m": 0.22, "track_pitching_deg": 8.0,
            "roundabout_enabled": True, "roundabout_arm_after_step_s": 5.0,
            "roundabout_ccw_seconds": 5.0, "roundabout_ccw_bias_rad_s": 0.12,
            "start_line_confirm_frames": 4, "start_line_min_band_height_px": 5,
            "line_black_value_max": 105, "line_black_saturation_max": 150,
            "roundabout_dash_close_kernel_height": 17,
            # Keep following a recently confirmed black line across the short
            # gaps in the roundabout's dashed marking.  This is deliberately
            # brief: a true, sustained loss still invokes the normal stop.
            "dashed_line_hold_seconds": 0.38,
            "dashed_line_hold_confidence": 0.38,
            "black_line_memory_frames": 14,
            "black_line_center_corridor_px": 70.0,
            "black_line_max_offset_px": 130.0,
            "black_line_track_jump_px": 78.0,
            "black_line_lookahead_gain": 0.55,
            "black_line_target_alpha": 0.72,
            "black_line_min_curve_points": 3,
            # A three-fragment dashed guide can mathematically produce a
            # wildly extrapolated parabola.  Keep the solid-line fit inside
            # a small image margin and require samples to span real depth.
            "black_line_fit_max_extrapolation_px": 26.0,
            "black_line_min_fit_row_span_px": 42.0,
            # A separate predictive profile for the three sharp solid-line
            # S bends.  It starts only after a *filtered* bend direction flip;
            # therefore straight road and an ordinary single bend keep the
            # inherited controller's original behaviour and parameters.
            "s_curve_enabled": True,
            "s_curve_curvature_filter_alpha": 0.28,
            "s_curve_curvature_threshold": 0.12,
            "s_curve_hold_frames": 18,
            "s_curve_enter_confirm_frames": 3,
            "s_curve_lookahead_extra": 0.35,
            "s_curve_expand_px": 28.0,
            "s_curve_preview_top_ratio": 0.30,
            "s_curve_preview_bottom_ratio": 0.58,
            "s_curve_speed_m_s": 0.20,
            "s_curve_min_speed_m_s": 0.12,
            "s_curve_max_angular_rad_s": 0.95,
            # Race-specific steering is deliberately stronger than the slow
            # classroom follower: at 0.40m/s the S bends need to start
            # turning while the line is still near image centre.
            "steering_kp": 2.60, "steering_kd": 0.065,
            "steering_error_filter_alpha": 0.78,
            "steering_slowdown_start": 0.045,
            "steering_center_deadband": 0.025,
            # Island markers are accepted only when they are close to the
            # robot.  This prevents distant dashed curves from arming entry.
            "roundabout_marker_roi_top_ratio": 0.70,
            "roundabout_marker_roi_bottom_ratio": 0.96,
            "roundabout_marker_min_component_area": 8,
            "roundabout_marker_max_component_area": 260,
            "roundabout_marker_max_component_extent_px": 34,
            "roundabout_marker_cluster_x_px": 50.0,
            "roundabout_marker_cluster_y_px": 90.0,
            "speed_scale": 1.0,
            "step_pass_confirm_seconds": 1.0,
            "step_pass_observation_max_distance_m": 0.70,
            # Both pre-island marks are on the LEFT in this course.  The first
            # is the exit marker; after it clears the camera, the second is
            # the actual island entry marker.
            "roundabout_first_marker_confirm_frames": 5,
            "roundabout_marker_cooldown_seconds": 1.00,
            # The nominal cooldown is expressed at this reference speed.
            # Runtime duration is rescaled to preserve the physical distance
            # driven past the first (exit) marker when speed is changed.
            "roundabout_marker_reference_speed_m_s": 0.08,
            "roundabout_marker_evidence_seconds": 0.45,
            "roundabout_marker_pass_speed_m_s": 0.10,
            "roundabout_marker_clear_frames": 5,
            "roundabout_dash_confirm_frames": 5,
            "roundabout_dash_min_components": 3,
            # The two dashed curves at the exit can otherwise be reported as
            # two separate left markers.  The true entry is also on the left,
            # but is a later, independently approaching dashed cluster.
            "roundabout_entry_min_travel_m": 0.30,
            "roundabout_entry_expected_side": "same",
            "roundabout_entry_approach_min_y_px": 12.0,
            "roundabout_entry_confirm_frames": 8,
            "roundabout_entry_track_max_x_jump_px": 45.0,
            "roundabout_entry_track_max_y_jump_px": 30.0,
            "roundabout_entry_y_jitter_px": 2.5,
            # After the first (exit) mark has cleared, do not let a lost-line
            # recovery spin toward another dashed fragment. This guard uses
            # only a small centred-line correction while waiting for the
            # later, independently confirmed entry mark.
            "roundabout_pre_entry_speed_m_s": 0.08,
            "roundabout_pre_entry_max_angular_rad_s": 0.20,
            "roundabout_pre_entry_max_line_error": 0.20,
            "roundabout_time_mode": False,
            "roundabout_min_start_seconds": 5.0,
            "roundabout_exit_confirm_frames": 5,
            "roundabout_exit_min_seconds": 3.0,
            "roundabout_min_seconds": 6.0,
            "roundabout_exit_solid_frames": 12,
            # The yaw at island entry is treated as the local 0-degree mark.
            # Exit is allowed only after one full CCW heading cycle returns to it.
            "roundabout_use_imu_heading": True,
            "roundabout_full_turn_rad": 6.00,
            "roundabout_heading_return_tolerance_rad": 0.40,
            # Heading alone can reach 360 degrees if the robot spins in
            # place.  Exit additionally requires a minimum chassis travel
            # distance and time measured from the island entry point.
            "roundabout_min_odom_travel_m": 0.50,
            "roundabout_min_complete_seconds": 8.0,
            "roundabout_heading_timeout_s": 35.0,
            "roundabout_ccw_imu_sign": 1.0,
            "roundabout_ccw_bias_rad_s": 0.24,
            # Never let a noisy dashed segment command the robot back in the
            # clockwise direction after it has committed to the island.
            "roundabout_min_ccw_angular_rad_s": 0.16,
            # Smooth the ring-specific angular command.  This is deliberately
            # not applied to the ordinary solid-line follower.
            "roundabout_angular_filter_alpha": 0.35,
            # Once the true entrance is confirmed, ignore every remote solid
            # target and apply the normal left-turn angular speed for this
            # short forced CCW commit before dashed-line tracking starts.
            "roundabout_entry_turn_seconds": 0.50,
            "roundabout_entry_speed_m_s": 0.07,
            "roundabout_entry_angular_rad_s": 0.55,
            "roundabout_lost_continue_seconds": 1.20,
            "roundabout_lost_speed_m_s": 0.06,
            # After a heading-confirmed 360-degree island lap, do not hand a
            # remaining dash straight back to normal line following.  Move
            # straight at low speed until a long solid line is stable.
            "roundabout_exit_search_speed_m_s": 0.08,
            "roundabout_exit_search_timeout_s": 4.0,
            "roundabout_exit_straight_min_seconds": 0.35,
            # Solid-line loss protection uses a robust median of the last
            # real visual samples, then returns to the original loss guard.
            "solid_reacquire_enabled": True,
            "solid_reacquire_history_frames": 5,
            "solid_reacquire_history_seconds": 0.40,
            "solid_reacquire_speed_m_s": 0.08,
            "solid_reacquire_min_speed_m_s": 0.05,
            "solid_reacquire_max_angular_rad_s": 0.55,
            "race_line_target_offset_px": 0.0,
            "debug_screen_width": 800, "debug_screen_height": 480,
            "debug_display_every_n_frames": 3,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
            setattr(self, name, self.get_parameter(name).value)
        self._apply_steering_tuning()
        line_follower_module.LINE_TARGET_OFFSET_PX = float(self.race_line_target_offset_px)
        self.detector = BlackLineDetector(
            self.line_black_value_max,
            self.line_black_saturation_max,
            self.roundabout_dash_close_kernel_height,
            self.black_line_memory_frames,
            self.black_line_center_corridor_px,
            self.black_line_max_offset_px,
            self.black_line_track_jump_px,
            self.black_line_lookahead_gain,
            self.black_line_target_alpha,
            self.black_line_min_curve_points,
            self.s_curve_curvature_filter_alpha,
            self.s_curve_curvature_threshold,
            self.s_curve_hold_frames,
            self.s_curve_enter_confirm_frames,
            self.black_line_fit_max_extrapolation_px,
            self.black_line_min_fit_row_span_px,
            self.s_curve_lookahead_extra,
            self.s_curve_expand_px,
            self.s_curve_preview_top_ratio,
            self.s_curve_preview_bottom_ratio,
        )
        # Keep YAML values as the unscaled baseline.  speed_scale can safely
        # be changed while the controller is running, without restarting it.
        self._base_normal_speed = float(self.normal_speed)
        self._base_min_speed = float(self.min_speed)
        self._apply_speed_scale()
        self.create_subscription(StepDetection, "/step_detector/detection", self._on_step, 10)
        self.create_subscription(Imu, "/imu/data", self._on_imu, 20)
        self.create_subscription(Odometry, "/odom", self._on_odom, 20)
        self.create_subscription(Bool, str(self.jump_confirm_topic), self._on_jump_confirm, 10)
        self.jump_pub = self.create_publisher(Bool, "/cmd_jump", 10)
        self.posture_pub = self.create_publisher(JointState, "/cmd_posture", 10)
        self.latest_step: Optional[StepDetection] = None
        self.latest_step_time = 0.0
        self.step_seen = False
        self.step_seen_time = 0.0
        self.step_detection_armed = not bool(self.first_turn_required)
        self.first_turn_yaw_zero = None
        self.first_turn_heading_complete = False
        self.first_turn_center_hits = 0
        self.jump_confirmed = False
        self.test_turn_reference_yaw = None
        self.test_turn_count = 0
        self.roundabout_turn_ready = False
        self.roundabout_debug_last_time = 0.0
        self.s_curve_debug_last_time = 0.0
        self.crosswalk_debug_last_time = 0.0
        self.crosswalk_gate_last_time = 0.0
        self.started = self.lap_started = time.monotonic()
        self.phase, self.deadline = self.FOLLOW, 0.0
        self.step_complete = self.crosswalk_complete = self.roundabout_complete = False
        self.crosswalk_hits = self.start_line_hits = self.roundabout_dash_hits = self.roundabout_exit_hits = 0
        self.roundabout_solid_hits = 0
        self.roundabout_first_marker_hits = self.roundabout_marker_clear_hits = 0
        self.roundabout_first_marker_seen = self.roundabout_marker_wait_clear = False
        self.roundabout_marker_last_seen = self.roundabout_entry_last_seen = 0.0
        self.roundabout_marker_cooldown_until = 0.0
        self.roundabout_pre_entry_guard_last_time = 0.0
        self.roundabout_marker_odom_last_xy = None
        self.roundabout_marker_travel_m = 0.0
        self.roundabout_first_marker_side = None
        self.roundabout_entry_candidate_start_y = None
        self.roundabout_entry_candidate_last_x = None
        self.roundabout_entry_candidate_last_y = None
        self.roundabout_used_this_lap = False
        self.roundabout_started_at = 0.0
        self.latest_yaw = None
        self.latest_odom_yaw = None
        self.latest_odom_xy = None
        self.latest_heading_yaw = None
        self.latest_odom_time = 0.0
        self.heading_source = "none"
        self.roundabout_heading_reference = None
        self.roundabout_heading_last = None
        self.roundabout_heading_accumulated = 0.0
        self.roundabout_odom_last_xy = None
        self.roundabout_odom_travel_m = 0.0
        self.roundabout_progress_last_time = 0.0
        self.roundabout_angular_filtered = None
        self.roundabout_lost_since = None
        self.roundabout_exit_search_started_at = 0.0
        self.dash_approach_started_at = 0.0
        self.lap, self.waiting_for_start_line = 1, False
        self.last_line_detection: Optional[DetectionResult] = None
        self.last_line_seen_time = 0.0
        self.solid_line_history = deque(maxlen=max(3, int(self.solid_reacquire_history_frames)))
        self.solid_reacquire_started_at = 0.0
        self.add_on_set_parameters_callback(self._on_runtime_parameters)
        self.get_logger().warn("比赛控制器：黑色中线巡线，RGB 和 /cmd_vel 均只由本节点占用")

    def _publish(self, linear, angular):
        """Publish exactly one final command; ring bias is applied before publish."""
        if getattr(self, "phase", None) == self.ROUNDABOUT:
            island_cap = min(
                float(self.max_angular_speed),
                float(self.roundabout_prepare_max_angular_rad_s),
            )
            desired = float(np.clip(
                float(angular) + float(self.roundabout_ccw_bias_rad_s),
                -island_cap, island_cap,
            ))
            # Positive ROS angular-z is counter-clockwise.  Once the entry
            # branch has committed, a dashed fragment may soften the turn but
            # must not reverse it and drive the robot back to the straight.
            desired = max(
                min(island_cap, float(self.roundabout_min_ccw_angular_rad_s)),
                desired,
            )
            angular = self._smooth_roundabout_angular(desired)
        return super()._publish(linear, angular)

    def stop_now(self):
        """Stop safely even when ROS has already begun SIGINT shutdown."""
        try:
            super().stop_now()
        except Exception as error:
            # ROS invalidates publishers while launch processes are stopping.
            # Suppress only that shutdown-time publish failure, never a live
            # controller command error.
            if rclpy.ok():
                self.get_logger().error("停车命令发布失败：%s" % error)

    def _apply_speed_scale(self):
        """Apply a runtime speed scale without exceeding the configured 0.50m/s cap."""
        scale = float(np.clip(self.speed_scale, 0.20, 1.0))
        self.speed_scale = scale
        self.normal_speed = min(0.50, self._base_normal_speed * scale)
        self.min_speed = min(self.normal_speed, self._base_min_speed * scale)

    def _remember_solid_line(self, detection, now):
        """Keep only real current-frame solid-line targets for short recovery."""
        if not detection.valid or detection.target_x is None:
            return
        self.solid_line_history.append((now, float(detection.target_x)))
        self.solid_reacquire_started_at = 0.0

    def _solid_reacquire_detection(self, detection, now):
        """Turn briefly toward the robust recent solid-line direction after loss."""
        if not bool(self.solid_reacquire_enabled):
            return detection
        max_age = max(.05, float(self.solid_reacquire_history_seconds))
        recent = [sample for sample in self.solid_line_history if now - sample[0] <= max_age]
        if len(recent) < min(3, max(3, int(self.solid_reacquire_history_frames))):
            return detection
        target_x = float(np.median([sample[1] for sample in recent]))
        target_x = float(np.clip(
            target_x,
            self.detector.center_x - self.detector.max_offset_px,
            self.detector.center_x + self.detector.max_offset_px,
        ))
        control_x = self.detector.center_x + float(line_follower_module.LINE_TARGET_OFFSET_PX)
        if self.solid_reacquire_started_at <= 0.0:
            self.solid_reacquire_started_at = now
            self.get_logger().warn(
                "黑色实线暂时丢失：使用最近 %d 帧中线方向低速找回（最长 %.2fs）"
                % (len(recent), max_age)
            )
        return replace(
            detection,
            valid=True,
            target_x=target_x,
            error=float(np.clip((target_x - control_x) / self.detector.center_x, -1.0, 1.0)),
            confidence=max(.35, float(detection.confidence)),
            reason="solid_reacquire_history",
        )

    def _apply_steering_tuning(self):
        """Tune the inherited controller only for this high-speed race node."""
        line_follower_module.KP = float(self.steering_kp)
        line_follower_module.KD = float(self.steering_kd)
        line_follower_module.ERROR_FILTER_ALPHA = float(self.steering_error_filter_alpha)
        line_follower_module.ERROR_SLOWDOWN_START = float(self.steering_slowdown_start)

    def _on_runtime_parameters(self, parameters):
        """Allow safe tuning with ros2 param set or race_tuner.py during a run."""
        numeric = {
            "speed_scale", "normal_speed", "min_speed", "max_angular_speed",
            "curve_slowdown", "lost_search_speed", "lost_search_angular",
            "track_height_m", "track_pitching_deg", "roundabout_min_start_seconds",
            "roundabout_prepare_speed_m_s", "roundabout_prepare_min_speed_m_s",
            "roundabout_prepare_max_angular_rad_s",
            "roundabout_marker_pass_speed_m_s", "roundabout_marker_cooldown_seconds",
            "roundabout_pre_entry_speed_m_s", "roundabout_pre_entry_max_angular_rad_s",
            "roundabout_pre_entry_max_line_error",
            "roundabout_min_ccw_angular_rad_s",
            "s_curve_speed_m_s", "s_curve_min_speed_m_s",
            "s_curve_max_angular_rad_s", "roundabout_angular_filter_alpha",
            "solid_reacquire_speed_m_s", "solid_reacquire_min_speed_m_s",
            "solid_reacquire_max_angular_rad_s", "roundabout_exit_search_speed_m_s",
            "steering_kp", "steering_kd", "steering_error_filter_alpha",
            "steering_slowdown_start",
        }
        updates = {parameter.name: parameter.value for parameter in parameters if parameter.name in numeric}
        if not updates:
            return SetParametersResult(successful=True)
        try:
            scale = float(updates.get("speed_scale", self.speed_scale))
            base_normal = float(updates.get("normal_speed", self._base_normal_speed))
            base_min = float(updates.get("min_speed", self._base_min_speed))
            max_angular = float(updates.get("max_angular_speed", self.max_angular_speed))
            prepare_speed = float(updates.get("roundabout_prepare_speed_m_s", self.roundabout_prepare_speed_m_s))
            prepare_min_speed = float(updates.get("roundabout_prepare_min_speed_m_s", self.roundabout_prepare_min_speed_m_s))
            prepare_max_angular = float(updates.get("roundabout_prepare_max_angular_rad_s", self.roundabout_prepare_max_angular_rad_s))
            marker_pass_speed = float(updates.get("roundabout_marker_pass_speed_m_s", self.roundabout_marker_pass_speed_m_s))
            marker_cooldown = float(updates.get("roundabout_marker_cooldown_seconds", self.roundabout_marker_cooldown_seconds))
            pre_entry_speed = float(updates.get("roundabout_pre_entry_speed_m_s", self.roundabout_pre_entry_speed_m_s))
            pre_entry_max_angular = float(updates.get("roundabout_pre_entry_max_angular_rad_s", self.roundabout_pre_entry_max_angular_rad_s))
            pre_entry_max_error = float(updates.get("roundabout_pre_entry_max_line_error", self.roundabout_pre_entry_max_line_error))
            ring_min_ccw = float(updates.get("roundabout_min_ccw_angular_rad_s", self.roundabout_min_ccw_angular_rad_s))
            s_curve_speed = float(updates.get("s_curve_speed_m_s", self.s_curve_speed_m_s))
            s_curve_min_speed = float(updates.get("s_curve_min_speed_m_s", self.s_curve_min_speed_m_s))
            s_curve_max_angular = float(updates.get("s_curve_max_angular_rad_s", self.s_curve_max_angular_rad_s))
            ring_filter_alpha = float(updates.get("roundabout_angular_filter_alpha", self.roundabout_angular_filter_alpha))
            solid_reacquire_speed = float(updates.get("solid_reacquire_speed_m_s", self.solid_reacquire_speed_m_s))
            solid_reacquire_min_speed = float(updates.get("solid_reacquire_min_speed_m_s", self.solid_reacquire_min_speed_m_s))
            solid_reacquire_max_angular = float(updates.get("solid_reacquire_max_angular_rad_s", self.solid_reacquire_max_angular_rad_s))
            exit_search_speed = float(updates.get("roundabout_exit_search_speed_m_s", self.roundabout_exit_search_speed_m_s))
            height = float(updates.get("track_height_m", self.track_height_m))
            steering_kp = float(updates.get("steering_kp", self.steering_kp))
            steering_kd = float(updates.get("steering_kd", self.steering_kd))
            steering_alpha = float(updates.get("steering_error_filter_alpha", self.steering_error_filter_alpha))
            slowdown_start = float(updates.get("steering_slowdown_start", self.steering_slowdown_start))
        except (TypeError, ValueError):
            return SetParametersResult(successful=False, reason="调参值必须是数字")
        if not 0.20 <= scale <= 1.0:
            return SetParametersResult(successful=False, reason="speed_scale 必须在 0.20 到 1.00")
        if not 0.02 <= base_min <= base_normal <= 0.50:
            return SetParametersResult(successful=False, reason="速度必须满足 0.02 <= min_speed <= normal_speed <= 0.50")
        if not 0.10 <= max_angular <= 1.20:
            return SetParametersResult(successful=False, reason="max_angular_speed 必须在 0.10 到 1.20")
        if not 0.05 <= prepare_min_speed <= prepare_speed <= 0.50:
            return SetParametersResult(successful=False, reason="环岛准备速度必须满足 0.05 <= min <= speed <= 0.50")
        if not 0.10 <= prepare_max_angular <= 1.20:
            return SetParametersResult(successful=False, reason="环岛准备角速度必须在 0.10 到 1.20")
        if not 0.03 <= marker_pass_speed <= 0.20:
            return SetParametersResult(successful=False, reason="首次环岛标记通过速度必须在 0.03 到 0.20")
        if not 0.20 <= marker_cooldown <= 3.00:
            return SetParametersResult(successful=False, reason="首次标记名义通过时间必须在 0.20 到 3.00")
        if not 0.03 <= pre_entry_speed <= 0.20:
            return SetParametersResult(successful=False, reason="环岛入口前保护速度必须在 0.03 到 0.20")
        if not 0.05 <= pre_entry_max_angular <= 0.50:
            return SetParametersResult(successful=False, reason="环岛入口前保护角速度必须在 0.05 到 0.50")
        if not 0.05 <= pre_entry_max_error <= 0.50:
            return SetParametersResult(successful=False, reason="环岛入口前保护误差阈值必须在 0.05 到 0.50")
        if not 0.05 <= ring_min_ccw <= min(max_angular, prepare_max_angular):
            return SetParametersResult(successful=False, reason="环岛最小逆时针角速度必须不大于环岛角速度上限")
        if not 0.05 <= s_curve_min_speed <= s_curve_speed <= 0.50:
            return SetParametersResult(successful=False, reason="S 弯速度必须满足 0.05 <= min <= speed <= 0.50")
        if not 0.10 <= s_curve_max_angular <= 1.20:
            return SetParametersResult(successful=False, reason="S 弯角速度必须在 0.10 到 1.20")
        if not 0.05 <= ring_filter_alpha <= 0.95:
            return SetParametersResult(successful=False, reason="环岛角速度平滑系数必须在 0.05 到 0.95")
        if not 0.03 <= solid_reacquire_min_speed <= solid_reacquire_speed <= 0.25:
            return SetParametersResult(successful=False, reason="实线找回速度必须满足 0.03 <= min <= speed <= 0.25")
        if not 0.10 <= solid_reacquire_max_angular <= 0.90:
            return SetParametersResult(successful=False, reason="实线找回角速度必须在 0.10 到 0.90")
        if not 0.03 <= exit_search_speed <= 0.20:
            return SetParametersResult(successful=False, reason="环岛出口直行速度必须在 0.03 到 0.20")
        if not 0.14 <= height <= 0.36:
            return SetParametersResult(successful=False, reason="track_height_m 必须在 0.14 到 0.36")
        if not 0.40 <= steering_kp <= 3.00 or not 0.0 <= steering_kd <= 0.30:
            return SetParametersResult(successful=False, reason="steering_kp/KD 超出安全范围")
        if not 0.30 <= steering_alpha <= 1.0 or not 0.02 <= slowdown_start <= 0.40:
            return SetParametersResult(successful=False, reason="巡线滤波或弯道减速阈值超出范围")
        self._base_normal_speed, self._base_min_speed, self.speed_scale = base_normal, base_min, scale
        self._apply_speed_scale()
        for name, value in updates.items():
            if name not in {"speed_scale", "normal_speed", "min_speed"}:
                setattr(self, name, float(value))
        self._apply_steering_tuning()
        self.get_logger().info(
            "运行时调参已生效：v=%.3f m/s (scale=%.2f), min=%.3f, w_max=%.2f, S=%.2f/%.2f, height=%.2f"
            % (self.normal_speed, self.speed_scale, self.min_speed,
               self.max_angular_speed, self.s_curve_speed_m_s,
               self.s_curve_max_angular_rad_s, self.track_height_m)
        )
        return SetParametersResult(successful=True)

    def _on_step(self, message):
        now = time.monotonic()
        self.latest_step, self.latest_step_time = message, now
        if (self.step_detection_armed and message.detected and message.confidence >= float(self.step_min_confidence)
                and float(self.step_min_height_m) <= message.height <= float(self.step_max_height_m)
                and message.distance <= float(self.step_pass_observation_max_distance_m)):
            self.step_seen = True
            self.step_seen_time = now

    @staticmethod
    def _wrap_angle(angle):
        return float(np.arctan2(np.sin(angle), np.cos(angle)))

    @staticmethod
    def _quaternion_yaw(orientation):
        return float(np.arctan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        ))

    def _accept_heading(self, yaw, source):
        """Use odometry yaw when available; IMU is only a short-gap fallback."""
        self.latest_heading_yaw = yaw
        self.heading_source = source
        if self.first_turn_yaw_zero is None:
            self.first_turn_yaw_zero = yaw
        self._update_step_bypass_for_test(yaw)
        if self.roundabout_heading_reference is not None:
            if self.roundabout_heading_last is not None:
                self.roundabout_heading_accumulated += self._wrap_angle(
                    yaw - self.roundabout_heading_last
                )
            self.roundabout_heading_last = yaw

    def _on_imu(self, message):
        yaw = self._quaternion_yaw(message.orientation)
        self.latest_yaw = yaw
        if time.monotonic() - self.latest_odom_time > 0.50:
            self._accept_heading(yaw, "imu_fallback")

    def _on_odom(self, message):
        yaw = self._quaternion_yaw(message.pose.pose.orientation)
        position = message.pose.pose.position
        xy = (float(position.x), float(position.y))
        self.latest_odom_xy = xy
        if getattr(self, "roundabout_odom_last_xy", None) is not None:
            dx = xy[0] - self.roundabout_odom_last_xy[0]
            dy = xy[1] - self.roundabout_odom_last_xy[1]
            delta = float(np.hypot(dx, dy))
            # Ignore a zero increment and implausible pose resets.  Normal
            # 20Hz wheel odometry changes are a few millimetres per frame.
            if 0.0005 <= delta <= 0.20:
                self.roundabout_odom_travel_m += delta
            self.roundabout_odom_last_xy = xy
        if getattr(self, "roundabout_marker_odom_last_xy", None) is not None:
            dx = xy[0] - self.roundabout_marker_odom_last_xy[0]
            dy = xy[1] - self.roundabout_marker_odom_last_xy[1]
            delta = float(np.hypot(dx, dy))
            if 0.0005 <= delta <= 0.20:
                self.roundabout_marker_travel_m += delta
            self.roundabout_marker_odom_last_xy = xy
        self.latest_odom_yaw = yaw
        self.latest_odom_time = time.monotonic()
        self._accept_heading(yaw, "odom")

    def _update_step_bypass_for_test(self, yaw):
        """Count right-angle turns for the independent island-entry gate."""
        step_turns = int(self.skip_step_after_turns_for_test)
        island_turns = int(self.roundabout_arm_after_turns)
        required_turns = max(step_turns, island_turns)
        # Each lap has only the two deliberately selected large turns used as
        # a gate.  Do not let later S-curves become a third/fourth "turn" and
        # reopen or perturb the roundabout state.
        if required_turns <= 0 or self.test_turn_count >= required_turns:
            return
        if self.test_turn_reference_yaw is None:
            self.test_turn_reference_yaw = yaw
            return
        if abs(self._wrap_angle(yaw - self.test_turn_reference_yaw)) < float(self.test_turn_angle_rad):
            return
        self.test_turn_count += 1
        self.test_turn_reference_yaw = yaw
        self.get_logger().info(
            "无台阶测试：已确认第 %d/%d 个约 90° 转弯"
            % (self.test_turn_count, required_turns)
        )
        if step_turns > 0 and self.test_turn_count >= step_turns and not self.step_complete:
            self.step_complete = True
            self.get_logger().warn(
                "无台阶测试模式：第二次转弯后已解锁环岛与人行道检测；真实比赛请关闭该模式"
            )
        if island_turns > 0 and self.test_turn_count >= island_turns and not self.roundabout_turn_ready:
            self.roundabout_turn_ready = True
            self.get_logger().warn(
                "第二次转弯已完成：已解锁环岛入口检测，并进入环岛准备低速档 "
                "v<=%.2fm/s、w<=%.2frad/s"
                % (float(self.roundabout_prepare_speed_m_s), float(self.roundabout_prepare_max_angular_rad_s))
            )

    def _log_roundabout_status(self, now, ready, side=None):
        """Expose ring-gate and dashed-marker decisions in the SSH terminal."""
        if now - self.roundabout_debug_last_time < float(self.roundabout_debug_log_seconds):
            return
        self.roundabout_debug_last_time = now
        required = int(self.roundabout_arm_after_turns)
        if not ready:
            delta = 0.0 if self.latest_heading_yaw is None or self.test_turn_reference_yaw is None else abs(
                self._wrap_angle(self.latest_heading_yaw - self.test_turn_reference_yaw)
            )
            self.get_logger().info(
                "RING_GATE_WAIT turns=%d/%d yaw_delta=%.2frad source=%s：尚未开启左侧虚线检测"
                % (self.test_turn_count, required, delta, self.heading_source)
            )
            return
        stage = "首次出口标记" if not self.roundabout_first_marker_seen else (
            "等待首次标记离开" if self.roundabout_marker_wait_clear else "等待同侧入口标记"
        )
        self.get_logger().info(
            "RING_DETECT_ON side=%s stage=%s first_hits=%d entry_hits=%d"
            % (side or "none", stage, self.roundabout_first_marker_hits, self.roundabout_dash_hits)
        )

    def _roundabout_marker_pass_profile(self):
        """Return a speed and duration that preserve first-marker clearance."""
        pass_speed = max(0.03, float(self.roundabout_marker_pass_speed_m_s))
        reference_speed = max(0.03, float(self.roundabout_marker_reference_speed_m_s))
        nominal_seconds = max(0.10, float(self.roundabout_marker_cooldown_seconds))
        distance = nominal_seconds * reference_speed
        return pass_speed, distance / pass_speed, distance

    def _roundabout_pre_entry_guard_active(self):
        """Protect the straight segment between the exit mark and true entry.

        This is a state condition rather than a timed blind drive: it ends as
        soon as the later dash cluster passes all visual checks and
        ``_begin_roundabout_entry`` changes phase.
        """
        return (
            self.phase == self.FOLLOW
            and bool(self.roundabout_first_marker_seen)
            and not bool(self.roundabout_marker_wait_clear)
            and not bool(self.roundabout_complete)
            and not bool(self.roundabout_used_this_lap)
        )

    def _roundabout_pre_entry_guard_control(self, detection, now):
        """Hold course gently instead of invoking normal lost-line search.

        The short dash fragments at the fork are not a reliable solid-line
        target. A current, nearly centred line may still make a small
        correction; an invalid or off-centre fragment is ignored rather than
        turning the chassis sharply toward it.
        """
        target_angular = 0.0
        usable_line = (
            detection.valid
            and np.isfinite(float(detection.error))
            and abs(float(detection.error)) <= float(self.roundabout_pre_entry_max_line_error)
        )
        if usable_line:
            target_angular = float(np.clip(
                float(line_follower_module.STEERING_SIGN)
                * float(self.steering_kp) * float(detection.error),
                -float(self.roundabout_pre_entry_max_angular_rad_s),
                float(self.roundabout_pre_entry_max_angular_rad_s),
            ))
        else:
            # Avoid a stale derivative/integral kick when the car later
            # commits to the ring or returns to ordinary solid tracking.
            self.previous_error = None
            self.filtered_error = None
            self.filtered_d_error = 0.0
            self.error_integral *= 0.85
        if now - self.roundabout_pre_entry_guard_last_time >= 0.75:
            self.roundabout_pre_entry_guard_last_time = now
            self.get_logger().info(
                "RING_ENTRY_GUARD：等待后续同侧入口，v=%.2f w<=%.2f valid=%s err=%+.2f travel=%.2fm"
                % (float(self.roundabout_pre_entry_speed_m_s),
                   float(self.roundabout_pre_entry_max_angular_rad_s),
                   "yes" if usable_line else "no", float(detection.error),
                   self.roundabout_marker_travel_m)
            )
        linear, angular = self._publish_smooth(
            float(self.roundabout_pre_entry_speed_m_s), target_angular,
            1 / CONTROL_RATE_HZ,
        )
        return linear, angular, 0.0, "ROUNDABOUT_PRE_ENTRY_GUARD"

    def _begin_roundabout_entry(self, now, reason):
        """Capture the entry heading as relative 0 degrees, then turn CCW."""
        self.phase = self.ROUNDABOUT_ENTRY
        self.roundabout_used_this_lap = True
        self.deadline = now + float(self.roundabout_entry_turn_seconds)
        self.roundabout_heading_reference = self.latest_heading_yaw
        self.roundabout_heading_last = self.latest_heading_yaw
        self.roundabout_heading_accumulated = 0.0
        self.roundabout_odom_last_xy = self.latest_odom_xy
        self.roundabout_odom_travel_m = 0.0
        self.roundabout_progress_last_time = 0.0
        self.roundabout_angular_filtered = None
        if bool(self.roundabout_use_imu_heading) and self.latest_heading_yaw is not None:
            self.get_logger().warn("%s；已记录环岛入口相对航向 0°（%s）" % (reason, self.heading_source))
        else:
            self.get_logger().warn("%s；未收到 IMU 航向，将使用黑实线退出兜底" % reason)

    def _smooth_roundabout_angular(self, desired_angular):
        """Avoid sharp yaw-command steps while following the dashed ring."""
        alpha = float(np.clip(self.roundabout_angular_filter_alpha, .05, .95))
        desired = float(desired_angular)
        if self.roundabout_angular_filtered is None:
            self.roundabout_angular_filtered = desired
        else:
            self.roundabout_angular_filtered = (
                alpha * desired + (1.0 - alpha) * self.roundabout_angular_filtered
            )
        return float(self.roundabout_angular_filtered)

    def _roundabout_heading_complete(self, now):
        if (not bool(self.roundabout_use_imu_heading)
                or self.roundabout_heading_reference is None
                or self.latest_heading_yaw is None):
            return False
        signed_turn = float(self.roundabout_ccw_imu_sign) * self.roundabout_heading_accumulated
        returned = abs(self._wrap_angle(self.latest_heading_yaw - self.roundabout_heading_reference))
        heading_cycle = (signed_turn >= float(self.roundabout_full_turn_rad)
                         and returned <= float(self.roundabout_heading_return_tolerance_rad))
        elapsed = now - self.roundabout_started_at
        # Do not allow a lifted wheel or a short self-spin to masquerade as
        # a completed island lap.  Absence of planar odometry is deliberately
        # treated as unsafe and remains in the ring until the timeout stop.
        travel_cycle = (
            self.roundabout_odom_last_xy is not None
            and self.roundabout_odom_travel_m >= float(self.roundabout_min_odom_travel_m)
        )
        return (heading_cycle
                and elapsed >= float(self.roundabout_min_complete_seconds)
                and travel_cycle)

    def _log_roundabout_progress(self, now):
        """Expose the three independent exit checks for field diagnosis."""
        if now - self.roundabout_progress_last_time < 1.0:
            return
        self.roundabout_progress_last_time = now
        signed_turn = float(self.roundabout_ccw_imu_sign) * self.roundabout_heading_accumulated
        returned = (0.0 if self.roundabout_heading_reference is None or self.latest_heading_yaw is None
                    else abs(self._wrap_angle(self.latest_heading_yaw - self.roundabout_heading_reference)))
        self.get_logger().info(
            "ROUNDABOUT_PROGRESS turn=%.2f/%.2frad return=%.2f/%.2frad travel=%.2f/%.2fm elapsed=%.1f/%.1fs"
            % (signed_turn, float(self.roundabout_full_turn_rad), returned,
               float(self.roundabout_heading_return_tolerance_rad),
               self.roundabout_odom_travel_m, float(self.roundabout_min_odom_travel_m),
               now - self.roundabout_started_at, float(self.roundabout_min_complete_seconds))
        )

    def _begin_roundabout_exit_search(self, now, reason):
        """Lock ring re-entry, then leave the island only on stable solid-line evidence."""
        self.phase = self.ROUNDABOUT_EXIT_SEARCH
        # Mark it complete immediately: no island/crosswalk gate may react to
        # another dashed ring fragment while we leave the island.
        self.roundabout_complete = True
        self.roundabout_exit_search_started_at = now
        self.roundabout_solid_hits = 0
        self.roundabout_angular_filtered = None
        self.last_line_detection = None
        self.last_line_seen_time = 0.0
        self.solid_line_history.clear()
        self.solid_reacquire_started_at = 0.0
        self.detector._reset_to_centre()
        self.get_logger().warn(
            "%s；环岛入口方向为相对 0°，现锁定环岛识别并低速直行寻找黑色实线"
            % reason
        )

    def _roundabout_exit_search_control(self, detection, frame, now):
        """Drive straight out of a completed island; never follow its dashes again."""
        elapsed = now - self.roundabout_exit_search_started_at
        solid = self._solid_line_after_roundabout_seen(frame)
        self.roundabout_solid_hits = self.roundabout_solid_hits + 1 if solid else 0
        if (elapsed >= float(self.roundabout_exit_straight_min_seconds)
                and detection.valid
                and self.roundabout_solid_hits >= int(self.roundabout_exit_solid_frames)):
            self.phase = self.FOLLOW
            self.roundabout_solid_hits = 0
            self.get_logger().info("环岛出口已连续确认黑色实线，恢复普通巡线")
            linear, angular, derivative, state = self._control(detection, now)
            return linear, angular, derivative, "ROUNDABOUT_EXIT_SOLID_" + state
        if elapsed >= float(self.roundabout_exit_search_timeout_s):
            self.stop_now()
            return 0.0, 0.0, 0.0, "ROUNDABOUT_EXIT_SOLID_NOT_FOUND_STOP"
        linear, angular = self._publish_smooth(
            float(self.roundabout_exit_search_speed_m_s), 0.0, 1 / CONTROL_RATE_HZ,
        )
        return linear, angular, 0.0, "ROUNDABOUT_EXIT_STRAIGHT_SEARCH"

    def _step_ok(self, now):
        """Permit one jump only in early course and only for a confirmed 5-10cm step."""
        s = self.latest_step
        if not (self.step_detection_armed and bool(self.enable_step_jump) and not self.step_complete and s): return False
        if not (float(self.step_arm_delay_s) <= now-self.lap_started <= float(self.step_arm_timeout_s)): return False
        if now-self.latest_step_time > .35 or not s.detected: return False
        if s.confirm_frames < max(int(self.step_min_confirm_frames), s.required_confirm_frames): return False
        if not float(self.step_min_height_m) <= s.height <= float(self.step_max_height_m): return False
        if not float(self.step_min_distance_m) <= s.distance <= float(self.step_max_distance_m): return False
        if s.confidence < float(self.step_min_confidence) or not s.pose_valid: return False
        return abs(s.lateral_offset_m) <= float(self.step_max_lateral_offset_m) and abs(s.edge_angle_rad) <= float(self.step_max_edge_angle_rad)

    def _crosswalk_bands(self, frame):
        """Measure near zebra geometry as (bars, confidence, dark, length_ratio, gap_ratio).

        Borrowing the useful part of the reference Zebra.cpp: a real crosswalk
        has repeated, broad, parallel bars with comparable lengths and regular
        gaps.  The island has short curved dashes, so it fails these geometric
        tests even before the separate state gate blocks crosswalk detection.
        """
        top = float(np.clip(self.crosswalk_roi_top_ratio, .45, .90))
        bottom = float(np.clip(self.crosswalk_roi_bottom_ratio, top + .05, 1.0))
        roi = frame[int(top * IMAGE_HEIGHT):int(bottom * IMAGE_HEIGHT)]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        dark = (((hsv[:, :, 2] <= int(self.crosswalk_black_value_max)) &
                 (hsv[:, :, 1] <= int(self.line_black_saturation_max))).astype(np.uint8) * 255)
        kernel = np.ones((3, 3), dtype=np.uint8)
        dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN, kernel)
        dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, kernel)
        binary = dark > 0
        dark_ratio = float(binary.mean())
        if not float(self.crosswalk_min_dark_ratio) <= dark_ratio <= .65:
            return 0, 0.0, dark_ratio, 0.0, 0.0

        # A row is valid only if one broad horizontal strip covers enough of
        # the image.  This rejects isolated island dashes and the centre line.
        covered_rows = binary.mean(axis=1) >= float(self.crosswalk_row_min_coverage)
        row_groups, start = [], None
        for index, covered in enumerate(covered_rows):
            if covered and start is None:
                start = index
            elif not covered and start is not None:
                if index - start >= int(self.crosswalk_min_band_height_px):
                    row_groups.append((start, index - 1))
                start = None
        if start is not None and len(covered_rows) - start >= int(self.crosswalk_min_band_height_px):
            row_groups.append((start, len(covered_rows) - 1))

        required = int(self.crosswalk_min_stripes)
        raw_bars = len(row_groups)
        best = None
        # Test every consecutive five-bar window.  This tolerates seeing only
        # part of the physical zebra while rejecting arbitrary sparse marks.
        for first in range(0, max(0, raw_bars - required + 1)):
            groups = row_groups[first:first + required]
            span = groups[-1][1] - groups[0][0]
            if span < .20 * dark.shape[0]:
                continue
            widths, coverages = [], []
            for top_y, bottom_y in groups:
                bar = binary[top_y:bottom_y + 1]
                occupied_x = np.flatnonzero(bar.any(axis=0))
                if occupied_x.size == 0:
                    widths.append(0.0)
                else:
                    widths.append((occupied_x[-1] - occupied_x[0] + 1) / float(dark.shape[1]))
                coverages.append(float(bar.mean()))
            gaps = [
                groups[index + 1][0] - groups[index][1] - 1
                for index in range(len(groups) - 1)
            ]
            if (min(widths) < float(self.crosswalk_min_bar_width_ratio)
                    or min(gaps) <= 0):
                continue
            length_ratio = max(widths) / max(1.0e-6, min(widths))
            gap_ratio = max(gaps) / max(1.0, min(gaps))
            if (length_ratio > float(self.crosswalk_length_ratio_max)
                    or gap_ratio > float(self.crosswalk_gap_ratio_max)):
                continue
            # Prefer the most uniform five-bar sequence when more than five
            # bars are visible in the lower ROI.
            uniformity = length_ratio + gap_ratio
            candidate = (uniformity, widths, coverages, length_ratio, gap_ratio)
            if best is None or candidate[0] < best[0]:
                best = candidate

        if best is None:
            return raw_bars, 0.0, dark_ratio, 0.0, 0.0
        _uniformity, widths, coverages, length_ratio, gap_ratio = best
        coverage = float(np.median(coverages))
        stripe_score = min(1.0, raw_bars / max(1, required))
        coverage_score = min(1.0, coverage / max(.01, float(self.crosswalk_row_min_coverage)))
        darkness_score = min(1.0, dark_ratio / max(.06, 2.0 * float(self.crosswalk_min_dark_ratio)))
        length_score = float(np.clip(
            (float(self.crosswalk_length_ratio_max) - length_ratio)
            / max(.01, float(self.crosswalk_length_ratio_max) - 1.0), 0.0, 1.0
        ))
        gap_score = float(np.clip(
            (float(self.crosswalk_gap_ratio_max) - gap_ratio)
            / max(.01, float(self.crosswalk_gap_ratio_max) - 1.0), 0.0, 1.0
        ))
        confidence = (.40 * stripe_score + .22 * coverage_score + .10 * darkness_score
                      + .14 * length_score + .14 * gap_score)
        return raw_bars, float(confidence), dark_ratio, length_ratio, gap_ratio

    def _control(self, detection, now):
        """Track solid lines; activate a predictive profile only for tight S bends."""
        solid_recovery_profile = detection.reason.startswith("solid_reacquire_history")
        if detection.valid and abs(float(detection.error)) <= float(self.steering_center_deadband):
            self.previous_error = 0.0
            self.filtered_error = 0.0
            self.filtered_d_error = 0.0
            self.error_integral = 0.0
            detection = replace(detection, error=0.0, reason=detection.reason + "_centred")
        # After the second confirmed 90-degree turn, the next two left-side
        # dashed groups are physically close together.  Cap both v and w in
        # FOLLOW/ROUNDABOUT so the vehicle cannot pass an entry during one
        # camera confirmation window, while preserving a similar turn radius.
        slow_island_profile = (
            self.roundabout_turn_ready and not self.roundabout_complete
            and self.phase in (self.FOLLOW, self.DASH_APPROACH, self.ROUNDABOUT)
        )
        crosswalk_align_profile = self.phase == self.CROSSWALK_APPROACH
        s_curve_profile = (
            bool(self.s_curve_enabled)
            and not slow_island_profile
            and not solid_recovery_profile
            and self.phase == self.FOLLOW
            and self.detector.curve_profile.s_curve_active
        )
        if (not slow_island_profile and not crosswalk_align_profile
                and not solid_recovery_profile and not s_curve_profile):
            return super()._control(detection, now)
        saved_normal, saved_min, saved_angular = (
            self.normal_speed, self.min_speed, self.max_angular_speed
        )
        if crosswalk_align_profile:
            self.normal_speed = min(float(saved_normal), float(self.crosswalk_approach_speed_m_s))
            self.min_speed = min(self.normal_speed, float(saved_min), 0.05)
            self.max_angular_speed = min(float(saved_angular), 0.45)
        elif slow_island_profile:
            self.normal_speed = min(float(saved_normal), float(self.roundabout_prepare_speed_m_s))
            self.min_speed = min(
                self.normal_speed,
                float(saved_min),
                float(self.roundabout_prepare_min_speed_m_s),
            )
            self.max_angular_speed = min(
                float(saved_angular), float(self.roundabout_prepare_max_angular_rad_s)
            )
        elif solid_recovery_profile:
            # The target is only a short history-based direction, never a new
            # visual observation.  Keep it deliberately slower than normal;
            # on expiry the inherited SEARCH/LOST_TIMEOUT safeguard resumes.
            self.normal_speed = min(float(saved_normal), float(self.solid_reacquire_speed_m_s))
            self.min_speed = min(
                self.normal_speed,
                float(saved_min),
                float(self.solid_reacquire_min_speed_m_s),
            )
            self.max_angular_speed = min(
                float(saved_angular), float(self.solid_reacquire_max_angular_rad_s)
            )
        else:
            # Target anticipation and the expanded far ROI have already been
            # applied by BlackLineDetector.  This branch only reduces v early
            # enough for that prediction to take effect and never changes the
            # normal/ordinary-bend controller outside S mode.
            self.normal_speed = min(float(saved_normal), float(self.s_curve_speed_m_s))
            self.min_speed = min(
                self.normal_speed,
                float(saved_min),
                float(self.s_curve_min_speed_m_s),
            )
            self.max_angular_speed = min(
                float(saved_angular), float(self.s_curve_max_angular_rad_s)
            )
        try:
            result = super()._control(detection, now)
        finally:
            self.normal_speed, self.min_speed, self.max_angular_speed = (
                saved_normal, saved_min, saved_angular
            )
        if solid_recovery_profile:
            linear, angular, derivative, _state = result
            return linear, angular, derivative, "SOLID_REACQUIRE_HISTORY"
        if s_curve_profile:
            linear, angular, derivative, state = result
            return linear, angular, derivative, "S_CURVE_" + state
        return result

    def _roundabout_dashes_side(self, frame):
        """Return only a *near* cluster of short dashed island-marker strokes."""
        top = float(np.clip(self.roundabout_marker_roi_top_ratio, .45, .90))
        bottom = float(np.clip(self.roundabout_marker_roi_bottom_ratio, top + .05, 1.0))
        roi = frame[int(top * IMAGE_HEIGHT):int(bottom * IMAGE_HEIGHT)]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        dark = (((hsv[:, :, 2] <= int(self.line_black_value_max)) &
                 (hsv[:, :, 1] <= int(self.line_black_saturation_max))).astype(np.uint8) * 255)
        dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN, np.ones((3, 3), dtype=np.uint8))
        count, _labels, stats, centers = cv2.connectedComponentsWithStats(dark, connectivity=8)
        points = []
        for label in range(1, count):
            area = int(stats[label, cv2.CC_STAT_AREA])
            width = int(stats[label, cv2.CC_STAT_WIDTH])
            height = int(stats[label, cv2.CC_STAT_HEIGHT])
            extent = max(width, height)
            aspect = extent / max(1.0, min(width, height))
            if (int(self.roundabout_marker_min_component_area) <= area <= int(self.roundabout_marker_max_component_area)
                    and 2 <= width <= int(self.roundabout_marker_max_component_extent_px)
                    and 2 <= height <= int(self.roundabout_marker_max_component_extent_px)
                    and aspect <= 4.5 and centers[label, 1] >= .40 * roi.shape[0]):
                points.append((float(centers[label, 0]), float(centers[label, 1])))
        if not points:
            return None
        clusters = []
        for x, y in points:
            nearby = [
                (other_x, other_y) for other_x, other_y in points
                if (abs(other_x - x) <= float(self.roundabout_marker_cluster_x_px)
                    and abs(other_y - y) <= float(self.roundabout_marker_cluster_y_px))
            ]
            clusters.append((len(nearby), float(np.mean([p[0] for p in nearby])),
                             float(np.mean([p[1] for p in nearby]))))
        count, center_x, center_y = max(clusters, key=lambda item: item[0])
        if count < int(self.roundabout_dash_min_components):
            return None
        width = roi.shape[1]
        if center_x < .45 * width:
            side = "left"
        elif center_x > .55 * width:
            side = "right"
        else:
            side = "center"
        return RingMarker(side, center_x, center_y, int(count))

    def _roundabout_dashes_seen(self, frame):
        return self._roundabout_dashes_side(frame) is not None

    def _roundabout_entry_marker_matches(self, marker):
        """Select the configured visual side relative to the first exit mark.

        On this course both the exit and the true entrance are left-side
        dash clusters.  The two are separated by travel and a mandatory
        visual-clear interval; the later cluster must then approach the
        camera continuously.  Heading is deliberately not an entrance gate.
        """
        if marker is None or marker.side not in ("left", "right"):
            return False
        expected = str(self.roundabout_entry_expected_side).strip().lower()
        first_side = self.roundabout_first_marker_side
        if expected == "same":
            return first_side in ("left", "right") and marker.side == first_side
        if expected == "opposite":
            return first_side in ("left", "right") and marker.side != first_side
        return marker.side == expected

    def _roundabout_junction_seen(self, frame):
        """Find a second black direction at the dashed-guide junction."""
        roi = frame[int(.50 * IMAGE_HEIGHT):int(.96 * IMAGE_HEIGHT)]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = (((hsv[:, :, 2] <= int(self.line_black_value_max)) &
                 (hsv[:, :, 1] <= int(self.line_black_saturation_max))).astype(np.uint8) * 255)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                                cv2.getStructuringElement(cv2.MORPH_RECT, (3, 11)))
        width = mask.shape[1]
        mask[:, :int(.15 * width)] = 0
        mask[:, int(.85 * width):] = 0
        lines = cv2.HoughLinesP(mask, 1, np.pi / 180.0, 14,
                                minLineLength=15, maxLineGap=9)
        if lines is None:
            return False
        angles = []
        for x1, y1, x2, y2 in lines.reshape(-1, 4):
            angle = abs(float(np.degrees(np.arctan2(y2 - y1, x2 - x1))))
            angles.append(angle if angle <= 90.0 else 180.0 - angle)
        return any(abs(first - second) >= 25.0 for first in angles for second in angles)

    def _solid_line_after_roundabout_seen(self, frame):
        """Require a genuinely long unbroken black line before leaving the island."""
        roi = frame[int(.52 * IMAGE_HEIGHT):int(.96 * IMAGE_HEIGHT)]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = (((hsv[:, :, 2] <= int(self.line_black_value_max)) &
                 (hsv[:, :, 1] <= int(self.line_black_saturation_max))).astype(np.uint8) * 255)
        lines = cv2.HoughLinesP(mask, 1, np.pi / 180.0, 18,
                                minLineLength=70, maxLineGap=3)
        return lines is not None

    def _start_line_seen(self, frame):
        """Start line is checked only after a completed crosswalk, never while racing."""
        roi = frame[int(.76*IMAGE_HEIGHT):int(.96*IMAGE_HEIGHT)]
        dark = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)[:, :, 2] <= int(self.crosswalk_black_value_max)
        rows = dark.mean(axis=1) >= .35
        run = longest = 0
        for on in rows:
            run = run + 1 if on else 0
            longest = max(longest, run)
        return longest >= int(self.start_line_min_band_height_px)

    def _begin_next_lap_or_finish(self, now):
        if self.lap >= 2:
            self.phase = self.FINISHED
            self.stop_now()
            self.get_logger().warn("第二次压过起点线，比赛完成，保持停车")
            return
        self.lap = 2
        self.lap_started = now
        self.step_complete = self.crosswalk_complete = self.roundabout_complete = False
        self.crosswalk_hits = self.start_line_hits = self.roundabout_dash_hits = self.roundabout_exit_hits = 0
        self.roundabout_solid_hits = 0
        self.roundabout_first_marker_hits = self.roundabout_marker_clear_hits = 0
        self.roundabout_first_marker_seen = self.roundabout_marker_wait_clear = False
        self.roundabout_marker_last_seen = self.roundabout_entry_last_seen = 0.0
        self.roundabout_marker_cooldown_until = 0.0
        self.roundabout_pre_entry_guard_last_time = 0.0
        self.roundabout_marker_odom_last_xy = None
        self.roundabout_marker_travel_m = 0.0
        self.roundabout_first_marker_side = None
        self.roundabout_entry_candidate_start_y = None
        self.roundabout_entry_candidate_last_x = None
        self.roundabout_entry_candidate_last_y = None
        self.roundabout_used_this_lap = False
        self.roundabout_started_at = 0.0
        self.roundabout_heading_reference = self.roundabout_heading_last = None
        self.roundabout_heading_accumulated = 0.0
        self.roundabout_odom_last_xy = None
        self.roundabout_odom_travel_m = 0.0
        self.roundabout_progress_last_time = 0.0
        self.roundabout_angular_filtered = None
        self.roundabout_exit_search_started_at = 0.0
        self.test_turn_reference_yaw = self.latest_heading_yaw
        self.test_turn_count = 0
        self.roundabout_turn_ready = False
        self.roundabout_lost_since = None
        self.dash_approach_started_at = 0.0
        self.step_seen = False
        self.solid_line_history.clear()
        self.solid_reacquire_started_at = 0.0
        self.waiting_for_start_line = False
        self.get_logger().info("第一次压过起点线，进入第 2 圈")

    def _posture(self):
        msg = JointState(); msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["joint_height", "joint_roll", "joint_pitching", "joint_slide"]
        msg.position = [float(self.stand_height_m), 0., 0., 0.]
        self.posture_pub.publish(msg)

    def _bridge_dashed_gap(self, detection: DetectionResult, now: float) -> DetectionResult:
        """Use the last trustworthy line direction only across a short dash gap."""
        if detection.valid:
            self.last_line_detection = detection
            self.last_line_seen_time = now
            return detection
        previous = self.last_line_detection
        if previous is None or now - self.last_line_seen_time > float(self.dashed_line_hold_seconds):
            return detection
        return replace(
            previous,
            valid=True,
            confidence=min(float(previous.confidence), float(self.dashed_line_hold_confidence)),
            reason="dashed_gap_hold",
        )

    def _track_posture(self):
        """Keep the camera-facing race stance while following the line."""
        msg = JointState(); msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["joint_height", "joint_roll", "joint_pitching", "joint_slide"]
        msg.position = [float(self.track_height_m), 0., float(self.track_pitching_deg), 0.]
        self.posture_pub.publish(msg)

    def _jump(self, active):
        msg = Bool(); msg.data = bool(active); self.jump_pub.publish(msg)

    def _on_jump_confirm(self, message):
        """Accept a deliberate terminal approval only while stopped at the step."""
        if message.data and self.phase == self.JUMP_WAIT:
            self.jump_confirmed = True
            self.get_logger().warn("已收到终端跳跃许可，开始后退准备助跑")

    def _update_first_turn_gate(self, detection):
        """Arm depth step detection only after the first 90-degree turn is settled."""
        if self.step_detection_armed or not bool(self.first_turn_required):
            return
        if self.first_turn_yaw_zero is None or self.latest_heading_yaw is None:
            return
        if not self.first_turn_heading_complete:
            turned = abs(self._wrap_angle(self.latest_heading_yaw - self.first_turn_yaw_zero))
            if turned >= float(self.first_turn_target_rad) - float(self.first_turn_tolerance_rad):
                self.first_turn_heading_complete = True
                self.first_turn_center_hits = 0
                self.get_logger().info("首个弯道已达到约 90°，等待黑色中线重新居中后开启台阶检测")
            return
        centred = detection.valid and abs(float(detection.error)) <= float(self.first_turn_center_error)
        self.first_turn_center_hits = self.first_turn_center_hits + 1 if centred else 0
        if self.first_turn_center_hits >= int(self.first_turn_center_frames):
            self.step_detection_armed = True
            self.get_logger().warn("首弯后黑色中线已居中，台阶检测现已开启")

    def _control_race(self, detection, frame, now):
        self._update_first_turn_gate(detection)
        if (not bool(self.enable_step_jump) and not self.step_complete and self.step_seen
                and now - self.step_seen_time >= float(self.step_pass_confirm_seconds)):
            self.step_complete = True
            self.get_logger().info("已确认通过台阶区域（跳跃关闭），等待环岛虚线入口")
        if self.phase == self.FOLLOW and self._step_ok(now):
            s = self.latest_step
            self.phase = self.STEP_APPROACH
            self.get_logger().warn(
                "锁定赛道台阶：h=%.3f d=%.3f conf=%.2f；将于 %.2fm 停车等待终端许可"
                % (s.height, s.distance, s.confidence, float(self.step_stop_distance_m))
            )
        if self.phase == self.STEP_APPROACH:
            s = self.latest_step
            if not self._step_ok(now):
                self.stop_now()
                return 0.0, 0.0, 0.0, "STEP_DATA_LOST_STOP"
            if s.distance <= float(self.step_stop_distance_m):
                self.phase = self.JUMP_WAIT
                self.jump_confirmed = False
                self.stop_now()
                self.get_logger().warn(
                    "距台阶 %.2fm，已停车。请在第二个终端运行 ros2 run wheel_robot jump_confirm.py，输入 1 确认跳跃。"
                    % s.distance
                )
                return 0.0, 0.0, 0.0, "JUMP_WAIT_CONFIRM"
            self._posture()
            return (*self._publish_smooth(float(self.step_approach_speed_m_s), 0., 1/CONTROL_RATE_HZ), 0., "STEP_APPROACH")
        if self.phase == self.JUMP_WAIT:
            self.stop_now()
            if self.jump_confirmed:
                self.jump_confirmed = False
                self.phase = self.JUMP_RETREAT
                self.deadline = now + float(self.jump_retreat_seconds)
                self.get_logger().warn("跳跃已确认：后退 %.2fs 后开始 0.30m/s 助跑" % float(self.jump_retreat_seconds))
            return 0.0, 0.0, 0.0, "JUMP_WAIT_CONFIRM"
        if self.phase == self.JUMP_RETREAT:
            self._posture()
            if now >= self.deadline:
                self.phase = self.JUMP_RUNUP
                self.deadline = now + float(self.jump_runup_seconds)
                self.get_logger().warn("开始助跑：%.2fm/s，持续 %.2fs" % (float(self.jump_runup_speed_m_s), float(self.jump_runup_seconds)))
            return (*self._publish_smooth(-float(self.jump_retreat_speed_m_s), 0., 1/CONTROL_RATE_HZ), 0., "JUMP_RETREAT")
        if self.phase == self.JUMP_RUNUP:
            self._posture()
            if now >= self.deadline:
                self.phase, self.deadline = self.JUMP, now + float(self.jump_pulse_seconds)
                self.get_logger().warn("助跑结束，发送跳跃指令")
            return (*self._publish_smooth(float(self.jump_runup_speed_m_s), 0., 1/CONTROL_RATE_HZ), 0., "JUMP_RUNUP")
        if self.phase == self.JUMP:
            self._posture(); active = now < self.deadline; self._jump(active)
            if not active: self.phase, self.deadline = self.LAND, now+float(self.landing_forward_seconds)
            return (*self._publish_smooth(float(self.jump_forward_speed_m_s), 0., 1/CONTROL_RATE_HZ), 0., "JUMP")
        if self.phase == self.LAND:
            if now >= self.deadline:
                self.phase, self.step_complete = self.FOLLOW, True
                self.get_logger().info("台阶跳跃完成，恢复巡线；后续进入逆时针环岛阶段")
            return (*self._publish_smooth(
                float(self.jump_forward_speed_m_s) * float(self.landing_forward_speed_ratio),
                0., 1 / CONTROL_RATE_HZ,
            ), 0., "LAND")
        if self.phase == self.DASH_APPROACH:
            linear, angular, derivative, _ = self._control(detection, now)
            if (now - self.dash_approach_started_at >= float(self.roundabout_approach_min_seconds)
                    and self._roundabout_junction_seen(frame)):
                self._begin_roundabout_entry(now, "确认环岛岔口：低速逆时针切入黑色虚线")
            return linear, angular, derivative, "DASH_APPROACH"
        if self.phase == self.ROUNDABOUT_ENTRY:
            if now >= self.deadline:
                self.phase = self.ROUNDABOUT
                self.roundabout_started_at = now
                self.roundabout_solid_hits = 0
                self.roundabout_lost_since = None
                # The last valid path was the incoming solid centre line.
                # Never bridge it into the dashed roundabout or it can steer
                # the first recovery command in the wrong direction.
                self.last_line_detection = None
                self.last_line_seen_time = 0.0
                self.detector._reset_to_centre()
                self.get_logger().info("入口几何已确认，恢复黑色虚线巡线；累计航向仅用于 360° 出环岛确认")
            linear, angular = self._publish_smooth(
                float(self.roundabout_entry_speed_m_s),
                float(self.roundabout_entry_angular_rad_s),
                1 / CONTROL_RATE_HZ,
            )
            return linear, angular, 0.0, "ROUNDABOUT_ENTRY_CCW"
        if self.phase == self.ROUNDABOUT_EXIT_SEARCH:
            return self._roundabout_exit_search_control(detection, frame, now)
        if self.phase == self.ROUNDABOUT:
            self._log_roundabout_progress(now)
            if self._roundabout_heading_complete(now):
                self._begin_roundabout_exit_search(
                    now, "航向、实际里程和最短时间均确认逆时针完成约 360°并回到入口相对 0°"
                )
                return self._roundabout_exit_search_control(detection, frame, now)
            if (bool(self.roundabout_use_imu_heading)
                    and self.roundabout_heading_reference is not None
                    and now - self.roundabout_started_at > float(self.roundabout_heading_timeout_s)):
                self.stop_now()
                return 0.0, 0.0, 0.0, "ROUNDABOUT_HEADING_TIMEOUT_STOP"
            if not detection.valid:
                if self.roundabout_lost_since is None:
                    self.roundabout_lost_since = now
                if now - self.roundabout_lost_since <= float(self.roundabout_lost_continue_seconds):
                    linear, angular = self._publish_smooth(
                        float(self.roundabout_lost_speed_m_s),
                        0.0,
                        1 / CONTROL_RATE_HZ,
                    )
                    return linear, angular, 0.0, "ROUNDABOUT_REACQUIRE"
                self.stop_now()
                return 0.0, 0.0, 0.0, "ROUNDABOUT_LOST_STOP"
            self.roundabout_lost_since = None
            linear, angular, derivative, _ = self._control(detection, now)
            # Only use the former solid-line exit when IMU data is absent or
            # heading mode was deliberately disabled; with IMU we must finish
            # the full 360-degree cycle first.
            if not bool(self.roundabout_use_imu_heading) or self.roundabout_heading_reference is None:
                solid_exit = self._solid_line_after_roundabout_seen(frame)
                self.roundabout_solid_hits = self.roundabout_solid_hits + 1 if solid_exit else 0
                if (now - self.roundabout_started_at >= float(self.roundabout_min_seconds)
                        and self.roundabout_solid_hits >= int(self.roundabout_exit_solid_frames)):
                    self._begin_roundabout_exit_search(now, "兜底确认环岛后黑色实线")
                    return self._roundabout_exit_search_control(detection, frame, now)
            return linear, angular, derivative, "ROUNDABOUT_CCW"
        if self.phase == self.CROSSWALK_APPROACH:
            if now >= self.deadline:
                self.phase = self.CROSSWALK_STOP
                self.deadline = now + max(3.0, float(self.crosswalk_stop_seconds))
                self.stop_now()
                self.get_logger().warn(
                    "人行道对齐距离已完成，停车至少 %.1fs" % max(3.0, float(self.crosswalk_stop_seconds))
                )
                return 0.0, 0.0, 0.0, "CROSSWALK_STOP"
            linear, angular, derivative, _ = self._control(detection, now)
            return linear, angular, derivative, "CROSSWALK_ALIGN"
        if self.phase == self.CROSSWALK_STOP:
            self.stop_now()
            if now >= self.deadline:
                self.phase, self.crosswalk_complete = self.FOLLOW, True
                self.waiting_for_start_line = True
                self.get_logger().info("人行道停车完成，等待本圈再次压过起点线")
            return 0., 0., 0., "CROSSWALK_STOP"
        if self.phase == self.FINISHED:
            self.stop_now()
            return 0., 0., 0., "FINISHED_TWO_LAPS"
        if self.waiting_for_start_line:
            self.start_line_hits = self.start_line_hits + 1 if self._start_line_seen(frame) else 0
            if self.start_line_hits >= int(self.start_line_confirm_frames):
                self._begin_next_lap_or_finish(now)
                return 0., 0., 0., "START_LINE"
        time_ready = (bool(self.roundabout_time_mode)
                      and now - self.lap_started >= float(self.roundabout_min_start_seconds))
        roundabout_ready = self.roundabout_turn_ready or self.step_complete or time_ready
        ring_marker = self._roundabout_dashes_side(frame) if roundabout_ready else None
        ring_side = ring_marker.side if ring_marker is not None else None
        self._log_roundabout_status(now, roundabout_ready, ring_side)
        if (bool(self.roundabout_enabled) and roundabout_ready
                and not self.roundabout_complete and not self.roundabout_used_this_lap):
            left_marker = ring_marker is not None and ring_marker.side == "left"
            evidence_timeout = float(self.roundabout_marker_evidence_seconds)
            if not self.roundabout_first_marker_seen:
                if left_marker:
                    self.roundabout_first_marker_hits += 1
                    self.roundabout_marker_last_seen = now
                elif now - self.roundabout_marker_last_seen > evidence_timeout:
                    self.roundabout_first_marker_hits = 0
                if left_marker and self.roundabout_first_marker_hits == 1:
                    self.get_logger().info("RING_LEFT_DASH：发现左侧黑色虚线，正在确认首次出口标记")
                if self.roundabout_first_marker_hits >= int(self.roundabout_first_marker_confirm_frames):
                    pass_speed, pass_seconds, pass_distance = self._roundabout_marker_pass_profile()
                    self.roundabout_first_marker_seen = True
                    self.roundabout_first_marker_side = ring_marker.side
                    self.roundabout_marker_wait_clear = True
                    self.roundabout_marker_cooldown_until = now + pass_seconds
                    self.roundabout_marker_odom_last_xy = self.latest_odom_xy
                    self.roundabout_marker_travel_m = 0.0
                    self.roundabout_entry_candidate_start_y = None
                    self.roundabout_entry_candidate_last_x = None
                    self.roundabout_entry_candidate_last_y = None
                    self.roundabout_marker_clear_hits = self.roundabout_dash_hits = 0
                    # This is the course's exit marker, not an entry branch.
                    # Reset the curve tracker and hold a straight command for
                    # the full cooldown so its short dashes cannot pull us left.
                    self.detector._reset_to_centre()
                    self.get_logger().info(
                        "确认首次左侧黑色虚线：按环岛出口标记直行 %.2fm（%.2fm/s，%.1fs），等待标记离开画面"
                        % (pass_distance, pass_speed, pass_seconds)
                    )
            elif self.roundabout_marker_wait_clear:
                self.roundabout_marker_clear_hits = (
                    self.roundabout_marker_clear_hits + 1 if not left_marker else 0
                )
                if (now >= self.roundabout_marker_cooldown_until
                        and self.roundabout_marker_clear_hits >= int(self.roundabout_marker_clear_frames)):
                    self.roundabout_marker_wait_clear = False
                    self.roundabout_dash_hits = 0
                    self.roundabout_entry_last_seen = 0.0
                    self.roundabout_entry_candidate_start_y = None
                    self.roundabout_entry_candidate_last_x = None
                    self.roundabout_entry_candidate_last_y = None
                    self.get_logger().info("首次左侧出口标记已通过，开始等待后续同侧的环岛入口虚线")
            else:
                entry_marker = self._roundabout_entry_marker_matches(ring_marker)
                if ring_marker is not None and not entry_marker:
                    if now - self.roundabout_debug_last_time >= .25:
                        self.roundabout_debug_last_time = now
                        expected = str(self.roundabout_entry_expected_side)
                        self.get_logger().info(
                            "RING_ENTRY_REJECT：side=%s x=%.1f，入口需要 %s 侧虚线簇"
                            % (ring_marker.side, ring_marker.center_x, expected)
                        )
                if entry_marker:
                    self.roundabout_entry_last_seen = now
                    distance_ready = (
                        self.roundabout_marker_travel_m >= float(self.roundabout_entry_min_travel_m)
                    )
                    if not distance_ready:
                        self.roundabout_dash_hits = 0
                        if now - self.roundabout_debug_last_time >= .25:
                            self.get_logger().info(
                                "RING_ENTRY_BLOCKED：忽略出口的前向汇聚虚线，已行驶 %.2f/%.2fm"
                                % (self.roundabout_marker_travel_m,
                                   float(self.roundabout_entry_min_travel_m))
                            )
                    else:
                        if self.roundabout_entry_candidate_start_y is None:
                            self.roundabout_entry_candidate_start_y = ring_marker.center_y
                            self.roundabout_entry_candidate_last_x = ring_marker.center_x
                            self.roundabout_entry_candidate_last_y = ring_marker.center_y
                            self.roundabout_dash_hits = 1
                            self.get_logger().info(
                                "RING_ENTRY_CANDIDATE：同侧入口候选 x=%.1f y=%.1f travel=%.2fm，跟踪其朝车头的方向"
                                % (ring_marker.center_x, ring_marker.center_y,
                                   self.roundabout_marker_travel_m)
                            )
                        else:
                            dx = ring_marker.center_x - float(self.roundabout_entry_candidate_last_x)
                            dy = ring_marker.center_y - float(self.roundabout_entry_candidate_last_y)
                            same_cluster = (
                                abs(dx) <= float(self.roundabout_entry_track_max_x_jump_px)
                                and abs(dy) <= float(self.roundabout_entry_track_max_y_jump_px)
                            )
                            approaching = dy >= -float(self.roundabout_entry_y_jitter_px)
                            if same_cluster and approaching:
                                self.roundabout_dash_hits += 1
                            else:
                                # The component selector jumped to another
                                # dash group or it receded.  Restart instead
                                # of combining blue-exit and true-entry data.
                                self.roundabout_entry_candidate_start_y = ring_marker.center_y
                                self.roundabout_dash_hits = 1
                                self.get_logger().info(
                                    "RING_ENTRY_RESTART：虚线簇跳变/远离 dx=%+.1f dy=%+.1f，重新跟踪"
                                    % (dx, dy)
                                )
                            self.roundabout_entry_candidate_last_x = ring_marker.center_x
                            self.roundabout_entry_candidate_last_y = ring_marker.center_y
                elif now - self.roundabout_entry_last_seen > evidence_timeout:
                    self.roundabout_dash_hits = 0
                    self.roundabout_entry_candidate_start_y = None
                    self.roundabout_entry_candidate_last_x = None
                    self.roundabout_entry_candidate_last_y = None
                entered_toward_vehicle = (
                    self.roundabout_entry_candidate_start_y is not None
                    and self.roundabout_entry_candidate_last_y is not None
                    and float(self.roundabout_entry_candidate_last_y)
                    - float(self.roundabout_entry_candidate_start_y)
                    >= float(self.roundabout_entry_approach_min_y_px)
                )
                if (self.roundabout_dash_hits >= int(self.roundabout_entry_confirm_frames)
                        and entered_toward_vehicle):
                    self._begin_roundabout_entry(now, "确认后续同侧黑色虚线为环岛入口，低速逆时针切入")
        # The first left dashed group is explicitly the exit marker.  Do not
        # feed it to the normal curve tracker during its short pass-over.
        # Keep the exit-mark pass command active until the first cluster has
        # actually disappeared, not merely until its nominal clearance time
        # has elapsed. This prevents a late visible dash from reaching the
        # normal curve tracker for a few unsafe frames.
        if self.roundabout_marker_wait_clear:
            pass_speed, _pass_seconds, _pass_distance = self._roundabout_marker_pass_profile()
            linear, angular = self._publish_smooth(
                pass_speed, 0.0,
                1 / CONTROL_RATE_HZ,
            )
            return linear, angular, 0.0, "ROUNDABOUT_EXIT_MARKER_STRAIGHT"
        # Between the exit marker and the confirmed entry, black dashes can
        # briefly displace the solid-line target. Use the dedicated low-speed
        # guard instead of the inherited SEARCH_* / history-reacquire turn.
        if self._roundabout_pre_entry_guard_active():
            return self._roundabout_pre_entry_guard_control(detection, now)
        # Crosswalk is recognised by its own multi-stripe geometry, not by a
        # previous island state.  This keeps it available even if an island
        # junction was missed during a practice run.
        if bool(self.crosswalk_enabled) and not self.crosswalk_complete:
            crosswalk_armed = (
                not bool(self.crosswalk_require_roundabout_complete)
                or self.roundabout_complete
            )
            if not crosswalk_armed:
                # Reset partial candidates: a dash group before/inside the
                # island must never contribute to a later zebra confirmation.
                self.crosswalk_hits = 0
                if now - self.crosswalk_gate_last_time >= 1.0:
                    self.crosswalk_gate_last_time = now
                    self.get_logger().info(
                        "CROSSWALK_BLOCKED：等待本圈环岛完成，当前不执行人行道识别"
                    )
            else:
                bands, confidence, dark_ratio, length_ratio, gap_ratio = self._crosswalk_bands(frame)
                confirmed_frame = (
                    bands >= int(self.crosswalk_min_stripes)
                    and confidence >= float(self.crosswalk_min_confidence)
                )
                self.crosswalk_hits = self.crosswalk_hits + 1 if confirmed_frame else 0
                if bool(self.crosswalk_debug_each_frame):
                    self.get_logger().info(
                        "CROSSWALK_FRAME bars=%d conf=%.2f dark=%.3f len_ratio=%.2f gap_ratio=%.2f hits=%d/%d roi=%.2f~%.2f" % (
                            bands, confidence, dark_ratio, length_ratio, gap_ratio, self.crosswalk_hits,
                            int(self.crosswalk_confirm_frames),
                            float(self.crosswalk_roi_top_ratio), float(self.crosswalk_roi_bottom_ratio),
                        )
                    )
                if self.crosswalk_hits >= int(self.crosswalk_confirm_frames):
                    distance = max(0.0, float(self.crosswalk_approach_distance_m))
                    speed = max(0.05, float(self.crosswalk_approach_speed_m_s))
                    self.phase = self.CROSSWALK_APPROACH
                    self.deadline = now + distance / speed
                    self.get_logger().warn(
                        "确认人行道：%d 条纹、置信度 %.2f、连续 %d 帧；以 %.2fm/s 再对齐 %.2fm 后停车" % (
                            bands, confidence, self.crosswalk_hits, speed, distance
                        )
                    )
                    linear, angular, derivative, _ = self._control(detection, now)
                    return linear, angular, derivative, "CROSSWALK_ALIGN"
        return self._control(detection, now)

    def run(self):
        self._open_camera(); next_tick = time.monotonic(); failed = None
        debug_stride = max(1, int(self.debug_display_every_n_frames))
        if self.show_debug_window:
            try:
                cv2.namedWindow("Task 3 race debug", cv2.WINDOW_NORMAL)
                cv2.resizeWindow("Task 3 race debug", int(self.debug_screen_width), int(self.debug_screen_height))
            except cv2.error as error:
                self.get_logger().error("无法打开车载调试屏幕：%s" % error)
                self.show_debug_window = False
        while rclpy.ok() and not self.stop_requested:
            rclpy.spin_once(self, timeout_sec=0.0); ok, frame = self.capture.read(); now = time.monotonic()
            if not ok or frame is None:
                failed = failed or now
                if now-failed >= IMAGE_TIMEOUT_SECONDS: raise RuntimeError("比赛 RGB 摄像头连续读取失败")
                time.sleep(.01); continue
            failed = None
            if frame.shape[:2] != (IMAGE_HEIGHT, IMAGE_WIDTH): frame = cv2.resize(frame, (IMAGE_WIDTH, IMAGE_HEIGHT))
            # The additional far preview must not influence the ordinary-line
            # controller or the island marker gate.  It is enabled only after
            # this lap's roundabout, where the known tight solid S section is.
            self.detector.s_curve_preview_enabled = (
                bool(self.s_curve_enabled)
                and self.phase == self.FOLLOW
                and bool(self.roundabout_complete)
            )
            # The dashed island is not a continuous polynomial path.  It is
            # followed only by its bounded gap bridge plus the ring state;
            # this prevents three dash fragments from creating a fictitious
            # curve that commands the vehicle into the island.
            self.detector.curve_fit_enabled = self.phase not in (
                self.ROUNDABOUT_ENTRY, self.ROUNDABOUT, self.ROUNDABOUT_EXIT_SEARCH,
            )
            raw_detection, fps = self.detector.detect(frame), self._update_fps(now)
            detection = raw_detection
            profile = self.detector.curve_profile
            if profile.valid and now - self.s_curve_debug_last_time >= 0.75:
                self.s_curve_debug_last_time = now
                mode = "S_CURVE" if profile.s_curve_active else "NORMAL_OR_SINGLE_BEND"
                preview = "none" if profile.preview_x is None else "%.1f" % profile.preview_x
                self.get_logger().info(
                    "CURVE_PROFILE mode=%s curv=%.3f sign=%+d flip=%s near=%.1f far=%.1f preview=%s samples=%d"
                    % (mode, profile.curvature_filtered, profile.bend_sign,
                       profile.alternated, profile.near_x, profile.far_x,
                       preview, profile.sample_count)
                )
            # Solid-line mode is strict: every command must be based on the
            # current frame's centred black line.  Direction hold is reserved
            # exclusively for physical gaps in the roundabout's dashed guide.
            if self.phase in (self.DASH_APPROACH, self.ROUNDABOUT_ENTRY, self.ROUNDABOUT):
                detection = self._bridge_dashed_gap(detection, now)
            elif raw_detection.valid:
                self.last_line_detection = detection
                self.last_line_seen_time = now
                if self.phase == self.FOLLOW:
                    self._remember_solid_line(raw_detection, now)
            elif self.phase == self.FOLLOW and not self._roundabout_pre_entry_guard_active():
                # Only a real current frame is admitted to history.  The
                # synthetic direction below expires quickly and cannot keep
                # itself alive or turn a sustained loss into blind driving.
                detection = self._solid_reacquire_detection(raw_detection, now)
            # wl_base_node applies posture updates in sync with motion.
            if self.phase not in (self.STEP_APPROACH, self.JUMP_WAIT, self.JUMP_RETREAT, self.JUMP_RUNUP, self.JUMP):
                self._track_posture()
            linear, angular, derivative, state = self._control_race(detection, frame, now)
            self._write_log(detection, fps, derivative, linear, angular, state, now)
            if self.show_debug_window and self.frame_id % debug_stride == 0:
                debug = self._make_debug(frame, detection, fps, derivative, linear, angular, state)
                debug = cv2.resize(debug, (int(self.debug_screen_width), int(self.debug_screen_height)), interpolation=cv2.INTER_AREA)
                cv2.imshow("Task 3 race debug", debug)
            if self.show_debug_window:
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), ord("Q"), 27):
                    self.get_logger().warn("车载屏幕收到 Q/Esc，立即停车退出")
                    self.stop_now(); self.stop_requested = True; break
            if now-self.last_status_time >= STATUS_LOG_SECONDS:
                self.get_logger().info("%s line=%s conf=%.2f v=%.2f w=%.2f" % (state, detection.reason, detection.confidence, linear, angular)); self.last_status_time = now
            self.frame_id += 1; next_tick += 1/CONTROL_RATE_HZ; time.sleep(max(0., next_tick-time.monotonic()))


def main(args=None):
    rclpy.init(args=args); node = None
    try: node = RaceController(); node.run()
    except KeyboardInterrupt: pass
    except Exception as exc:
        if node: node.get_logger().error("比赛控制器异常: %s" % exc)
    finally:
        if node: node.close(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == "__main__": main()
