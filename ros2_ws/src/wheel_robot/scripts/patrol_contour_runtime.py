#!/usr/bin/env python3
"""ROS-free helpers for the opt-in patrol experiment.

7.25 reference sequence: road binary -> contour -> row edges -> short-gap
repair -> row midpoints. V13 retains near-priority NORMALIZED errors and
separates tracking from ordered ring/crosswalk tasks. No servo/PWM copied.
"""
from dataclasses import dataclass, fields, replace
import hashlib
import math
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import yaml


@dataclass(frozen=True)
class PatrolTuning:
    enabled: int = 1
    fixed_speed_m_s: float = 0.65
    adaptive_speed_enabled: int = 1
    startup_speed_m_s: float = 0.40
    bend_speed_m_s: float = 0.45
    sharp_bend_speed_m_s: float = 0.45
    lost_speed_m_s: float = 0.12
    roundabout_approach_speed_m_s: float = 0.40
    roundabout_speed_m_s: float = 0.30
    speed_stable_seconds: float = 0.50
    speed_stable_confidence: float = 0.40
    bend_preview_error: float = 0.45
    bend_release_seconds: float = 0.50
    linear_decel_m_s2: float = 1.20
    steering_gain: float = 1.08
    steering_kd: float = 0.018
    error_filter_alpha: float = 0.48
    max_error_derivative: float = 3.0
    max_turn_rad_s: float = 2.0
    linear_accel_m_s2: float = 0.65
    angular_accel_rad_s2: float = 3.0
    corner_start_error: float = 0.25
    corner_full_error: float = 0.70
    corner_angular_accel_rad_s2: float = 10.0
    unwind_angular_accel_rad_s2: float = 10.0
    rescue_corner_confidence: float = 0.55
    path_jump_error: float = 0.65
    path_jump_confirm_frames: int = 2
    target_x_ratio: float = 0.50
    black_enabled: int = 1
    black_min_confidence: float = 0.26
    contour_enabled: int = 1
    black_return_frames: int = 3
    handoff_seconds: float = 0.15
    road_max_near_jump_ratio: float = 0.25
    road_single_edge_hold_seconds: float = 0.40
    road_width_probe_seconds: float = 0.20
    loss_hold_seconds: float = 0.80
    search_enabled: int = 1
    search_max_seconds: float = 3.0
    search_max_distance_m: float = 1.8
    search_turn_rad_s: float = 0.80
    recovery_turn_guard_enabled: int = 1
    search_turn_max_rad_s: float = 0.35
    weak_path_turn_max_rad_s: float = 0.45
    recovery_angular_accel_rad_s2: float = 1.0
    search_default_direction: int = 1
    reacquire_confirm_frames: int = 3
    maintain_cruise_during_search: int = 1
    image_timeout_seconds: float = 0.35
    roi_top_ratio: float = 0.25
    roi_bottom_ratio: float = 0.98
    blue_h_min: int = 90
    blue_h_max: int = 140
    blue_s_min: int = 90
    blue_v_min: int = 35
    blue_weak_s_min: int = 25
    blue_weak_br_delta: int = 8
    blue_weak_min_area_ratio: float = 0.005
    blue_context_radius_ratio: float = 0.10
    road_blue_contrast_gray: int = 25
    white_s_max: int = 110
    white_v_min: int = 90
    close_kernel_px: int = 3
    black_gap_fill_px: int = 12
    road_max_row_step_ratio: float = 0.08
    road_history_weight: float = 0.25
    min_road_area_ratio: float = 0.04
    seed_bottom_ratio: float = 0.85
    seed_half_width_ratio: float = 0.15
    min_road_width_ratio: float = 0.12
    min_observed_rows_ratio: float = 0.20
    max_repair_gap_rows: int = 6
    road_near_y_ratio: float = 0.86
    road_near_gap_ratio: float = 0.18
    road_mid_y_ratio: float = 0.60
    road_far_y_ratio: float = 0.38
    display_fps: float = 10.0
    posture_height_m: float = 0.15
    posture_roll_deg: float = 0.0
    posture_pitch_deg: float = 25.0
    posture_slide_m: float = 0.0
    roundabout_enabled: int = 1
    roundabout_arm_seconds: float = 6.0
    roundabout_arm_distance_m: float = 1.0
    roundabout_timeout_enabled: int = 1
    roundabout_first_timeout_seconds: float = 30.0
    roundabout_second_timeout_seconds: float = 45.0
    roundabout_gate_strong_score: float = 0.82
    roundabout_gate_weak_score: float = 0.78
    roundabout_gate_evidence: float = 2.4
    roundabout_approach_distance_m: float = 0.05
    roundabout_entry_gate_near_ratio: float = 0.66
    roundabout_entry_confirm_frames: int = 2
    roundabout_entry_near_hold_seconds: float = 0.25
    roundabout_approach_max_seconds: float = 8.0
    roundabout_entry_min_seconds: float = 0.6
    roundabout_entry_distance_m: float = 0.45
    roundabout_entry_min_yaw_rad: float = 0.35
    roundabout_path_confirm_frames: int = 3
    roundabout_follow_min_seconds: float = 2.0
    roundabout_follow_min_distance_m: float = 0.50
    roundabout_exit_min_yaw_rad: float = 1.5
    roundabout_exit_solid_score: float = 0.65
    roundabout_exit_left_max_ratio: float = 0.65
    roundabout_follow_max_seconds: float = 28.0
    roundabout_exit_distance_m: float = 0.45
    roundabout_route_max_seconds: float = 10.0
    roundabout_rearm_seconds: float = 12.0
    roundabout_rearm_distance_m: float = 3.0
    roundabout_search_turn_rad_s: float = 0.8
    roundabout_entry_turn_rad_s: float = 1.2
    roundabout_first_marker_cooldown_seconds: float = 1.0
    roundabout_first_marker_pass_distance_m: float = 0.20
    vv_dash_min_blocks: int = 4
    vv_dash_confirm_frames: int = 1
    vv_dash_roi_top_ratio: float = 0.22
    vv_dash_roi_bottom_ratio: float = 0.90
    vv_dash_max_x_spread_px80: float = 18.0
    vv_dash_min_gap_rows56: float = 3.0
    ring_gyro_exit_enabled: int = 1
    ring_gyro_exit_degrees: float = 360.0
    ring_gyro_min_distance_m: float = 0.80
    ring_gyro_timeout_seconds: float = 0.20
    ring_gyro_rate_limit_rad_s: float = 8.0
    ring_gyro_deadband_rad_s: float = 0.03
    ring_gyro_exit_seek_seconds: float = 1.0
    ring_gyro_exit_speed_m_s: float = 0.20
    roundabout_fork_change_ratio: float = 0.06
    roundabout_fork_confirm_frames: int = 3
    roundabout_path_agreement: float = 0.30
    roundabout_heading_gain: float = 0.60
    roundabout_left_preview_enabled: int = 1
    roundabout_entry_stable_seconds: float = 0.20
    roundabout_entry_path_delta: float = 0.18
    roundabout_path_max_gap_seconds: float = 0.20
    roundabout_route_memory_seconds: float = 1.20
    roundabout_route_max_delta: float = 0.30
    roundabout_gate_tracking_enabled: int = 1
    roundabout_gate_track_min_score: float = 0.70
    roundabout_gate_track_frames: int = 4
    roundabout_gate_track_max_gap_seconds: float = 0.20
    roundabout_gate_track_max_x_ratio: float = 0.10
    roundabout_exit_corner_enabled: int = 1
    roundabout_exit_corner_min_span_ratio: float = 0.12
    roundabout_exit_corner_min_shift_ratio: float = 0.08
    crosswalk_enabled: int = 1
    crosswalk_confirm_frames: int = 2
    crosswalk_min_stripes: int = 4
    crosswalk_good_rows: int = 3
    crosswalk_roi_top_ratio: float = 0.24
    crosswalk_roi_bottom_ratio: float = 0.82
    crosswalk_min_span_ratio: float = 0.35
    crosswalk_delay_after_roundabout: float = 1.5
    crosswalk_first_min_seconds: float = 0.0
    crosswalk_repeat_min_seconds: float = 0.0
    crosswalk_visual_release_enabled: int = 0
    crosswalk_visual_release_seconds: float = 18.0
    crosswalk_visual_release_frames: int = 3
    crosswalk_visual_release_min_stripes: int = 5
    crosswalk_approach_distance_m: float = 0.25
    crosswalk_stop_seconds: float = 3.0
    crosswalk_clear_seconds: float = 3.0
    crosswalk_clear_speed_m_s: float = 0.0
    crosswalk_heading_gain: float = 1.2
    finish_after_second_crosswalk: int = 1
    second_crosswalk_finish_distance_m: float = 1.0
    crosswalk_task_max_seconds: float = 12.0

    def validate(self):
        """Reject an entire bad save; never silently clip a requested speed."""
        for field in fields(self):
            value = getattr(self, field.name)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{field.name} 必须是数值（开关用0/1）")
            if not math.isfinite(value):
                raise ValueError(f"{field.name} 必须是有限数")
        if not 0 < self.roundabout_entry_turn_rad_s <= self.max_turn_rad_s:
            raise ValueError("入环角速度必须大于0且不超过最大角速度")
        if not 0 < self.roundabout_fork_change_ratio < .5:
            raise ValueError("虚线分叉变化阈值必须在(0,0.5)")
        if self.roundabout_first_marker_cooldown_seconds <= 0 or self.roundabout_first_marker_pass_distance_m <= 0:
            raise ValueError("通过首次标记需要正的冷却时间和前进距离")
        if int(self.roundabout_fork_confirm_frames) != self.roundabout_fork_confirm_frames or not 2 <= self.roundabout_fork_confirm_frames <= 20:
            raise ValueError("分叉确认帧数必须为2..20整数")
        for name in ("enabled", "adaptive_speed_enabled", "recovery_turn_guard_enabled", "black_enabled", "contour_enabled", "search_enabled", "roundabout_enabled", "roundabout_timeout_enabled", "roundabout_left_preview_enabled", "maintain_cruise_during_search", "crosswalk_enabled", "crosswalk_visual_release_enabled", "finish_after_second_crosswalk"):
            if getattr(self, name) not in (0, 1):
                raise ValueError(f"{name} 必须为0或1")
        for name in ("startup_speed_m_s","bend_speed_m_s","sharp_bend_speed_m_s","lost_speed_m_s","roundabout_speed_m_s","roundabout_approach_speed_m_s","ring_gyro_exit_speed_m_s"):
            if getattr(self,name)<0:
                raise ValueError(f"{name} 必须非负")
        for name in ("speed_stable_seconds","bend_release_seconds","linear_decel_m_s2"):
            if getattr(self,name)<=0:
                raise ValueError(f"{name} 必须大于0")
        for name in ("speed_stable_confidence","bend_preview_error"):
            if not 0<getattr(self,name)<=1:
                raise ValueError(f"{name} 必须在(0,1]")
        if not self.lost_speed_m_s<=self.startup_speed_m_s or not self.sharp_bend_speed_m_s<=self.bend_speed_m_s:
            raise ValueError("找线速度须<=起步速度，急弯速度须<=普通弯速度")
        for name in ("search_turn_max_rad_s","weak_path_turn_max_rad_s","recovery_angular_accel_rad_s2"):
            if getattr(self,name)<=0:
                raise ValueError(f"{name} 必须大于0")
        for name,low,high in (("vv_dash_min_blocks",2,12),("vv_dash_confirm_frames",1,10),("ring_gyro_exit_enabled",0,1)):
            value=getattr(self,name)
            if int(value)!=value or not low<=value<=high:
                raise ValueError(f"{name} 必须为{low}..{high}整数")
        if not 0<=self.vv_dash_roi_top_ratio<self.vv_dash_roi_bottom_ratio<=1:
            raise ValueError("VV虚线ROI必须有序且在0..1")
        if not 0<self.vv_dash_max_x_spread_px80<=40 or not 0<=self.vv_dash_min_gap_rows56<=10:
            raise ValueError("VV虚线横向范围/白色间隔超出参考像素范围")
        if not 180<=self.ring_gyro_exit_degrees<=720:
            raise ValueError("陀螺仪出环角度必须在180..720度")
        for name in ("ring_gyro_min_distance_m","ring_gyro_timeout_seconds","ring_gyro_rate_limit_rad_s","ring_gyro_exit_seek_seconds"):
            if getattr(self,name)<=0:
                raise ValueError(f"{name} 必须大于0")
        if not 0<=self.ring_gyro_deadband_rad_s<self.ring_gyro_rate_limit_rad_s:
            raise ValueError("陀螺仪死区须小于角速度有效上限")
        for name in ("roundabout_gate_tracking_enabled", "roundabout_exit_corner_enabled"):
            if getattr(self, name) not in (0, 1):
                raise ValueError(f"{name} 必须为0或1")
        for name in ("roundabout_entry_stable_seconds", "roundabout_path_max_gap_seconds", "roundabout_route_memory_seconds", "roundabout_gate_track_max_gap_seconds"):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} 必须大于0")
        for name in ("roundabout_entry_path_delta", "roundabout_route_max_delta", "roundabout_gate_track_max_x_ratio", "roundabout_exit_corner_min_span_ratio", "roundabout_exit_corner_min_shift_ratio"):
            if not 0 < getattr(self, name) <= 1:
                raise ValueError(f"{name} 必须在(0,1]")
        if not 0 <= self.roundabout_gate_track_min_score <= self.roundabout_gate_weak_score:
            raise ValueError("跨帧入口最低分数必须在0..weak之间")
        if int(self.roundabout_gate_track_frames) != self.roundabout_gate_track_frames or not 2 <= self.roundabout_gate_track_frames <= 20:
            raise ValueError("跨帧入口确认数必须为2..20整数")
        if self.roundabout_entry_stable_seconds >= self.roundabout_route_max_seconds:
            raise ValueError("入环稳定观察时间必须小于入环超时")
        if not self.black_enabled and not self.contour_enabled:
            raise ValueError("黑线与轮廓不能同时关闭；停车请设enabled=0")
        for name in ("fixed_speed_m_s", "max_turn_rad_s", "steering_gain", "steering_kd", "max_error_derivative", "handoff_seconds", "loss_hold_seconds", "road_single_edge_hold_seconds", "search_turn_rad_s", "roundabout_arm_seconds", "roundabout_arm_distance_m", "roundabout_entry_min_seconds", "roundabout_entry_distance_m", "roundabout_entry_min_yaw_rad", "roundabout_follow_min_seconds", "roundabout_exit_distance_m", "roundabout_rearm_seconds", "roundabout_rearm_distance_m", "roundabout_search_turn_rad_s"):
            if getattr(self, name) < 0:
                raise ValueError(f"{name} 必须非负")
        for name in ("linear_accel_m_s2", "angular_accel_rad_s2", "corner_angular_accel_rad_s2", "unwind_angular_accel_rad_s2", "search_max_seconds", "search_max_distance_m", "image_timeout_seconds", "display_fps", "roundabout_gate_evidence", "roundabout_follow_max_seconds", "roundabout_route_max_seconds"):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} 必须大于0")
        for name in ("target_x_ratio", "black_min_confidence", "roi_top_ratio", "roi_bottom_ratio", "min_road_area_ratio", "seed_bottom_ratio", "seed_half_width_ratio", "min_road_width_ratio", "min_observed_rows_ratio", "road_near_y_ratio", "road_mid_y_ratio", "road_far_y_ratio", "road_max_row_step_ratio", "road_history_weight", "road_max_near_jump_ratio", "error_filter_alpha"):
            if not 0 <= getattr(self, name) <= 1:
                raise ValueError(f"{name} 必须在0..1")
        if not 0 <= self.roi_top_ratio < self.roi_bottom_ratio <= 1:
            raise ValueError("ROI上下界必须有序且有面积")
        if not self.roi_top_ratio <= self.road_far_y_ratio < self.road_mid_y_ratio < self.road_near_y_ratio <= self.roi_bottom_ratio:
            raise ValueError("道路远/中/近采样行必须有序且位于ROI内")
        if self.error_filter_alpha <= 0:
            raise ValueError("error_filter_alpha必须大于0")
        for name in ("rescue_corner_confidence", "path_jump_error", "roundabout_path_agreement", "blue_weak_min_area_ratio", "blue_context_radius_ratio"):
            if not 0 < getattr(self, name) <= 1:
                raise ValueError(f"{name} 必须在(0,1]")
        if not 0 <= self.roundabout_gate_weak_score <= self.roundabout_gate_strong_score <= 1:
            raise ValueError("环岛证据阈值必须满足0<=weak<=strong<=1")
        if self.roundabout_gate_evidence > 5 or not 0 <= self.roundabout_heading_gain <= 1:
            raise ValueError("环岛证据触发值须<=5，航向预瞄权重须在0..1")
        if self.roundabout_follow_min_seconds >= self.roundabout_follow_max_seconds:
            raise ValueError("环内最短观察时间必须小于故障超时")
        if self.roundabout_entry_min_seconds >= self.roundabout_route_max_seconds:
            raise ValueError("入环最短时间必须小于路线锁定超时")
        for name in ("roundabout_approach_distance_m", "roundabout_follow_min_distance_m", "roundabout_exit_min_yaw_rad", "crosswalk_delay_after_roundabout", "crosswalk_first_min_seconds", "crosswalk_repeat_min_seconds", "crosswalk_approach_distance_m", "crosswalk_stop_seconds", "crosswalk_clear_seconds", "crosswalk_clear_speed_m_s", "crosswalk_heading_gain", "second_crosswalk_finish_distance_m"):
            if getattr(self, name) < 0:
                raise ValueError(f"{name} 必须非负")
        if self.roundabout_approach_max_seconds <= 0 or self.crosswalk_task_max_seconds <= 0:
            raise ValueError("接近环岛/横道运动任务的故障时间必须大于0")
        if self.roundabout_entry_near_hold_seconds <= 0:
            raise ValueError("近场入口证据保留时间必须大于0")
        if self.roundabout_first_timeout_seconds <= 0 or self.roundabout_second_timeout_seconds <= 0:
            raise ValueError("两次环岛兜底时间必须大于0；关闭请用roundabout_timeout_enabled=0")
        if self.road_width_probe_seconds<=0 or self.crosswalk_visual_release_seconds<0:
            raise ValueError("路宽采样间隔必须>0，横道视觉兜底时间必须>=0")
        for name,low,high in (("crosswalk_visual_release_frames",2,20),("crosswalk_visual_release_min_stripes",4,12)):
            value=getattr(self,name)
            if int(value)!=value or not low<=value<=high:
                raise ValueError(f"{name} 必须为{low}..{high}整数")
        if int(self.roundabout_entry_confirm_frames) != self.roundabout_entry_confirm_frames or not 1 <= self.roundabout_entry_confirm_frames <= 20:
            raise ValueError("近场入口确认帧数必须为1..20整数")
        for name in ("roundabout_entry_gate_near_ratio", "roundabout_exit_solid_score", "roundabout_exit_left_max_ratio", "crosswalk_min_span_ratio"):
            if not 0 < getattr(self, name) <= 1:
                raise ValueError(f"{name} 必须在(0,1]")
        if not 0 <= self.crosswalk_roi_top_ratio < self.crosswalk_roi_bottom_ratio <= 1:
            raise ValueError("横道ROI必须有序且有面积")
        if self.crosswalk_stop_seconds > 15 or self.crosswalk_clear_seconds > 6:
            raise ValueError("横道停车应在0..15秒，清线在0..6秒")
        for name, lo, hi in (("crosswalk_confirm_frames",1,20),("crosswalk_min_stripes",2,12),("crosswalk_good_rows",1,40)):
            value=getattr(self,name)
            if int(value) != value or not lo <= value <= hi:
                raise ValueError(f"{name} 必须是{lo}..{hi}整数")
        if not 0 <= self.corner_start_error < self.corner_full_error <= 1:
            raise ValueError("急弯起始/满转向误差必须满足0<=start<full<=1")
        if not 0 <= self.road_near_gap_ratio <= self.road_near_y_ratio-self.road_mid_y_ratio:
            raise ValueError("道路近端缺口必须在0..(近端行-中端行)")
        if self.search_default_direction not in (-1, 1):
            raise ValueError("search_default_direction必须为-1右或1左")
        if int(self.reacquire_confirm_frames) != self.reacquire_confirm_frames or not 1 <= self.reacquire_confirm_frames <= 20:
            raise ValueError("reacquire_confirm_frames必须为1..20整数")
        if self.min_observed_rows_ratio <= 0 or self.min_road_width_ratio <= 0 or self.road_max_row_step_ratio <= 0:
            raise ValueError("有效边界行比例和最小路宽必须大于0")
        for name, lo, hi in (("blue_h_min", 0, 179), ("blue_h_max", 0, 179), ("blue_s_min", 0, 255), ("blue_v_min", 0, 255), ("blue_weak_s_min", 0, 255), ("blue_weak_br_delta", 0, 255), ("road_blue_contrast_gray", 0, 255), ("path_jump_confirm_frames", 1, 20), ("roundabout_path_confirm_frames", 1, 20), ("white_s_max", 0, 255), ("white_v_min", 0, 255), ("black_return_frames", 1, 20), ("close_kernel_px", 1, 9), ("max_repair_gap_rows", 0, 40), ("black_gap_fill_px", 0, 40)):
            value = getattr(self, name)
            if int(value) != value or not lo <= value <= hi:
                raise ValueError(f"{name} 必须是{lo}..{hi}整数")
        if self.close_kernel_px % 2 != 1 or self.blue_h_min > self.blue_h_max:
            raise ValueError("闭运算核必须是奇数，蓝色H上下界必须有序")
        for name, lo, hi in (("posture_height_m", .14, .36), ("posture_roll_deg", -15, 15), ("posture_pitch_deg", -35, 35), ("posture_slide_m", -.08, .08)):
            if not lo <= getattr(self, name) <= hi:
                raise ValueError(f"{name} 超出底盘接口范围{lo}..{hi}")
        return self


class PatrolFileWatcher:
    """Read a complete file snapshot, including atomic replace saves."""
    def __init__(self, path: Path):
        self.path = Path(path)
        self.signature = None

    def poll(self):
        try:
            data = self.path.read_bytes()
        except OSError as error:
            signature = ("missing", str(error))
            if signature == self.signature:
                return None, ""
            self.signature = signature
            return None, f"参数文件不可读，保留上一套：{error}"
        signature = hashlib.sha256(data).hexdigest()
        if signature == self.signature:
            return None, ""
        self.signature = signature
        try:
            document = yaml.safe_load(data.decode("utf-8-sig"))
            values = document["patrol_test"]
            if not isinstance(values, dict):
                raise ValueError("patrol_test 必须为参数表")
            names = {field.name for field in fields(PatrolTuning)}
            if set(values) != names:
                raise ValueError(f"参数名/完整性错误，缺少{sorted(names-set(values))}，未知{sorted(set(values)-names)}")
            tuning = PatrolTuning(**values).validate()
        except (ValueError, TypeError, KeyError, yaml.YAMLError) as error:
            return None, f"参数保存未生效，整套保留上一值：{error}"
        return tuning, "已完整加载 patrol_test.yaml"


@dataclass
class PathEstimate:
    valid: bool
    error: float = 0.0  # NORMALIZED lateral error: +right; NOT radians
    confidence: float = 0.0
    points: Optional[np.ndarray] = None
    mask: Optional[np.ndarray] = None
    reason: str = ""
    near_error: float = 0.0
    mid_error: float = 0.0
    far_error: float = 0.0
    rescue: bool = False
    source: str = ""
    handoff: bool = False
    widths: Optional[np.ndarray] = None  # [visible dual-edge width, full-frame y]
    dual_fraction: float = 0.0
    single_edge: bool = False
    near_gap_ratio: float = 0.0
    branch_rows: int = 0
    route_locked: bool = False
    route_observed: bool = False  # near+mid geometry actually observed, not a lock flag
    exit_boundary: Optional[np.ndarray] = None  # validated virtual RIGHT boundary


def classify_gate_fork(components, width, height, tuning, reference_x_ratio=None, diagnostics=None):
    """Classify a LEFT fork relative to its straight main dash axis.

    Forward is image-up. Increasing left separation forward is entry;
    decreasing separation is the first exit marker. Reuses neutral-component
    extraction, never imports another navigation state machine. Work is capped.
    Returns (kind, separation change / image width, observed main path).
    """
    invalid = PathEstimate(False, reason="fork_no_main")
    info={} if diagnostics is None else diagnostics
    info.clear()
    info.update(reason="few_components", main=None, left=None)
    candidates = sorted(components, key=lambda c: c[4], reverse=True)[:32]
    if len(candidates) < 5:
        return "none", 0., invalid
    points = np.asarray([(c[0], c[1]) for c in candidates], dtype=float)
    seed = (tuning.target_x_ratio if reference_x_ratio is None else float(np.clip(reference_x_ratio,.25,.75))) * width
    axis = points[abs(points[:, 0]-seed) <= .24*width]
    info["reason"]="main_axis_missing"
    if len(axis) < 3:
        return "none", 0., invalid
    a, b = np.triu_indices(len(axis), 1)
    dy = axis[b, 1]-axis[a, 1]
    usable = abs(dy) >= .10*height
    a, b, dy = a[usable], b[usable], dy[usable]
    if not len(a):
        info["reason"]="main_span_short"
        return "none", 0., invalid
    slopes = (axis[b, 0]-axis[a, 0])/dy
    intercepts = axis[a, 0]-slopes*axis[a, 1]
    inliers = abs(axis[None, :, 0]-slopes[:, None]*axis[None, :, 1]-intercepts[:, None]) <= .025*width
    spans = np.max(np.where(inliers, axis[None, :, 1], -np.inf), axis=1)-np.min(np.where(inliers, axis[None, :, 1], np.inf), axis=1)
    scores = inliers.sum(axis=1)+spans/height-1.5*abs(slopes)-abs(slopes*.85*height+intercepts-seed)/width
    scores[(inliers.sum(axis=1)<3) | (spans<.22*height) | (abs(slopes)>.75)] = -np.inf
    if not np.isfinite(scores).any():
        info["reason"]="main_consensus_failed"
        return "none", 0., invalid
    main = axis[inliers[int(np.argmax(scores))]]
    fit = np.polyfit(main[:, 1], main[:, 0], 1)
    if abs(np.polyval(fit, .85*height)-seed) > .20*width:
        info["reason"]="main_off_reference"
        return "none", 0., invalid
    main = main[np.argsort(main[:, 1])]
    estimate = PathEstimate(True, confidence=.8, points=main, reason="fork_main_axis")
    estimate.error, estimate.near_error, estimate.mid_error, estimate.far_error = path_errors(main, width, height, tuning)
    gaps = np.polyval(fit, points[:, 1])-points[:, 0]
    left = points[(gaps>=.035*width) & (gaps<=.50*width)]
    info.update(main=main, left=left, reason="left_branch_short_or_far")
    if len(left)<2 or np.ptp(left[:, 1])<.12*height or left[:, 1].max()<tuning.roundabout_entry_gate_near_ratio*height:
        return "none", 0., estimate
    forward = 1-left[:, 1]/height
    gaps = np.polyval(fit, left[:, 1])-left[:, 0]
    separation = np.polyfit(forward, gaps, 1)
    if np.median(abs(gaps-np.polyval(separation, forward))) > .035*width:
        info["reason"]="left_branch_inconsistent"
        return "none", 0., estimate
    change = float(separation[0]*np.ptp(forward)/width)
    kind = "none" if abs(change)<tuning.roundabout_fork_change_ratio else "diverging" if change>0 else "converging"
    info["reason"]="parallel_or_small_change" if kind=="none" else "current_fork"
    return kind, change, estimate


class ForkTopologyTracker:
    """Short, geometrically associated evidence; no old-image-only entry.

    Bridge a missing branch block only while BOTH branches are still observed
    at matching positions in the new frame. Expiry is based on the last FULL
    classification, never renewed indefinitely by partial detections.
    """
    def __init__(self):
        self.reset()

    def reset(self):
        self.at=None
        self.kind="none"
        self.main=self.left=None
        self.current_left=None
        self.change=0.
        self.reason="not_armed"

    def update(self,components,width,height,now,tuning,reference_x_ratio=None):
        info={}
        kind,change,main=classify_gate_fork(components,width,height,tuning,reference_x_ratio,info)
        self.current_left=info.get("left")
        self.reason=info["reason"]
        if kind!="none":
            self.at,self.kind,self.change=now,kind,change
            self.main,self.left=info["main"].copy(),info["left"].copy()
            return kind,change,main
        # Do not carry evidence over a conflicting parallel fork or reset.
        fresh=self.at is not None and 0<now-self.at<=min(.15,tuning.roundabout_gate_track_max_gap_seconds)
        current_left=info.get("left")
        if (not fresh or not main.valid or current_left is None or len(current_left)<1
                or self.main is None or self.left is None
                or info["reason"]!="left_branch_short_or_far"):
            return kind,change,main
        old_main=np.polyfit(self.main[:,1],self.main[:,0],1)
        old_left=np.polyfit(self.left[:,1],self.left[:,0],1)
        current_main=info["main"]
        near_left=current_left[:,1].max()>=tuning.roundabout_entry_gate_near_ratio*height
        main_agrees=np.max(abs(current_main[:,0]-np.polyval(old_main,current_main[:,1])))<=.04*width
        branch_agrees=np.max(abs(current_left[:,0]-np.polyval(old_left,current_left[:,1])))<=.04*width
        if near_left and main_agrees and branch_agrees:
            self.reason="associated_partial_fork"
            return self.kind,self.change,main
        self.reason="partial_fork_mismatch"
        return kind,change,main


class GateChainTracker:
    """Associate the SAME longitudinal chain across fresh frames, SEEK only.

    Raw detector score is never rewritten. A lower-score chain must independently
    satisfy span/spacing/shape and temporal association; no blue visibility gate.
    """
    def __init__(self):
        self.reset()

    def reset(self):
        self.anchor = self.at = self.since = None
        self.frames = 0
        self.reason = "no_chain"

    def update(self, components, score, width, height, now, tuning):
        if not tuning.roundabout_gate_tracking_enabled or score < tuning.roundabout_gate_track_min_score or len(components) < 5:
            self.reset()
            return False
        values = np.asarray(components, dtype=float)
        xs, ys = values[:, 0]/width, values[:, 1]/height
        order = np.argsort(ys)
        xs, ys = xs[order], ys[order]
        gaps = np.diff(ys)
        span = float(np.ptp(ys))
        if span < .22 or np.min(gaps) < .018 or np.max(gaps) > .19 or np.std(gaps)/max(np.mean(gaps), 1e-6) > 1.05:
            self.reset()
            self.reason = "chain_spacing_or_span"
            return False
        coeff = np.polyfit(ys, xs, 1)
        if np.sqrt(np.mean((xs-np.polyval(coeff, ys))**2)) > .06:
            self.reset()
            self.reason = "chain_shape"
            return False
        anchor = (float(np.polyval(coeff, .60)), float(np.max(ys)), len(xs))
        related = (self.anchor is not None and self.at is not None and 0 < now-self.at <= tuning.roundabout_gate_track_max_gap_seconds
                   and abs(anchor[0]-self.anchor[0]) <= tuning.roundabout_gate_track_max_x_ratio
                   and abs(anchor[1]-self.anchor[1]) <= .15 and abs(anchor[2]-self.anchor[2]) <= 2)
        if not related:
            self.frames, self.since = 0, now
        self.frames += 1
        self.anchor, self.at = anchor, now
        ready = self.frames >= tuning.roundabout_gate_track_frames and now-self.since >= .08
        self.reason = "tracked_chain" if ready else "tracking_chain"
        return ready


class RingRouteMemory:
    """Preserve route ownership across ENTRY/FOLLOW and temporary path loss.

    This stores geometry, not a forced yaw sign. A continuous right correction
    remains valid. Expired memory does not authorize any arbitrary outside line.
    """
    def __init__(self):
        self.reset()

    def reset(self):
        self.path = self.at = None

    def agrees(self, path, now, tuning, max_delta=None):
        if self.path is None or self.at is None or now-self.at > tuning.roundabout_route_memory_seconds:
            return None
        limit=tuning.roundabout_route_max_delta if max_delta is None else min(max_delta,tuning.roundabout_route_max_delta)
        return max(abs(path.near_error-self.path.near_error), abs(path.mid_error-self.path.mid_error)) <= limit

    def remember(self, path, now):
        if path.valid and path.route_observed and not path.handoff:
            self.path = replace(path, points=None, mask=None, exit_boundary=None)
            self.at = now

    def seed_entry(self,path,now,width,height,tuning):
        """Actual partial guide owns entry, but does NOT prove acquisition.

        VV correctly rejects the two merged near-junction blocks when their
        sides are not white. Retain the observed mid/near branch as a bounded
        reference; accept_path still requires fully observed near+mid frames.
        """
        if not path.valid or path.rescue or path.points is None or len(path.points)<2:
            return False
        points=np.asarray(path.points)
        if (not np.isfinite(points).all() or np.min(points[:,1])>(tuning.road_mid_y_ratio+.04)*height
                or np.max(points[:,1])<tuning.roundabout_entry_gate_near_ratio*height):
            return False
        self.path=replace(path,points=None,mask=None,reason="entry_partial_route_reference")
        self.at=now
        return True


def observed_route(path, width, height, tuning):
    """At least two observed points span near and mid; rescue is not acquisition."""
    if not path.valid or path.rescue or path.points is None or path.near_gap_ratio > .02:
        return False
    points = np.asarray(path.points).reshape(-1, 2)
    points = points[np.isfinite(points).all(axis=1)]
    return bool(len(points) >= 2 and np.min(points[:, 1]) <= (tuning.road_mid_y_ratio+.04)*height
                and np.max(points[:, 1]) >= (tuning.road_near_y_ratio-.08)*height)


def path_errors(points, width, height, tuning):
    """Near-dominant normalized road geometry; never average away near lines.

    Black uses the BASELINE extractor's own error, including its curve and
    single-band semantics. Road errors use the same units, not an angle.
    """
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) < 1:
        return None
    points = points[np.argsort(points[:, 1])]
    ys, unique = np.unique(points[:, 1], return_index=True)
    xs = points[unique, 0]
    # np.interp clamps to nearest observation; no polynomial extrapolation.
    near, mid, far = [float(np.clip((np.interp(r*height, ys, xs)-width*tuning.target_x_ratio)/(width*.5), -1., 1.))
                      for r in (tuning.road_near_y_ratio, tuning.road_mid_y_ratio, tuning.road_far_y_ratio)]
    shape = max(abs(mid-near), .8*abs(far-mid), .65*abs(far-near))
    if near*mid < -.015:
        error = .98*near+.02*mid
    elif shape >= .12:
        error = .96*near+.04*mid+.02*(mid-near)
    elif abs(near) >= .52:
        error = .90*near+.10*mid
    else:
        error = .78*near+.22*mid+.06*(mid-near)
    return float(np.clip(error, -1., 1.)), near, mid, far


def center_error(points, width, height, tuning):
    errors = path_errors(points, width, height, tuning)
    return None if errors is None else errors[0]


def black_usable(path, tuning):
    return bool(tuning.black_enabled and path.valid and
                (path.rescue or path.confidence >= tuning.black_min_confidence))


def _repair_short_gaps(values, max_gap):
    result = values.copy()
    valid = np.flatnonzero(np.isfinite(values))
    for start, end in zip(valid[:-1], valid[1:]):
        gap = end - start - 1
        if 0 < gap <= max_gap:
            result[start:end + 1] = np.linspace(values[start], values[end], end-start+1)
    return result


def road_masks(hsv, tuning):
    """Strong blue plus sufficiently large/connected low-saturation blue.

    Weak blue still needs blue hue and B-R chroma, not brightness alone.
    Warm white and neutral glare are not globally relabelled blue.
    """
    blue = cv2.inRange(hsv, (int(tuning.blue_h_min), int(tuning.blue_s_min), int(tuning.blue_v_min)), (int(tuning.blue_h_max), 255, 255))
    weak = cv2.inRange(hsv, (int(tuning.blue_h_min), int(tuning.blue_weak_s_min), int(tuning.blue_v_min)), (int(tuning.blue_h_max), 255, 255))
    bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR).astype(np.int16)
    weak[(bgr[:, :, 0]-bgr[:, :, 2] < tuning.blue_weak_br_delta) |
         (bgr[:, :, 0] < bgr[:, :, 1]-8)] = 0
    count, labels, stats, _ = cv2.connectedComponentsWithStats(weak)
    if count > 1:
        keep = stats[:, cv2.CC_STAT_AREA] >= tuning.blue_weak_min_area_ratio*hsv.shape[0]*hsv.shape[1]
        keep[np.unique(labels[blue > 0])] = True
        keep[0] = False
        blue[keep[labels]] = 255
    white = cv2.inRange(hsv, (0, 0, int(tuning.white_v_min)), (179, int(tuning.white_s_max), 255))
    # Neutral glare INSIDE blue may have H=0/S=0, so hue alone cannot fix it.
    # Compare to nearby observed blue, not an absolute scene brightness.
    # Ambiguous low-contrast pixels are forbidden/unknown, never filled back.
    if tuning.road_blue_contrast_gray:
        radius = max(1, int(hsv.shape[1]*tuning.blue_context_radius_ratio))
        kernel = (2*radius+1, 2*radius+1)
        known = (blue > 0).astype(np.float32)
        coverage = cv2.boxFilter(known, -1, kernel)
        gray = (.114*bgr[:, :, 0]+.587*bgr[:, :, 1]+.299*bgr[:, :, 2]).astype(np.float32)
        brightness = cv2.boxFilter(known*gray, -1, kernel)/np.maximum(coverage, 1e-6)
        ambiguous = (white > 0) & (coverage >= .20) & (gray < brightness+tuning.road_blue_contrast_gray)
        blue[ambiguous] = 255
    return blue, white


def exit_corner_guide(road, blue, starts, ends, offsets, seed, full_height, tuning):
    """Virtual right edge: observed inner-right corner -> upper-left edge.

    EXIT caller only. The corner is the last left-channel right edge at an
    observed split, NOT an arbitrary image corner. Both the connector and every
    returned center sample must stay on connected observed white road. No
    inpainted blue, no global leftmost point from an unrelated contour.
    """
    height, width = road.shape
    empty = np.full(height, np.nan)
    near_x = int(np.clip(round(seed*width), 0, width-1))
    bottom = height-1
    if not road[bottom, near_x]:
        return empty, None
    min_width = tuning.min_road_width_ratio*width

    def runs(y):
        return [(int(starts[i]), int(ends[i])-1) for i in range(offsets[y], offsets[y+1])
                if ends[i]-starts[i]-1 >= min_width]

    for cy in range(height-2, 0, -1):
        channels = runs(cy)
        if len(channels) < 2:
            continue
        left, right = min(channels)
        # A real corner: the narrow left channel opens into a wider connected
        # near region on the next rows. Reject detached left patches/straight
        # parallel lanes without any observed opening.
        opening = next((r for r in runs(cy+1) if r[0] <= left+2 and r[1] >= right+tuning.roundabout_exit_corner_min_shift_ratio*width), None)
        if opening is None or left <= 0:
            continue
        cx = right-1
        far_left, far_right, far_y = left, right, cy
        for y in range(cy-1, -1, -1):
            connected = [r for r in runs(y) if r[0] <= far_right and r[1] >= far_left and r[1] <= cx+.08*width and r[0] > 0]
            if not connected:
                break
            far_left, far_right = min(connected)
            far_y = y
        tx = far_left+1
        if cy-far_y < tuning.roundabout_exit_corner_min_span_ratio*full_height or cx-tx < tuning.roundabout_exit_corner_min_shift_ratio*width:
            continue
        ys = np.arange(far_y, cy+1)
        virtual_right = np.interp(ys, [far_y, cy], [tx, cx])
        pixels = np.clip(np.rint(virtual_right).astype(int), 0, width-1)
        if np.any(road[ys, pixels] == 0) or np.any(blue[ys, pixels] > 0):
            continue
        guide = empty.copy()
        boundary = []
        for y, x in zip(ys, virtual_right):
            local = [r for r in runs(y) if r[0] <= x <= r[1]]
            if not local:
                continue
            l, _ = min(local)
            if x-l >= min_width:
                guide[y] = (l+x)/2
                boundary.append((float(x), float(y)))
        # Preview the selected exit from the current near anchor so the
        # controller does not wait for the blue tip to reach its near row.
        near_ys = np.arange(cy, bottom+1)
        near_xs = np.interp(near_ys, [cy, bottom], [(left+cx)/2, near_x])
        near_pixels = np.clip(np.rint(near_xs).astype(int), 0, width-1)
        if len(boundary) < 3 or np.any(road[near_ys, near_pixels] == 0) or np.any(blue[near_ys, near_pixels] > 0):
            continue
        guide[near_ys] = near_xs
        return guide, np.asarray(boundary)
    return empty, None


def road_centerline(hsv, tuning, seed_x_ratio=None, previous_points=None, previous_widths=None, branch="nearest", masks=None, exit_guide=False):
    """Preserve blue holes; trace ONE continuous white run per row.

    Uses the 7.25 edge/midpoint sequence without its unsafe whole-outer-fill
    shortcut. Both returned points and steering geometry use full-frame y.
    """
    full_h, width = hsv.shape[:2]
    y0 = int(full_h * tuning.roi_top_ratio)
    y1 = max(y0 + 1, int(full_h * tuning.roi_bottom_ratio))
    roi = hsv[y0:y1]
    height = len(roi)
    full_blue, full_white = road_masks(hsv, tuning) if masks is None else masks
    blue, white = full_blue[y0:y1], full_white[y0:y1]
    binary = cv2.bitwise_and(cv2.bitwise_not(blue), white)
    kernel = np.ones((int(tuning.close_kernel_px),) * 2, np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
    if tuning.black_gap_fill_px:
        # Only close narrow NEUTRAL-DARK paint gaps, never blue islands or a
        # large arbitrary hole. Inverting/filling an outer contour is unsafe.
        paint_kernel = np.ones((1, int(tuning.black_gap_fill_px)+1), np.uint8)
        bridged = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, paint_kernel)
        neutral_dark = (roi[:, :, 1] <= tuning.white_s_max) & (roi[:, :, 2] < tuning.white_v_min)
        binary[(bridged > 0) & neutral_dark] = 255
    binary[blue > 0] = 0  # reapply AFTER every morphology/repair operation
    contours, hierarchy = cv2.findContours(binary, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    seed = tuning.target_x_ratio if seed_x_ratio is None else float(np.clip(seed_x_ratio, 0, 1))
    sx0 = int(max(0, seed - tuning.seed_half_width_ratio) * width)
    sx1 = max(sx0 + 1, int(min(1, seed + tuning.seed_half_width_ratio) * width))
    # Permit a REAL dual-edge channel ending slightly above the requested
    # near row, anchored against the recent route. Never invent a white road.
    sy0 = min(int(tuning.seed_bottom_ratio * height), max(0, int((tuning.road_near_y_ratio-tuning.road_near_gap_ratio)*full_h)-y0))
    best = None
    best_score = None
    anchor_y=int(np.clip(tuning.road_near_y_ratio*full_h-y0,0,height-1))
    for i, contour in enumerate(contours):
        if hierarchy[0, i, 3] != -1:
            continue
        area = cv2.contourArea(contour)
        if area < tuning.min_road_area_ratio * width * height:
            continue
        filled = np.zeros_like(binary)
        cv2.drawContours(filled, [contour], -1, 255, cv2.FILLED)
        child = hierarchy[0, i, 2]
        while child != -1:
            cv2.drawContours(filled, [contours[child]], -1, 0, cv2.FILLED)
            child = hierarchy[0, child, 0]
        # Component selection may use an outer polygon, but its drivable
        # pixels MUST remain exactly within the repaired, non-blue road mask.
        filled = cv2.bitwise_and(filled, binary)
        seed_pixels = int(np.count_nonzero(filled[sy0:, sx0:sx1]))
        if seed_pixels == 0:
            continue
        # A large reflection outside a blue separator is not our road. Rank
        # by the nearest observed row's distance to the recent route anchor,
        # NOT contour area or number of seed-band pixels. Area is only a
        # minimum-noise rejection above, never a route preference.
        ys,xs=np.nonzero(filled[sy0:])
        if not len(xs):
            continue
        ys=ys+sy0
        dy=np.abs(ys-anchor_y)
        nearest=dy==dy.min()
        anchor_x=int(np.clip(round(seed*width),0,width-1))
        available=xs[nearest]
        nearest_x=int(available[np.argmin(np.abs(available-anchor_x))])
        row=int(ys[nearest][0])
        if np.any(blue[row,min(anchor_x,nearest_x):max(anchor_x,nearest_x)+1]>0):
            continue  # do not cross a visible blue stripe to another white patch
        score=(abs(nearest_x-anchor_x),int(dy.min()),-int(np.count_nonzero(filled[:,anchor_x])))
        if best_score is None or score < best_score:
            best, best_score = filled, score
    if best is None:
        return PathEstimate(False, mask=binary, reason="no_near_connected_road")
    # Precompute all row runs in one vectorized pass. Taking the extrema of
    # two separate white runs would put their average INSIDE a blue island.
    delta = np.diff(np.pad((best > 0).astype(np.int8), ((0, 0), (1, 1))), axis=1)
    run_y, starts = np.where(delta == 1)
    _, ends = np.where(delta == -1)  # exclusive right bounds
    offsets = np.concatenate(([0], np.cumsum(np.bincount(run_y, minlength=height))))
    branch_guide=np.full(height,np.nan)
    if branch=="left" and tuning.roundabout_left_preview_enabled:
        # Preview a REAL observed split before reaching its last row. A
        # greedy row-only trace otherwise reaches the blue tip centrally,
        # cannot move into the left run within max_step and rejects all road.
        # This connector is a center guide, not invented traversable pixels:
        # every guide sample must remain inside the observed non-blue mask.
        bottom=height-1
        near_x=int(np.clip(round(seed*width),0,width-1))
        if best[bottom,near_x]:
            for y in range(height-2,-1,-1):
                runs=[(int(starts[i]),int(ends[i])-1) for i in range(offsets[y],offsets[y+1])
                      if ends[i]-starts[i]-1>=tuning.min_road_width_ratio*width]
                if len(runs)<2:
                    continue
                left_run=min(runs)
                target=(left_run[0]+left_run[1])/2
                ys=np.arange(y,bottom+1)
                xs=np.interp(ys,[y,bottom],[target,near_x])
                pixels=np.clip(np.rint(xs).astype(int),0,width-1)
                if np.all(best[ys,pixels]>0) and np.all(blue[ys,pixels]==0):
                    branch_guide[ys]=xs
                    break
    exit_boundary = None
    if exit_guide and branch == "left" and tuning.roundabout_exit_corner_enabled:
        corner_guide, exit_boundary = exit_corner_guide(best, blue, starts, ends, offsets, seed, full_h, tuning)
        if exit_boundary is not None:
            branch_guide = corner_guide
            exit_boundary = exit_boundary+np.array([0, y0])
    references = np.full(height, np.nan)
    if previous_points is not None:
        previous = np.asarray(previous_points, dtype=float).reshape(-1, 2)
        previous = previous[np.isfinite(previous).all(axis=1)]
        if len(previous) >= 2:
            previous = previous[np.argsort(previous[:, 1])]
            references = np.interp(np.arange(y0, y1), previous[:, 1], previous[:, 0], left=np.nan, right=np.nan)
    center = np.full(height, np.nan)
    observed = np.zeros(height, dtype=bool)
    dual = np.zeros(height, dtype=bool)
    widths = np.full(height, np.nan)
    width_reference = np.full(height, np.nan)
    if previous_widths is not None and len(previous_widths) >= 2:
        wp = np.asarray(previous_widths, dtype=float)
        wp = wp[np.isfinite(wp).all(axis=1)]
        if len(wp) >= 2:
            wp = wp[np.argsort(wp[:, 1])]
            width_reference = np.interp(np.arange(y0, y1), wp[:, 1], wp[:, 0], left=np.nan, right=np.nan)
    last_x = seed * width
    max_step = tuning.road_max_row_step_ratio * width
    missing = 0
    found = False
    last_run = None
    branch_rows = 0
    for y in range(height-1, -1, -1):
        candidates = []
        historical = float(references[y])
        has_history = math.isfinite(historical)
        reference = .65*last_x+.35*historical if has_history else last_x
        for index in range(offsets[y], offsets[y+1]):
            left, right = int(starts[index]), int(ends[index])-1
            if right-left < tuning.min_road_width_ratio * width:
                continue
            both_edges = left > 0 and right < width-1
            one_edge = (left > 0) != (right < width-1)
            # A frame edge is not a measured road boundary. Only estimate the
            # missing side from a recent, explicitly dual-edge road width.
            if not both_edges:
                if not one_edge or not math.isfinite(width_reference[y]):
                    continue
                midpoint = left+width_reference[y]/2 if left > 0 else right-width_reference[y]/2
                if not left <= midpoint <= right:
                    continue  # estimated center is outside the visible road
            else:
                midpoint = (left+right)/2
            distance = max(left-reference, reference-right, 0.)
            if last_run is not None:
                # A branch must overlap the previous row's reachable road;
                # choosing the leftmost disconnected island is forbidden.
                if left > last_run[1]+1 or right < last_run[0]-1:
                    continue
            score = distance + .25*abs(midpoint-reference)
            candidates.append((score, left, right, midpoint))
        if not candidates:
            missing += 1
            if found and missing > tuning.max_repair_gap_rows:
                break
            continue
        if branch == "left" and found and len(candidates) > 1:
            _, left, right, midpoint = min(candidates, key=lambda item: item[3])
            branch_rows += 1
        else:
            _, left, right, midpoint = min(candidates)
        margin = min(2., (right-left)/4)
        desired = midpoint
        if math.isfinite(branch_guide[y]):
            desired=float(branch_guide[y])
        elif has_history:
            # A broad near-road region may connect two channels farther up.
            # Preserve the recent route instead of recentering between them.
            desired = (1-tuning.road_history_weight)*midpoint+tuning.road_history_weight*reference
        # Scalar min/max avoids hundreds of NumPy dispatches per image row.
        value = min(max(desired, last_x-max_step*(missing+1)), last_x+max_step*(missing+1))
        value = min(max(value, left+margin), right-margin)
        if found and abs(value-last_x) > max_step*(missing+1)+1:
            break  # a disconnected lateral jump is not a valid continuation
        center[y] = value
        observed[y] = left > 0 or right < width-1
        dual[y] = left > 0 and right < width-1
        if dual[y]:
            widths[y] = right-left
        last_x, missing, found = value, 0, True
        last_run = (left, right)
    if np.count_nonzero(observed) < max(3, tuning.min_observed_rows_ratio * height):
        return PathEstimate(False, mask=best, reason="all_white_or_insufficient_boundary")
    center = _repair_short_gaps(center, int(tuning.max_repair_gap_rows))
    valid = np.isfinite(center)
    # Interpolation must not create a path through a known blue forbidden area.
    valid_y = np.flatnonzero(valid)
    valid_x = np.clip(np.rint(center[valid]).astype(int), 0, width-1)
    forbidden_y = valid_y[blue[valid_y, valid_x] > 0]
    if len(forbidden_y):
        # Truncate, rather than letting the angle fitter interpolate back
        # across a rejected blue row toward far-side points.
        center[:int(np.max(forbidden_y))+1] = np.nan
    valid = np.isfinite(center)
    observed &= valid
    if np.count_nonzero(observed) < max(3, tuning.min_observed_rows_ratio * height):
        return PathEstimate(False, mask=best, reason="blue_blocked_or_short_channel")
    points = np.column_stack((center[valid], np.flatnonzero(valid)))
    global_points = points + np.array([0, y0])
    errors = path_errors(global_points, width, full_h, tuning)
    near_gap = max(0., tuning.road_near_y_ratio-float(np.max(global_points[:, 1]))/full_h)
    if errors is None or near_gap > tuning.road_near_gap_ratio:
        return PathEstimate(False, points=global_points, mask=best, reason="no_center_in_error_rows")
    if near_gap > 0:
        # Missing near pixels may be neutral/occluded, but never silently
        # bridge a visibly BLUE forbidden strip to a distant white island.
        closest = int(np.argmax(global_points[:, 1]))
        anchor_x = int(np.clip(round(global_points[closest, 0]), 0, width-1))
        gap_start = int(global_points[closest, 1])-y0+1
        gap_end = min(height, int(tuning.road_near_y_ratio*full_h)-y0+1)
        if np.any(blue[gap_start:gap_end, anchor_x] > 0):
            return PathEstimate(False, points=global_points, mask=best, reason="blue_blocks_near_gap", near_gap_ratio=near_gap)
    dual &= valid
    dual_fraction = np.count_nonzero(dual)/height
    confidence = min(1., (np.count_nonzero(dual)+.5*np.count_nonzero(observed & ~dual))/height)
    width_y = np.flatnonzero(np.isfinite(widths) & valid)
    width_points = np.column_stack((widths[width_y], width_y+y0))
    near_y = int(min(tuning.road_near_y_ratio*full_h,np.max(global_points[:, 1])))-y0
    # Mixed scenes are single-edge fallback if the control-near row lacks
    # dual boundaries, even when distant rows have both edges.
    single_edge = dual_fraction < tuning.min_observed_rows_ratio or not dual[min(max(near_y, 0), height-1)]
    return PathEstimate(True, errors[0], confidence, global_points, best,
                        "road_single_edge_width" if single_edge else "road_dual_edge",
                        *errors[1:], widths=width_points, dual_fraction=dual_fraction, single_edge=single_edge, near_gap_ratio=near_gap,
                        branch_rows=branch_rows, route_locked=branch == "left" and (branch_rows > 0 or errors[0] < -.10 or errors[2] < errors[1]-.10),
                        route_observed=bool(not single_edge and near_gap <= .02 and np.min(global_points[:,1]) <= (tuning.road_mid_y_ratio+.04)*full_h),
                        exit_boundary=exit_boundary)


def detect_crosswalk(dark, hsv, tuning):
    """Multiple regular dark bars with bright gaps; never a lone track line."""
    h,w=dark.shape
    x0,x1=int(.07*w),int(.93*w)
    y0,y1=int(tuning.crosswalk_roi_top_ratio*h),int(tuning.crosswalk_roi_bottom_ratio*h)
    good=maximum=0
    for y in range(y0,y1,3):
        strip=dark[max(y0,y-2):min(y1,y+3),x0:x1]
        occupied=np.mean(strip>0,axis=0)>=.38
        delta=np.diff(np.pad(occupied.astype(np.int8),(1,1)))
        starts,ends=np.flatnonzero(delta==1),np.flatnonzero(delta==-1)
        runs=[(int(l),int(r)) for l,r in zip(starts,ends) if 3<=r-l<=.15*w]
        if len(runs)<tuning.crosswalk_min_stripes:
            continue
        widths=np.array([r-l for l,r in runs],dtype=float)
        centers=np.array([(r+l)/2 for l,r in runs])
        if np.ptp(centers)<tuning.crosswalk_min_span_ratio*(x1-x0) or np.std(widths)/np.mean(widths)>.65:
            continue
        gaps=[(r,l) for (_,r),(l,_) in zip(runs,runs[1:]) if l-r>=2]
        bright=[np.mean((hsv[y,x0+l:x0+r,2]>=90)&(hsv[y,x0+l:x0+r,1]<=185)) for l,r in gaps]
        if len(bright)<tuning.crosswalk_min_stripes-1 or np.mean(bright)<.60:
            continue
        good+=1
        maximum=max(maximum,len(runs))
    score=min(1.,.55*maximum/max(1,tuning.crosswalk_min_stripes)+.45*good/max(1,tuning.crosswalk_good_rows))
    return good>=tuning.crosswalk_good_rows,score


class RoundaboutRouter:
    """Ordered task tokens: ring -> zebra -> next ring -> zebra -> finish."""
    def __init__(self):
        self.phase="SEEK"
        self.started_at=self.last_time=self.phase_at=None
        self.distance=self.phase_distance=self.left_yaw=self.ring_yaw=0.
        self.phase_measured_distance=0.
        self.evidence=0.
        self.clear_frames=self.path_streak=self.near_frames=self.solid_frames=0
        self.crosswalk_frames=self.crosswalk_count=0
        self.gate_cleared=self.branch_acquired=False
        self.fault=""
        self.last_pose=None
        self.progress_source="command_estimate"
        self.completed=0
        self.after_ring_at=self.last_crosswalk_at=None
        self.entry_block_until=self.entry_block_distance=0.
        self.paused_at=None
        self.heading_target=None
        self.near_seen_at=None
        self.release_cycle=-1
        self.ring_release_source="none"
        self.timed_releases=0
        self.deadline_overdue=False
        self.navigation_reset_serial=0
        self.visual_release_frames=0
        self.visual_release_at=None
        self.visual_releases=0
        self.route_status="seeking_gate"
        self.path_stable_since=self.last_path_observation=None
        self.last_path_errors=None
        self.first_marker_at=None
        self.first_marker_distance=0.
        self.fork_kind="none"
        self.fork_frames=self.first_marker_clear_frames=0
        self.fork_at=self.fork_since=None
        self.vv_streak=0
        self.vv_at=None
        self.gyro_yaw=0.
        self.gyro_at=None
        self.last_motion_linear=0.
        self.gyro_exit_requested=False

    @property
    def left_lock(self):
        return self.phase in ("ENTRY","EXIT")

    @property
    def ring_tracking(self):
        return self.phase in ("ENTRY", "FOLLOW", "EXIT")

    @property
    def search_left(self):
        # FOLLOW keeps task identity too; ordinary tracking can still steer
        # right on valid geometry. This only selects the LOST-path direction.
        return self.ring_tracking

    @property
    def active(self):
        return self.phase in ("APPROACH","ENTRY","FOLLOW","EXIT","RING_WAIT_DEADLINE")

    def ring_deadline(self,tuning):
        if not tuning.roundabout_enabled or not tuning.roundabout_timeout_enabled:
            return None
        if self.crosswalk_count==0 and self.started_at is not None:
            return self.started_at+tuning.roundabout_first_timeout_seconds
        if self.crosswalk_count==1 and self.last_crosswalk_at is not None:
            return self.last_crosswalk_at+tuning.roundabout_second_timeout_seconds
        return None

    def _release_ring(self,now,tuning,source):
        self.ring_release_source=source
        self.release_cycle=self.crosswalk_count
        self.after_ring_at=now
        self.entry_block_until=now+tuning.roundabout_rearm_seconds
        self.entry_block_distance=self.distance+tuning.roundabout_rearm_distance_m
        self.navigation_reset_serial+=1
        if source!="visual_exit":
            if source=="time_fallback":
                self.timed_releases+=1
            else:
                self.visual_releases+=1
        else:
            self.completed+=1
        self._phase("SEEK_CROSSWALK" if tuning.crosswalk_enabled else "COOLDOWN",now)

    def crosswalk_visual_release_allowed(self,now,tuning):
        if (not tuning.crosswalk_enabled or not tuning.crosswalk_visual_release_enabled
                or self.fault or self.paused_at is not None
                or self.phase not in ("SEEK","APPROACH","RING_WAIT_DEADLINE")
                or self.release_cycle==self.crosswalk_count):
            return False
        origin=self.started_at if self.crosswalk_count==0 else self.last_crosswalk_at
        return origin is not None and now-origin>=tuning.crosswalk_visual_release_seconds

    def observe_crosswalk_visual_release(self,detected,now,tuning):
        if not self.crosswalk_visual_release_allowed(now,tuning):
            self.visual_release_frames=0
            self.visual_release_at=None
            return False
        if self.visual_release_at is not None and now-self.visual_release_at>.20:
            self.visual_release_frames=0
        self.visual_release_frames=self.visual_release_frames+1 if detected else 0
        self.visual_release_at=now
        if self.visual_release_frames<tuning.crosswalk_visual_release_frames:
            return False
        self._release_ring(now,tuning,"zebra_evidence_fallback")
        # These stronger consecutive frames already confirmed the zebra.
        # The normal observer below consumes the task exactly once.
        self.crosswalk_frames=tuning.crosswalk_confirm_frames-1
        return True

    def apply_time_fallback(self,now,tuning):
        # Compatibility name: the old method released the task on a timer.
        # A deadline is diagnostic ONLY, never proof of a physical ring exit.
        deadline=self.ring_deadline(tuning)
        self.deadline_overdue=bool(not self.fault and self.paused_at is None
            and self.phase in ("SEEK","APPROACH","ENTRY","FOLLOW","EXIT","RING_WAIT_DEADLINE")
            and self.release_cycle!=self.crosswalk_count and deadline is not None and now>=deadline)
        return False

    def _phase(self,name,now):
        preserve = self.phase in ("ENTRY", "FOLLOW") and name in ("FOLLOW", "EXIT")
        acquired, cleared = self.branch_acquired, self.gate_cleared
        self.phase,self.phase_at=name,now
        self.phase_distance=self.left_yaw=self.evidence=0.
        self.phase_measured_distance=0.
        self.path_streak=self.clear_frames=self.near_frames=self.solid_frames=0
        self.gate_cleared=self.branch_acquired=False
        if preserve:
            self.branch_acquired, self.gate_cleared = acquired, cleared
        self.path_stable_since=self.last_path_observation=None
        self.last_path_errors=None
        self.route_status={"ENTRY":"acquiring_left_route", "FOLLOW":"following_owned_route", "EXIT":"seeking_left_exit"}.get(name,name.lower())
        self.near_seen_at=None
        self.visual_release_frames=0
        self.visual_release_at=None
        if name=="ENTRY":
            self.ring_yaw=0.
        if name=="FOLLOW":
            self.gyro_yaw=0.
            self.gyro_at=None
        if name not in ("FOLLOW","EXIT"):
            self.gyro_exit_requested=False

    def pause(self,now):
        if self.paused_at is None:
            self.paused_at=now
        self.last_time=self.last_pose=None
        self.crosswalk_frames=self.near_frames=self.path_streak=0
        self.path_stable_since=self.last_path_observation=None
        self.last_path_errors=None
        self.near_seen_at=None
        self.visual_release_frames=0
        self.visual_release_at=None
        self.fork_frames=0
        self.fork_at=self.fork_since=None
        self.vv_streak=0
        self.vv_at=self.gyro_at=None

    def gate_detection_allowed(self,now,tuning):
        if not tuning.roundabout_enabled:
            return False
        if self.phase in ("APPROACH","FOLLOW","COOLDOWN"):
            return True
        elapsed=0. if self.started_at is None else now-self.started_at
        return (self.phase=="SEEK" and elapsed>=tuning.roundabout_arm_seconds
                and self.distance>=tuning.roundabout_arm_distance_m
                and now>=self.entry_block_until and self.distance>=self.entry_block_distance)

    def crosswalk_block_reason(self,now,tuning):
        if not tuning.crosswalk_enabled:
            return "disabled"
        if self.phase!="SEEK_CROSSWALK" or self.after_ring_at is None:
            return "ring_not_complete" if self.phase in ("SEEK","APPROACH","ENTRY","FOLLOW","EXIT","COOLDOWN","RING_WAIT_DEADLINE") else self.phase.lower()
        if self.ring_release_source not in ("time_fallback","zebra_evidence_fallback") and now-self.after_ring_at<tuning.crosswalk_delay_after_roundabout:
            return "after_ring_delay"
        if self.crosswalk_count==0 and (self.started_at is None or now-self.started_at<tuning.crosswalk_first_min_seconds):
            return "first_time_gate"
        if self.crosswalk_count and (self.last_crosswalk_at is None or now-self.last_crosswalk_at<tuning.crosswalk_repeat_min_seconds):
            return "repeat_time_gate"
        return "armed"

    def resume(self,now,clear_fault=False):
        if self.paused_at is None:
            return
        delay=max(0.,now-self.paused_at)
        for name in ("started_at","phase_at","after_ring_at","last_crosswalk_at","first_marker_at"):
            if getattr(self,name) is not None:
                setattr(self,name,getattr(self,name)+delay)
        self.entry_block_until+=delay
        if self.fault and clear_fault:
            # Only explicit operator confirmation can grant a new task budget.
            self.fault=""
            self.phase_at=now
        self.paused_at=None
        self.last_time=self.last_pose=None

    def crosswalk_armed(self,now,tuning):
        return self.crosswalk_block_reason(now,tuning)=="armed"

    def observe_gyro(self,rate,now,tuning):
        """Signed measured gyro, FOLLOW only; entry spins cannot fill a lap."""
        if (not tuning.ring_gyro_exit_enabled or self.phase!="FOLLOW" or self.paused_at is not None
                or self.fault or self.last_motion_linear<=.03 or not math.isfinite(rate)
                or abs(rate)>tuning.ring_gyro_rate_limit_rad_s):
            self.gyro_at=None
            return
        dt=0. if self.gyro_at is None else now-self.gyro_at
        self.gyro_at=now
        if 0<dt<=min(.15,tuning.ring_gyro_timeout_seconds):
            self.gyro_yaw+=(rate if abs(rate)>=tuning.ring_gyro_deadband_rad_s else 0.)*dt

    def _observe_vv_entry(self,cue,now,tuning,command):
        fresh=self.vv_at is not None and 0<now-self.vv_at<=tuning.roundabout_path_max_gap_seconds
        self.vv_streak=(self.vv_streak+1 if fresh else 1) if cue.valid and command[0]>1e-4 else 0
        self.vv_at=now
        confirmed=self.vv_streak>=tuning.vv_dash_confirm_frames
        if self.phase=="SEEK" and self.gate_detection_allowed(now,tuning) and confirmed:
            self.first_marker_at=now
            self.first_marker_distance=self.distance
            self._phase("APPROACH",now)
        if self.phase!="APPROACH":
            return
        passed=(self.first_marker_at is not None
                and now-self.first_marker_at>=tuning.roundabout_first_marker_cooldown_seconds
                and self.distance-self.first_marker_distance>=tuning.roundabout_first_marker_pass_distance_m)
        self.route_status="vv_pass_exit_5_on_main" if not passed else "vv_wait_left_entrance_4"
        if passed and confirmed and cue.near_ratio>=tuning.roundabout_entry_gate_near_ratio and cue.branch.valid:
            self._phase("ENTRY",now)
        elif now-self.phase_at>=tuning.roundabout_approach_max_seconds:
            self.entry_block_until=now+1.
            self._phase("SEEK",now)
            self.first_marker_at=None
            self.vv_streak=0

    def observe(self,gate,score,now,tuning,command=(0.,0.),pose=None,
                gate_near=0.,exit_solid=False,crosswalk=False,gate_tracked=False,gate_kind=None,dash_cue=None):
        self.resume(now)
        if self.started_at is None:
            self.started_at=now
        if self.phase_at is None:
            self.phase_at=now
        elapsed=0. if self.last_time is None else max(0.,now-self.last_time)
        dt=min(.1,elapsed)
        travel,yaw=abs(command[0])*dt,command[1]*dt
        measured_travel=0.
        self.progress_source="command_estimate"
        if pose is not None:
            self.progress_source="odom"
            travel=yaw=0.
            if self.last_pose is not None:
                step=math.hypot(pose[0]-self.last_pose[0],pose[1]-self.last_pose[1])
                turn=(pose[2]-self.last_pose[2]+math.pi)%(2*math.pi)-math.pi
                if step<=max(.2,2*abs(command[0])*dt) and abs(turn)<=.5:
                    travel,yaw=step,turn
                    measured_travel=step
        # Odom drift or manually moving a stopped car cannot arm a task.
        if abs(command[0])+abs(command[1])<1e-4:
            travel=yaw=0.
            measured_travel=0.
        self.last_pose,self.last_time=pose,now
        self.last_motion_linear=command[0]
        self.distance+=travel
        self.phase_distance+=travel
        self.phase_measured_distance+=measured_travel
        self.left_yaw+=yaw
        if self.phase in ("ENTRY","FOLLOW","EXIT"):
            self.ring_yaw+=yaw
        if self.fault:
            return
        self.apply_time_fallback(now,tuning)
        age=now-self.phase_at
        if ((self.left_lock and age>=tuning.roundabout_route_max_seconds) or
                (self.phase=="FOLLOW" and age>=tuning.roundabout_follow_max_seconds)):
            self.fault="roundabout_route_timeout"
            return
        if self.phase in ("CW_APPROACH","FINISH_RUN") and age>=tuning.crosswalk_task_max_seconds:
            self.fault="crosswalk_motion_timeout"
            return
        if dash_cue is not None and self.phase in ("SEEK","APPROACH"):
            self._observe_vv_entry(dash_cue,now,tuning,command)
            return  # exclusive live entry path; old topology is NOT consulted
        # Temporal association is ENTRY-only evidence, never an exit shortcut.
        tracked = bool(gate_tracked and self.phase in ("SEEK", "APPROACH"))
        topology_candidate=gate_kind in ("converging","diverging") and self.phase in ("SEEK","APPROACH")
        strong=gate and (score>=tuning.roundabout_gate_strong_score or tracked or topology_candidate)
        weak=score>=tuning.roundabout_gate_weak_score or tracked or topology_candidate
        self.clear_frames=self.clear_frames+1 if not gate and not weak else 0
        self.gate_cleared|=self.clear_frames>=tuning.roundabout_path_confirm_frames

        # None preserves the legacy raw-observation helper API. The live vision
        # node ALWAYS supplies a classification (including explicit "none").
        topology_ready = gate_kind is None
        if gate_kind is not None and self.phase in ("SEEK", "APPROACH"):
            fresh = self.fork_at is not None and 0 < now-self.fork_at <= tuning.roundabout_gate_track_max_gap_seconds
            usable = weak and gate_kind in ("converging", "diverging") and gate_near>=tuning.roundabout_entry_gate_near_ratio and command[0]>1e-4
            if not usable or not fresh or gate_kind!=self.fork_kind:
                self.fork_frames=0
                self.fork_since=now if usable else None
            self.fork_kind=gate_kind
            self.fork_at=now
            self.fork_frames=self.fork_frames+1 if usable else 0
            confirmed = self.fork_frames>=tuning.roundabout_fork_confirm_frames and self.fork_since is not None and now-self.fork_since>=.08
            if confirmed and gate_kind=="converging" and self.first_marker_at is None:
                self.first_marker_at=now
                self.first_marker_distance=self.distance
                self.first_marker_clear_frames=0
            if self.first_marker_at is not None:
                self.first_marker_clear_frames=self.first_marker_clear_frames+1 if gate_kind!="converging" and fresh else 0
                passed = (now-self.first_marker_at>=tuning.roundabout_first_marker_cooldown_seconds
                          and self.distance-self.first_marker_distance>=tuning.roundabout_first_marker_pass_distance_m
                          and self.first_marker_clear_frames>=tuning.roundabout_fork_confirm_frames)
                topology_ready = passed and confirmed and gate_kind=="diverging"
                self.route_status="entry_fork_confirmed" if topology_ready else "pass_first_marker" if not passed else "wait_diverging_entry"
            else:
                self.route_status="wait_converging_exit_marker"

        if self.phase=="SEEK" and tuning.roundabout_enabled:
            allowed=(now-self.started_at>=tuning.roundabout_arm_seconds
                     and self.distance>=tuning.roundabout_arm_distance_m
                     and now>=self.entry_block_until and self.distance>=self.entry_block_distance)
            if allowed:
                self.evidence=max(0.,min(5.,self.evidence+(1. if strong else .35 if weak else -.30)))
                if self.evidence>=tuning.roundabout_gate_evidence:
                    self._phase("APPROACH",now)
        elif self.phase=="APPROACH":
            # A previously confirmed chain may lose one block at close range.
            # Keep only recent real near evidence, not an indefinite timer latch.
            if self.near_seen_at is not None and now-self.near_seen_at>tuning.roundabout_entry_near_hold_seconds:
                self.near_frames=0
                self.near_seen_at=None
            if weak and topology_ready and gate_near>=tuning.roundabout_entry_gate_near_ratio and command[0]>1e-4:
                self.near_frames+=1
                self.near_seen_at=now
            elif gate_kind is not None:
                self.near_frames=0
                self.near_seen_at=None
            if (self.phase_distance>=tuning.roundabout_approach_distance_m
                    and self.near_seen_at is not None
                    and self.near_frames>=tuning.roundabout_entry_confirm_frames):
                self._phase("ENTRY",now)
            elif age>=tuning.roundabout_approach_max_seconds:
                # A distant/false candidate is NOT permission to turn blindly.
                self.entry_block_until=now+3.
                self.entry_block_distance=self.distance+tuning.roundabout_arm_distance_m
                self._phase("SEEK",now)
        elif self.phase=="FOLLOW":
            measured_turn=(tuning.ring_gyro_exit_enabled and self.gyro_at is not None
                and 0<=now-self.gyro_at<=tuning.ring_gyro_timeout_seconds
                and self.progress_source=="odom"
                and age>=tuning.roundabout_follow_min_seconds
                and self.phase_measured_distance>=tuning.ring_gyro_min_distance_m
                and self.gyro_yaw>=math.radians(tuning.ring_gyro_exit_degrees))
            if measured_turn:
                self.gyro_exit_requested=True
                self._phase("EXIT",now)
                self.route_status="gyro_exit_seek_not_complete"
                return
            ready=(age>=tuning.roundabout_follow_min_seconds
                   and self.phase_distance>=tuning.roundabout_follow_min_distance_m
                   and self.ring_yaw>=tuning.roundabout_exit_min_yaw_rad
                   and self.gate_cleared)
            if ready:
                self.solid_frames=self.solid_frames+1 if exit_solid else 0
                self.evidence=max(0.,min(5.,self.evidence+(1. if strong else .35 if weak else -.30)))
                if (self.evidence>=tuning.roundabout_gate_evidence
                        or self.solid_frames>=tuning.roundabout_path_confirm_frames):
                    self._phase("EXIT",now)
        elif self.phase=="COOLDOWN":
            if (age>=tuning.roundabout_rearm_seconds
                    and self.phase_distance>=tuning.roundabout_rearm_distance_m and self.gate_cleared):
                self._phase("SEEK",now)
        elif self.phase=="SEEK_CROSSWALK":
            self.crosswalk_frames=self.crosswalk_frames+1 if self.crosswalk_armed(now,tuning) and crosswalk else 0
            if self.crosswalk_frames>=tuning.crosswalk_confirm_frames:
                self.crosswalk_count+=1
                self.last_crosswalk_at=now
                self.heading_target=pose[2] if pose is not None else None
                self._phase("FINISH_RUN" if self.crosswalk_count>=2 and tuning.finish_after_second_crosswalk else "CW_APPROACH",now)
        elif self.phase=="CW_APPROACH":
            if self.phase_distance>=tuning.crosswalk_approach_distance_m:
                self._phase("CW_STOP",now)
        elif self.phase=="CW_STOP":
            if age>=tuning.crosswalk_stop_seconds:
                self._phase("CW_CLEAR",now)
        elif self.phase=="CW_CLEAR":
            if age>=tuning.crosswalk_clear_seconds:
                # This is the ONLY normal release of the next-ring token.
                self.first_marker_at=None
                self.fork_frames=self.first_marker_clear_frames=0
                self.fork_at=self.fork_since=None
                self.vv_at=None
                self.vv_streak=0
                self._phase("SEEK",now)
        elif self.phase=="FINISH_RUN":
            if self.phase_distance>=tuning.second_crosswalk_finish_distance_m:
                self._phase("FINISHED",now)

    def accept_path(self,path,now,tuning):
        if not self.ring_tracking or self.fault:
            return
        if self.phase=="FOLLOW":
            self.route_status="following_owned_route" if path.valid else "left_route_recovery"
            return
        good=bool(path.valid and path.route_locked)
        if self.phase=="ENTRY":
            good=good and path.route_observed and not path.rescue
        else:
            # Fork/connector frames cannot pre-fill the stable EXIT counter.
            # Require several actual unbranched observations after guidance.
            good=good and not path.rescue and path.branch_rows==0 and path.near_gap_ratio==0 and path.exit_boundary is None and abs(path.near_error-path.mid_error)<=tuning.roundabout_path_agreement
        errors=(path.near_error,path.mid_error)
        gap=self.last_path_observation is None or not 0 < now-self.last_path_observation <= tuning.roundabout_path_max_gap_seconds
        jump=self.last_path_errors is not None and max(abs(a-b) for a,b in zip(errors,self.last_path_errors))>tuning.roundabout_entry_path_delta
        if not good or gap or jump:
            self.path_streak=0
            self.path_stable_since=None
        self.last_path_observation=now
        self.last_path_errors=errors if good else None
        if good:
            if self.path_stable_since is None:
                self.path_stable_since=now
            self.path_streak+=1
        self.branch_acquired|=bool(good and (path.route_observed or self.phase=="EXIT"))
        if self.path_streak<tuning.roundabout_path_confirm_frames or not self.branch_acquired:
            self.route_status="waiting_continuous_near_mid_route" if self.phase=="ENTRY" else "waiting_exit_route"
            return
        if self.phase=="ENTRY":
            if (now-self.phase_at>=tuning.roundabout_entry_min_seconds
                    and self.phase_distance>=tuning.roundabout_entry_distance_m
                    and self.left_yaw>=tuning.roundabout_entry_min_yaw_rad
                    and now-self.path_stable_since>=tuning.roundabout_entry_stable_seconds):
                self._phase("FOLLOW",now)
        elif (self.phase_distance>=tuning.roundabout_exit_distance_m and not path.rescue
              and path.branch_rows==0 and path.near_gap_ratio==0
              and abs(path.near_error-path.mid_error)<=tuning.roundabout_path_agreement):
            self._release_ring(now,tuning,"visual_exit")

    def motion_override(self,now,tuning,pose=None):
        if self.phase=="EXIT" and self.gyro_exit_requested and now-self.phase_at<tuning.ring_gyro_exit_seek_seconds:
            return min(tuning.fixed_speed_m_s,tuning.ring_gyro_exit_speed_m_s),0.,"gyro_exit_seek"
        if self.phase=="ENTRY" and self.left_yaw<tuning.roundabout_entry_min_yaw_rad:
            # Establish the verified LEFT entry before any outside/right line
            # may take over. Bounded by route timeout, not perpetual blind yaw.
            return min(tuning.fixed_speed_m_s,tuning.roundabout_speed_m_s),min(tuning.roundabout_entry_turn_rad_s,tuning.max_turn_rad_s),"roundabout_left_acquire"
        if self.phase in ("CW_STOP","FINISHED"):
            return 0.,0.,"crosswalk_stop" if self.phase=="CW_STOP" else "finished"
        if self.phase not in ("CW_APPROACH","CW_CLEAR","FINISH_RUN"):
            return None
        angular=0.
        if self.heading_target is not None and pose is not None:
            error=(self.heading_target-pose[2]+math.pi)%(2*math.pi)-math.pi
            angular=float(np.clip(tuning.crosswalk_heading_gain*error,-tuning.max_turn_rad_s,tuning.max_turn_rad_s))
        speed=tuning.fixed_speed_m_s
        if self.phase=="CW_CLEAR" and tuning.crosswalk_clear_speed_m_s>0:
            speed=tuning.crosswalk_clear_speed_m_s
        return speed,angular,{"CW_APPROACH":"crosswalk_approach","CW_CLEAR":"crosswalk_clear","FINISH_RUN":"finish_pass"}[self.phase]


class PathSelector:
    """Immediate fallback, confirmed black return, short angular handoff."""
    def __init__(self):
        self.source = "none"
        self.black_streak = 0
        self.last_error = 0.0
        self.transition_from = 0.0
        self.transition_at = -math.inf
        self.last_observed_error = None
        self.jump_candidate = None
        self.jump_streak = 0

    def select(self, black, road, now, tuning):
        black_ok = black_usable(black, tuning)
        road_ok = bool(tuning.contour_enabled and road.valid)
        self.black_streak = self.black_streak + 1 if black_ok else 0
        if black_ok and (black.rescue or self.source != "road" or not road_ok or self.black_streak >= tuning.black_return_frames):
            estimate, source = black, "black"
        elif road_ok:
            estimate, source = road, "road"
        else:
            return PathEstimate(False, reason=f"no_path:black={black.reason};road={road.reason}"), "none"
        if (source == "black" and self.last_observed_error is not None
                and abs(estimate.error-self.last_observed_error) > tuning.path_jump_error
                and estimate.error*self.last_observed_error < 0 and not estimate.route_locked):
            self.jump_streak = self.jump_streak+1 if self.jump_candidate is not None and abs(estimate.error-self.jump_candidate) < .20 else 1
            self.jump_candidate = estimate.error
            if self.jump_streak < tuning.path_jump_confirm_frames:
                return PathEstimate(False, reason="path_jump_confirm"), "none"
        self.jump_candidate, self.jump_streak = None, 0
        self.last_observed_error = estimate.error
        if source != self.source:
            self.transition_from = self.last_error
            self.transition_at = now if self.source != "none" and not (estimate.rescue or estimate.route_locked) else -math.inf
            self.source = source
        ratio = 1.0 if tuning.handoff_seconds <= 0 else min(1., max(0., (now-self.transition_at)/tuning.handoff_seconds))
        # Convex blend: always inside the OLD OUTPUT and CURRENT target.
        # Never add a fixed source delta to a changing new error (V2 overshot).
        if estimate.rescue:
            ratio = 1.
            self.transition_at = -math.inf
        error = (1-ratio)*self.transition_from+ratio*estimate.error
        self.last_error = error
        return replace(estimate, error=error, source=source, handoff=ratio < 1.), source


class FixedSpeedController:
    """Cruise ceiling, debounced preview limits and bounded active recovery."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.last_valid_at = None
        self.last_good_turn = 0.
        self.last_time = None
        self.last_command = (0., 0.)
        self.loss_command = None
        self.loss_started_at = None
        self.loss_distance = 0.
        self.reacquire_streak = 0
        self.last_curve_turn = 0.
        self.last_curve_at = -math.inf
        self.search_turn_goal = 0.
        self.fault_latched = False
        self.fault_reason = ""
        self.corner_strength = 0.
        self.raw_angular = 0.
        self.startup_until=None
        self.bend_until=-math.inf
        self.speed_reason="waiting_for_path"
        self.speed_target=0.
        self.reset_error()

    def reset_error(self):
        """Clear PD only; never manufacture a cruise restart."""
        self.filtered_error = None
        self.previous_source = ""

    def loss_age(self, now):
        return 0. if self.loss_started_at is None else max(0., now-self.loss_started_at)

    def planned_speed(self,path,now,tuning,ring_tracking=False,ring_approach=False,recovering=False):
        """One speed owner for normal/curve/recovery paths; yaw PD unchanged."""
        cruise=tuning.fixed_speed_m_s
        if not tuning.adaptive_speed_enabled:
            self.speed_reason="legacy_fixed_target"
            self.speed_target=cruise
            return cruise
        # Startup is a bounded time window, NOT a confidence latch that gets
        # restarted by normal .45/.63 black-line observations every frame.
        if path.valid and self.startup_until is None:
            self.startup_until=now+tuning.speed_stable_seconds
        if not path.valid:
            target,reason=tuning.lost_speed_m_s,"lost_path_limit"
        else:
            shape=max(abs(path.error),.65*abs(path.near_error),abs(path.mid_error-path.near_error),.7*abs(path.far_error-path.mid_error))
            # Immediate braking on a real preview; release requires a quiet
            # interval, so an S-bend sign reversal cannot re-enable cruise.
            if shape>=tuning.bend_preview_error and not path.rescue and path.confidence>=tuning.speed_stable_confidence:
                self.bend_until=now+tuning.bend_release_seconds
            target,reason=cruise,"stable_cruise"
            if now<self.bend_until:
                target=tuning.sharp_bend_speed_m_s if shape>=tuning.corner_full_error else tuning.bend_speed_m_s
                reason="bend_preview_limit"
            if (now<(self.startup_until if self.startup_until is not None else now)
                    or recovering or path.rescue or path.confidence<tuning.speed_stable_confidence):
                target=min(target,tuning.startup_speed_m_s)
                reason="brief_start_or_weak_reacquire"
            if ring_approach:
                target=min(target,tuning.roundabout_approach_speed_m_s)
                reason="ring_detected_approach_limit"
            if ring_tracking:
                target=min(target,tuning.roundabout_speed_m_s)
                reason="ring_route_limit"
        self.speed_target=min(cruise,target)
        self.speed_reason=reason
        return self.speed_target

    @staticmethod
    def _approach(previous, target, delta):
        return previous+float(np.clip(target-previous, -delta, delta))

    def _steer(self, path, now, tuning, dt, recovering=False):
        magnitude = abs(path.error)
        blend = float(np.clip((magnitude-tuning.corner_start_error) /
                              (tuning.corner_full_error-tuning.corner_start_error), 0., 1.))
        self.corner_strength = blend*blend*(3-2*blend)
        # A lone weak edge remains usable, but no longer commands full sharp
        # turn authority solely because it happens to be near the image edge.
        if path.rescue:
            self.corner_strength *= min(1., path.confidence/tuning.rescue_corner_confidence)
        alpha = tuning.error_filter_alpha+(1-tuning.error_filter_alpha)*self.corner_strength
        reset_pd = self.filtered_error is None or recovering or path.source != self.previous_source
        filtered = path.error if reset_pd or path.rescue else alpha*path.error+(1-alpha)*self.filtered_error
        derivative = 0. if reset_pd or path.rescue or dt <= 0 or path.handoff else float(np.clip(
            (filtered-self.filtered_error)/dt, -tuning.max_error_derivative, tuning.max_error_derivative))
        self.filtered_error, self.previous_source = filtered, path.source
        normal = tuning.steering_gain*filtered
        corner = math.copysign(tuning.max_turn_rad_s, filtered) if filtered else 0.
        self.raw_angular = -((1-self.corner_strength)*normal+self.corner_strength*corner+tuning.steering_kd*derivative)
        return float(np.clip(self.raw_angular, -tuning.max_turn_rad_s, tuning.max_turn_rad_s))

    def _fault(self, reason, now):
        self.fault_latched, self.fault_reason = True, reason
        self.last_command, self.last_time = (0., 0.), now
        self.raw_angular = 0.
        self.reset_error()
        return 0., 0., "localization_fault_latched"

    def task_command(self, linear, angular, now, tuning, immediate_stop=False, left_only=False):
        """Explicit zebra/finish heading guidance, never an implicit speed cap."""
        elapsed=1/30 if self.last_time is None else max(0.,now-self.last_time)
        dt=min(.1,elapsed)
        if immediate_stop:
            linear=angular=0.
        else:
            target_v=min(linear,tuning.fixed_speed_m_s)
            rate=tuning.linear_decel_m_s2 if target_v<self.last_command[0] else tuning.linear_accel_m_s2
            linear=self._approach(self.last_command[0],target_v,rate*dt)
            angular=self._approach(self.last_command[1],angular,tuning.unwind_angular_accel_rad_s2*dt)
            if left_only:
                linear=min(linear,target_v)
                angular=max(0.,angular)  # cancel stale RIGHT output at entry
        self.last_command,self.last_time=(linear,angular),now
        # A task heading is current navigation evidence, NOT a black-line
        # detection. Resume a bounded visual search from the end of guidance.
        self.last_valid_at=now
        self.loss_started_at,self.loss_distance,self.loss_command=None,0.,None
        self.reacquire_streak=0
        self.last_good_turn=angular
        self.last_curve_at=-math.inf
        self.raw_angular=angular
        self.speed_target=min(tuning.fixed_speed_m_s,tuning.roundabout_speed_m_s) if left_only else linear
        self.speed_reason="verified_ring_entry" if left_only else "task_heading"
        self.corner_strength=0.
        self.reset_error()
        return linear,angular

    def command(self, path, now, tuning, search_direction=None, ring_tracking=False,ring_approach=False):
        if not tuning.enabled:
            self.reset()
            return 0., 0., "disabled"
        if self.fault_latched:
            return 0., 0., "localization_fault_latched"
        elapsed = max(0., now-self.last_time) if self.last_time is not None else 1/30
        dt = min(.1, elapsed)
        recovering = self.loss_started_at is not None
        if recovering:
            # Count real issued travel and elapsed time, including brief valid
            # flickers. A single line flash cannot renew either budget.
            self.loss_distance += abs(self.last_command[0])*elapsed

        if path.valid:
            reliable=not path.rescue and path.confidence>=tuning.speed_stable_confidence
            self.reacquire_streak = self.reacquire_streak+1 if recovering and (reliable or not tuning.recovery_turn_guard_enabled) else 0
            confirmed = not recovering or self.reacquire_streak >= tuning.reacquire_confirm_frames
            if confirmed:
                self.last_valid_at = now
                self.loss_started_at, self.loss_distance = None, 0.
                self.loss_command = None
                mode, linear = "fixed_speed", tuning.fixed_speed_m_s
            else:
                mode = "reacquire_confirm"
                linear = tuning.fixed_speed_m_s if tuning.maintain_cruise_during_search else min(self.loss_command[0], tuning.fixed_speed_m_s)
            angular = self._steer(path, now, tuning, elapsed, recovering)
            # A real new observation may correct the search DIRECTION at
            # once; only confirmed consecutive observations renew budgets.
            if not tuning.recovery_turn_guard_enabled or (reliable and (not recovering or confirmed)):
                self.last_good_turn = angular
            if (abs(path.error) >= tuning.corner_start_error or path.rescue) and (not tuning.recovery_turn_guard_enabled or (reliable and (not recovering or confirmed))):
                self.last_curve_turn, self.last_curve_at = angular, now
                if recovering:
                    self.search_turn_goal = angular
        elif self.last_valid_at is None:
            self.last_time, self.last_command = now, (0., 0.)
            return 0., 0., "waiting_for_path"
        else:
            self.reacquire_streak = 0
            if not recovering:
                self.loss_started_at = self.last_valid_at
                self.loss_distance = abs(self.last_command[0])*elapsed
                self.loss_command = self.last_command
                self.search_turn_goal = self.last_curve_turn if now-self.last_curve_at <= tuning.search_max_seconds else self.last_good_turn
            if search_direction is not None:
                self.search_turn_goal = math.copysign(tuning.roundabout_search_turn_rad_s, search_direction)
            age = self.loss_age(now)
            linear = tuning.fixed_speed_m_s if tuning.maintain_cruise_during_search else min(self.loss_command[0], tuning.fixed_speed_m_s)
            recent_curve = now-self.last_curve_at <= tuning.search_max_seconds
            # A confirmed sharp-turn direction starts active recovery right
            # away, instead of driving almost straight for another 0.8s.
            active = bool(tuning.search_enabled and (search_direction is not None or age >= tuning.loss_hold_seconds or
                          (recent_curve and abs(self.last_curve_turn) >= min(tuning.search_turn_rad_s,tuning.max_turn_rad_s)*.5)))
            if active:
                # Freeze the chosen episode direction; elapsed time alone
                # must not flip a right search into the default left turn.
                direction = self.search_turn_goal
                sign = math.copysign(1., direction) if abs(direction) > .05 else tuning.search_default_direction
                angular = sign*min(tuning.max_turn_rad_s, max(tuning.search_turn_rad_s, abs(direction)))
                mode = "active_search"
                self.raw_angular = angular
            elif age < tuning.loss_hold_seconds:
                angular = float(np.clip(self.loss_command[1], -tuning.max_turn_rad_s, tuning.max_turn_rad_s))
                self.raw_angular = angular
                mode = "path_memory_bridge"
            else:
                return self._fault("search_disabled", now)

        if self.loss_started_at is not None:
            if self.loss_age(now) >= tuning.search_max_seconds:
                return self._fault("time_limit", now)
            if self.loss_distance >= tuning.search_max_distance_m:
                return self._fault("distance_limit", now)
        previous = self.last_command
        linear=min(linear,self.planned_speed(path,now,tuning,ring_tracking,ring_approach,
            recovering and self.loss_started_at is not None))
        rate=tuning.linear_decel_m_s2 if linear<previous[0] else tuning.linear_accel_m_s2
        linear = self._approach(previous[0], linear, rate*dt)
        if tuning.adaptive_speed_enabled and not path.valid:
            # A safety output ceiling, not only a slowly approached goal.
            linear=min(linear,tuning.lost_speed_m_s,tuning.fixed_speed_m_s)
        fast_turn = self.corner_strength > 0 or path.rescue or mode == "active_search"
        gentle=tuning.recovery_turn_guard_enabled and (not path.valid or mode=="reacquire_confirm" or path.rescue or path.confidence<tuning.speed_stable_confidence)
        turn_limit=min(tuning.max_turn_rad_s,tuning.search_turn_max_rad_s if not path.valid else tuning.weak_path_turn_max_rad_s) if gentle else tuning.max_turn_rad_s
        if gentle:
            angular=float(np.clip(angular,-turn_limit,turn_limit))
            self.raw_angular=angular
        rate = tuning.recovery_angular_accel_rad_s2 if gentle else tuning.corner_angular_accel_rad_s2 if fast_turn else tuning.angular_accel_rad_s2
        # Unwind the OLD turn quickly, independently of how gently a new turn
        # should build. Crossing zero never spends leftover braking authority
        # accelerating into a new direction within the same frame.
        unwinding = abs(previous[1]) > 1e-6 and (previous[1]*angular <= 0 or abs(angular) < abs(previous[1]))
        if unwinding:
            brake_rate = rate if gentle else max(rate, tuning.unwind_angular_accel_rad_s2)
            if previous[1]*angular < 0:
                angular = self._approach(previous[1], 0., brake_rate*dt)
            else:
                angular = self._approach(previous[1], angular, brake_rate*dt)
        else:
            angular = self._approach(previous[1], angular, rate*dt)
        # Final output ceiling also cancels a stale +/-2rad/s command on the
        # FIRST lost/weak frame, rather than waiting for the slew to unwind.
        angular = float(np.clip(angular, -turn_limit, turn_limit))
        self.last_time, self.last_command = now, (linear, angular)
        return linear, angular, mode
