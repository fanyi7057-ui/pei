#!/usr/bin/env python3
"""ROS-free helpers for the opt-in fixed-speed patrol experiment.

7.25 reference sequence: road binary -> contour -> row edges -> short-gap
repair -> row midpoints. V14 retains V11 near-priority NORMALIZED errors and
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
    search_degraded_enabled: int = 1
    search_degraded_speed_m_s: float = 0.12
    search_hard_max_seconds: float = 8.0
    search_hard_max_distance_m: float = 2.6
    search_turn_rad_s: float = 0.80
    search_default_direction: int = 1
    reacquire_confirm_frames: int = 3
    maintain_cruise_during_search: int = 0
    search_speed_m_s: float = 0.20
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
    vv_dash_min_blocks: int = 4
    vv_dash_confirm_frames: int = 1
    vv_dash_roi_top_ratio: float = 0.22
    vv_dash_roi_bottom_ratio: float = 0.90
    vv_dash_max_x_spread_px80: float = 18.0
    vv_dash_min_gap_rows56: float = 3.0
    vv_dash_entry_near_ratio: float = 0.66
    roundabout_first_marker_cooldown_seconds: float = 0.5
    roundabout_first_marker_pass_distance_m: float = 0.73
    roundabout_speed_m_s: float = 0.30
    ring_gyro_exit_enabled: int = 1
    ring_gyro_exit_degrees: float = 360.0
    ring_gyro_min_distance_m: float = 0.80
    ring_gyro_exit_seek_seconds: float = 7.0
    ring_gyro_exit_seek_distance_m: float = 0.73
    ring_recovery_speed_m_s: float = 0.12
    ring_recovery_turn_rad_s: float = 0.35
    ring_recovery_seconds: float = 1.0
    roundabout_entry_gate_near_ratio: float = 0.72
    roundabout_entry_confirm_frames: int = 2
    roundabout_entry_near_hold_seconds: float = 0.25
    roundabout_entry_observe_seconds: float = 0.35
    roundabout_entry_near_advance_ratio: float = 0.03
    roundabout_entry_left_gap_ratio: float = 0.025
    roundabout_entry_branch_divergence_ratio: float = 0.015
    roundabout_retry_seconds: float = 2.0
    roundabout_retry_distance_m: float = 0.50
    roundabout_approach_memory_seconds: float = 0.80
    roundabout_approach_gap_seconds: float = 0.30
    roundabout_approach_route_delta: float = 0.30
    roundabout_path_dropout_seconds: float = 0.25
    roundabout_observed_exit_seconds: float = 0.35
    roundabout_observed_exit_agreement: float = 0.12
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
    roundabout_partial_gate_enabled: int = 1
    roundabout_partial_gate_min_blocks: int = 3
    roundabout_partial_gate_min_left_blocks: int = 2
    roundabout_partial_gate_left_ratio: float = 0.55
    roundabout_partial_gate_min_span_ratio: float = 0.16
    roundabout_partial_gate_frames: int = 4
    roundabout_partial_gate_min_seconds: float = 0.10
    roundabout_exit_corner_enabled: int = 0
    roundabout_exit_corner_min_span_ratio: float = 0.12
    roundabout_exit_corner_min_shift_ratio: float = 0.08
    roundabout_entry_turn_rad_s: float = 1.20
    roundabout_entry_acquire_yaw_deg: float = 55.0
    roundabout_entry_commit_enabled: int = 1
    roundabout_entry_min_observed_span_ratio: float = 0.16
    roundabout_entry_check_seconds: float = 2.0
    roundabout_entry_required_turn_deg: float = 30.0
    roundabout_entry_acquire_max_seconds: float = 3.0
    roundabout_heading_enabled: int = 1
    roundabout_heading_source: int = 0  # 0 valid IMU then odom; 1 IMU only; 2 odom only
    roundabout_heading_timeout_seconds: float = 0.35
    roundabout_heading_max_rate_rad_s: float = 6.0
    roundabout_heading_disagreement_deg: float = 45.0
    roundabout_require_exit_route: int = 1
    roundabout_exit_acquire_max_seconds: float = 8.0
    roundabout_exit_commit_enabled: int = 1
    roundabout_exit_commit_seconds: float = 5.0
    roundabout_exit_commit_distance_m: float = 3.5
    roundabout_exit_commit_max_delta: float = 0.28
    roundabout_exit_commit_straight_frames: int = 3
    roundabout_exit_recovery_seconds: float = 0.40
    roundabout_loop_min_turn_deg: float = 300.0
    roundabout_loop_return_tolerance_deg: float = 35.0
    roundabout_loop_min_distance_m: float = 1.0
    roundabout_loop_min_coherence: float = 0.65
    roundabout_visual_exit_min_turn_deg: float = 240.0
    roundabout_follow_speed_multiplier: float = 1.30
    roundabout_follow_boost_delay_seconds: float = 1.0
    roundabout_follow_boost_min_turn_deg: float = 100.0
    roundabout_scale_tracking_turn: int = 1
    roundabout_yaw_exit_recovery_seconds: float = 0.80
    roundabout_yaw_exit_heading_gain: float = 1.40
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
    crosswalk_resume_tracking: int = 1
    crosswalk_ignore_after_stop_seconds: float = 5.0
    crosswalk_heading_gain: float = 1.2
    finish_after_second_crosswalk: int = 1
    second_crosswalk_finish_distance_m: float = 1.0
    crosswalk_task_max_seconds: float = 12.0

    ring_candidate_hold_seconds: float = 0.6
    ring_candidate_turn_limit_rad_s: float = 0.6
    ring_entry_progress_max_seconds: float = 6.0
    corner_exit_confirm_frames: int = 3
    corner_exit_error: float = 0.12
    remote_guard_enabled: int = 1
    remote_auto_channel_index: int = 4
    remote_auto_channel_min: int = 1792
    remote_signal_timeout_seconds: float = 0.35
    ring_curve_reacquire_enabled: int = 1
    ring_curve_reacquire_max_shift_ratio: float = 0.22
    ring_prediction_seconds: float = 0.35
    ring_prediction_max_shift_ratio: float = 0.10

    def validate(self):
        if not math.isfinite(self.search_speed_m_s) or not 0 < self.search_speed_m_s <= .30:
            raise ValueError("search_speed_m_s必须在(0,0.30]米/秒")
        """Reject an entire bad save; never silently clip a requested speed."""
        for field in fields(self):
            value = getattr(self, field.name)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{field.name} 必须是数值（开关用0/1）")
            if not math.isfinite(value):
                raise ValueError(f"{field.name} 必须是有限数")
        if self.remote_guard_enabled not in (0,1) or int(self.remote_auto_channel_index)!=self.remote_auto_channel_index or not 0<=self.remote_auto_channel_index<10:
            raise ValueError("遥控保护须0/1，SBUS通道下标须0..9整数")
        if int(self.remote_auto_channel_min)!=self.remote_auto_channel_min or not 192<=self.remote_auto_channel_min<=2047 or not .1<=self.remote_signal_timeout_seconds<=1:
            raise ValueError("自动档门槛须192..2047整数，信号期限须0.1..1秒")
        if not 0<self.ring_gyro_exit_seek_distance_m<=1.0:
            raise ValueError("环岛出口低速寻找距离须在(0,1.0]米")
        if not 0<self.ring_prediction_seconds<=.5 or not 0<self.ring_prediction_max_shift_ratio<=.15:
            raise ValueError("环内预测期限须在(0,.5]秒，最大位移须在(0,.15]图宽")
        if self.ring_curve_reacquire_enabled not in (0,1) or not .12<=self.ring_curve_reacquire_max_shift_ratio<=.25:
            raise ValueError("环内虚线重定位开关须0/1，最大平移范围须在0.12..0.25图宽")
        if not self.roundabout_entry_acquire_max_seconds < self.ring_entry_progress_max_seconds <= self.roundabout_route_max_seconds:
            raise ValueError("有效虚线延长获取期限必须大于初始获取期限且不超过环岛控制上限")
        if int(self.corner_exit_confirm_frames)!=self.corner_exit_confirm_frames or not 2<=self.corner_exit_confirm_frames<=10 or not 0<self.corner_exit_error<=.25:
            raise ValueError("出弯确认须为2..10整数帧，回正误差阈值须在(0,.25]")
        if not 0 < self.ring_candidate_hold_seconds <= 2 or not 0 < self.ring_candidate_turn_limit_rad_s <= self.max_turn_rad_s:
            raise ValueError("环岛候选保持时间须在(0,2]秒，候选转向限制须为正且不超过总角速度限制")
        for name in ("roundabout_approach_memory_seconds","roundabout_approach_gap_seconds","roundabout_path_dropout_seconds","roundabout_observed_exit_seconds"):
            if not 0<getattr(self,name)<=2:
                raise ValueError(f"{name} 必须在(0,2]秒")
        if self.roundabout_approach_gap_seconds>self.roundabout_approach_memory_seconds:
            raise ValueError("入口短缺帧窗口不能大于独立主路记忆窗口")
        for name in ("roundabout_approach_route_delta","roundabout_observed_exit_agreement"):
            if not 0<getattr(self,name)<=1:
                raise ValueError(f"{name} 必须在(0,1]")
        if self.crosswalk_visual_release_enabled!=0 or self.crosswalk_resume_tracking!=1:
            raise ValueError("V14禁止横道越级放行/盲直行：visual_release_enabled须0，resume_tracking须1")
        for name,low,high in (("vv_dash_min_blocks",2,12),("vv_dash_confirm_frames",1,10)):
            value=getattr(self,name)
            if int(value)!=value or not low<=value<=high:
                raise ValueError(f"{name} 必须为{low}..{high}整数")
        if not 0<=self.vv_dash_roi_top_ratio<self.vv_dash_roi_bottom_ratio<=1:
            raise ValueError("VV虚线ROI上下界必须有序且位于0..1")
        if not self.vv_dash_roi_top_ratio<self.vv_dash_entry_near_ratio<=self.vv_dash_roi_bottom_ratio:
            raise ValueError("VV入口近端比例必须位于虚线ROI内")
        if not 0<self.vv_dash_max_x_spread_px80<=40 or not 0<=self.vv_dash_min_gap_rows56<=10:
            raise ValueError("VV虚线列宽/间隙超出80x56参考网格范围")
        if self.roundabout_first_marker_cooldown_seconds<=0 or self.roundabout_first_marker_pass_distance_m<=0:
            raise ValueError("首次标记冷却时间和通过距离必须为正数")
        if self.roundabout_first_marker_cooldown_seconds>=self.roundabout_approach_max_seconds:
            raise ValueError("首次标记冷却必须短于入口搜寻最长时间，避免永远等不到入环")
        if self.roundabout_speed_m_s<=0 or self.ring_gyro_min_distance_m<=0:
            raise ValueError("环岛速度和陀螺仪最低实测行程须为正数")
        if not 0<self.ring_recovery_speed_m_s<=self.roundabout_speed_m_s or self.ring_recovery_turn_rad_s<=0 or self.ring_recovery_seconds<=0:
            raise ValueError("环岛恢复速度须大于0且不超过环岛上限，转向和恢复期限须为正数")
        if self.ring_gyro_exit_enabled not in (0,1) or not 180<=self.ring_gyro_exit_degrees<=720:
            raise ValueError("陀螺仪保底开关须0/1，目标角度须180..720度")
        if not 0<self.ring_gyro_exit_seek_seconds<=self.roundabout_exit_acquire_max_seconds:
            raise ValueError("短时找出口须大于0且不超过出口获取期限")
        for name in ("roundabout_entry_observe_seconds", "roundabout_retry_seconds"):
            if not math.isfinite(getattr(self,name)) or getattr(self,name)<=0:
                raise ValueError(f"{name} 必须为有限正数")
        for name in ("roundabout_entry_near_advance_ratio", "roundabout_entry_left_gap_ratio", "roundabout_entry_branch_divergence_ratio"):
            if not 0<getattr(self,name)<.5:
                raise ValueError(f"{name} 必须在(0,0.5)")
        if not math.isfinite(self.roundabout_retry_distance_m) or self.roundabout_retry_distance_m<0:
            raise ValueError("环岛重找距离必须有限非负")
        for name in ("roundabout_partial_gate_enabled", "roundabout_require_exit_route", "roundabout_exit_commit_enabled", "roundabout_entry_commit_enabled"):
            if getattr(self,name) not in (0,1):
                raise ValueError(f"{name} 必须为0或1")
        if self.roundabout_follow_boost_delay_seconds<0 or not 0<=self.roundabout_follow_boost_min_turn_deg<=360:
            raise ValueError("环内倍率延迟须>=0、累计实测转角门须在0..360度")
        if not 0<self.roundabout_exit_recovery_seconds<=2:
            raise ValueError("出口弱路径恢复窗口须在(0,2]秒，不能靠弱观测无限续期")
        for name in ("roundabout_partial_gate_min_seconds", "roundabout_exit_acquire_max_seconds", "roundabout_exit_commit_seconds", "roundabout_exit_commit_distance_m"):
            if getattr(self,name)<=0:
                raise ValueError(f"{name} 必须>0，关闭功能请使用对应开关")
        for name in ("roundabout_partial_gate_left_ratio", "roundabout_partial_gate_min_span_ratio", "roundabout_exit_commit_max_delta"):
            if not 0<getattr(self,name)<=1:
                raise ValueError(f"{name} 必须在(0,1]")
        for name in ("roundabout_partial_gate_frames", "roundabout_exit_commit_straight_frames"):
            if int(getattr(self,name))!=getattr(self,name) or not 1<=getattr(self,name)<=20:
                raise ValueError(f"{name} 必须是1..20整数")
        if not 0<self.roundabout_heading_disagreement_deg<=180:
            raise ValueError("heading_disagreement须在(0,180]度，冲突时禁止偏航证明而非盲信IMU")
        if (int(self.roundabout_partial_gate_min_blocks)!=self.roundabout_partial_gate_min_blocks
                or not 3<=self.roundabout_partial_gate_min_blocks<=10
                or int(self.roundabout_partial_gate_min_left_blocks)!=self.roundabout_partial_gate_min_left_blocks
                or not 2<=self.roundabout_partial_gate_min_left_blocks<=self.roundabout_partial_gate_min_blocks):
            raise ValueError("部分入口须3..10块，左侧块数为2..总块数整数")
        for name in ("enabled", "black_enabled", "contour_enabled", "search_enabled", "roundabout_enabled", "roundabout_timeout_enabled", "roundabout_left_preview_enabled", "maintain_cruise_during_search", "crosswalk_enabled", "crosswalk_visual_release_enabled", "finish_after_second_crosswalk"):
            if getattr(self, name) not in (0, 1):
                raise ValueError(f"{name} 必须为0或1")
        for name in ("roundabout_gate_tracking_enabled", "roundabout_exit_corner_enabled"):
            if getattr(self, name) not in (0, 1):
                raise ValueError(f"{name} 必须为0或1")
        for name in ("search_degraded_enabled", "roundabout_heading_enabled", "roundabout_scale_tracking_turn", "crosswalk_resume_tracking"):
            if getattr(self,name) not in (0,1):
                raise ValueError(f"{name} 必须为0或1")
        if self.roundabout_heading_source not in (0,1,2):
            raise ValueError("roundabout_heading_source必须为0自动/1IMU/2odom")
        for name in ("search_degraded_speed_m_s", "roundabout_entry_turn_rad_s", "roundabout_entry_check_seconds", "roundabout_entry_acquire_max_seconds", "roundabout_heading_timeout_seconds", "roundabout_heading_max_rate_rad_s", "roundabout_follow_speed_multiplier", "roundabout_yaw_exit_heading_gain"):
            if getattr(self,name)<=0:
                raise ValueError(f"{name} 必须大于0")
        if self.search_hard_max_seconds<=self.search_max_seconds or self.search_hard_max_distance_m<=self.search_max_distance_m:
            raise ValueError("最终定位保护边界必须大于低速恢复起点")
        if self.roundabout_entry_acquire_max_seconds<=self.roundabout_entry_check_seconds:
            raise ValueError("入环路线获取期限必须大于早期转向检查时间")
        for name in ("roundabout_entry_min_observed_span_ratio", "roundabout_loop_min_coherence"):
            if not 0<getattr(self,name)<=1:
                raise ValueError(f"{name} 必须在(0,1]")
        if not 0<self.roundabout_loop_return_tolerance_deg<90 or not 180<=self.roundabout_loop_min_turn_deg<=360:
            raise ValueError("环行回向容差须在0..90度，最少累计转角在180..360度")
        for name in ("roundabout_entry_acquire_yaw_deg", "roundabout_entry_required_turn_deg", "roundabout_visual_exit_min_turn_deg"):
            if not 0<=getattr(self,name)<=360:
                raise ValueError(f"{name} 必须在0..360度")
        if self.roundabout_loop_min_distance_m<=0 or self.crosswalk_ignore_after_stop_seconds<0:
            raise ValueError("环行最小实测行程必须>0，横道冷却时间必须>=0")
        if not all(math.isfinite(value*self.roundabout_follow_speed_multiplier) for value in (self.fixed_speed_m_s,self.steering_gain,self.steering_kd,self.angular_accel_rad_s2)):
            raise ValueError("环内倍率与速度/增益相乘必须仍为有限数；不是额外限速")
        if not 0<=self.roundabout_yaw_exit_recovery_seconds<=2.:
            raise ValueError("回向后无路线短恢复时间必须在0..2秒；0关闭")
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


class PartialGateTracker:
    """Entry-only close/partial dashed chain; never three arbitrary black blobs.

    Components still come from the baseline neutral/bright-surround filter.
    Search a small, bounded family of longitudinal chains, then associate the
    same chain over fresh frames. A zebra's horizontal row cannot pass.
    """
    def __init__(self):
        self.reset()

    def reset(self):
        self.anchor=self.at=self.since=None
        self.frames=0
        self.chain=[]
        self.reason="partial_no_chain"

    def update(self,components,width,height,now,tuning):
        if not tuning.roundabout_partial_gate_enabled:
            self.reset()
            return False
        if self.at is not None and now<=self.at:
            return False
        items=sorted(components,key=lambda c:c[4],reverse=True)[:24]
        items=sorted(items,key=lambda c:c[1])
        chains=[[c] for c in items]
        best=[]
        for i,c in enumerate(items):
            for j,p in enumerate(items[:i]):
                dy=(c[1]-p[1])/height
                if .025<=dy<=.22 and abs(c[0]-p[0])/width<=.075+.35*dy:
                    proposal=chains[j]+[c]
                    if len(proposal)>len(chains[i]):
                        chains[i]=proposal
            chain=chains[i]
            if len(chain)<tuning.roundabout_partial_gate_min_blocks:
                continue
            a=np.asarray(chain,float)
            xs,ys=a[:,0]/width,a[:,1]/height
            gaps=np.diff(ys)
            coeff=np.polyfit(ys,xs,1)
            rmse=float(np.sqrt(np.mean((xs-np.polyval(coeff,ys))**2)))
            if (np.ptp(ys)<tuning.roundabout_partial_gate_min_span_ratio
                    or np.count_nonzero(xs<tuning.roundabout_partial_gate_left_ratio)<tuning.roundabout_partial_gate_min_left_blocks
                    or ys[-1]<.45 or np.std(gaps)/max(np.mean(gaps),1e-6)>.90
                    or rmse>.035 or xs[0]>xs[-1]+.04
                    or np.max(a[:,2])/max(np.min(a[:,2]),1)>3.5
                    or np.max(a[:,3])/max(np.min(a[:,3]),1)>3.5):
                continue
            if len(chain)>len(best):
                best=chain
        if not best:
            self.reset()
            return False
        xs=np.array([c[0] for c in best])/width
        ys=np.array([c[1] for c in best])/height
        anchor=(float(np.polyval(np.polyfit(ys,xs,1),.60)),float(ys[-1]))
        related=(self.anchor is not None and 0<now-self.at<=tuning.roundabout_gate_track_max_gap_seconds
                 and abs(anchor[0]-self.anchor[0])<=tuning.roundabout_gate_track_max_x_ratio
                 and abs(anchor[1]-self.anchor[1])<=.15)
        if not related:
            self.frames,self.since=0,now
        self.frames+=1
        self.anchor,self.at,self.chain=anchor,now,best
        ready=self.frames>=tuning.roundabout_partial_gate_frames and now-self.since>=tuning.roundabout_partial_gate_min_seconds
        self.reason="partial_tracked" if ready else "partial_confirming"
        return ready


class EntryApproachProof:
    """A moving SAME left branch, not a distant/parallel exit dashed chain.

    This is a conservative geometric cue, not a physical entrance classifier.
    Coordinates are component CENTERS from the original detector.
    """
    def __init__(self):
        self.reset()

    def reset(self):
        self.anchor=self.at=self.since=self.first_near=None
        self.frames=0
        self.advance=self.left_gap=self.divergence=0.
        self.reason="entry_no_chain"

    def update(self,chain,reference,width,height,now,tuning,moving=True):
        if self.at is not None and now==self.at:
            return False  # duplicate image never adds a frame or renews proof
        if not moving:
            self.reset();self.reason="entry_not_moving";return False
        if len(chain)<3 or reference is None or len(reference)<2:
            # A brief real-observation dropout neither proves entry nor
            # destroys an otherwise associated chain. It never renews age.
            if self.at is None or now-self.at>tuning.roundabout_approach_gap_seconds or now<self.at:
                self.reset()
            self.reason="entry_missing_chain" if len(chain)<3 else "entry_missing_independent_reference"
            return False
        a=np.asarray(chain,float)
        order=np.argsort(a[:,1]); a=a[order]
        ref=np.asarray(reference,float);ref=ref[np.argsort(ref[:,1])]
        xs,ys=a[:,0]/width,a[:,1]/height
        if np.ptp(ys)<tuning.roundabout_partial_gate_min_span_ratio:
            self.reset();self.reason="entry_chain_span";return False
        # Never extrapolate a remembered route into unseen far-field geometry.
        inside=(a[:,1]>=ref[0,1])&(a[:,1]<=ref[-1,1])
        if np.count_nonzero(inside)<2:
            self.reset();self.reason="entry_reference_span";return False
        offsets=(np.interp(a[inside,1],ref[:,1],ref[:,0])-a[inside,0])/width
        self.left_gap=float(np.median(offsets))
        self.divergence=float(offsets[0]-offsets[-1])
        if (self.left_gap<tuning.roundabout_entry_left_gap_ratio
                or self.divergence<tuning.roundabout_entry_branch_divergence_ratio):
            self.reset();self.reason="entry_not_left_opening_branch";return False
        anchor=(float(np.polyval(np.polyfit(ys,xs,1),.60)),float(np.max(ys+a[:,3]/height*.5)))
        related=(self.anchor is not None and 0<now-self.at<=tuning.roundabout_approach_gap_seconds
                 and abs(anchor[0]-self.anchor[0])<=tuning.roundabout_gate_track_max_x_ratio
                 and -.12<=anchor[1]-self.anchor[1]<=.15)
        if not related:
            self.frames=0;self.since=now;self.first_near=anchor[1]
        self.frames+=1
        self.advance=max(0.,anchor[1]-self.first_near)
        self.anchor,self.at=anchor,now
        ready=(now-self.since>=tuning.roundabout_entry_observe_seconds
               and self.frames>=tuning.roundabout_entry_confirm_frames
               and self.advance>=tuning.roundabout_entry_near_advance_ratio
               and anchor[1]>=tuning.roundabout_entry_gate_near_ratio)
        self.reason="entry_left_branch_ready" if ready else "entry_wait_approaching_branch"
        return ready


class ApproachRouteMemory:
    """Bounded independent main-route reference and candidate-only ownership.

    A dashed chain cannot refresh the road against which it is evaluated.
    No fabricated path is returned on missing observations.
    """
    def __init__(self):
        self.reset()

    def reset(self):
        self.path=None
        self.at=None
        self.reason="approach_no_reference"

    def reference(self,now,tuning):
        if self.at is None or not 0<=now-self.at<=tuning.roundabout_approach_memory_seconds:
            self.reset()
            return None
        return self.path

    @staticmethod
    def independent(path,chain,width):
        if not chain:
            return True
        pts=path.points[np.argsort(path.points[:,1])]
        overlapping=[c for c in chain if pts[0,1]<=c[1]<=pts[-1,1]]
        if len(overlapping)<2:
            return False
        delta=[abs(np.interp(c[1],pts[:,1],pts[:,0])-c[0])/width for c in overlapping]
        return float(np.median(delta))>.025

    def remember(self,path,chain,width,now,tuning):
        if (not path.valid or not path.route_observed or path.rescue or path.handoff
                or path.points is None or len(path.points)<2
                or not self.independent(path,chain,width)):
            return False
        previous=self.reference(now,tuning)
        if previous is not None and max(abs(path.near_error-previous.near_error),abs(path.mid_error-previous.mid_error))>tuning.roundabout_approach_route_delta:
            return False
        self.path=replace(path,points=path.points.copy(),mask=None)
        self.at=now
        self.reason="approach_independent_reference"
        return True

    def select(self,black,road,now,tuning):
        ref=self.reference(now,tuning)
        if ref is None:
            return None
        candidates=[p for p in (black,road) if p.valid and p.route_observed and not p.rescue and p.points is not None
                    and max(abs(p.near_error-ref.near_error),abs(p.mid_error-ref.mid_error))<=tuning.roundabout_approach_route_delta]
        if not candidates:
            self.reason="approach_missing_current_route"
            if now-self.at<=tuning.roundabout_approach_gap_seconds:
                # Let the unchanged V11 short-loss recovery bridge this gap,
                # not a just-appeared opposite branch. No fabricated pixels.
                return PathEstimate(False,reason="approach_route_gap")
            return None  # bounded wait only; never lock out ordinary rescue
        chosen=min(candidates,key=lambda p:max(abs(p.near_error-ref.near_error),abs(p.mid_error-ref.mid_error)))
        self.reason="approach_same_route"
        return replace(chosen)  # caller retains original normal estimates


class ExitRouteCommit:
    """Prefer the confirmed EXIT road, not five seconds of blind straight yaw.

    Does not mint a path, renew its deadline or require positive steering.
    Black fallback must agree with the same route; ordinary bends remain valid.
    """
    def __init__(self):
        self.reset()

    def reset(self):
        self.path=None
        self.started_at=None
        self.last_at=None
        self.observed_at=None
        self.distance=0.
        self.straight_frames=0
        self.reason="inactive"

    def begin(self,path,now,tuning):
        self.reset()
        if not tuning.roundabout_exit_commit_enabled or not path.valid or not path.route_observed or path.rescue:
            self.reason="no_confirmed_exit_geometry"
            return False
        self.started_at=self.last_at=now
        self.observed_at=now
        self.path=replace(path,mask=None,exit_boundary=None)
        self.reason="exit_contour_priority"
        return True

    def active(self,now,tuning):
        return (bool(tuning.roundabout_exit_commit_enabled) and self.started_at is not None
                and 0<=now-self.started_at<tuning.roundabout_exit_commit_seconds
                and self.distance<tuning.roundabout_exit_commit_distance_m)

    def select(self,black,road,now,command_speed,tuning):
        if not self.active(now,tuning):
            self.reason="expired" if self.started_at is not None else "inactive"
            return None
        if self.last_at is not None:
            self.distance+=abs(command_speed)*min(.1,max(0.,now-self.last_at))
        self.last_at=now
        if not self.active(now,tuning):
            self.reason="distance_expired"
            return None
        def agrees(p):
            if not p.valid or self.path is None:
                return False
            weak=p.rescue or not p.route_observed
            if weak and (self.observed_at is None or now-self.observed_at>tuning.roundabout_exit_recovery_seconds
                         or p.points is None or len(p.points)<1):
                return False
            # A single near band has no independently observed middle row.
            delta=abs(p.near_error-self.path.near_error)
            if not p.rescue:
                delta=max(delta,abs(p.mid_error-self.path.mid_error))
            return delta<=tuning.roundabout_exit_commit_max_delta
        # An unbranched observed contour is preferred, not the largest region.
        if agrees(road):
            candidate=replace(road,source="road")
            self.reason="exit_contour_priority"
        elif black_usable(black,tuning) and agrees(black):
            candidate=replace(black,source="black")
            self.reason="exit_same_route_black_fallback"
        else:
            self.straight_frames=0
            if self.observed_at is not None and now-self.observed_at>tuning.roundabout_exit_recovery_seconds:
                self.reset()
                self.reason="exit_unobservable_release"
                return None
            self.reason="exit_route_missing"
            return PathEstimate(False,reason=self.reason)
        straight=(candidate.branch_rows==0 and abs(candidate.near_error)<.18
                  and abs(candidate.mid_error-candidate.near_error)<.18)
        self.straight_frames=self.straight_frames+1 if straight else 0
        if self.straight_frames>=tuning.roundabout_exit_commit_straight_frames:
            self.reason+="_straight"
        # Only stable observations move the anchor; a pending branch cannot
        # repeatedly inch the route identity back onto the roundabout.
        if candidate.branch_rows==0 and candidate.route_observed and not candidate.rescue:
            self.path=replace(candidate,mask=None,exit_boundary=None)
            self.observed_at=now
        return candidate


class RingRouteMemory:
    """Preserve route ownership across ENTRY/FOLLOW and temporary path loss.

    This stores geometry, not a forced yaw sign. A continuous right correction
    remains valid. Expired memory does not authorize any arbitrary outside line.
    """
    def __init__(self):
        self.reset()

    def reset(self):
        self.path = self.at = None

    def agrees(self, path, now, tuning):
        if self.path is None or self.at is None or now-self.at > tuning.roundabout_route_memory_seconds:
            return None
        return max(abs(path.near_error-self.path.near_error), abs(path.mid_error-self.path.mid_error)) <= tuning.roundabout_route_max_delta

    def remember(self, path, now):
        if path.valid and path.route_locked and path.route_observed and not path.handoff:
            self.path = replace(path, points=None, mask=None, exit_boundary=None)
            self.at = now


def observed_route(path, width, height, tuning):
    """Multi-row near geometry; do not require the exact 60% row to be visible.

    At a tight entry the far half may leave frame while 3 real near bands still
    show the bend. Their actual vertical span matters; one rescue band cannot
    acquire ownership. Width-based single-edge estimates are separately gated.
    """
    if not path.valid or path.rescue or path.points is None or path.near_gap_ratio > .02:
        return False
    points = np.asarray(path.points).reshape(-1, 2)
    points = points[np.isfinite(points).all(axis=1)]
    return bool(len(points) >= 2 and np.ptp(points[:,1]) >= tuning.roundabout_entry_min_observed_span_ratio*height
                and np.max(points[:, 1]) >= (tuning.road_near_y_ratio-.08)*height)


class MeasuredRingTurn:
    """Unwrap fresh MEASURED yaw, never integrate commanded angular velocity.

    Source changes/gaps/jumps invalidate loop proof for this attempt. A return
    to the original heading without net rotation is not a completed circle.
    """
    def __init__(self):
        self.reset()

    def reset(self):
        self.source="none"
        self.angle=self.absolute=self.travel=0.
        self.observed_seconds=0.
        self.last_yaw=self.stamp=None
        self.reliable=True
        self.reason="waiting_measured_yaw"

    @property
    def available(self):
        return self.stamp is not None and self.reliable

    def fresh(self,now,tuning):
        return self.available and 0<=now-self.stamp<=tuning.roundabout_heading_timeout_seconds

    def update(self,sample,now,moving,measured_travel,tuning):
        if not tuning.roundabout_heading_enabled:
            return
        if moving:
            self.travel+=max(0.,measured_travel)
        if sample is None or not moving:
            if self.stamp is not None and now-self.stamp>tuning.roundabout_heading_timeout_seconds:
                self.reliable=False
                self.reason="heading_gap"
            return
        source,yaw,stamp=sample
        if source not in ("imu","odom") or not all(math.isfinite(x) for x in (yaw,stamp)) or not 0<=now-stamp<=tuning.roundabout_heading_timeout_seconds:
            return
        if self.stamp is None:
            self.source,self.last_yaw,self.stamp=source,yaw,stamp
            self.reason="measured_turn"
            return
        if stamp<=self.stamp:
            return
        dt=stamp-self.stamp
        delta=(yaw-self.last_yaw+math.pi)%(2*math.pi)-math.pi
        if source!=self.source or dt>tuning.roundabout_heading_timeout_seconds or abs(delta)>tuning.roundabout_heading_max_rate_rad_s*dt+.05:
            self.reliable=False
            self.reason="heading_source_gap_or_jump"
        elif self.reliable:
            self.angle+=delta
            self.absolute+=abs(delta)
            self.observed_seconds+=dt
        self.source,self.last_yaw,self.stamp=source,yaw,stamp

    def loop_ready(self,now,tuning):
        if not self.fresh(now,tuning):
            return False
        tolerance=math.radians(tuning.roundabout_loop_return_tolerance_deg)
        target=math.radians(tuning.ring_gyro_exit_degrees)
        returned=abs((self.angle-target+math.pi)%(2*math.pi)-math.pi)<=tolerance
        return (tuning.ring_gyro_exit_enabled and self.angle>=target
                and self.angle<=target+tolerance and returned
                and self.travel>=tuning.ring_gyro_min_distance_m
                and self.angle/max(self.absolute,1e-6)>=tuning.roundabout_loop_min_coherence)


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


class CourseProgress:
    """Per-lap receipts. No timeout, visual zebra, or numeric counter mints one.

    Step slots are intentionally non-blocking SKIPPED_TEST, never jump success.
    Original step implementation remains separate and frozen.
    """
    def __init__(self):
        self.lap=0
        self.steps=["SKIPPED_TEST","PENDING"]
        self.rings=[False,False]
        self.zebras=[False,False]

    def confirm_ring(self):
        if self.rings[self.lap] or (self.lap==1 and not self.zebras[0]):
            return False
        self.rings[self.lap]=True
        return True

    def confirm_zebra(self):
        if not self.rings[self.lap] or self.zebras[self.lap]:
            return False
        self.zebras[self.lap]=True
        return True

    def next_lap(self):
        if self.lap!=0 or not self.zebras[0]:
            return False
        self.lap=1
        self.steps[1]="SKIPPED_TEST"
        return True

    @property
    def finish_allowed(self):
        return self.lap==1 and all(self.rings) and all(self.zebras)


class RoundaboutRouter:
    """Strict per-lap receipts: skipped step -> ring -> zebra, twice -> finish."""
    def __init__(self):
        self.phase="SEEK"
        self.started_at=self.last_time=self.phase_at=None
        self.distance=self.phase_distance=self.left_yaw=self.ring_yaw=0.
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
        self.navigation_reset_serial=0
        self.visual_release_frames=0
        self.visual_release_at=None
        self.visual_releases=0
        self.route_status="seeking_gate"
        self.path_stable_since=self.last_path_observation=None
        self.last_path_errors=None
        self.measured_turn=MeasuredRingTurn()
        self.yaw_releases=self.aborted_rings=0
        self.crosswalk_block_until=0.
        self.yaw_exit_recovery_until=0.
        self.yaw_exit_heading=None
        self._clear_exit_seek()
        self.release_serial=0
        self.follow_stable_since=None
        self.course=CourseProgress()
        self.ring_route_acquired=False
        self.retry_started_at=None
        self.retry_origin_distance=None
        self.crosswalk_observed_at=None
        self.finish_authorized=False
        self.exit_observed_since=None
        self.last_good_path_at=None

        self.first_marker_at=None
        self.first_marker_distance=0.
        self.marker_moving_seconds=0.
        self.vv_streak=0
        self.vv_entry_streak=0
        self.vv_last_at=None
        self.vv_kind=None
        self.vv_candidate_at=None
        self.entry_guide_at=None
        self.entry_detection_reason="vv_wait_first_marker"
        self.ring_recovery_until=None
        self.ring_recovery_streak=0
        self.ring_recovery_observed_at=None

    @property
    def task_name(self):
        if self.phase in ("FINISH_RUN","FINISHED"):
            return self.phase
        return f"L{self.course.lap+1}_"+("ZEBRA" if self.course.rings[self.course.lap] else "RING")

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
        if self.retry_started_at is not None:
            budget=tuning.roundabout_first_timeout_seconds if self.course.lap==0 else tuning.roundabout_second_timeout_seconds
            return self.retry_started_at+budget
        if self.course.lap==0 and self.started_at is not None:
            return self.started_at+tuning.roundabout_first_timeout_seconds
        if self.course.lap==1 and self.last_crosswalk_at is not None:
            return self.last_crosswalk_at+tuning.roundabout_second_timeout_seconds
        return None

    def _clear_exit_seek(self):
        self.loop_candidate_at=None
        self.loop_seek_pose=None
        self.loop_seek_yaw=None
        self.loop_seek_distance=0.

    def _release_ring(self,now,tuning,source):
        if source=="visual_exit":
            if (self.phase!="EXIT" or not self.ring_route_acquired
                    or not self.course.confirm_ring()):
                self.route_status="exit_without_ordered_acquisition"
                return False
        self.release_serial+=1
        self._clear_exit_seek()
        self.yaw_exit_recovery_until=0.
        self.yaw_exit_heading=None
        if source=="yaw_loop_release" and self.measured_turn.last_yaw is not None:
            reference=(self.measured_turn.last_yaw-self.measured_turn.angle+math.pi)%(2*math.pi)-math.pi
            self.yaw_exit_heading=(self.measured_turn.source,reference)
            self.yaw_exit_recovery_until=now+tuning.roundabout_yaw_exit_recovery_seconds
        self.ring_release_source=source
        if source!="visual_exit":
            self.ring_recovery_until=now+tuning.ring_recovery_seconds
            self.ring_recovery_streak=0
            self.ring_recovery_observed_at=None
            if source=="time_fallback":
                self.timed_releases+=1
            elif source=="zebra_evidence_fallback":
                self.visual_releases+=1
            elif source=="yaw_loop_release":
                self.yaw_releases+=1
            else:
                self.aborted_rings+=1
            self.navigation_reset_serial+=1
            # Release CONTROL, not the ring task. A failed attempt cannot
            # authorize zebra or finish, nor loop an expired global timer.
            self.after_ring_at=None
            self.entry_block_until=now+tuning.roundabout_retry_seconds
            self.entry_block_distance=self.distance+tuning.roundabout_retry_distance_m
            self.retry_started_at=now
            self.retry_origin_distance=self.distance
            self.ring_route_acquired=False
            self._phase("SEEK",now)
            self.route_status="retry_current_ring:"+source
            return False
        else:
            self.completed=sum(self.course.rings)
        self.release_cycle=self.course.lap
        self.after_ring_at=now
        self.retry_started_at=None
        self.retry_origin_distance=None
        self.entry_block_until=now+tuning.roundabout_rearm_seconds
        self.entry_block_distance=self.distance+tuning.roundabout_rearm_distance_m
        self._phase("SEEK_CROSSWALK",now)
        return True

    def crosswalk_visual_release_allowed(self,now,tuning):
        # Retired compatibility fields remain loadable, never an authorization.
        return False

    def observe_crosswalk_visual_release(self,detected,now,tuning):
        self.visual_release_frames=0
        self.visual_release_at=None
        return False  # compatibility API: no zebra evidence can complete a ring

    def apply_time_fallback(self,now,tuning):
        # A scheduled handoff is not a physical exit and cannot clear faults,
        # restart a paused robot, consume a zebra, or advance a finished course.
        if self.fault or self.paused_at is not None or self.course.rings[self.course.lap]:
            return False
        # Only an approach SEARCH budget. Active ring phases have their own
        # bounded phase/acquisition budgets; never interrupt them with a clock
        # started on the starting line. SEEK has no ring ownership to release.
        if self.phase not in ("APPROACH","RING_WAIT_DEADLINE"):
            return False
        deadline=self.ring_deadline(tuning)
        if deadline is not None and self.phase_at is not None:
            budget=tuning.roundabout_first_timeout_seconds if self.course.lap==0 else tuning.roundabout_second_timeout_seconds
            deadline=max(deadline,self.phase_at+budget)
        if deadline is None or now<deadline:
            return False
        self._release_ring(now,tuning,"time_fallback")
        return True

    def _phase(self,name,now):
        self.exit_observed_since=None
        self.last_good_path_at=None
        self.follow_stable_since=None
        self.crosswalk_frames=0
        self.crosswalk_observed_at=None
        preserve = self.phase in ("ENTRY", "FOLLOW") and name in ("FOLLOW", "EXIT")
        acquired, cleared = self.branch_acquired, self.gate_cleared
        self.phase,self.phase_at=name,now
        self.phase_distance=self.left_yaw=self.evidence=0.
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
        if name=="SEEK":
            self.first_marker_at=None
            self.first_marker_distance=self.distance
            self.marker_moving_seconds=0.
            self.vv_streak=0
            self.vv_entry_streak=0
            self.vv_last_at=None
            self.vv_kind=None
            self.entry_detection_reason="vv_wait_first_marker"
        if name=="ENTRY":
            self.entry_guide_at=None
            self.ring_route_acquired=False
            self.ring_yaw=0.
            self.measured_turn.reset()

    def pause(self,now):
        self.entry_guide_at=None
        self.vv_candidate_at=None
        self.vv_streak=0
        self.vv_entry_streak=0
        self.vv_last_at=None
        self.vv_kind=None
        self.ring_recovery_streak=0
        self.ring_recovery_observed_at=None
        self.exit_observed_since=None
        self.last_good_path_at=None
        self.follow_stable_since=None
        if self.paused_at is None:
            self.paused_at=now
        self.last_time=self.last_pose=None
        self.crosswalk_frames=self.near_frames=self.path_streak=0
        self.path_stable_since=self.last_path_observation=None
        self.last_path_errors=None
        self.near_seen_at=None
        self.visual_release_frames=0
        self.visual_release_at=None
        self.measured_turn.reset()  # a manually moved paused car is not loop evidence
        self._clear_exit_seek()
        self.yaw_exit_recovery_until=0.

    def gate_detection_allowed(self,now,tuning):
        if not tuning.roundabout_enabled:
            return False
        if self.ring_recovery_until is not None:
            return False
        if self.phase in ("APPROACH","ENTRY","FOLLOW","COOLDOWN"):
            return True
        elapsed=0. if self.started_at is None else now-self.started_at
        if self.retry_started_at is not None:
            unblocked=(now>=self.retry_started_at+tuning.roundabout_retry_seconds
                       and self.retry_origin_distance is not None
                       and self.distance>=self.retry_origin_distance+tuning.roundabout_retry_distance_m)
        else:
            unblocked=now>=self.entry_block_until and self.distance>=self.entry_block_distance
        return (self.phase=="SEEK" and elapsed>=tuning.roundabout_arm_seconds
                and self.distance>=tuning.roundabout_arm_distance_m
                and unblocked)

    def crosswalk_block_reason(self,now,tuning):
        if not tuning.crosswalk_enabled:
            return "disabled"
        if not self.course.rings[self.course.lap]:
            return f"lap{self.course.lap+1}_ring_not_confirmed"
        if self.course.zebras[self.course.lap]:
            return self.phase.lower()
        if self.phase!="SEEK_CROSSWALK" or self.after_ring_at is None:
            return "ring_not_complete" if self.phase in ("SEEK","APPROACH","ENTRY","FOLLOW","EXIT","COOLDOWN","RING_WAIT_DEADLINE") else self.phase.lower()
        if now-self.after_ring_at<tuning.crosswalk_delay_after_roundabout:
            return "after_ring_delay"
        if self.crosswalk_count==0 and (self.started_at is None or now-self.started_at<tuning.crosswalk_first_min_seconds):
            return "first_time_gate"
        if self.crosswalk_count and (self.last_crosswalk_at is None or now-self.last_crosswalk_at<tuning.crosswalk_repeat_min_seconds):
            return "repeat_time_gate"
        if now<self.crosswalk_block_until:
            return "crosswalk_cooldown"
        return "armed"

    def resume(self,now,clear_fault=False):
        if self.paused_at is None:
            return
        delay=max(0.,now-self.paused_at)
        for name in ("started_at","phase_at","after_ring_at","last_crosswalk_at","retry_started_at","ring_recovery_until"):
            if getattr(self,name) is not None:
                setattr(self,name,getattr(self,name)+delay)
        self.entry_block_until+=delay
        if self.crosswalk_block_until:
            self.crosswalk_block_until+=delay
        self.yaw_exit_recovery_until=0.  # never resume an old blind recovery after manual pause
        if self.fault and clear_fault:
            # Only explicit operator confirmation can grant a new task budget.
            self.fault=""
            self.phase_at=now
            self.ring_recovery_until=None
            self.ring_recovery_streak=0
            self.ring_recovery_observed_at=None
        self.paused_at=None
        self.last_time=self.last_pose=None

    def crosswalk_armed(self,now,tuning):
        return self.crosswalk_block_reason(now,tuning)=="armed"

    def candidate_active(self,now,tuning):
        """Fresh main dashes slow observation, but NEVER authorize left entry."""
        return (self.phase=="SEEK" and not self.fault and self.ring_recovery_until is None
                and self.gate_detection_allowed(now,tuning) and self.vv_candidate_at is not None
                and 0<=now-self.vv_candidate_at<=tuning.ring_candidate_hold_seconds)

    def entry_guide_fresh(self,now,tuning):
        return (self.entry_guide_at is not None
                and 0<=now-self.entry_guide_at<=max(tuning.roundabout_path_max_gap_seconds,tuning.roundabout_path_dropout_seconds))

    def entry_acquire_limit(self,now,tuning):
        return tuning.ring_entry_progress_max_seconds if self.entry_guide_fresh(now,tuning) else tuning.roundabout_entry_acquire_max_seconds

    def _observe_vv_entry(self,cue,now,tuning,elapsed,travel,command):
        """Only entrance authorizer: pass first junction, then current left fork.

        Cooling is moving-time, not a stop/forced-yaw command. With odometry,
        actual forward progress is required; command estimation is explicitly
        the existing fallback, never a measured-distance claim.
        """
        if self.vv_last_at is not None and now<=self.vv_last_at:
            return
        fresh=self.vv_last_at is None or now-self.vv_last_at<=tuning.roundabout_path_max_gap_seconds
        self.vv_last_at=now
        moving=command[0]>1e-4 and travel>1e-6
        if self.phase=="APPROACH" and moving:
            # Odom can update more slowly than image processing: using one
            # image dt per changed pose would stretch a 1s pass into 2..3s.
            # Accumulate equivalent forward-command time from measured travel.
            self.marker_moving_seconds+=travel/command[0] if self.progress_source=="odom" else elapsed
        hit=cue is not None and cue.valid
        if hit and self.gate_detection_allowed(now,tuning):
            self.vv_candidate_at=now
        kind=("entry" if cue.branch.valid else "exit" if cue.exit_branch.valid else "main") if hit else None
        self.vv_streak=(self.vv_streak+1 if fresh and kind==self.vv_kind else 1) if hit and moving else 0
        self.vv_kind=kind
        if self.phase=="SEEK":
            if self.gate_detection_allowed(now,tuning) and self.vv_streak>=tuning.vv_dash_confirm_frames:
                if cue.branch.valid:
                    self._phase("ENTRY",now)
                    self.entry_detection_reason="vv_direct_verified_entry_4"
                    return
                if not cue.exit_branch.valid:
                    self.entry_detection_reason="vv_main_only_not_exit_5"
                    return
                self.first_marker_at=now
                self.first_marker_distance=self.distance
                self.marker_moving_seconds=0.
                self._phase("APPROACH",now)
                self.entry_detection_reason="vv_first_marker_pass_5"
            return
        passed=(self.marker_moving_seconds>=tuning.roundabout_first_marker_cooldown_seconds
                and self.distance-self.first_marker_distance>=tuning.roundabout_first_marker_pass_distance_m)
        entry_hit=passed and hit and cue.branch.valid and moving
        self.vv_entry_streak=(self.vv_entry_streak+1 if fresh else 1) if entry_hit else 0
        if not passed:
            self.entry_detection_reason="vv_cooling_keep_tracking"
        elif not hit:
            self.entry_detection_reason="vv_wait_current_dashes"
        elif not cue.branch.valid:
            self.entry_detection_reason="vv_reject_return_or_no_left_fork"
        elif not moving:
            self.entry_detection_reason="vv_wait_forward_progress"
        elif self.vv_entry_streak>=tuning.vv_dash_confirm_frames:
            self._phase("ENTRY",now)
            self.entry_detection_reason="vv_current_left_diverging_entry_4"
        if self.phase=="APPROACH" and now-self.phase_at>=tuning.roundabout_approach_max_seconds:
            self._release_ring(now,tuning,"entry_candidate_expired")

    def observe(self,gate,score,now,tuning,command=(0.,0.),pose=None,
                gate_near=0.,exit_solid=False,crosswalk=False,gate_tracked=False,heading=None,entry_ready=False,dash_cue=None):
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
        self.last_pose,self.last_time=pose,now
        self.distance+=travel
        self.phase_distance+=travel
        self.left_yaw+=yaw
        if self.phase in ("ENTRY","FOLLOW","EXIT"):
            self.ring_yaw+=yaw
            self.measured_turn.update(heading,now,abs(command[0])+abs(command[1])>1e-4,measured_travel,tuning)
            if (self.progress_source=="odom" and self.measured_turn.source=="imu"
                    and self.measured_turn.fresh(now,tuning)
                    and abs(self.measured_turn.angle-self.ring_yaw)>math.radians(tuning.roundabout_heading_disagreement_deg)):
                # Unit norm + freshness do not establish semantic correctness.
                # Disagreement invalidates proof, it never silently changes
                # source halfway through a supposed circle or proves no-turn.
                self.measured_turn.reliable=False
                self.measured_turn.reason="imu_odom_turn_conflict"
        if self.fault:
            return
        if tuning.roundabout_heading_enabled and self.ring_tracking and self.measured_turn.fresh(now,tuning):
            # This releases software task ownership, NOT a claim of road/pose
            # closure. Ordinary tracking resumes; ring_count stays honest.
            if (self.phase=="FOLLOW" and self.ring_route_acquired
                    and self.measured_turn.loop_ready(now,tuning) and self.loop_candidate_at is None):
                if tuning.roundabout_require_exit_route:
                    self.loop_candidate_at=now
                    self.loop_seek_pose=tuple(pose) if pose is not None else None
                    self.loop_seek_yaw=self.measured_turn.last_yaw
                    self.loop_seek_distance=0.
                    if self.phase!="EXIT":
                        self._phase("EXIT",now)
                    self.route_status="loop_progress_seek_real_exit"
                else:
                    self._release_ring(now,tuning,"yaw_loop_release")
                    return
            age=now-self.phase_at
            angle_deg=math.degrees(self.measured_turn.angle)
            if angle_deg>tuning.ring_gyro_exit_degrees+tuning.roundabout_loop_return_tolerance_deg:
                self._release_ring(now,tuning,"yaw_overturn_release")
                return
            if (self.phase=="ENTRY" and not self.entry_guide_fresh(now,tuning) and age>=tuning.roundabout_entry_check_seconds
                    and self.measured_turn.observed_seconds>=min(.5,.75*tuning.roundabout_entry_check_seconds)
                    and angle_deg<tuning.roundabout_entry_required_turn_deg):
                self._release_ring(now,tuning,"entry_no_turn_abort")
                return
        if self.phase=="ENTRY" and now-self.phase_at>=self.entry_acquire_limit(now,tuning):
            if tuning.roundabout_timeout_enabled:
                self._release_ring(now,tuning,"entry_no_route_abort")
            else:
                self.fault="roundabout_route_timeout"
            return
        if (self.loop_candidate_at is not None
                and now-self.loop_candidate_at>=tuning.roundabout_exit_acquire_max_seconds):
            # A bounded failed handoff is NOT a completed ring and must not
            # arm the contour-only exit commitment from unobserved geometry.
            self._release_ring(now,tuning,"yaw_exit_unconfirmed")
            return
        self.apply_time_fallback(now,tuning)
        age=now-self.phase_at
        if ((self.left_lock and age>=tuning.roundabout_route_max_seconds) or
                (self.phase=="FOLLOW" and age>=tuning.roundabout_follow_max_seconds)):
            if self.ring_deadline(tuning) is not None and not self.course.rings[self.course.lap]:
                # Do not keep forcing left while waiting for the global handoff.
                # Ordinary tracking/search and its data/localization safeguards
                # still apply; no fake exit or early zebra permission is issued.
                self._release_ring(now,tuning,"route_budget_retry")
            else:
                self.fault="roundabout_route_timeout"
            return
        if self.phase in ("CW_APPROACH","FINISH_RUN") and age>=tuning.crosswalk_task_max_seconds:
            self.fault="crosswalk_motion_timeout"
            return
        # Temporal association is ENTRY-only evidence, never an exit shortcut.
        tracked = bool(gate_tracked and self.phase in ("SEEK", "APPROACH"))
        strong=gate and (score>=tuning.roundabout_gate_strong_score or tracked)
        weak=score>=tuning.roundabout_gate_weak_score or tracked
        self.clear_frames=self.clear_frames+1 if not gate and not weak else 0
        self.gate_cleared|=self.clear_frames>=tuning.roundabout_path_confirm_frames

        if self.phase in ("SEEK","APPROACH") and tuning.roundabout_enabled:
            self._observe_vv_entry(dash_cue,now,tuning,dt,travel,command)
        elif self.phase=="FOLLOW":
            actual=self.measured_turn.fresh(now,tuning) and tuning.roundabout_heading_enabled
            turn_progress=self.measured_turn.angle if actual else self.ring_yaw
            ready=(age>=tuning.roundabout_follow_min_seconds
                   and self.phase_distance>=tuning.roundabout_follow_min_distance_m
                   and turn_progress>=tuning.roundabout_exit_min_yaw_rad
                   and self.gate_cleared)
            if actual:
                ready=ready and self.measured_turn.angle>=math.radians(tuning.roundabout_visual_exit_min_turn_deg)
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
            if self.crosswalk_observed_at is not None and not 0<now-self.crosswalk_observed_at<=tuning.roundabout_path_max_gap_seconds:
                self.crosswalk_frames=0
            self.crosswalk_frames=self.crosswalk_frames+1 if self.crosswalk_armed(now,tuning) and crosswalk else 0
            self.crosswalk_observed_at=now
            if self.crosswalk_frames>=tuning.crosswalk_confirm_frames:
                if not self.course.confirm_zebra():
                    self.crosswalk_frames=0
                    return
                self.crosswalk_count=sum(self.course.zebras)
                self.last_crosswalk_at=now
                self.heading_target=pose[2] if pose is not None else None
                self.finish_authorized=self.course.finish_allowed
                self._phase("FINISH_RUN" if self.finish_authorized and tuning.finish_after_second_crosswalk else "CW_APPROACH",now)
        elif self.phase=="CW_APPROACH":
            if self.phase_distance>=tuning.crosswalk_approach_distance_m:
                self._phase("CW_STOP",now)
        elif self.phase=="CW_STOP":
            if age>=tuning.crosswalk_stop_seconds:
                self.crosswalk_block_until=now+tuning.crosswalk_ignore_after_stop_seconds
                self._phase("CW_CLEAR",now)
        elif self.phase=="CW_CLEAR":
            if age>=tuning.crosswalk_clear_seconds:
                # This is the ONLY normal release of the next-ring token.
                if self.course.next_lap():
                    self._phase("SEEK",now)
                elif self.course.finish_allowed:
                    self.finish_authorized=True
                    self._phase("FINISH_RUN",now)
        elif self.phase=="FINISH_RUN":
            if not self.course.finish_allowed or not self.finish_authorized:
                self.fault="finish_without_ordered_receipts"
            elif self.phase_distance>=tuning.second_crosswalk_finish_distance_m:
                self._phase("FINISHED",now)

    def accept_path(self,path,now,tuning):
        if not self.ring_tracking or self.fault:
            return
        if self.phase=="ENTRY" and path.valid and path.route_locked and not path.rescue and not path.handoff:
            self.entry_guide_at=now  # current actual pixels, not route completion
        if self.phase=="FOLLOW":
            self.route_status="following_owned_route" if path.valid else "left_route_recovery"
            healthy=(path.valid and path.route_observed and not path.rescue and not path.handoff
                     and path.confidence>=tuning.black_min_confidence
                     and abs(path.near_error-path.mid_error)<=tuning.roundabout_path_agreement)
            if not healthy:
                self.follow_stable_since=None
            elif self.follow_stable_since is None:
                self.follow_stable_since=now
            # Observed departure geometry can nominate EXIT without waiting
            # for the entry-chain detector to also recognize an exit marker.
            # Requires actual measured turn, acquired route, travel, and a
            # sustained straight unbranched path. Never authorizes zebra here.
            departure=(healthy and path.branch_rows==0 and path.near_gap_ratio==0
                       and path.exit_boundary is None and abs(path.near_error)<.25
                       and abs(path.mid_error-path.near_error)<=tuning.roundabout_observed_exit_agreement
                       and self.gate_cleared and self.ring_route_acquired
                       and now-self.phase_at>=tuning.roundabout_follow_min_seconds
                       and self.phase_distance>=tuning.roundabout_follow_min_distance_m
                       and self.measured_turn.fresh(now,tuning)
                       and self.measured_turn.angle>=math.radians(tuning.roundabout_visual_exit_min_turn_deg))
            gap=self.last_path_observation is None or not 0<now-self.last_path_observation<=tuning.roundabout_path_max_gap_seconds
            if not departure or gap:
                self.exit_observed_since=None
            self.last_path_observation=now
            if departure:
                if self.exit_observed_since is None:self.exit_observed_since=now
                if now-self.exit_observed_since>=tuning.roundabout_observed_exit_seconds:
                    self._phase("EXIT",now)
                    self.route_status="measured_progress_observed_departure"
            return
        good=bool(path.valid and path.route_locked)
        if self.phase=="ENTRY":
            good=good and path.route_observed and not path.rescue
            # Route acquisition is an observation, not a 55-degree timer gate.
            # FOLLOW still requires measured progress/time/travel below.
        else:
            # Fork/connector frames cannot pre-fill the stable EXIT counter.
            # Require several actual unbranched observations after guidance.
            good=good and path.route_observed and not path.rescue and path.branch_rows==0 and path.near_gap_ratio==0 and path.exit_boundary is None and abs(path.near_error-path.mid_error)<=tuning.roundabout_path_agreement
        errors=(path.near_error,path.mid_error)
        gap=self.last_path_observation is None or not 0 < now-self.last_path_observation <= tuning.roundabout_path_max_gap_seconds
        jump=self.last_path_errors is not None and max(abs(a-b) for a,b in zip(errors,self.last_path_errors))>tuning.roundabout_entry_path_delta
        short_missing=(not good and not path.valid and self.last_good_path_at is not None
                       and 0<now-self.last_good_path_at<=tuning.roundabout_path_dropout_seconds)
        if short_missing:
            self.route_status="short_observation_dropout"
            return  # no new streak, no expiry renewal, no phase transition
        if not good or (gap and (self.last_good_path_at is None or now-self.last_good_path_at>tuning.roundabout_path_dropout_seconds)) or jump:
            self.path_streak=0
            self.path_stable_since=None
        self.last_path_observation=now
        self.last_path_errors=errors if good else None
        if good:
            self.last_good_path_at=now
            if self.path_stable_since is None:
                self.path_stable_since=now
            self.path_streak+=1
        self.branch_acquired|=bool(good and (path.route_observed or self.phase=="EXIT"))
        if self.path_streak<tuning.roundabout_path_confirm_frames or not self.branch_acquired:
            self.route_status="waiting_continuous_near_mid_route" if self.phase=="ENTRY" else "waiting_exit_route"
            return
        if self.phase=="ENTRY":
            turn_progress=self.measured_turn.angle if tuning.roundabout_heading_enabled and self.measured_turn.fresh(now,tuning) else self.left_yaw
            if (now-self.phase_at>=tuning.roundabout_entry_min_seconds
                    and self.phase_distance>=tuning.roundabout_entry_distance_m
                    and turn_progress>=tuning.roundabout_entry_min_yaw_rad
                    and now-self.path_stable_since>=tuning.roundabout_entry_stable_seconds):
                self.ring_route_acquired=True
                self._phase("FOLLOW",now)
        elif (self.phase_distance>=tuning.roundabout_exit_distance_m and not path.rescue
              and path.branch_rows==0 and path.near_gap_ratio==0
              and abs(path.near_error-path.mid_error)<=tuning.roundabout_path_agreement):
            self._release_ring(now,tuning,"visual_exit")

    def motion_override(self,now,tuning,pose=None,path=None):
        reliable=(path is not None and path.valid and path.route_observed and not path.rescue and not path.handoff)
        if self.ring_recovery_until is not None:
            if self.ring_recovery_observed_at is None or now>self.ring_recovery_observed_at:
                contiguous=(self.ring_recovery_observed_at is not None
                            and now-self.ring_recovery_observed_at<=tuning.roundabout_path_max_gap_seconds)
                self.ring_recovery_streak=(self.ring_recovery_streak+1 if contiguous else 1) if reliable else 0
                self.ring_recovery_observed_at=now
            if self.ring_recovery_streak>=tuning.reacquire_confirm_frames:
                self.ring_recovery_until=None
                return None
            if now>=self.ring_recovery_until:
                self.fault="ring_recovery_no_reliable_path"
                return 0.,0.,"ring_recovery_stop"
            return min(tuning.ring_recovery_speed_m_s,tuning.fixed_speed_m_s),min(tuning.ring_recovery_turn_rad_s,tuning.max_turn_rad_s),"ring_failure_recovery"
        if self.phase=="EXIT" and self.loop_candidate_at is not None and not reliable:
            if not self.measured_turn.fresh(now,tuning):
                self.fault="gyro_exit_heading_stale"
                return 0.,0.,"ring_recovery_stop"
            if self.loop_seek_yaw is not None:
                error=(self.measured_turn.last_yaw-self.loop_seek_yaw+math.pi)%(2*math.pi)-math.pi
                if abs(error)>math.radians(tuning.roundabout_loop_return_tolerance_deg):
                    self.fault="gyro_exit_seek_heading_departure"
                    return 0.,0.,"ring_recovery_stop"
            if pose is not None and self.loop_seek_pose is not None:
                x,y,yaw=self.loop_seek_pose
                self.loop_seek_distance=max(0.,(pose[0]-x)*math.cos(yaw)+(pose[1]-y)*math.sin(yaw))
                if self.loop_seek_distance>=tuning.ring_gyro_exit_seek_distance_m:
                    self.fault="gyro_exit_seek_distance_no_reliable_path"
                    return 0.,0.,"ring_recovery_stop"
            if now-self.loop_candidate_at<tuning.ring_gyro_exit_seek_seconds:
                return min(tuning.ring_recovery_speed_m_s,tuning.fixed_speed_m_s),0.,"gyro_exit_seek"
            self.fault="gyro_exit_seek_no_reliable_path"
            return 0.,0.,"ring_recovery_stop"
        if self.phase=="FINISH_RUN" and (not self.course.finish_allowed or not self.finish_authorized):
            self.fault="finish_without_ordered_receipts"
            return 0.,0.,"unauthorized_finish"
        turn_progress=self.measured_turn.angle if tuning.roundabout_heading_enabled and self.measured_turn.fresh(now,tuning) else self.left_yaw
        establish=turn_progress<tuning.roundabout_entry_min_yaw_rad or ((tuning.roundabout_entry_commit_enabled or not self.branch_acquired) and turn_progress<math.radians(tuning.roundabout_entry_acquire_yaw_deg))
        if self.phase=="ENTRY" and not (reliable and path.route_locked):
            if path is not None and path.valid and path.route_locked and not path.rescue and not path.handoff:
                # Current owned pixels can steer before full acquisition proof.
                # No turn/time credit is invented; accept_path remains strict.
                turn=float(np.clip(-tuning.steering_gain*path.error,
                                   -tuning.ring_candidate_turn_limit_rad_s,tuning.ring_candidate_turn_limit_rad_s))
                return min(tuning.fixed_speed_m_s,tuning.ring_recovery_speed_m_s),turn,"ring_curve_preview"
            # Establish the verified LEFT entry before any outside/right line
            # may take over. Bounded by route timeout, not perpetual blind yaw.
            speed=min(tuning.fixed_speed_m_s,tuning.roundabout_speed_m_s)
            turn=tuning.roundabout_entry_turn_rad_s if establish else tuning.ring_recovery_turn_rad_s
            return speed if establish else min(speed,tuning.ring_recovery_speed_m_s),min(turn,tuning.max_turn_rad_s),"roundabout_left_acquire"
        if self.phase in ("CW_STOP","FINISHED"):
            return 0.,0.,"crosswalk_stop" if self.phase=="CW_STOP" else "finished"
        if self.phase=="CW_CLEAR":
            return None  # zebra cooldown is NOT blind straight driving
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

    def follow_boost_allowed(self,path,now,tuning):
        return (self.phase=="FOLLOW" and self.branch_acquired and path.valid and path.route_observed
                and not path.rescue and path.confidence>=tuning.black_min_confidence
                and abs(path.near_error-path.mid_error)<=tuning.roundabout_path_agreement
                and now-self.phase_at>=tuning.roundabout_follow_boost_delay_seconds
                and self.follow_stable_since is not None
                and now-self.follow_stable_since>=tuning.roundabout_follow_boost_delay_seconds
                and self.measured_turn.fresh(now,tuning)
                and self.measured_turn.angle>=math.radians(tuning.roundabout_follow_boost_min_turn_deg))

    def exit_search_turn(self,path,now,tuning,heading):
        """Only a brief measured-heading bridge if exit has NO usable path.

        Valid fresh vision always wins. This cannot renew the controller's
        localization budget, survive a pause, or outlive heading freshness.
        """
        if (path.valid or self.phase!="SEEK_CROSSWALK" or now>=self.yaw_exit_recovery_until
                or self.yaw_exit_heading is None or heading is None):
            return None
        source,yaw,stamp=heading
        if source!=self.yaw_exit_heading[0] or not 0<=now-stamp<=tuning.roundabout_heading_timeout_seconds:
            return None
        error=(self.yaw_exit_heading[1]-yaw+math.pi)%(2*math.pi)-math.pi
        return float(np.clip(error*tuning.roundabout_yaw_exit_heading_gain,-tuning.max_turn_rad_s,tuning.max_turn_rad_s))


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


class RemoteAuthorityGuard:
    """Firmware SelectTelecontrol=1: map(SBUS[4],192,1792,0,2)==2.

    robot_mode is chassis/joint mode, NOT remote/ROS ownership. Raw samples
    only establish the switch position, never proof that motors executed ROS.
    """
    def __init__(self):
        self.stamp=None
        self.raw=None
        self.auto=False

    def update(self,channels,now,tuning):
        index=int(tuning.remote_auto_channel_index)
        if len(channels)!=10 or any(not isinstance(v,(int,np.integer)) or not 0<=v<=2047 for v in channels):
            self.stamp=None
            self.auto=False
            return
        self.stamp=now
        self.raw=int(channels[index])
        self.auto=self.raw>=tuning.remote_auto_channel_min

    def state(self,now,tuning):
        if not tuning.remote_guard_enabled:return "disabled"
        if self.stamp is None:return "unknown"
        if not 0<=now-self.stamp<=tuning.remote_signal_timeout_seconds:return "stale"
        return "auto_switch" if self.auto else "manual_switch"


class CornerExitGuard:
    """Three fresh centered near/mid observations clear only stale PD."""
    def __init__(self):
        self.streak=0
        self.last_at=None

    def observe(self,path,controller,now,tuning,allowed=True):
        fresh=self.last_at is None or 0<now-self.last_at<=tuning.roundabout_path_max_gap_seconds
        centered=(allowed and path.valid and path.route_observed and not path.rescue and not path.handoff
                  and max(abs(path.near_error),abs(path.mid_error),abs(path.error))<=tuning.corner_exit_error)
        self.streak=(self.streak+1 if fresh else 1) if centered else 0
        self.last_at=now
        if self.streak==tuning.corner_exit_confirm_frames:
            controller.reset_error()  # preserve actual previous speed/turn and smooth unwind


class FixedSpeedController:
    """Fixed cruise, explicit bounded active recovery, no automatic stop/go."""

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
        self.degraded = False
        self.corner_strength = 0.
        self.raw_angular = 0.
        self.reset_error()

    def reset_error(self):
        """Clear PD only; never manufacture a cruise restart."""
        self.filtered_error = None
        self.previous_source = ""

    def loss_age(self, now):
        return 0. if self.loss_started_at is None else max(0., now-self.loss_started_at)

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
            linear=self._approach(self.last_command[0],linear,tuning.linear_accel_m_s2*dt)
            angular=self._approach(self.last_command[1],angular,tuning.unwind_angular_accel_rad_s2*dt)
            if left_only:
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
        self.corner_strength=0.
        self.reset_error()
        return linear,angular

    def command(self, path, now, tuning, search_direction=None, search_heading_turn=None):
        if not tuning.enabled:
            self.reset()
            return 0., 0., "disabled"
        if self.fault_latched:
            return 0., 0., "localization_fault_latched"
        elapsed = max(0., now-self.last_time) if self.last_time is not None else 1/30
        self.degraded=False
        dt = min(.1, elapsed)
        recovering = self.loss_started_at is not None
        if recovering:
            # Count real issued travel and elapsed time, including brief valid
            # flickers. A single line flash cannot renew either budget.
            self.loss_distance += abs(self.last_command[0])*elapsed

        if path.valid:
            self.reacquire_streak = self.reacquire_streak+1 if recovering else 0
            confirmed = not recovering or self.reacquire_streak >= tuning.reacquire_confirm_frames
            if confirmed:
                self.last_valid_at = now
                self.loss_started_at, self.loss_distance = None, 0.
                self.loss_command = None
                mode, linear = "fixed_speed", tuning.fixed_speed_m_s
            else:
                mode = "reacquire_confirm"
                linear = tuning.fixed_speed_m_s if tuning.maintain_cruise_during_search else min(self.loss_command[0], tuning.search_speed_m_s, tuning.fixed_speed_m_s)
            angular = self._steer(path, now, tuning, elapsed, recovering)
            # A real new observation may correct the search DIRECTION at
            # once; only confirmed consecutive observations renew budgets.
            self.last_good_turn = angular
            if abs(path.error) >= tuning.corner_start_error or path.rescue:
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
            linear = tuning.fixed_speed_m_s if tuning.maintain_cruise_during_search else min(self.loss_command[0], tuning.search_speed_m_s, tuning.fixed_speed_m_s)
            recent_curve = now-self.last_curve_at <= tuning.search_max_seconds
            # A confirmed sharp-turn direction starts active recovery right
            # away, instead of driving almost straight for another 0.8s.
            active = bool(tuning.search_enabled and (search_direction is not None or age >= tuning.loss_hold_seconds or
                          (recent_curve and abs(self.last_curve_turn) >= min(tuning.search_turn_rad_s,tuning.max_turn_rad_s)*.5)))
            if search_heading_turn is not None:
                angular=float(np.clip(search_heading_turn,-tuning.max_turn_rad_s,tuning.max_turn_rad_s))
                self.raw_angular=angular
                mode="yaw_exit_recovery"
            elif active:
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
            soft_time=self.loss_age(now)>=tuning.search_max_seconds
            soft_distance=self.loss_distance>=tuning.search_max_distance_m
            if tuning.search_degraded_enabled:
                if self.loss_age(now)>=tuning.search_hard_max_seconds:
                    return self._fault("hard_time_limit",now)
                if self.loss_distance>=tuning.search_hard_max_distance_m:
                    return self._fault("hard_distance_limit",now)
                if soft_time or soft_distance:
                    self.degraded=True
                    linear=min(linear,tuning.search_degraded_speed_m_s)
                    if not path.valid and search_heading_turn is None:
                        angular=math.copysign(min(tuning.search_turn_rad_s,tuning.max_turn_rad_s),self.search_turn_goal or tuning.search_default_direction)
                        self.raw_angular=angular
                    mode="degraded_reacquire" if path.valid else "degraded_search"
            else:
                if soft_time:
                    return self._fault("time_limit", now)
                if soft_distance:
                    return self._fault("distance_limit", now)
        # Loss and unconfirmed reacquisition share a bounded search speed.
        # Do not slew down from cruise while blind: cap the issued value too.
        search_capped = self.loss_started_at is not None and not tuning.maintain_cruise_during_search
        if search_capped:
            linear = min(linear, tuning.search_speed_m_s)
        previous = self.last_command
        linear = self._approach(previous[0], linear, tuning.linear_accel_m_s2*dt)
        if search_capped:
            linear = min(linear, tuning.search_speed_m_s)
        fast_turn = self.corner_strength > 0 or path.rescue or mode in ("active_search","degraded_search")
        rate = tuning.corner_angular_accel_rad_s2 if fast_turn else tuning.angular_accel_rad_s2
        # Unwind the OLD turn quickly, independently of how gently a new turn
        # should build. Crossing zero never spends leftover braking authority
        # accelerating into a new direction within the same frame.
        unwinding = abs(previous[1]) > 1e-6 and (previous[1]*angular <= 0 or abs(angular) < abs(previous[1]))
        if unwinding:
            brake_rate = max(rate, tuning.unwind_angular_accel_rad_s2)
            if previous[1]*angular < 0:
                reliable_opposite = (recovering and path.valid and not path.rescue
                    and not path.handoff and path.confidence >= tuning.rescue_corner_confidence
                    and abs(path.error) >= tuning.corner_start_error
                    and path.near_error*path.error > 0)
                # Cancel a stale wrong-way search immediately, but start the
                # new turn from zero on the next frame (no angular jump).
                angular = 0. if reliable_opposite else self._approach(previous[1], 0., brake_rate*dt)
            else:
                angular = self._approach(previous[1], angular, brake_rate*dt)
        else:
            angular = self._approach(previous[1], angular, rate*dt)
        angular = float(np.clip(angular, -tuning.max_turn_rad_s, tuning.max_turn_rad_s))
        self.last_time, self.last_command = now, (linear, angular)
        return linear, angular, mode
