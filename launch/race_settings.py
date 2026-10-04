"""Race-wide settings.  Edit this file, rebuild, then start race.launch.py.

All values here override the same-named values in config/race_controller.yaml.
Speeds use m/s; angular speeds use rad/s; durations use seconds.
"""

RACE_SETTINGS = {
    # Master switches
    "enable_motion": True,           # False: vision/debug only, wheels never move
    "enable_step_jump": False,       # No physical step in current tuning run
    "show_debug_window": True,
    "speed_scale": 1.00,             # Multiplies normal/min line-follow speeds

    # Solid-line following: straight, tight bends, and command turn limit
    "normal_speed": 0.30,
    "min_speed": 0.16,
    "max_angular_speed": 0.95,
    "curve_slowdown": 0.85,
    "lost_search_speed": 0.03,
    "lost_search_angular": 0.65,
    "steering_kp": 2.60,
    "steering_kd": 0.065,
    "steering_error_filter_alpha": 0.78,
    "steering_slowdown_start": 0.045,
    "steering_center_deadband": 0.025,

    # Tight solid S bends only: activate after a filtered curvature-direction
    # flip.  Normal straight and ordinary single-bend parameters stay above.
    "s_curve_enabled": True,
    "s_curve_curvature_filter_alpha": 0.28,
    "s_curve_curvature_threshold": 0.12,
    "s_curve_hold_frames": 18,
    "s_curve_enter_confirm_frames": 3,
    # Reject a polynomial inferred from sparse, unrelated dash fragments.
    "black_line_fit_max_extrapolation_px": 26.0,
    "black_line_min_fit_row_span_px": 42.0,
    "s_curve_lookahead_extra": 0.35,
    "s_curve_expand_px": 28.0,
    "s_curve_preview_top_ratio": 0.30,
    "s_curve_preview_bottom_ratio": 0.58,
    "s_curve_speed_m_s": 0.20,
    "s_curve_min_speed_m_s": 0.12,
    "s_curve_max_angular_rad_s": 0.95,

    # Step: stop at stop_distance, wait for terminal confirmation, then move
    "step_approach_speed_m_s": 0.20,
    "step_stop_distance_m": 0.30,
    "jump_retreat_speed_m_s": 0.10,
    "jump_retreat_seconds": 0.50,
    "jump_runup_speed_m_s": 0.30,
    "jump_runup_seconds": 1.00,
    "jump_forward_speed_m_s": 0.50,
    "jump_pulse_seconds": 0.20,
    "landing_forward_speed_ratio": 0.55,
    "landing_forward_seconds": 1.20,

    # Roundabout manoeuvres
    # This profile begins immediately after the second large turn and stays
    # active through the island; it prevents skipping the close entry marks.
    "roundabout_prepare_speed_m_s": 0.20,
    "roundabout_prepare_min_speed_m_s": 0.14,
    "roundabout_prepare_max_angular_rad_s": 0.40,
    # Confirmed forward-converging exit: travel about 0.50 m (0.20 m farther
    # than before) with a controlled slight-left search before entry decision.
    "roundabout_marker_pass_speed_m_s": 0.30,
    "roundabout_marker_cooldown_seconds": 1.67,
    "roundabout_marker_search_ccw_bias_rad_s": 0.10,
    "roundabout_marker_evidence_seconds": 0.45,
    # The search must release after the same 1.67-second interval.
    "roundabout_marker_max_hold_seconds": 1.67,
    # Both exit and entrance are left-side curved clusters.  Only the true
    # entrance has two dashed arms that diverge toward the camera.
    "roundabout_marker_curve_min_slope": 0.06,
    "roundabout_marker_curve_min_residual_px": 1.0,
    "roundabout_fork_min_separation_px": 32.0,
    "roundabout_fork_min_divergence_px": 7.0,
    "roundabout_entry_require_diverging_fork": True,
    "roundabout_entry_allow_outward_curve_fallback": True,
    "roundabout_entry_outward_slope_min": 0.15,
    "roundabout_entry_min_travel_m": 0.30,
    "roundabout_entry_expected_side": "same",
    "roundabout_entry_approach_min_y_px": 8.0,
    "roundabout_entry_confirm_frames": 5,
    "roundabout_entry_track_max_x_jump_px": 60.0,
    "roundabout_entry_track_max_y_jump_px": 30.0,
    "roundabout_entry_y_jitter_px": 2.5,
    "roundabout_pre_entry_speed_m_s": 0.12,
    "roundabout_pre_entry_max_angular_rad_s": 0.25,
    "roundabout_pre_entry_max_line_error": 0.35,
    "roundabout_pre_entry_ccw_bias_rad_s": 0.10,
    # Dedicated tight-circle profile: it leaves approach and normal curves unchanged.
    "roundabout_track_speed_m_s": 0.09,
    "roundabout_track_min_speed_m_s": 0.06,
    "roundabout_track_max_angular_rad_s": 0.90,
    "roundabout_entry_turn_seconds": 0.50,
    "roundabout_entry_speed_m_s": 0.08,
    "roundabout_entry_angular_rad_s": 0.60,
    "roundabout_lost_continue_seconds": 1.40,
    "roundabout_lost_speed_m_s": 0.04,
    "roundabout_lost_angular_rad_s": 0.55,
    "roundabout_ccw_bias_rad_s": 0.22,
    "roundabout_min_ccw_angular_rad_s": 0.32,
    "roundabout_angular_filter_alpha": 0.55,
    # A 360-degree yaw cycle is insufficient: the chassis must also travel
    # around the island rather than spin in place.
    "roundabout_min_odom_travel_m": 0.50,
    "roundabout_min_complete_seconds": 8.0,
    "roundabout_exit_search_speed_m_s": 0.08,
    "roundabout_exit_search_timeout_s": 4.0,
    "roundabout_exit_straight_min_seconds": 0.35,

    # Solid line short-loss fallback: recent real-frame median direction only.
    "solid_reacquire_enabled": True,
    "solid_reacquire_history_frames": 5,
    "solid_reacquire_history_seconds": 0.40,
    "solid_reacquire_speed_m_s": 0.08,
    "solid_reacquire_min_speed_m_s": 0.05,
    "solid_reacquire_max_angular_rad_s": 0.55,

    # Crosswalk: lower ROI only, five wide horizontal bars, then align 6 cm
    # with the line before publishing the required full stop.
    "crosswalk_enabled": True,
    "crosswalk_require_roundabout_complete": True,
    "crosswalk_min_stripes": 5,
    "crosswalk_confirm_frames": 8,
    "crosswalk_min_confidence": 0.72,
    "crosswalk_roi_top_ratio": 0.64,
    "crosswalk_roi_bottom_ratio": 0.98,
    "crosswalk_min_dark_ratio": 0.055,
    "crosswalk_row_min_coverage": 0.35,
    "crosswalk_min_bar_width_ratio": 0.32,
    "crosswalk_length_ratio_max": 1.65,
    "crosswalk_gap_ratio_max": 2.50,
    "crosswalk_approach_speed_m_s": 0.10,
    "crosswalk_approach_distance_m": 0.06,
    "crosswalk_stop_seconds": 3.0,
    "crosswalk_debug_each_frame": True,

    # Current course rule: enable island detection after the second turn.
    "roundabout_arm_after_turns": 2,
    "skip_step_after_turns_for_test": 0,
    "test_turn_angle_rad": 0.95,
}

# Launch options which are not controller speed parameters.
LAUNCH_SETTINGS = {
    "start_step_detector": True,
    "local_display": ":0",
}
