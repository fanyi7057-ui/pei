#!/usr/bin/env python3
"""Opt-in patrol/ring test. Existing race/step source files stay unchanged."""
import os
import math
from dataclasses import replace
from pathlib import Path
import signal
import threading
import time

import cv2
import numpy as np
import icn as base
from patrol_contour_runtime import (
    FixedSpeedController, CornerExitGuard, RemoteAuthorityGuard, PathEstimate, PathSelector, PatrolFileWatcher,
    PatrolTuning, RoundaboutRouter, ExitRouteCommit, RingRouteMemory, observed_route,
    black_usable, road_centerline, road_masks, path_errors, detect_crosswalk,
)

from patrol_dash_entry import DashObservation, RingSearchPredictor, detect_vv_dashes, track_vv_curve


class PatrolVision(base.CourseVision):
    """Share baseline masks/gate primitives, with separate ordered tasks."""
    def __init__(self):
        super().__init__()
        self.selector = PathSelector()
        self.seed_x_ratio = None
        self.source = "none"
        self.last_debug = None
        self.previous_points = None
        self.previous_route_observed = False
        self.last_path_at = None
        self.last_black_at = None
        self.road_widths = None
        self.last_dual_at = None
        self.last_width_probe_at = -float("inf")
        self.black_estimate = PathEstimate(False, reason="not_processed")
        self.road_estimate = PathEstimate(False, reason="not_processed")
        self.preview_kind = "black"
        self.router = RoundaboutRouter()
        self.dash_observation = DashObservation()
        self.entry_ready = False
        self.entry_trigger = "none"
        self.exit_commit = ExitRouteCommit()
        self.ring_memory = RingRouteMemory()
        self.ring_predictor=RingSearchPredictor()
        self.ring_phase = "SEEK"
        self.gate_tracked = False
        self.motion_command = (0., 0.)
        self.motion_pose = None
        self.motion_heading = None
        self.gate_score = 0.
        self.gate_chain_count=0
        self.gate_raw_count=0
        self.partial_gate=False
        self.gate_near = self.crosswalk_score = self.exit_solid_score = 0.

    def reset_course(self):
        self.reset_path()
        self.dash_observation = DashObservation()
        self.entry_trigger = "none"
        self.router = RoundaboutRouter()
        self.ring_memory.reset()
        self.ring_phase = "SEEK"
        self.motion_command, self.motion_pose = (0., 0.), None
        self.motion_heading=None

    def reset_path(self,preserve_approach=False):
        self.ring_predictor.reset()
        self.dash_observation = DashObservation()
        self.entry_ready = False
        self.exit_commit.reset()
        self.reset_tracking()
        self._last_good_band_x = [None for _ in base.LOOKAHEAD_Y_RATIOS]
        self.selector = PathSelector()
        self.seed_x_ratio = None
        self.previous_points = None
        self.previous_route_observed = False
        self.last_path_at = None
        self.last_black_at = None
        self.road_widths = None
        self.last_dual_at = None
        self.last_width_probe_at = -float("inf")
        self.source = "none"
        self.last_debug = None
        self.black_estimate = PathEstimate(False, reason="reset")
        self.road_estimate = PathEstimate(False, reason="reset")

    def estimate(self, frame, tuning, now, draw=False):
        self.ring_prediction_used=False
        self.ring_match_reason="inactive"
        self.ring_route_delta=0.
        self.ring_candidate_points=0
        self.ring_reference_age=now-self.last_path_at if self.last_path_at is not None else -1.
        release_before=self.router.release_serial
        if self.router.paused_at is not None and self.ring_memory.at is not None:
            # Operator pause freezes task-relative memory age too; the NEW
            # image still has to agree, so moving the car cannot mint a path.
            self.ring_memory.at+=max(0.,now-self.router.paused_at)
        navigation_serial=self.router.navigation_reset_serial
        self.router.apply_time_fallback(now,tuning)
        if navigation_serial!=self.router.navigation_reset_serial:
            self.reset_path()  # discard the old left-locked contour/template
            self.ring_memory.reset()
        navigation_serial=self.router.navigation_reset_serial
        if self.last_path_at is not None and now-self.last_path_at >= tuning.loss_hold_seconds:
            self.reset_path(preserve_approach=self.router.phase in ("SEEK","APPROACH"))
        work = cv2.resize(frame, (base.PROCESS_WIDTH, base.PROCESS_HEIGHT), interpolation=cv2.INTER_AREA)
        height, width = work.shape[:2]
        # Share masks across tracking and ordered ring/zebra tasks. Never run
        # depth or jump methods; the validated original step files stay frozen.
        if tuning.black_enabled or tuning.roundabout_enabled or tuning.crosswalk_enabled:
            dark, events, hsv, _blue = self._make_dark_mask(work)
        else:
            hsv = cv2.cvtColor(work, cv2.COLOR_BGR2HSV)
            dark = np.zeros((height, width), np.uint8)
            events = dark.copy()
        blue, _white = road_masks(hsv, tuning)
        guard = cv2.dilate(blue, np.ones((3, 3), np.uint8))
        dark[guard > 0] = 0
        events[guard > 0] = 0
        gate, self.gate_score = False, 0.
        self.gate_tracked=False
        self.gate_chain_count=0
        self.gate_raw_count=0
        self.partial_gate=False
        self.gate_near=self.crosswalk_score=self.exit_solid_score=0.
        self.entry_ready=False
        self.dash_observation=DashObservation()
        if self.router.gate_detection_allowed(now,tuning):
            self.dash_observation=detect_vv_dashes(events,_white,tuning,self.seed_x_ratio)
            cue=self.dash_observation
            self.gate_chain_count=cue.blocks
            self.gate_near=cue.near_ratio
            # 5's return branch is exit evidence, never reverse entry.
            gate=cue.valid and (cue.exit_branch.valid if self.router.phase=="FOLLOW" else True)
            self.gate_score=1. if gate else 0.
            self.entry_ready=cue.branch.valid
        self.entry_chain_n=self.dash_observation.blocks
        exit_solid=False
        if tuning.roundabout_enabled and self.router.phase=="FOLLOW" and not gate:
            solid,self.exit_solid_score,box=self._detect_roundabout_exit_solid(events)
            if solid and box is not None:
                x,y,cw,ch=box
                exit_solid=(self.exit_solid_score>=tuning.roundabout_exit_solid_score
                            and (x+.5*cw)/width<=tuning.roundabout_exit_left_max_ratio
                            and ch/height>=base.ROUNDABOUT_EXIT_SOLID_MIN_HEIGHT_RATIO
                            and y/height<=base.ROUNDABOUT_EXIT_SOLID_TOP_MAX_RATIO
                            and (y+ch)/height>=base.ROUNDABOUT_EXIT_SOLID_BOTTOM_MIN_RATIO)
        crosswalk=False
        if tuning.crosswalk_enabled:
            # Diagnostic preview is independent of task permission. This raw
            # score must NEVER consume a zebra token before this lap's exit.
            visual_release=self.router.crosswalk_visual_release_allowed(now,tuning)
            if self.router.crosswalk_armed(now,tuning) or visual_release or draw:
                zebra_tuning=replace(tuning,crosswalk_min_stripes=max(tuning.crosswalk_min_stripes,tuning.crosswalk_visual_release_min_stripes)) if visual_release else tuning
                detected,self.crosswalk_score=detect_crosswalk(events,hsv,zebra_tuning)
                self.router.observe_crosswalk_visual_release(detected,now,tuning)
                crosswalk=detected and self.router.crosswalk_armed(now,tuning)
        self.router.observe(gate,self.gate_score,now,tuning,self.motion_command,self.motion_pose,
                            gate_near=self.gate_near,exit_solid=exit_solid,crosswalk=crosswalk,gate_tracked=self.gate_tracked,
                            heading=self.motion_heading,dash_cue=self.dash_observation)
        if navigation_serial!=self.router.navigation_reset_serial:
            self.reset_path()
            self.ring_memory.reset()
        if self.router.phase=="ENTRY" and self.ring_phase!="ENTRY":
            cue=self.dash_observation
            entry=self.dash_observation.branch
            self.entry_trigger=self.router.entry_detection_reason
            # Current real guide seeds identity, NOT acquisition proof.
            self.reset_path()
            self.dash_observation=cue  # retain this fresh frame for diagnostics
            self.ring_memory.reset()
            if entry.valid and entry.points is not None:
                self.ring_memory.remember(entry,now)
                self.previous_points=entry.points.copy()
                self.last_path_at=now
                self.seed_x_ratio=float(entry.points[np.argmax(entry.points[:,1]),0]/width)
        if not self.router.ring_tracking:
            self.ring_memory.reset()
        self.ring_phase=self.router.phase
        black = PathEstimate(False, reason="black_disabled")
        if tuning.black_enabled:
            if self.last_black_at is not None and now-self.last_black_at >= tuning.loss_hold_seconds:
                # Road observations must not keep an old BLACK template alive.
                self.reset_tracking()
                self._last_good_band_x = [None for _ in base.LOOKAHEAD_Y_RATIOS]
                self.last_black_at = None
            extracted = self._extract_line(work, dark, hsv, False, use_dynamic_offset=False)
            valid, error, confidence = extracted[:3]
            if valid:
                self.last_black_at = now
            band_x = extracted[8]
            points = np.array([(x, int(ratio * height)) for ratio, x in zip(base.LOOKAHEAD_Y_RATIOS, band_x) if x is not None], dtype=float).reshape(-1, 2)
            # Use the baseline near-dominant error. Its single-band rescue
            # must not be discarded by another 25..75% row-average rule.
            shift = 2*(tuning.target_x_ratio-.5)
            errors = [float(np.clip(e-shift, -1., 1.)) for e in extracted[3:6]]
            black = PathEstimate(bool(valid), float(np.clip(error-shift, -1., 1.)), confidence,
                                 points, dark, extracted[9], *errors,
                                 rescue=bool(valid and extracted[9] == "line_edge_single_rescue"))
            black.route_observed=observed_route(black,width,height,tuning)
        road = PathEstimate(False, reason="contour_not_needed")
        # While returning from road fallback, compute both until black is
        # confirmed. Sample measured dual-edge width at a low rate while
        # black is healthy; waiting until loss left no width for single-edge
        # recovery. A probe cannot replace an otherwise healthy black path.
        routing = bool(tuning.roundabout_enabled and self.router.phase in ("ENTRY","FOLLOW","EXIT"))
        width_probe=now-self.last_width_probe_at>=tuning.road_width_probe_seconds
        committing=self.exit_commit.active(now,tuning)
        if tuning.contour_enabled and (self.router.phase=="APPROACH" or routing or committing or not black_usable(black, tuning) or self.selector.source == "road" or width_probe):
            self.last_width_probe_at=now
            widths = self.road_widths if self.last_dual_at is not None and now-self.last_dual_at < tuning.road_single_edge_hold_seconds else None
            road = road_centerline(hsv, tuning, self.seed_x_ratio, self.previous_points, widths,
                                   branch="left" if self.router.left_lock and routing else "nearest", masks=(blue, _white),
                                   exit_guide=self.router.phase=="EXIT")
            if road.valid and road.single_edge and widths is None:
                road.valid, road.reason = False, "single_edge_width_expired"
            if road.valid:
                road.route_observed=observed_route(road,width,height,tuning) and (not road.single_edge or widths is not None)
            if road.valid and self.previous_points is not None and len(self.previous_points):
                previous = self.previous_points[np.argsort(self.previous_points[:, 1])]
                y = tuning.road_near_y_ratio*height
                previous_x = float(np.interp(y, previous[:, 1], previous[:, 0]))
                road_points = road.points[np.argsort(road.points[:, 1])]
                current_x = float(np.interp(y, road_points[:, 1], road_points[:, 0]))
                if abs(current_x-previous_x) > tuning.road_max_near_jump_ratio*width and not (self.router.phase=="EXIT" and road.exit_boundary is not None):
                    road.valid, road.reason = False, "road_near_disagrees_with_recent_path"
            width_agrees=not black_usable(black,tuning) or max(abs(road.near_error-black.near_error),abs(road.mid_error-black.mid_error))<=tuning.roundabout_path_agreement
            if road.valid and width_agrees and not road.single_edge and road.widths is not None and len(road.widths) >= 2:
                # Single-edge continuation NEVER renews the width deadline.
                self.road_widths = road.widths.copy()
                self.last_dual_at = now
        if routing:
            self.ring_candidate_points=len(black.points) if black.points is not None else 0
            if black.valid and self.ring_memory.path is not None:
                self.ring_route_delta=max(abs(black.near_error-self.ring_memory.path.near_error),abs(black.mid_error-self.ring_memory.path.mid_error))
            # Select a ROUTE before choosing a sensor. Ordinary black priority
            # must not pull us back onto the straight/right fork.
            if self.router.phase=="ENTRY" and road.valid and not self.router.branch_acquired and not road.route_locked:
                road.valid,road.reason=False,"entry_unacquired_right_road"
            if self.router.phase in ("ENTRY","FOLLOW") and road.valid and self.ring_memory.path is not None:
                agrees=self.ring_memory.agrees(road,now,tuning)
                left_recovery=road.route_observed and road.error<-.10 and road.mid_error<=.10
                if agrees is False or (agrees is None and self.ring_memory.path is not None and not left_recovery):
                    road.valid,road.reason=False,"road_outside_owned_ring_route"
            reference = road if road.valid else None
            if reference is None and self.previous_points is not None and self.last_path_at is not None and now-self.last_path_at < tuning.loss_hold_seconds:
                errors = path_errors(self.previous_points, width, height, tuning)
                if errors is not None:
                    reference = PathEstimate(True, errors[0], near_error=errors[1], mid_error=errors[2], far_error=errors[3])
            if road.valid and self.router.branch_acquired:
                road.route_locked = True
            if black.valid:
                agrees = reference is not None and max(abs(black.near_error-reference.near_error), abs(black.mid_error-reference.mid_error)) <= tuning.roundabout_path_agreement
                left_shape = black.error < -.10 and black.mid_error <= .10
                if self.router.phase=="FOLLOW":
                    # No contour does NOT grant an arbitrary outside line
                    # ownership. Blue visibility is NOT required: a near/mid
                    # black path agreeing with persistent memory is enough.
                    memory_agrees=self.ring_memory.agrees(black,now,tuning)
                    unseeded_center=(self.ring_memory.path is None and abs(black.near_error)<.15 and abs(black.mid_error)<.20)
                    allowed=black.route_observed and (agrees if road.valid else memory_agrees is True or (memory_agrees is None and (left_shape or unseeded_center)))
                elif self.router.left_lock and not road.valid and not self.router.branch_acquired:
                    allowed = (left_shape and (reference is None or agrees)) or (black.route_observed and self.ring_memory.agrees(black,now,tuning) is True)
                elif reference is not None:
                    allowed = agrees
                else:
                    allowed = left_shape if self.router.left_lock else False
                if self.router.phase=="EXIT" and road.valid and road.exit_boundary is not None:
                    # A validated virtual RIGHT edge defines this task's
                    # center; even a nearby straight black must not undo it.
                    allowed=False
                if allowed:
                    black.route_locked = bool((road.valid and road.route_locked) or self.router.branch_acquired or left_shape
                                             or (self.router.phase=="ENTRY" and black.route_observed and self.ring_memory.agrees(black,now,tuning) is True))
                    if road.valid:
                        black.branch_rows = road.branch_rows
                        black.exit_boundary = road.exit_boundary
                else:
                    black.valid, black.reason = False, "exit_corner_route_priority" if self.router.phase=="EXIT" and road.valid and road.exit_boundary is not None else "black_outside_locked_route"
            # Route geometry provides preview only AFTER task arming. It is
            # not a constant left yaw: a right correction is still permitted.
            for candidate in (black, road):
                if candidate.valid:
                    candidate.error = float(np.clip(candidate.near_error+tuning.roundabout_heading_gain*(candidate.mid_error-candidate.near_error), -1., 1.))
        if (self.router.phase in ("ENTRY","FOLLOW") and self.previous_points is not None
                and self.last_path_at is not None and now-self.last_path_at<=tuning.roundabout_route_memory_seconds):
            guide=track_vv_curve(events,_white,tuning,self.previous_points,reacquire=self.router.phase=="FOLLOW")
            if not guide.valid and self.router.phase=="FOLLOW":
                predicted=self.ring_predictor.reference(now,width,tuning)
                if predicted is not None:
                    guide=track_vv_curve(events,_white,tuning,predicted,reacquire=True)
                    self.ring_prediction_used=True
            self.ring_match_reason=guide.reason
            reliable_current=(black.valid and black.route_observed) or (road.valid and road.route_observed)
            if guide.valid and (guide.reason!="vv_owned_curve_reacquired" or not reliable_current):
                # Already-owned entry only: no need for the straight fork
                # to remain visible. Current measured points, never old pixels.
                guide.error=float(np.clip(guide.near_error+tuning.roundabout_heading_gain*(guide.mid_error-guide.near_error),-1.,1.))
                black=guide
        self.black_estimate, self.road_estimate = black, road
        committed=self.exit_commit.select(black,road,now,self.motion_command[0],tuning)
        if (committed is None and (self.router.phase=="APPROACH" or self.router.candidate_active(now,tuning))
                and self.dash_observation.main.valid
                and self.dash_observation.main.route_observed):
            # Current main dash chain, not a fabricated remembered path.
            # No fixed yaw or cooldown stop; ordinary controller still steers.
            committed=replace(self.dash_observation.main,source="black")
        if committed is None:
            chosen, self.source = self.selector.select(black, road, now, tuning)
        else:
            chosen,self.source=committed,committed.source if committed.valid else "none"
            if chosen.valid:
                self.selector.source=self.source
                self.selector.last_error=self.selector.last_observed_error=chosen.error
                self.selector.transition_at=-float("inf")
        self.router.accept_path(chosen, now, tuning)
        if self.router.release_serial!=release_before:
            if self.router.ring_release_source=="visual_exit":
                self.exit_commit.begin(chosen,now,tuning)
            else:
                self.exit_commit.reset()
        if self.router.ring_tracking:
            self.ring_memory.remember(chosen,now)
        if self.router.phase=="FOLLOW":
            self.ring_predictor.remember(chosen,now)
        else:
            self.ring_predictor.reset()
        self.ring_phase=self.router.phase
        if chosen.valid and chosen.points is not None and len(chosen.points):
            self.previous_points = chosen.points.copy()
            self.previous_route_observed=bool(chosen.route_observed and not chosen.rescue and not chosen.handoff)
            self.last_path_at = now
            order = np.argsort(chosen.points[:, 1])
            self.seed_x_ratio = float(np.clip(np.interp(tuning.road_near_y_ratio*height,
                                                       chosen.points[order, 1], chosen.points[order, 0]) / width, 0, 1))
        if draw:
            debug = work.copy()
            if chosen.exit_boundary is not None and len(chosen.exit_boundary)>1:
                cv2.polylines(debug,[np.rint(chosen.exit_boundary).astype(np.int32)],False,(255,0,255),2)
            if chosen.points is not None and len(chosen.points) > 1:
                points = np.round(chosen.points).astype(np.int32)
                cv2.polylines(debug, [points], False, (0, 255, 0), 2)
            elif chosen.valid and chosen.points is not None and len(chosen.points) == 1:
                cv2.circle(debug, tuple(np.round(chosen.points[0]).astype(int)), 4, (0,255,0), 2)
            elif self.previous_points is not None and len(self.previous_points) > 1:
                cv2.polylines(debug, [np.round(self.previous_points).astype(np.int32)], False, (0, 160, 255), 2)
            cv2.line(debug, (int(width*tuning.target_x_ratio), 0), (int(width*tuning.target_x_ratio), height-1), (0, 200, 255), 1)
            # Preview reference rows only; black keeps the baseline's actual
            # eight-band extraction. Rendering remains display-rate limited.
            for ratio, label, color in ((tuning.road_near_y_ratio, "near", (0,255,0)),
                                        (tuning.road_mid_y_ratio, "mid", (0,200,255)),
                                        (tuning.road_far_y_ratio, "far", (255,160,0))):
                row = min(height-1, int(ratio*height))
                cv2.line(debug, (0,row), (width-1,row), color, 1)
                cv2.putText(debug, label, (2,max(10,row-2)), cv2.FONT_HERSHEY_SIMPLEX, .3, color, 1)
            self.preview_kind = "road" if road.mask is not None and self.source != "black" else "black"
            preview = road.mask if self.preview_kind == "road" else dark
            if preview is not None:
                small = cv2.resize(preview, (width//3, height//3), interpolation=cv2.INTER_NEAREST)
                debug[-small.shape[0]:, -small.shape[1]:] = cv2.cvtColor(small, cv2.COLOR_GRAY2BGR)
            self.last_debug = cv2.resize(debug, (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_NEAREST)
        return chosen


class PatrolTestNode(base.WheelLegRaceNode):
    def __init__(self):
        self._patrol_ready = False
        config = Path(os.environ.get("PATROL_TUNING_FILE", str(Path(__file__).resolve().parents[1] / "config" / "patrol_test.yaml")))
        self.patrol_watcher = PatrolFileWatcher(config)
        self.patrol_tuning, message = self.patrol_watcher.poll()
        if self.patrol_tuning is None:
            raise ValueError(message)  # fail before any camera/control initialization
        super().__init__()
        self.vision = PatrolVision()
        self.patrol_controller = FixedSpeedController()
        self._last_display_at = -1e9
        self._stats_at = time.monotonic()
        self._stats_sequence = 0
        self._processed_count = self._displayed_count = 0
        self._capture_fps = self._process_fps = self._display_fps = 0.
        self._last_patrol_log = 0.
        self._patrol_ready = True
        from std_msgs.msg import Int16MultiArray
        self.remote_guard=RemoteAuthorityGuard()
        self.create_subscription(Int16MultiArray,"/sbus_channels",self._on_remote_channels,10)
        self._set_patrol_posture()
        self.create_subscription(base.Bool, "/patrol_test_stop", self._on_patrol_stop, 10)
        self.create_subscription(base.Bool, "/patrol_test_reset_course", self._on_patrol_reset_course, 10)
        self.get_logger().warn(
            f"巡线/环岛/横道V21（普通巡线V11基线，VV单帧4进5出）：普通目标速度={self.patrol_tuning.fixed_speed_m_s:.2f}m/s，环岛上限={self.patrol_tuning.roundabout_speed_m_s:.2f}；"
            f"环岛={self.patrol_tuning.roundabout_enabled}，横道={self.patrol_tuning.crosswalk_enabled}；台阶1/2仅保留位置、测试跳过；两圈环岛/横道完成凭据齐全才冲线，失败/超时不越级。实时文件：{config}"
        )

    def _poll_live_tuning_file(self):
        # The parent's timer calls this override. Never watch race_tuning.yaml
        # or adaptive_tuning.yaml in this test: only this file is authoritative.
        if not self._patrol_ready:
            return
        updated, message = self.patrol_watcher.poll()
        if updated is not None:
            old = self.patrol_tuning
            router = getattr(self.vision, "router", None)
            if router is not None and router.active and old.roundabout_enabled and not updated.roundabout_enabled:
                self.get_logger().error("整套参数未应用：环岛任务进行中不能关闭；先按3重置新场再保存")
                self.patrol_watcher.signature = None  # retry this saved snapshot after operator stop
                return
            if (router is not None and router.phase in ("SEEK_CROSSWALK","CW_APPROACH","CW_STOP","CW_CLEAR","FINISH_RUN")
                    and old.crosswalk_enabled and not updated.crosswalk_enabled):
                self.get_logger().error("整套参数未应用：横道/终点任务进行中不能关闭；先按3重置新场再保存")
                self.patrol_watcher.signature=None
                return
            self.patrol_tuning = updated
            posture_changed = any(getattr(old, key) != getattr(updated, key) for key in ("posture_height_m", "posture_roll_deg", "posture_pitch_deg", "posture_slide_m"))
            if posture_changed:
                self._set_patrol_posture()
            # Saving speed/gain/display changes must not throw away a trusted
            # path and manufacture a new dropout. Reset only geometry/masks.
            geometry = ("target_x_ratio", "black_enabled", "contour_enabled", "roi_top_ratio", "roi_bottom_ratio", "blue_h_min", "blue_h_max", "blue_s_min", "blue_v_min", "blue_weak_s_min", "blue_weak_br_delta", "blue_weak_min_area_ratio", "blue_context_radius_ratio", "road_blue_contrast_gray", "white_s_max", "white_v_min", "close_kernel_px", "black_gap_fill_px", "road_max_row_step_ratio", "min_road_area_ratio", "min_observed_rows_ratio", "seed_bottom_ratio", "seed_half_width_ratio", "min_road_width_ratio", "max_repair_gap_rows", "road_near_y_ratio", "road_mid_y_ratio", "road_far_y_ratio", "road_near_gap_ratio")
            if posture_changed or any(getattr(old, key) != getattr(updated, key) for key in geometry):
                self.vision.reset_path()
                self.patrol_controller.reset_error()
            if not updated.enabled:
                self._publish_zero()
                self.patrol_controller.reset()
                self.vision.reset_path()
                if router is not None:
                    router.pause(time.monotonic())
            self.get_logger().warn(
                f"{message}：固定速度 {old.fixed_speed_m_s:.2f}->{updated.fixed_speed_m_s:.2f}m/s，"
                f"最大转向={updated.max_turn_rad_s:.2f}rad/s，目标中心={updated.target_x_ratio:.3f}；无旧弯道限速覆盖"
            )
        elif message:
            self.get_logger().error(message)

    def _set_patrol_posture(self):
        t = self.patrol_tuning
        self._posture_transition.set_target(base.RacePosture.from_values(
            t.posture_height_m, t.posture_roll_deg, t.posture_pitch_deg, t.posture_slide_m,
        ), time.monotonic())

    def _on_step_execute_jump(self, _message):
        # The inherited subscription must never arm the frozen jump controller.
        return

    def _on_odom(self,message):
        super()._on_odom(message)
        # Telemetry only: do not bypass the lower-level balancing controller.
        self._patrol_odom_v=float(message.twist.twist.linear.x)
        self._patrol_odom_w=float(message.twist.twist.angular.z)

    def _on_imu(self,message):
        super()._on_imu(message)
        q=message.orientation
        values=(float(q.x),float(q.y),float(q.z),float(q.w))
        norm=math.sqrt(sum(x*x for x in values))
        covariance=getattr(message,"orientation_covariance",[])
        if (not all(math.isfinite(x) for x in values) or not .8<=norm<=1.2
                or (len(covariance) and covariance[0]<0)):
            self._patrol_imu_heading=None
            return
        yaw=base.quaternion_to_yaw(*(x/norm for x in values))
        self._patrol_imu_heading=("imu",yaw,time.monotonic())

    def _heading_snapshot(self,now,tuning,pose):
        if not tuning.roundabout_heading_enabled:
            return None
        imu=getattr(self,"_patrol_imu_heading",None)
        if tuning.roundabout_heading_source!=2 and imu is not None and 0<=now-imu[2]<=tuning.roundabout_heading_timeout_seconds:
            return imu
        if tuning.roundabout_heading_source!=1 and pose is not None:
            stamp=getattr(self,"_last_odom_time",-math.inf)
            if math.isfinite(pose[2]) and 0<=now-stamp<=tuning.roundabout_heading_timeout_seconds:
                return ("odom",float(pose[2]),stamp)
        return None

    def _on_step_detection(self, _message):
        return

    def _on_patrol_stop(self, message):
        if not message.data:
            return
        self._wait_for_operator("人工停车")

    def _on_patrol_reset_course(self,message):
        if not message.data:
            return
        self._wait_for_operator("人工重置新场")
        self.vision.reset_course()
        self.patrol_watcher.signature=None
        self.get_logger().warn("任务已重置到起点：环岛/横道次数清零；放回起点后输入1开始，不会自行运动")

    def _wait_for_operator(self, reason, localization=False):
        self._publish_zero()
        self.patrol_controller.reset()
        self.vision.reset_path()
        router=getattr(self.vision,"router",None)
        if router is not None:
            router.pause(time.monotonic())
        self._localization_wait=localization
        if localization:
            self._recovery_preview=PatrolVision()
            self._recovery_ready_frames=0
            self._recovery_preview_at=None
            self._recovery_ready_announced=False
            self._recovery_ready_log_at=-math.inf
        else:
            self._stand_gate = base.StandGate(enabled=True, settle_seconds=base.STAND_SETTLE_SECONDS)
        self._stand_confirm_prompted = False
        self._start_cue_until = 0.
        self.get_logger().warn(f"{reason}；已清零并保留任务阶段。{'定位恢复等待（不是退出/重新站立），保持检测；' if localization else ''}1继续当前任务，3从起点重置新场，不会自动重启。")

    def _on_remote_channels(self,message):
        self.remote_guard.update(message.data,time.monotonic(),self.patrol_tuning)

    def _remote_state(self,now):
        guard=getattr(self,"remote_guard",None)
        return guard.state(now,self.patrol_tuning) if guard is not None else "unmonitored"

    def _remote_allows_motion(self,now):
        return not base.ENABLE_MOTION or self._remote_state(now) in ("auto_switch","disabled","unmonitored")

    def _hold_remote_if_needed(self,now):
        if self._remote_allows_motion(now) or getattr(self,"_localization_wait",False):
            return False
        router=getattr(self.vision,"router",None)
        if router is not None and router.phase in ("APPROACH","ENTRY","FOLLOW","EXIT","RING_WAIT_DEADLINE"):
            router._release_ring(now,self.patrol_tuning,"remote_takeover_retry")
            router.ring_recovery_until=None  # no old blind turn after manual relocation
        self._wait_for_operator("遥控/信号接管："+self._remote_state(now)+"；当前未完成环岛作废，已完成圈凭据保留",localization=True)
        return True

    def _on_race_start_confirm(self, message):
        if message.data and self._patrol_ready and not self._remote_allows_motion(time.monotonic()):
            self.get_logger().warn(f"遥控保护：{self._remote_state(time.monotonic())}，SWA原值={getattr(self.remote_guard,'raw',None)}；本包ROS档门槛={self.patrol_tuning.remote_auto_channel_min}。请确认自动档及新鲜SBUS数据，再输入1；未恢复运动")
            return
        if getattr(self,"_localization_wait",False) and not message.data:
            self.get_logger().warn("未收到有效继续确认，保持定位恢复等待")
            return
        if self._patrol_ready and not self.patrol_tuning.enabled:
            self.get_logger().warn("enabled=0，先修改实时文件再确认开始")
            return
        if self._patrol_ready and message.data:
            router=getattr(self.vision,"router",None)
            if router is not None:
                router.resume(time.monotonic(),clear_fault=True)
            if getattr(self,"_localization_wait",False):
                self._localization_wait=False
                self.patrol_controller.reset()
                self.get_logger().warn("人工确认定位恢复，继续当前任务，不重复赛前站立")
                return
        super()._on_race_start_confirm(message)

    def _run_localization_preview(self,frame,tuning,now,draw):
        """Independent perception only: never advance the paused task/router."""
        self._publish_zero()
        preview=self._recovery_preview
        path=preview.estimate(frame,replace(tuning,roundabout_enabled=0,crosswalk_enabled=0),now,draw=draw)
        fresh=self._recovery_preview_at is not None and 0<now-self._recovery_preview_at<=tuning.roundabout_path_max_gap_seconds
        reliable=path.valid and path.route_observed and not path.rescue and not path.handoff
        self._recovery_ready_frames=(self._recovery_ready_frames+1 if fresh else 1) if reliable else 0
        self._recovery_preview_at=now
        ready=self._recovery_ready_frames>=tuning.reacquire_confirm_frames
        if ready and not self._recovery_ready_announced and now-self._recovery_ready_log_at>=1.:
            self.get_logger().info("定位恢复：连续可靠路线已出现，输入1继续；小车仍保持停车")
            self._recovery_ready_log_at=now
        self._recovery_ready_announced=ready
        if draw and self._debug_window_ready:
            debug=preview.last_debug if preview.last_debug is not None else frame.copy()
            cv2.putText(debug,"LOCALIZATION WAIT - press 1 to resume",(8,24),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,180,255),1)
            self._last_display_at=now
            try:
                cv2.imshow("WheelLeg Race Vision",debug)
                if cv2.waitKey(1)&0xFF in (ord("q"),ord("Q"),27):
                    self.request_exit()
            except cv2.error:
                self._debug_window_ready=False

    def _draw_stand_gate_debug(self, now):
        # Keep HDMI previews without drawing/encoding on every control tick.
        if not self._patrol_ready or now-self._last_display_at < 1/self.patrol_tuning.display_fps:
            return
        self._last_display_at = now
        super()._draw_stand_gate_debug(now)

    def _control_timer(self):
        try:
            self._run_control_cycle()
        except Exception as error:
            self._fault_stop("control_exception", time.monotonic())
            self.get_logger().error(f"普通巡线测试异常，已停车：{error}")

    def _fault_stop(self, reason, now):
        """True data/process failures bypass all visual memory immediately."""
        self._publish_zero()
        if self._patrol_ready:
            self.patrol_controller.reset()
            reset = getattr(self.vision, "reset_path", None)
            if reset is not None:
                reset()
            router=getattr(self.vision,"router",None)
            if router is not None:
                router.pause(now)
        if reason != getattr(self, "_last_fault_reason", "") or now-getattr(self, "_last_fault_at", 0.) >= 1.:
            self.get_logger().warn(f"PATROL_FAULT reason={reason} cmd_v=0.00 cmd_w=0.00；真实故障停车，未使用失线记忆")
            self._last_fault_reason, self._last_fault_at = reason, now

    def _run_control_cycle(self):
        if not self._patrol_ready:
            return
        now = time.monotonic()
        tuning = self.patrol_tuning
        # Preserve the original standing/start gate. Never execute the race
        # state machine, _try_lock_step, or any jump/recovery method here.
        if self._run_stand_gate(now) or self._run_start_cue(now):
            self.patrol_controller.reset()
            return
        self._publish_race_posture()
        if not tuning.enabled:
            self._publish_zero()
            self.patrol_controller.reset()
            return
        if self._hold_remote_if_needed(now):
            return
        with self._frame_lock:
            healthy = self._camera_healthy and self._latest_frame is not None
            captured_at = self._latest_frame_time
            sequence = self._frame_sequence
            new = sequence != self._last_processed_sequence
            # Do not copy the same old image on duplicate timer callbacks.
            frame = self._latest_frame.copy() if new and self._latest_frame is not None else None
        if not healthy or now-captured_at > tuning.image_timeout_seconds:
            self._fault_stop("camera_timeout", now)
            return
        if not new:
            self._publish_command(*self._last_command)
            return
        if frame is None:
            self._fault_stop("missing_frame", now)
            return
        self._last_processed_sequence = sequence
        draw = base.SHOW_DEBUG and now-self._last_display_at >= 1/tuning.display_fps
        if getattr(self,"_localization_wait",False):
            self._run_localization_preview(frame,tuning,now,draw)
            return
        router = getattr(self.vision, "router", None)
        route_before = router.phase if router is not None else "SEEK"
        if router is not None:
            self.vision.motion_command = self._last_command
            self.vision.motion_pose = self._odom_snapshot() if now-getattr(self, "_last_odom_time", 0.) <= .35 else None
            self.vision.motion_heading=self._heading_snapshot(now,tuning,self.vision.motion_pose)
        path = self.vision.estimate(frame, tuning, now, draw=draw)
        done = time.monotonic()
        if router is not None and router.phase != route_before:
            self.get_logger().warn(f"ROUTE 阶段 {route_before}->{router.phase}；任务={router.task_name}，台阶位置={router.course.steps}，环岛凭据={router.course.rings}，横道凭据={router.course.zebras}；左路线锁定={int(router.left_lock)}，视觉完成环岛={router.completed}，时间释放={router.timed_releases}，来源={router.ring_release_source}")
        if router is not None and router.navigation_reset_serial!=getattr(self,"_navigation_reset_seen",0):
            self.patrol_controller.reset()  # discard the failed ring search episode too
            self._navigation_reset_seen=router.navigation_reset_serial
        age = done-captured_at
        if age > tuning.image_timeout_seconds:
            # A fresh-at-entry frame can become stale during a slow operation.
            self._fault_stop("processing_timeout", done)
            return
        if self._hold_remote_if_needed(done):
            return
        loss_age = 0. if path.valid else self.patrol_controller.loss_age(done)
        if router is not None and router.fault:
            self.get_logger().error(f"PATROL_ROUTE_FAULT {router.fault}：不靠超时假装出环，回站立等待人工")
            self._wait_for_operator(router.fault,localization=True)
            return
        override=router.motion_override(done,tuning,getattr(self.vision,"motion_pose",None),path=path) if router is not None else None
        candidate=router is not None and router.candidate_active(done,tuning)
        if not hasattr(self,"_corner_exit_guard"):
            self._corner_exit_guard=CornerExitGuard()
        self._corner_exit_guard.observe(path,self.patrol_controller,done,tuning,
            allowed=override is None and not candidate and (router is None or router.phase=="SEEK"))
        if override is not None:
            target_v,target_w,mode=override
            planned_target=target_v
            linear,angular=self.patrol_controller.task_command(target_v,target_w,done,tuning,
                immediate_stop=mode in ("crosswalk_stop","finished","ring_recovery_stop","unauthorized_finish"),
                left_only=mode=="roundabout_left_acquire")
            if mode in ("roundabout_left_acquire","ring_curve_preview","gyro_exit_seek","ring_failure_recovery","ring_recovery_stop"):
                linear=min(linear,target_v)  # no stale 0.65m/s during deceleration
        else:
            control_tuning=tuning
            if router is not None and (candidate or router.phase in ("APPROACH","ENTRY","FOLLOW","EXIT")):
                control_tuning=replace(tuning,fixed_speed_m_s=min(tuning.fixed_speed_m_s,tuning.roundabout_speed_m_s))
            exit_turn=router.exit_search_turn(path,done,tuning,getattr(self.vision,"motion_heading",None)) if router is not None else None
            linear, angular, mode = self.patrol_controller.command(path, done, control_tuning,
                search_direction=1 if router is not None and router.search_left and tuning.roundabout_enabled else None,
                search_heading_turn=exit_turn)
            planned_target=min(control_tuning.fixed_speed_m_s,control_tuning.search_degraded_speed_m_s) if self.patrol_controller.degraded else control_tuning.fixed_speed_m_s
        if router is not None and (candidate or router.phase in ("APPROACH","ENTRY","FOLLOW","EXIT")):
            linear=min(linear,tuning.roundabout_speed_m_s)
        if router is not None and (candidate or router.phase=="APPROACH"):
            angular=float(np.clip(angular,-tuning.ring_candidate_turn_limit_rad_s,tuning.ring_candidate_turn_limit_rad_s))
        source = self.vision.source
        previous_source = getattr(self, "_previous_path_source", source)
        self._source_changes_total = getattr(self, "_source_changes_total", 0)+int(source != previous_source)
        self._previous_path_source = source
        sign = 1 if angular > .02 else -1 if angular < -.02 else 0
        previous_sign = getattr(self, "_previous_yaw_sign", 0)
        self._yaw_reversals_total = getattr(self, "_yaw_reversals_total", 0)+int(bool(sign and previous_sign and sign != previous_sign))
        if sign:
            self._previous_yaw_sign = sign
        if not base.ENABLE_MOTION:
            linear, angular = 0., 0.
        self._publish_command(linear, angular)
        self._last_command = (linear, angular)
        self.patrol_controller.last_command=self._last_command
        if mode == "localization_fault_latched":
            self.get_logger().error(
                f"PATROL_LOCALIZATION_FAULT reason={self.patrol_controller.fault_reason} "
                f"loss_age={self.patrol_controller.loss_age(done):.2f}s "
                f"search_distance={self.patrol_controller.loss_distance:.2f}m；持续无定位，等待人工确认")
            self._wait_for_operator("定位保护停车，节点未退出；持续无定位达到找线边界",localization=True)
            return
        self._processed_count += 1
        elapsed = done-self._stats_at
        if elapsed >= 1:
            self._capture_fps = (sequence-self._stats_sequence)/elapsed
            self._process_fps = self._processed_count/elapsed
            self._display_fps = self._displayed_count/elapsed
            self._processed_count = self._displayed_count = 0
            self._stats_at, self._stats_sequence = done, sequence
        if done-self._last_patrol_log >= 1:
            dash=getattr(self.vision,"dash_observation",DashObservation())
            black_path = getattr(self.vision, "black_estimate", PathEstimate(False))
            road_path = getattr(self.vision, "road_estimate", PathEstimate(False))
            reason = "_".join(path.reason.split())
            self.get_logger().info(
                f"PATROL source={self.vision.source} mode={mode} version=21 target={tuning.fixed_speed_m_s:.2f} "
                f"ring_predict={int(getattr(self.vision,'ring_prediction_used',False))} ring_match={getattr(self.vision,'ring_match_reason','inactive')} "
                f"ring_delta={getattr(self.vision,'ring_route_delta',0.):.3f} ring_ref_age={getattr(self.vision,'ring_reference_age',-1.):.3f} ring_points={getattr(self.vision,'ring_candidate_points',0)} "
                f"authority={self._remote_state(done)} "
                f"remote_raw={getattr(getattr(self,'remote_guard',None),'raw',None)} "
                f"ring_multiplier={tuning.roundabout_follow_speed_multiplier:.2f} "
                f"ring_cap={tuning.roundabout_speed_m_s:.2f} "
                f"planned_target={planned_target:.3f} "
                f"cmd_v={linear:.2f} cmd_w={angular:+.2f} conf={path.confidence:.2f} "
                f"odom_v={getattr(self,'_patrol_odom_v',float('nan')) if done-getattr(self,'_last_odom_time',0.)<=.35 else float('nan'):.2f} "
                f"capture={self._capture_fps:.1f} process={self._process_fps:.1f}fps "
                f"age={age*1000:.1f}ms algo={(done-now)*1000:.1f}ms "
                f"black_ok={int(black_usable(black_path,tuning))} road_ok={int(road_path.valid)} "
                f"black_conf={black_path.confidence:.2f} road_conf={road_path.confidence:.2f} "
                f"black_e={black_path.error:+.3f} road_e={road_path.error:+.3f} "
                f"selected_e={path.error:+.3f} near={path.near_error:+.3f} mid={path.mid_error:+.3f} "
                f"raw_w={self.patrol_controller.raw_angular:+.2f} handoff={int(path.handoff)} "
                f"corner={self.patrol_controller.corner_strength:.2f} search_distance={self.patrol_controller.loss_distance:.2f}m "
                f"patrol_baseline=V11 "
                f"near_gap={road_path.near_gap_ratio:.2f} reacquire={self.patrol_controller.reacquire_streak}/{tuning.reacquire_confirm_frames} "
                f"switches={self._source_changes_total} reversals={self._yaw_reversals_total} "
                f"route={router.phase if router is not None else 'SEEK'} gate={getattr(self.vision, 'gate_score', 0.):.2f} "
                f"task={router.task_name if router is not None else 'none'} "
                f"step_slots={','.join(router.course.steps) if router is not None else 'none'} "
                f"ring_receipts={''.join(str(int(v)) for v in router.course.rings) if router is not None else '00'} "
                f"zebra_receipts={''.join(str(int(v)) for v in router.course.zebras) if router is not None else '00'} "
                f"finish_authorized={int(router.finish_authorized) if router is not None else 0} "
                f"entry_ready={int(getattr(self.vision,'entry_ready',False))} "
                f"entry_detector=vv entry_reason={router.entry_detection_reason if router is not None else 'none'} "
                f"vv_reason={dash.reason} vv_blocks={dash.blocks} "
                f"vv_raw={dash.raw_blocks} vv_white={dash.white_blocks} vv_column={dash.column_blocks} vv_branch={dash.branch_blocks} candidate={int(candidate)} "
                f"vv_frames={router.vv_streak if router is not None else 0} "
                f"vv_entry_frames={router.vv_entry_streak if router is not None else 0} "
                f"vv_entry={int(dash.branch.valid)} vv_exit={int(dash.exit_branch.valid)} "
                f"cooldown_remaining={max(0.,tuning.roundabout_first_marker_cooldown_seconds-router.marker_moving_seconds) if router is not None and router.phase=='APPROACH' else 0.:.2f} "
                f"marker_travel={router.distance-router.first_marker_distance if router is not None and router.phase=='APPROACH' else 0.:.2f} "
                f"last_entry_proof={getattr(self.vision,'entry_trigger','none')} "
                f"route_status={router.route_status if router is not None else 'none'} "
                f"entry_limit={router.entry_acquire_limit(done,tuning) if router is not None else 0.:.2f} "
                f"route_owned={int(router.branch_acquired) if router is not None else 0} "
                f"exit_corner={int(path.exit_boundary is not None)} "
                f"heading_source={router.measured_turn.source if router is not None else 'none'} "
                f"heading_ok={int(router.measured_turn.fresh(done,tuning)) if router is not None else 0} "
                f"heading_reason={router.measured_turn.reason if router is not None else 'none'} "
                f"loop_candidate={int(router.loop_candidate_at is not None) if router is not None else 0} "
                f"exit_seek_m={router.loop_seek_distance if router is not None else 0.:.3f} "
                f"exit_commit={getattr(getattr(self.vision,'exit_commit',None),'reason','none')} "
                f"measured_turn_deg={math.degrees(router.measured_turn.angle) if router is not None else 0.:+.1f} "
                f"measured_travel={router.measured_turn.travel if router is not None else 0.:.2f} "
                f"yaw_releases={router.yaw_releases if router is not None else 0} ring_aborts={router.aborted_rings if router is not None else 0} "
                f"route_progress={router.progress_source if router is not None else 'none'} "
                f"route_evidence={router.evidence if router is not None else 0.:.2f} "
                f"route_travel={router.distance if router is not None else 0.:.2f}m "
                f"route_phase_distance={router.phase_distance if router is not None else 0.:.2f}m "
                f"route_left_yaw={router.left_yaw if router is not None else 0.:+.2f} "
                f"route_frames={router.path_streak if router is not None else 0} "
                f"gate_near={getattr(self.vision,'gate_near',0.):.2f} ring_yaw={router.ring_yaw if router is not None else 0.:+.2f} "
                f"exit_solid={getattr(self.vision,'exit_solid_score',0.):.2f} ring_count={router.completed if router is not None else 0} "
                f"cw_count={router.crosswalk_count if router is not None else 0} cw_score={getattr(self.vision,'crosswalk_score',0.):.2f} "
                f"cw_armed={int(router.crosswalk_armed(done,tuning)) if router is not None else 0} "
                f"cw_block={router.crosswalk_block_reason(done,tuning) if router is not None else 'no_router'} "
                f"entry_near_frames={router.near_frames if router is not None else 0} "
                f"ring_release={router.ring_release_source if router is not None else 'none'} "
                f"ring_timeouts={router.timed_releases if router is not None else 0} "
                f"zebra_releases={router.visual_releases if router is not None else 0} "
                f"memory_age={loss_age:.2f}s reason={reason}"
            )
            self._last_patrol_log = done
        if draw and self.vision.last_debug is not None and self._debug_window_ready:
            self._last_display_at = done
            labels = [
                (f"V17 第{router.course.lap+1 if router is not None else 1}圈 {router.phase if router is not None else 'SEEK'}；横道 {router.crosswalk_count if router is not None else 0}/2", (0, 220, 255)),
                (f"台阶本次跳过；任务{router.task_name if router is not None else 'none'}；环岛凭据{router.course.rings if router is not None else []}", (0,220,255)),
                (f"横道分数{self.vision.crosswalk_score:.2f}；许可 {router.crosswalk_block_reason(done,tuning)}", (0, 220, 255)),
                (f"路径：{self.vision.source}  {mode}  可信度{path.confidence:.2f}", (0, 255, 0)),
                (f"目标{tuning.fixed_speed_m_s:.2f}  输出{linear:.2f}m/s  转向{angular:+.2f}rad/s", (255, 255, 255)),
                (f"采集{self._capture_fps:.1f} 处理{self._process_fps:.1f} 显示{self._display_fps:.1f}fps 图龄{age*1000:.0f}ms", (255, 255, 255)),
                (f"小窗={self.vision.preview_kind}；无定位{loss_age:.2f}/{tuning.search_max_seconds:.2f}s；Q退出", (255, 200, 80)),
            ]
            debug = base.draw_overlay(self.vision.last_debug, labels, font_size=18, line_height=25)
            try:
                cv2.imshow("WheelLeg Race Vision", debug)
                self._displayed_count += 1
                if cv2.waitKey(1) & 0xFF in (ord("q"), ord("Q"), 27):
                    self.request_exit()
            except cv2.error as error:
                self._debug_window_ready = False
                self.get_logger().error(f"车载显示失败：{error}")


def main(args=None):
    node = None
    exiting = threading.Event()
    def stop(_signum, _frame):
        exiting.set()
        if node is not None:
            node.request_exit()
    old_int = signal.signal(signal.SIGINT, stop)
    old_term = signal.signal(signal.SIGTERM, stop)
    base.rclpy.init(args=args, signal_handler_options=base.SignalHandlerOptions.NO)
    try:
        node = PatrolTestNode()
        while base.rclpy.ok() and not exiting.is_set() and not node.exit_requested:
            try:
                base.rclpy.spin_once(node, timeout_sec=.05)
            except Exception as error:
                if not base.rclpy.ok() or exiting.is_set():
                    break
                # A callback fault is a zero-command/operator-wait condition,
                # not permission to silently drop the node or blindly resume.
                node._wait_for_operator("ROS回调异常")
                node.get_logger().error(f"PATROL_CALLBACK_FAULT 节点保持运行、等待人工：{type(error).__name__}: {error}")
    finally:
        if node is not None:
            node.get_logger().warn(f"PATROL_PROCESS_EXIT signal={int(exiting.is_set())} requested={int(node.exit_requested)} ros_ok={int(base.rclpy.ok())}")
            node.close()
            node.destroy_node()
        if base.rclpy.ok():
            base.rclpy.shutdown()
        signal.signal(signal.SIGINT, old_int)
        signal.signal(signal.SIGTERM, old_term)


if __name__ == "__main__":
    main()
