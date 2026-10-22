#!/usr/bin/env python3
"""Opt-in ring navigation, independent of the original competition FSM.

Image +x is right, ROS +yaw is left/CCW. Fork topology chooses the entry;
signed gyro, measured displacement and solid exit jointly confirm completion.
No camera frame or commanded Twist is counted as measured rotation/travel.
"""
from dataclasses import dataclass, field
import math

import cv2
import numpy as np

from patrol_contour_runtime import PathEstimate


@dataclass
class RingObservation:
    kind: str = "none"  # converging=first exit; diverging=left entry
    main: PathEstimate = field(default_factory=lambda: PathEstimate(False))
    branch: PathEstimate = field(default_factory=lambda: PathEstimate(False))
    dashed: PathEstimate = field(default_factory=lambda: PathEstimate(False))
    solid: PathEstimate = field(default_factory=lambda: PathEstimate(False))
    component_count: int = 0
    separation_change: float = 0.


def _path(points, width, height, reason, mask=None, target_x_ratio=.5):
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    if len(points) < 2 or not np.isfinite(points).all():
        return PathEstimate(False, reason=reason+":insufficient")
    points = points[np.argsort(-points[:, 1])]
    near = np.mean(points[:min(2, len(points))], axis=0)
    far = np.mean(points[-min(2, len(points)):], axis=0)
    lateral = math.atan2(near[0]-target_x_ratio*width, .65*height)
    heading = math.atan2(far[0]-near[0], max(1., near[1]-far[1]))
    angle = lateral + float(np.clip(.35*heading, -.35, .35))
    return PathEstimate(True, angle, min(.95, .55+.07*len(points)), points, mask, reason)


def _dash_components(mask, tuning):
    height, width = mask.shape
    roi = mask.copy()
    roi[:int(height*tuning.ring_roi_top_ratio)] = 0
    count, labels, stats, centers = cv2.connectedComponentsWithStats(roi, connectivity=8)
    candidates = []
    for index in range(1, count):
        x, y, bw, bh, area = stats[index]
        if (12 <= area <= .02*width*height and 2 <= bw <= .15*width
                and 3 <= bh <= .24*height and .18 <= bw/max(1, bh) <= 5
                and .025*width < centers[index, 0] < .975*width):
            candidates.append((centers[index], area))
    # Bound work in textured scenes; closest large dash candidates only.
    candidates.sort(key=lambda item: item[1], reverse=True)
    return np.asarray([item[0] for item in candidates[:40]], dtype=float).reshape(-1, 2), labels, stats


def _solid_exit(mask, stats, labels, tuning):
    height, width = mask.shape
    best = PathEstimate(False, reason="ring_no_solid_exit")
    for index in range(1, len(stats)):
        x, y, bw, bh, area = stats[index]
        if bh < .45*height or y+bh < .90*height or bw > .35*width or area < 50:
            continue
        points = []
        for row in np.linspace(max(y, int(.5*height)), min(height-1, y+bh-1), 8).astype(int):
            xs = np.flatnonzero(labels[row] == index)
            if len(xs):
                points.append((float(np.median(xs)), row))
        candidate = _path(points, width, height, "ring_solid_exit", mask, tuning.target_x_ratio)
        if candidate.valid and abs(candidate.angle_rad) < .25:
            if not best.valid or abs(candidate.angle_rad) < abs(best.angle_rad):
                best = candidate
    return best


def _track_dashes(points, reference, width, height, mask, tuning):
    if len(points) < 2:
        return PathEstimate(False, reason="ring_dash_gap", mask=mask)
    points = points[np.argsort(-points[:, 1])]
    ref = None if reference is None else np.asarray(reference)
    if ref is not None and len(ref) >= 2:
        ref = ref[np.argsort(ref[:, 1])]
    def expected(y):
        return float(np.interp(y, ref[:, 1], ref[:, 0])) if ref is not None and len(ref) >= 2 else .5*width
    near = points[points[:, 1] >= .60*height]
    if not len(near):
        return PathEstimate(False, reason="ring_no_near_dash", mask=mask)
    # Prefer the last locked curved branch, not the straight-line alternative.
    anchor = min(near, key=lambda p: abs(p[0]-expected(p[1])) + .12*(height-p[1]))
    if abs(anchor[0]-expected(anchor[1])) > .20*width:
        return PathEstimate(False, reason="ring_anchor_jump", mask=mask)
    selected = [anchor]
    for point in points:
        last = selected[-1]
        dy = last[1]-point[1]
        if dy < .045*height:
            continue
        if dy > .28*height:
            break  # do not splice across a long unknown gap
        predicted = expected(point[1])
        if len(selected) >= 2:
            previous = selected[-2]
            predicted = last[0]+(last[0]-previous[0])*dy/max(1., previous[1]-last[1])
        if abs(point[0]-last[0]) <= .23*width and abs(point[0]-predicted) <= .13*width:
            selected.append(point)
    if len(selected) < 2 or selected[0][1]-selected[-1][1] < .10*height:
        return PathEstimate(False, reason="ring_dash_span_short", mask=mask)
    return _path(selected, width, height, "ring_dashed_track", mask, tuning.target_x_ratio)


def observe_ring(mask, tuning, seed_x_ratio=.5, reference=None):
    """Fit a near straight dash branch and the separation of its LEFT branch.

    Forward is image-up: separation increasing forward means diverging entry;
    decreasing forward means converging exit. A mere count of dark blobs is
    never sufficient. All topology is measured in a lower image ROI.
    """
    height, width = mask.shape
    points, labels, stats = _dash_components(mask, tuning)
    result = RingObservation(component_count=len(points))
    result.dashed = _track_dashes(points, reference, width, height, mask, tuning)
    result.solid = _solid_exit(mask, stats, labels, tuning)
    # The ordinary tracker may already be attracted toward a fork branch.
    # Do not let its seed drag the independent main-axis search off screen.
    seed = float(np.clip(seed_x_ratio, tuning.target_x_ratio-.05, tuning.target_x_ratio+.05))*width
    near_axis = points[abs(points[:, 0]-seed) <= .18*width]
    n = tuning.ring_min_branch_dashes
    if len(near_axis) < n or np.ptp(near_axis[:, 1]) < .22*height:
        return result
    # Vectorized two-point consensus avoids fitting an average of both fork
    # branches. Prefer a long, near-forward axis over an oblique side branch.
    first, second = np.triu_indices(len(near_axis), 1)
    dy = near_axis[second, 1]-near_axis[first, 1]
    valid = abs(dy) >= .10*height
    first, second, dy = first[valid], second[valid], dy[valid]
    if not len(first):
        return result
    slopes = (near_axis[second, 0]-near_axis[first, 0])/dy
    intercepts = near_axis[first, 0]-slopes*near_axis[first, 1]
    inliers = abs(near_axis[None, :, 0]-slopes[:, None]*near_axis[None, :, 1]-intercepts[:, None]) <= .025*width
    spans = (np.max(np.where(inliers, near_axis[None, :, 1], -math.inf), axis=1)
             -np.min(np.where(inliers, near_axis[None, :, 1], math.inf), axis=1))
    score = inliers.sum(axis=1)+spans/height-1.5*abs(slopes)-.5*abs(slopes*.85*height+intercepts-seed)/width
    score[(inliers.sum(axis=1) < n) | (spans < .22*height) | (abs(slopes) > .75)] = -math.inf
    if not np.isfinite(score).any():
        return result
    near_axis = near_axis[inliers[int(np.argmax(score))]]
    fit = np.polyfit(near_axis[:, 1], near_axis[:, 0], 1)
    if abs(fit[0]) > .75 or abs(np.polyval(fit, .85*height)-seed) > .15*width:
        return result
    result.main = _path(near_axis, width, height, "ring_main_dash", mask, tuning.target_x_ratio)
    gaps = np.polyval(fit, points[:, 1])-points[:, 0]
    left = points[(gaps >= .035*width) & (gaps <= .50*width)]
    if (len(left) < n or np.ptp(left[:, 1]) < .18*height
            or left[:, 1].max() < tuning.ring_fork_near_ratio*height):
        return result
    gaps = np.polyval(fit, left[:, 1])-left[:, 0]
    forward = 1-left[:, 1]/height
    separation_fit = np.polyfit(forward, gaps, 1)
    if np.median(abs(gaps-np.polyval(separation_fit, forward))) > .035*width:
        return result
    change = float(separation_fit[0]*np.ptp(forward)/width)
    result.separation_change = change
    if abs(change) < tuning.ring_fork_change_ratio:
        return result
    result.kind = "diverging" if change > 0 else "converging"
    result.branch = _path(left, width, height, "ring_left_curved_branch", mask, tuning.target_x_ratio)
    return result


@dataclass
class RingDecision:
    path: PathEstimate
    phase: str
    speed: float = -1.
    search_turn: float = 0.
    stop: bool = False


class PatrolRingNavigator:
    ACTIVE = {"ENTER", "FOLLOW", "EXIT"}

    def __init__(self):
        self.reset()

    def reset(self):
        self.phase = "SEEK_EXIT"
        self.started_at = None
        self.entered_at = -math.inf
        self.last_line_at = -math.inf
        self.reference = None
        self.main_reference = None
        self.exit_pose = None
        self.exit_heading = 0.
        self.odom_heading = 0.
        self.turn_rad = 0.
        self.travel_m = 0.
        self.imu_at = self.odom_at = -math.inf
        self.last_rate = None
        self.pose = None
        self.last_odom = None
        self.kind = "none"
        self.hits = self.entry_hits = self.exit_hits = self.clear_hits = 0
        self.armed_notice = False
        self.events = []
        self.reason = "等待近场出口标记"

    @property
    def locked(self):
        return self.phase in self.ACTIVE

    def fail(self, reason):
        if self.phase != "FAILED":
            self.events.append("RING_FAILED_STOP："+reason+"；请停车检查后重新输入1开始")
        self.phase, self.reason = "FAILED", reason

    def observe_imu(self, rate, now):
        if not math.isfinite(rate) or abs(rate) > 4.2:
            if self.locked:
                self.fail("陀螺仪异常跳变")
            return
        dt = now-self.imu_at
        if self.locked and 0 < dt <= .15 and self.last_rate is not None:
            self.turn_rad += .5*(rate+self.last_rate)*dt  # signed, not abs()
        self.last_rate, self.imu_at = rate, now

    def observe_odom(self, x, y, now, heading=0.):
        if not all(math.isfinite(v) for v in (x, y, heading)):
            return
        dt = now-self.odom_at
        if self.locked and self.last_odom is not None and 0 < dt <= .3:
            step = math.hypot(x-self.last_odom[0], y-self.last_odom[1])
            if step > .8*dt+.025:
                self.fail("里程计跳变，可能搬动车辆")
            else:
                self.travel_m += step
        self.pose = self.last_odom = (x, y)
        self.odom_heading = heading
        self.odom_at = now

    def _fresh(self, now):
        return 0 <= now-self.imu_at <= .30 and 0 <= now-self.odom_at <= .40

    def _set(self, phase, reason):
        self.phase, self.reason = phase, reason
        self.events.append(f"RING_{phase}：{reason}")

    def step(self, observation, normal, now, tuning):
        if self.phase == "FAILED":
            return RingDecision(PathEstimate(False), self.phase, stop=True)
        if not tuning.ring_enabled:
            if self.locked:
                self.fail("环岛运行中关闭功能，禁止直接抢回普通巡线")
                return RingDecision(PathEstimate(False), self.phase, stop=True)
            return RingDecision(normal, "DISABLED")
        if self.started_at is None:
            self.started_at = now
        if self.phase == "DONE":
            return RingDecision(normal, self.phase)
        if now-self.started_at < tuning.ring_start_delay_s:
            return RingDecision(normal, "ARM_DELAY")
        if not self.armed_notice:
            self.events.append("RING_ARMED：已开启近场环岛检测；先出口、后左侧发散入口")
            self.armed_notice = True
        kind = observation.kind if observation.main.valid and observation.branch.valid else "none"
        self.hits = self.hits+1 if kind != "none" and kind == self.kind else (1 if kind != "none" else 0)
        self.kind = kind
        if self.phase == "SEEK_EXIT":
            if kind == "converging" and self.hits >= tuning.ring_fork_confirm_frames:
                self.exit_pose = self.pose
                self.exit_heading = self.odom_heading
                self.main_reference = observation.main.points
                self.clear_hits = 0
                self._set("PASS_EXIT", "确认左侧朝前汇聚虚线为出口，沿主虚线前进通过")
                return RingDecision(observation.main, self.phase, tuning.ring_speed_m_s)
            if kind == "diverging" and self.hits == tuning.ring_fork_confirm_frames:
                self.events.append("RING_ENTRY_IGNORED：未确认先前出口，不从当前分叉直接入环")
            # Unconfirmed/incorrect-stage forks are telemetry ONLY. They must
            # not change either the ordinary target or its speed for one frame.
            return RingDecision(normal, self.phase)
        if self.phase in {"PASS_EXIT", "SEEK_ENTRY"}:
            # Ignore a curve before confirmation: only the straight dash axis.
            path = observation.main
            if not path.valid and normal.valid and self.main_reference is not None and normal.points is not None:
                fit = np.polyfit(self.main_reference[:, 1], self.main_reference[:, 0], 1)
                if np.median(abs(normal.points[:, 0]-np.polyval(fit, normal.points[:, 1]))) < .07*320:
                    path = normal
            if self.phase == "PASS_EXIT":
                if not self._fresh(now) or self.exit_pose is None:
                    self.fail("出口通过阶段缺少新鲜IMU/里程计")
                    return RingDecision(PathEstimate(False), self.phase, stop=True)
                self.clear_hits = self.clear_hits+1 if kind != "converging" else 0
                traveled = ((self.pose[0]-self.exit_pose[0])*math.cos(self.exit_heading)
                            +(self.pose[1]-self.exit_pose[1])*math.sin(self.exit_heading))
                if self._fresh(now) and traveled >= tuning.ring_exit_pass_m and self.clear_hits >= 2:
                    self.hits = 0
                    self._set("SEEK_ENTRY", "出口已通过，等待左侧朝前发散的弯曲虚线入口")
                return RingDecision(path, self.phase, tuning.ring_speed_m_s)
            if kind == "diverging" and self.hits >= tuning.ring_fork_confirm_frames:
                if not self._fresh(now):
                    self.fail("入口已识别，但IMU/里程计不新鲜，拒绝盲目入环")
                    return RingDecision(PathEstimate(False), self.phase, stop=True)
                self.entered_at = self.last_line_at = now
                self.turn_rad = self.travel_m = 0.
                self.imu_at = self.odom_at = now
                self.last_odom = self.pose
                self.reference = observation.branch.points
                self.entry_hits = 0
                self._set("ENTER", "入口确认，锁定左侧弯曲虚线；进入前方向记为0°")
                return RingDecision(observation.branch, self.phase, tuning.ring_entry_speed_m_s)
            return RingDecision(path, self.phase, tuning.ring_speed_m_s)
        # Locked ring phases never consume normal solid/road fallback targets.
        if not self._fresh(now):
            self.fail("环岛中IMU或里程计超时")
        speed = min(tuning.fixed_speed_m_s, tuning.ring_speed_m_s)
        scale = .20/max(.01, speed)  # reference-time gates adapt to speed changes
        entry_scale = .18/max(.01, min(tuning.fixed_speed_m_s, tuning.ring_entry_speed_m_s))
        loss_limit = tuning.ring_loss_timeout_s*.10/max(.01, min(tuning.fixed_speed_m_s, tuning.ring_loss_speed_m_s))
        elapsed = now-self.entered_at
        if elapsed > tuning.ring_max_duration_s*scale or math.degrees(self.turn_rad) > tuning.ring_max_turn_deg:
            self.fail("环岛绕行超时/过转，不能以盲绕圈替代出口确认")
        if self.phase == "FAILED":
            return RingDecision(PathEstimate(False), self.phase, stop=True)
        if self.phase == "EXIT":
            self.exit_hits = self.exit_hits+1 if observation.solid.valid else 0
            if self.exit_hits >= tuning.ring_exit_confirm_frames:
                self._set("DONE", "出口实线连续确认，恢复普通循线；本次开始不再入环")
            elif not observation.solid.valid and now-self.last_line_at > loss_limit:
                self.fail("出环实线丢失")
                return RingDecision(PathEstimate(False), self.phase, stop=True)
            if observation.solid.valid:
                self.last_line_at = now
            return RingDecision(observation.solid, self.phase, tuning.ring_speed_m_s)
        progress = (elapsed >= tuning.ring_min_duration_s*scale
                    and self.travel_m >= tuning.ring_min_travel_m
                    and math.degrees(self.turn_rad) >= tuning.ring_min_turn_deg)
        self.exit_hits = self.exit_hits+1 if progress and observation.solid.valid else 0
        if self.phase == "FOLLOW" and self.exit_hits >= tuning.ring_exit_confirm_frames:
            self.exit_hits = 0
            self._set("EXIT", "转角/距离/时间满足且出口实线出现，沿实线出环")
            self.last_line_at = now
            return RingDecision(observation.solid, self.phase, tuning.ring_speed_m_s)
        path = observation.dashed
        if self.phase == "ENTER" and observation.kind == "diverging" and observation.branch.valid:
            path = observation.branch
        if path.valid:
            self.reference = path.points
            self.last_line_at = now
            self.entry_hits += 1
            if (self.phase == "ENTER" and self.entry_hits >= tuning.ring_entry_confirm_frames
                    and self.turn_rad >= math.radians(8) and self.travel_m >= .06):
                self._set("FOLLOW", "已沿弯曲虚线进入环内，继续专用虚线跟踪")
            if self.phase == "ENTER" and elapsed > tuning.ring_entry_timeout_s*entry_scale:
                self.fail("入环未形成有效行驶进度")
                return RingDecision(PathEstimate(False), self.phase, stop=True)
            return RingDecision(path, self.phase, tuning.ring_entry_speed_m_s if self.phase == "ENTER" else tuning.ring_speed_m_s)
        self.entry_hits = 0
        if now-self.last_line_at > loss_limit:
            self.fail("虚线连续丢失超时")
            return RingDecision(path, self.phase, stop=True)
        return RingDecision(path, self.phase, tuning.ring_loss_speed_m_s, tuning.ring_search_turn_rad_s)
