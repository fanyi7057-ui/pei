"""Contour (left/right boundary) line + corner navigation for the patrol chain.

This module implements the requested algorithm:

* build a connected-domain road mask (white + uncertain gray, blue excluded),
* extract the left/right road boundaries and their centerline,
* detect "corner points" used to steer into / out of the roundabout:
  - entry: an upper-left corner of the road; connect the right half of the
    current centerline to that corner and treat it as the right boundary, which
    pulls the centerline left;
  - exit: a right-side corner where y stays roughly constant while x suddenly
    decreases (a leftward horizontal step); pull the line left so the car can
    turn back.

The black centre line stays the normal follower while it is visible; the road
contour is used when black is lost, and the corner pull is applied as a short,
bounded lateral offset on top of whichever path is currently valid.
"""
from dataclasses import dataclass, field
from typing import Optional, Tuple

import cv2
import numpy as np

from patrol_contour_runtime import road_masks


@dataclass
class CornerObservation:
    valid: bool = False
    reason: str = "no_road"
    left: Optional[np.ndarray] = None
    right: Optional[np.ndarray] = None
    center: Optional[np.ndarray] = None
    near_error: float = 0.0
    mid_error: float = 0.0
    far_error: float = 0.0
    error: float = 0.0
    confidence: float = 0.0
    entry_corner: Optional[Tuple[int, int]] = None
    exit_corner: Optional[Tuple[int, int]] = None
    mask: Optional[np.ndarray] = None
    # Corridor quality: a reliable "single lane between two observed edges".
    corridor: bool = False
    dual_fraction: float = 0.0
    width_ratio: float = 0.0
    width_mad_ratio: float = 1.0
    branch_ratio: float = 0.0
    area_ratio: float = 0.0


# A connected white/gray region is only a drivable lane when both edges are
# actually observed, the width is plausible and stable, and it is not a wide
# open hall or a branched fork. These are deliberately conservative: without
# them the "contour" follows the widest white region and drives straight into
# the scenery instead of searching (observed on 2026-11-17).
CORRIDOR_MIN_DUAL = 0.55
CORRIDOR_MIN_WIDTH = 0.18
CORRIDOR_MAX_WIDTH = 0.80
CORRIDOR_MAX_WIDTH_MAD = 0.22
CORRIDOR_MAX_BRANCH = 0.10
CORRIDOR_MAX_AREA = 0.85


def _component_mask(hsv, tuning):
    height, width = hsv.shape[:2]
    blue, white = road_masks(hsv, tuning)
    gray = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)
    gray_ok = gray >= int(getattr(tuning, "contour_corner_gray_v_min", 70))
    road = ((white > 0) | gray_ok) & (blue == 0)
    road = road.astype(np.uint8)
    road = cv2.morphologyEx(road, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    min_area = max(64, int(getattr(tuning, "contour_corner_min_area_ratio", .04) * height * width))
    n, labels, stats, _centroids = cv2.connectedComponentsWithStats(road, 8)
    bottom = max(0, height - max(4, int(.12 * height)))
    best, best_area = 0, 0
    for idx in range(1, n):
        area = int(stats[idx, cv2.CC_STAT_AREA])
        if area < min_area:
            continue
        ys = labels[bottom:height] == idx
        if not ys.any():
            continue
        if area > best_area:
            best, best_area = idx, area
    if not best:
        return None
    comp = (labels == best).astype(np.uint8)
    # Fill small holes inside the road component (shadows / dirt).
    filled = comp.copy()
    contours, _ = cv2.findContours(comp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        cv2.drawContours(filled, contours, -1, 1, -1)
    return filled


def _boundaries(mask, width, height):
    left = np.full(height, np.nan)
    right = np.full(height, np.nan)
    center = np.full(height, np.nan)
    prev = None
    missing = 0
    stats = {'rows': 0, 'dual': 0, 'multi': 0}
    min_run = max(3, int(.015 * width))
    for y in range(height - 1, -1, -1):
        row = mask[y]
        runs = []
        x = 0
        while x < width:
            if row[x]:
                s = x
                while x < width and row[x]:
                    x += 1
                if x - s >= min_run:
                    runs.append((s, x - 1))
            else:
                x += 1
        if not runs:
            missing += 1
            if missing > max(6, int(.15 * height)):
                break
            if prev is not None:
                center[y] = prev
            continue
        if prev is None:
            run = min(runs, key=lambda r: abs((r[0] + r[1]) * .5 - width * .5))
        else:
            run = min(runs, key=lambda r: abs((r[0] + r[1]) * .5 - prev))
            if abs((run[0] + run[1]) * .5 - prev) > .35 * width:
                missing += 1
                if missing > max(6, int(.15 * height)):
                    break
                center[y] = prev
                continue
        missing = 0
        left[y], right[y] = run[0], run[1]
        center[y] = (run[0] + run[1]) * .5
        prev = center[y]
        stats['rows'] += 1
        stats['dual'] += int(run[0] > 1 and run[1] < width - 2)
        stats['multi'] += int(len(runs) > 1)
    return left, right, center, stats


def _corner_candidates(left, right, width, height, tuning):
    """Return (entry_corner, exit_corner) in image coordinates."""
    min_angle = np.radians(float(getattr(tuning, "contour_corner_min_angle_deg", 55.)))
    entry = exit_ = None
    # Left boundary: highest-curvature point in the upper-left area. Ignore
    # points on the ROI border: those are crop edges, not road corners.
    pts = [(x, y) for y, x in enumerate(left)
           if np.isfinite(x) and .08 * width < x < .62 * width]
    best_angle = 0.
    for i in range(8, len(pts) - 8):
        x0, y0 = pts[i - 8]
        x1, y1 = pts[i]
        x2, y2 = pts[i + 8]
        if not (.10 * width < x1 < .58 * width and .12 * height < y1 < .58 * height):
            continue
        if abs(y1 - y0) < 5 or abs(y2 - y1) < 5:
            continue
        v1 = np.array([x1 - x0, y1 - y0], float)
        v2 = np.array([x2 - x1, y2 - y1], float)
        n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
        if n1 < 1e-6 or n2 < 1e-6:
            continue
        cosv = float(np.clip(np.dot(v1, v2) / (n1 * n2), -1., 1.))
        angle = float(np.arccos(cosv))
        if angle < min_angle:
            continue
        # Going downward the left boundary must turn right (x grows) enough to
        # count as a real corner (a small ripple is not a corner).
        if v2[0] < .05 * width:
            continue
        if angle > best_angle:
            best_angle, entry = angle, (int(round(x1)), int(round(y1)))
    # Right boundary: a nearly horizontal leftward step (dy small, dx negative)
    # in the upper part; this is the exit mouth seen without the black line.
    rpts = [(x, y) for y, x in enumerate(right)
            if np.isfinite(x) and .12 * width < x < .94 * width]
    best_dx = 0.
    for i in range(4, len(rpts) - 4):
        x0, y0 = rpts[i - 4]
        x1, y1 = rpts[i + 4]
        if not (.18 * width < x1 < .92 * width and .10 * height < y1 < .55 * height):
            continue
        if abs(y1 - y0) > .05 * height:
            continue
        dx = x1 - x0
        if dx < -.09 * width and dx < best_dx:
            best_dx = dx
            exit_ = (int(round(rpts[i][0])), int(round(rpts[i][1])))
    return entry, exit_


def _blue_runs(row, width):
    runs = []
    x = 0
    min_run = max(2, int(.008 * width))
    while x < width:
        if row[x]:
            s = x
            while x < width and row[x]:
                x += 1
            if x - s >= min_run:
                runs.append((s, x - 1))
        else:
            x += 1
    return runs


def _blue_boundaries(blue, width, height, prior_width, ref_x=None):
    """Left/right road edges from the BLUE tape, not the white floor.

    The track is a wide white floor with blue tape marking the lane edges; the
    white connected region is therefore not an enclosed corridor. The blue
    tape's inner edges are the real left/right boundaries. Each row pairs the
    nearest blue tape left/right of the reference centre (the black line when
    available, otherwise the image centre). When only one side is visible, a
    recent dual-edge width keeps the centre usable (single edge).
    """
    left = np.full(height, np.nan)
    right = np.full(height, np.nan)
    center = np.full(height, np.nan)
    if ref_x is None:
        ref_x = np.full(height, width * .5)
    rows_for_width = []
    for y in range(height):
        runs = _blue_runs(blue[y], width)
        ref = float(ref_x[y])
        left_run = max((r for r in runs if r[1] < ref), key=lambda r: r[1], default=None)
        right_run = min((r for r in runs if r[0] > ref), key=lambda r: r[0], default=None)
        if left_run is not None and right_run is not None:
            w = right_run[0] - left_run[1]
            if .10 * width <= w <= .95 * width:
                rows_for_width.append(w)
    prior = prior_width
    if len(rows_for_width) >= max(4, int(.05 * height)):
        prior = float(np.median(rows_for_width))
    dual_rows = single_rows = rows = 0
    widths = []
    prev = None
    for y in range(height - 1, -1, -1):
        runs = _blue_runs(blue[y], width)
        ref = float(ref_x[y])
        left_run = max((r for r in runs if r[1] < ref), key=lambda r: r[1], default=None)
        right_run = min((r for r in runs if r[0] > ref), key=lambda r: r[0], default=None)
        if left_run is not None and right_run is not None:
            l, r = left_run[1], right_run[0]
            w = r - l
            if not .10 * width <= w <= .95 * width:
                if prev is not None:
                    center[y] = prev
                continue
            dual_rows += 1
            widths.append(w)
        elif left_run is not None and prior:
            l = left_run[1]
            r = l + prior
            single_rows += 1
        elif right_run is not None and prior:
            r = right_run[0]
            l = r - prior
            single_rows += 1
        else:
            if prev is not None:
                center[y] = prev
            continue
        c = (l + r) * .5
        if not 0 <= c < width:
            continue
        if prev is not None and abs(c - prev) > .22 * width:
            continue
        left[y], right[y], center[y] = l, r, c
        prev = c
        rows += 1
    stats = {'rows': rows, 'dual': dual_rows, 'single': single_rows,
             'widths': np.asarray(widths, dtype=float)}
    return left, right, center, stats


def observe_blue_road(hsv, tuning, prior_width=None, ref_points=None) -> CornerObservation:
    height, width = hsv.shape[:2]
    blue, _white = road_masks(hsv, tuning)
    ref_x = None
    if ref_points is not None:
        pts = np.asarray(ref_points, dtype=float).reshape(-1, 2)
        pts = pts[np.isfinite(pts).all(axis=1)]
        if len(pts) >= 2:
            pts = pts[np.argsort(pts[:, 1])]
            ys = np.arange(height, dtype=float)
            ref_x = np.interp(ys, pts[:, 1], pts[:, 0])
    left, right, center, stats = _blue_boundaries(blue, width, height, prior_width, ref_x)
    rows = max(1, stats['rows'])
    if rows < max(6, int(.12 * height)):
        return CornerObservation(False, "few_blue_rows")
    widths = stats['widths'] if len(stats['widths']) else None
    width_med = float(np.median(widths)) if widths is not None else float(prior_width or 0.)
    width_ratio = width_med / width if width_med else 0.
    width_mad_ratio = (float(np.median(np.abs(widths - width_med))) / width_med
                       if widths is not None and len(widths) > 1 and width_med > 1 else 0.)
    dual_fraction = stats['dual'] / rows
    single_edge = stats['dual'] < stats['single']
    corridor = bool((dual_fraction >= .40
                     or (prior_width is not None and stats['single'] >= .50 * rows))
                    and .15 <= width_ratio <= .85
                    and width_mad_ratio <= .25)
    if corridor:
        reason = "blue_single_edge" if single_edge else "blue_dual_edge"
    elif width_ratio > .85:
        reason = "blue_too_wide"
    elif width_mad_ratio > .25:
        reason = "blue_width_unstable"
    else:
        reason = "blue_single_edge_only"

    def err(ratio):
        y = int(np.clip(ratio * height, 0, height - 1))
        ys = np.flatnonzero(np.isfinite(center))
        if len(ys) < 2:
            return 0.
        cx = float(np.interp(y, ys, center[ys]))
        return float(np.clip((cx - .5 * width) / (.5 * width), -1., 1.))
    near, mid, far = (err(getattr(tuning, "road_near_y_ratio", .85)),
                      err(getattr(tuning, "road_mid_y_ratio", .60)),
                      err(getattr(tuning, "road_far_y_ratio", .35)))
    entry, exit_ = _corner_candidates(left, right, width, height, tuning)
    ys = np.flatnonzero(np.isfinite(center))
    points = np.column_stack((center[ys], ys)).astype(float)
    confidence = float(np.clip(dual_fraction * max(0., 1. - width_mad_ratio / .25), 0., 1.))
    return CornerObservation(True, reason, left, right, points, near, mid, far,
                             float(np.clip((near + mid) * .5, -1., 1.)),
                             confidence, entry, exit_, None, corridor,
                             dual_fraction, width_ratio, width_mad_ratio, 0.,
                             1. - dual_fraction)


def observe_corners(hsv, tuning) -> CornerObservation:
    height, width = hsv.shape[:2]
    mask = _component_mask(hsv, tuning)
    if mask is None:
        return CornerObservation(False, "no_connected_road")
    area_ratio = float(np.count_nonzero(mask)) / float(width * height)
    left, right, center, stats = _boundaries(mask, width, height)
    valid_rows = max(1, stats['rows'])
    if valid_rows < max(6, int(.12 * height)):
        return CornerObservation(False, "few_boundary_rows", mask=mask, area_ratio=area_ratio)
    widths = (right - left)[np.isfinite(right) & np.isfinite(left)]
    width_med = float(np.median(widths)) if len(widths) else 0.
    width_ratio = width_med / width
    width_mad_ratio = (float(np.median(np.abs(widths - width_med))) / width_med
                       if width_med > 1 else 1.)
    dual_fraction = stats['dual'] / valid_rows
    branch_ratio = stats['multi'] / valid_rows
    corridor = bool(dual_fraction >= CORRIDOR_MIN_DUAL
                    and CORRIDOR_MIN_WIDTH <= width_ratio <= CORRIDOR_MAX_WIDTH
                    and width_mad_ratio <= CORRIDOR_MAX_WIDTH_MAD
                    and branch_ratio <= CORRIDOR_MAX_BRANCH
                    and area_ratio <= CORRIDOR_MAX_AREA)
    if corridor:
        reason = "ok"
    elif area_ratio > CORRIDOR_MAX_AREA:
        reason = "open_area"
    elif dual_fraction < CORRIDOR_MIN_DUAL:
        reason = "single_edge"
    elif width_ratio > CORRIDOR_MAX_WIDTH:
        reason = "too_wide"
    elif width_mad_ratio > CORRIDOR_MAX_WIDTH_MAD:
        reason = "width_unstable"
    elif branch_ratio > CORRIDOR_MAX_BRANCH:
        reason = "branched"
    else:
        reason = "corridor_reject"
    # Error at the configured near/mid/far rows.
    def err(ratio):
        y = int(np.clip(ratio * height, 0, height - 1))
        ys = np.flatnonzero(np.isfinite(center))
        if len(ys) < 2:
            return 0.
        cx = float(np.interp(y, ys, center[ys]))
        return float(np.clip((cx - .5 * width) / (.5 * width), -1., 1.))
    near, mid, far = (err(getattr(tuning, "road_near_y_ratio", .85)),
                      err(getattr(tuning, "road_mid_y_ratio", .60)),
                      err(getattr(tuning, "road_far_y_ratio", .35)))
    entry, exit_ = _corner_candidates(left, right, width, height, tuning)
    ys = np.flatnonzero(np.isfinite(center))
    points = np.column_stack((center[ys], ys)).astype(float)
    confidence = (dual_fraction
                  * max(0., 1. - width_mad_ratio / max(1e-6, CORRIDOR_MAX_WIDTH_MAD))
                  * max(0., 1. - branch_ratio))
    return CornerObservation(True, reason, left, right, points, near, mid, far,
                             float(np.clip((near + mid) * .5, -1., 1.)),
                             float(np.clip(confidence, 0., 1.)), entry, exit_, mask,
                             corridor, dual_fraction, width_ratio, width_mad_ratio,
                             branch_ratio, area_ratio)


class CornerRingState:
    """Bounded entry/exit corner state machine (short lateral pull windows)."""

    def __init__(self):
        self.entry_until = 0.
        self.exit_until = 0.
        self.mode = "idle"
        self.last_entry = -1e9
        self.last_exit = -1e9
        self.in_ring = False
        self.events = 0

    def reset(self):
        self.__init__()

    def update(self, obs, now, tuning, has_black, agree=True):
        # Only a reliable single-lane corridor that still agrees with the last
        # black route may drive the corner state.
        if obs.corridor and agree:
            if (obs.entry_corner is not None
                    and now - self.last_entry > tuning.contour_corner_cooldown_seconds):
                self.entry_until = now + tuning.contour_corner_entry_seconds
                self.last_entry = now
                self.in_ring = True
                self.mode = "entry_pull"
                self.events += 1
            if (self.in_ring and obs.exit_corner is not None and not has_black
                    and now - self.last_entry >= tuning.contour_corner_min_ring_seconds
                    and now - self.last_exit > tuning.contour_corner_cooldown_seconds):
                self.exit_until = now + tuning.contour_corner_exit_seconds
                self.last_exit = now
                self.mode = "exit_pull"
                self.events += 1
        if self.entry_until > now:
            self.mode = "entry_pull"
        elif self.exit_until > now:
            self.mode = "exit_pull"
        elif self.in_ring:
            self.mode = "ring"
        else:
            self.mode = "idle"

    def pull(self, now, tuning):
        if self.entry_until > now:
            return float(tuning.contour_corner_entry_pull)
        if self.exit_until > now:
            return float(tuning.contour_corner_exit_pull)
        return 0.


def debug_draw(frame, obs, state=None):
    out = frame.copy()
    if obs.mask is not None:
        out[obs.mask > 0] = (out[obs.mask > 0] * .5 + np.array([0, 60, 0]) * .5).astype(np.uint8)
    for arr, color in ((obs.left, (255, 0, 0)), (obs.right, (255, 0, 255))):
        if arr is None:
            continue
        for y, x in enumerate(arr):
            if np.isfinite(x):
                cv2.circle(out, (int(round(float(x))), y), 1, color, -1)
    if obs.center is not None:
        for x, y in np.asarray(obs.center).reshape(-1, 2):
            if np.isfinite(x) and np.isfinite(y):
                cv2.circle(out, (int(round(float(x))), int(round(float(y)))), 1, (0, 255, 0), -1)
    if obs.entry_corner is not None:
        cv2.circle(out, obs.entry_corner, 7, (0, 255, 255), 2)
        cv2.putText(out, "L-corner", (obs.entry_corner[0] - 30, obs.entry_corner[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 255, 255), 1)
    if obs.exit_corner is not None:
        cv2.circle(out, obs.exit_corner, 7, (0, 128, 255), 2)
        cv2.putText(out, "R-corner", (obs.exit_corner[0] + 8, obs.exit_corner[1]),
                    cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 128, 255), 1)
    if state is not None:
        cv2.putText(out, f"corner={state.mode}", (8, 18), cv2.FONT_HERSHEY_SIMPLEX, .55,
                    (255, 255, 255), 1)
        cv2.putText(out, f"corr={int(obs.corridor)} dual={obs.dual_fraction:.2f} "
                         f"w={obs.width_ratio:.2f} mad={obs.width_mad_ratio:.2f} "
                         f"br={obs.branch_ratio:.2f}",
                    (8, 38), cv2.FONT_HERSHEY_SIMPLEX, .42,
                    (0, 255, 0) if obs.corridor else (0, 128, 255), 1)
    return out
