#!/usr/bin/env python3
"""VV ringEntryMeasure adapted to ROS image coordinates, not VV motor/PWM.

Source: user-supplied vv/src/NewTrack/NewTrack.cpp ringEntryMeasure.
Black = nonzero events; white is an independently verified neutral road mask.
Measurements use an 80x56 reference grid like VV. No secondary state machine.
"""
from dataclasses import dataclass, field

import cv2
import numpy as np

from patrol_contour_runtime import PathEstimate, path_errors, observed_route


@dataclass
class DashObservation:
    valid: bool = False
    blocks: int = 0
    raw_blocks: int = 0
    sided_blocks: int = 0
    near_ratio: float = 0.
    main: PathEstimate = field(default_factory=lambda: PathEstimate(False))
    branch: PathEstimate = field(default_factory=lambda: PathEstimate(False))
    exit_branch: PathEstimate = field(default_factory=lambda: PathEstimate(False))
    reason: str = "no_dash_blocks"
    mask_source: str = "corrected"


def _path(points,width,height,tuning,reason):
    points=np.asarray(points,dtype=float).reshape(-1,2)
    if len(points)<2:
        return PathEstimate(False,reason=reason+"_short")
    result=PathEstimate(True,confidence=.8,points=points,reason=reason)
    result.error,result.near_error,result.mid_error,result.far_error=path_errors(points,width,height,tuning)
    result.route_observed=observed_route(result,width,height,tuning)
    return result


def detect_vv_dashes(events,white,tuning,reference_x_ratio=None):
    """VV's small dark blocks + white sides + x spread + vertical gaps.

    Unlike the original global x-range check, score separate columns: a
    second curved branch must not invalidate a legitimate straight dash chain.
    Thresholds are in the reference grid, never reused as ROS speed units.
    """
    result=DashObservation()
    height,width=events.shape
    black=cv2.resize(events,(80,56),interpolation=cv2.INTER_AREA)>=96
    road=cv2.resize(white,(80,56),interpolation=cv2.INTER_AREA)>=160
    black[:int(56*tuning.vv_dash_roi_top_ratio)]=False
    black[int(56*tuning.vv_dash_roi_bottom_ratio):]=False
    count,_,stats,centers=cv2.connectedComponentsWithStats(black.astype(np.uint8),8)
    result.raw_blocks=count-1
    candidates=[]
    for i in range(1,count):
        x,y,bw,bh,area=stats[i]
        if not (3<=area<=70 and 1<=bw<=12 and 1<=bh<=14):
            continue
        cx,cy=centers[i]
        left=int(np.floor(cx-bw/2-2))
        right=int(np.ceil(cx+bw/2+2))
        row=int(round(cy))
        if left<0 or right>=80 or not road[row,left] or not road[row,right]:
            continue
        candidates.append((float(cx),float(cy),int(bh)))
    result.sided_blocks=len(candidates)
    candidates=sorted(candidates,key=lambda c:c[1],reverse=True)[:40]
    if len(candidates)<tuning.vv_dash_min_blocks:
        result.reason="vv_few_white_sided_blocks"
        return result
    seed=80*(tuning.target_x_ratio if reference_x_ratio is None else float(np.clip(reference_x_ratio,.20,.80)))
    best=None
    short_main=None
    # One block per horizontal band; do not double-count two fork branches.
    for anchor in candidates:
        column=sorted((c for c in candidates if abs(c[0]-anchor[0])<=tuning.vv_dash_max_x_spread_px80/2),key=lambda c:c[1])
        rows=[]
        for c in column:
            if rows and c[1]-rows[-1][1]<max(c[2],rows[-1][2]):
                if abs(c[0]-anchor[0])<abs(rows[-1][0]-anchor[0]):
                    rows[-1]=c
            else:
                rows.append(c)
        if len(rows)<max(3,tuning.vv_dash_min_blocks-1):
            continue
        gaps=sum(b[1]-(a[1]+a[2])>=tuning.vv_dash_min_gap_rows56 for a,b in zip(rows,rows[1:]))
        if gaps*2<len(rows)-1:
            continue
        score=(len(rows)+np.ptp([c[1] for c in rows])/56
               -abs(rows[-1][0]-seed)/20-np.std([c[0] for c in rows])/4)
        if len(rows) >= tuning.vv_dash_min_blocks:
            if best is None or score>best[0]:
                best=(score,rows)
        elif short_main is None or score>short_main[0]:
            short_main=(score,rows)
    short = best is None
    if short:
        best = short_main
    if best is None:
        result.reason="vv_column_or_gap_reject"
        return result
    rows=best[1]
    main=np.asarray([(x*width/80,y*height/56) for x,y,_ in rows])
    result.main=_path(main,width,height,tuning,"vv_main_dashes")
    result.blocks=len(rows)
    result.near_ratio=max((y+h/2)/56 for _,y,h in rows)
    # Verify gaps on source pixels: low-resolution aliasing of a solid line
    # must not manufacture independent dash blocks.
    source_gaps=0
    for first,second in zip(rows,rows[1:]):
        y0=int((first[1]+first[2]/2)*height/56)+1
        y1=int((second[1]-second[2]/2)*height/56)
        if y1<=y0:
            continue
        xs=np.linspace(first[0],second[0],y1-y0)*width/80
        gaps=[]
        for y,x in zip(range(y0,y1),xs):
            x=int(round(x));lo=max(0,x-1);hi=min(width,x+2)
            gaps.append(np.mean(events[y,lo:hi]>0)<.25 and np.mean(white[y,lo:hi]>0)>.5)
        source_gaps+=bool(gaps and np.mean(gaps)>=.5)
    if source_gaps*2<len(rows)-1:
        result.reason="vv_source_gaps_reject"
        return result
    # Compare only within the main chain's observed row span. A detached
    # near block must not reverse the inferred direction by extrapolation.
    branch = []
    for x, y, _ in candidates:
        if main[0, 1] <= y*height/56 <= main[-1, 1]:
            gap = np.interp(y*height/56, main[:, 1], main[:, 0]) - x*width/80
            if .035*width <= gap <= .45*width:
                branch.append((x*width/80, y*height/56, gap))
    branch.sort(key=lambda point: point[1])
    direction = ""
    if len(branch) >= 2 and branch[-1][1]-branch[0][1] >= .10*height:
        changes = np.diff([point[2] for point in branch])
        delta = branch[-1][2]-branch[0][2]
        if delta <= -.0375*width and np.all(changes <= 0):
            direction = "entry"
        elif delta >= .0375*width and np.all(changes >= 0):
            direction = "exit"
    # A short main chain is evidence only with an independently measured fork.
    near_limit = tuning.vv_dash_entry_near_ratio
    if short and direction:
        near_limit = max(.54, near_limit-.08)
    if result.near_ratio < near_limit or (short and not direction):
        result.reason = "vv_column_or_gap_reject" if short else "vv_dashes_not_near"
        result.blocks = 0
        return result
    result.valid = True
    result.reason = "vv_dash_hit"
    if direction:
        curve = _path(np.asarray(branch)[:, :2], width, height, tuning, "vv_left_curved_dashes")
        if direction == "entry":
            curve.route_locked = True
            result.branch = curve
            result.reason = "vv_entry_diverging_left"
        else:
            result.exit_branch = curve
            result.reason = "vv_exit_converging_left"
    return result


def detect_vv_dashes_compatible(events,white,tuning,reference_x_ratio=None,legacy_events=None):
    """Try corrected pixels first; retain valid legacy chain when needed."""
    result=detect_vv_dashes(events,white,tuning,reference_x_ratio)
    if not result.valid and legacy_events is not None:
        legacy=detect_vv_dashes(legacy_events,white,tuning,reference_x_ratio)
        if legacy.valid:
            legacy.mask_source="legacy_compatible"
            return legacy
    return result
