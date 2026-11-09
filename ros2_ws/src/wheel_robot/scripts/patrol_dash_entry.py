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
    diagnostics: dict = field(default_factory=dict)


def detect_color_dashes(frame, tuning, reference_x_ratio=None):
    """Independent colour evidence for tasks; never modifies patrol masks.

    Relative darkness recovers reflective tape, while neutral colour, local
    contrast and a dark core reject blue and broad shadows. No open/close is
    applied to small blocks. Geometry and source-gap proof remain shared.
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hue, sat, val = cv2.split(hsv)
    blue = ((hue >= tuning.blue_h_min) & (hue <= tuning.blue_h_max)
            & (sat >= tuning.blue_weak_s_min))
    local = cv2.blur(val.astype(np.float32), (21, 21))
    neutral = (sat <= 100) & ~blue
    candidate = neutral & ((val <= 85) | ((val <= 175) & (local-val >= 25)))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(candidate.astype(np.uint8), 8)
    accepted = np.zeros(count, dtype=np.uint8)
    rejected_core = 0
    for index in range(1, count):
        x,y,w,h,area = stats[index]
        pixels = labels[y:y+h,x:x+w] == index
        if np.mean(val[y:y+h,x:x+w][pixels] <= 110) < .06:
            rejected_core += 1
            continue
        accepted[index] = 255
    events = accepted[labels]
    white = ((sat <= tuning.white_s_max) & (val >= tuning.white_v_min)
             & ~blue & ~candidate).astype(np.uint8)*255
    result = detect_vv_dashes(events, white, tuning, reference_x_ratio)
    result.diagnostics.update(color_components=count-1, core_rejected=rejected_core,
                              blue_pixels=int(np.count_nonzero(blue)))
    return result


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
    result.diagnostics = {"shape_rejected": 0, "white_sides_rejected": 0}
    candidates=[]
    for i in range(1,count):
        x,y,bw,bh,area=stats[i]
        if not (3<=area<=70 and 1<=bw<=12 and 1<=bh<=14):
            result.diagnostics["shape_rejected"] += 1
            continue
        cx,cy=centers[i]
        left=int(np.floor(cx-bw/2-2))
        right=int(np.ceil(cx+bw/2+2))
        row=int(round(cy))
        if left<0 or right>=80 or not road[row,left] or not road[row,right]:
            result.diagnostics["white_sides_rejected"] += 1
            continue
        candidates.append((float(cx),float(cy),int(bh)))
    result.sided_blocks=len(candidates)
    candidates=sorted(candidates,key=lambda c:c[1],reverse=True)[:40]
    if len(candidates)<tuning.vv_dash_min_blocks:
        result.reason="vv_few_white_sided_blocks"
        return result
    seed=80*(tuning.target_x_ratio if reference_x_ratio is None else float(np.clip(reference_x_ratio,.20,.80)))
    best=None
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
        if len(rows)<tuning.vv_dash_min_blocks:
            continue
        gaps=sum(b[1]-(a[1]+a[2])>=tuning.vv_dash_min_gap_rows56 for a,b in zip(rows,rows[1:]))
        if gaps*2<len(rows)-1:
            continue
        score=(len(rows)+np.ptp([c[1] for c in rows])/56
               -abs(rows[-1][0]-seed)/20-np.std([c[0] for c in rows])/4)
        if best is None or score>best[0]:
            best=(score,rows)
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
    if result.near_ratio<tuning.vv_dash_entry_near_ratio:
        result.reason="vv_dashes_not_near"
        return result
    result.valid=True
    result.reason="vv_dash_hit"
    # Curved LEFT guide is an actual observation, not a fabricated connector.
    fit=np.polyfit(main[:,1],main[:,0],1)
    branch=[]
    for x,y,_ in candidates:
        px,py=x*width/80,y*height/56
        gap=np.polyval(fit,py)-px
        if .035*width<=gap<=.45*width:
            branch.append((px,py))
    if len(branch)>=2:
        branch=np.asarray(branch)
        if np.ptp(branch[:,1])>=.10*height and branch[:,1].max()/height>=tuning.vv_dash_entry_near_ratio:
            curve=_path(branch,width,height,tuning,"vv_left_curved_dashes")
            # For 4-entry the selected guide bends LEFT as we look forward.
            # At 5-exit the returning curve points back toward the main line;
            # do not follow that branch backwards, even after a long delay.
            near_gap=result.main.near_error-curve.near_error
            mid_gap=result.main.mid_error-curve.mid_error
            # Compare TWO guides, not their absolute screen slope. Camera
            # yaw/ordinary main-road slant must not reverse fork identity.
            if mid_gap>.04 and mid_gap>near_gap+.02:
                curve.route_locked=True
                result.branch=curve
                result.reason="vv_entry_diverging_left"
            elif near_gap>mid_gap+.02:
                result.exit_branch=curve
                result.reason="vv_exit_converging_left"
    return result
