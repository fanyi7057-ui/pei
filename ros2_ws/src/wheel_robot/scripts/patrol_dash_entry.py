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
    near_ratio: float = 0.
    main: PathEstimate = field(default_factory=lambda: PathEstimate(False))
    branch: PathEstimate = field(default_factory=lambda: PathEstimate(False))
    reason: str = "no_dash_blocks"


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
        if np.ptp(branch[:,1])>=.10*height and branch[:,1].max()/height>=tuning.roundabout_entry_gate_near_ratio:
            curve=_path(branch,width,height,tuning,"vv_left_curved_dashes")
            # For 4-entry the selected guide bends LEFT as we look forward.
            # At 5-exit the returning curve points back toward the main line;
            # do not follow that branch backwards, even after a long delay.
            if curve.mid_error<result.main.mid_error-.04 and curve.mid_error<curve.near_error-.02:
                curve.route_locked=True
                result.branch=curve
    return result
