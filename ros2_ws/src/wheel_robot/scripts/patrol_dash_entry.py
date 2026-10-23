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
    exit_branch: PathEstimate = field(default_factory=lambda: PathEstimate(False))
    reason: str = "no_dash_blocks"
    white_blocks: int = 0
    column_blocks: int = 0
    branch_blocks: int = 0
    raw_blocks: int = 0


def _path(points,width,height,tuning,reason):
    points=np.asarray(points,dtype=float).reshape(-1,2)
    if len(points)<2:
        return PathEstimate(False,reason=reason+"_short")
    result=PathEstimate(True,confidence=.8,points=points,reason=reason)
    result.error,result.near_error,result.mid_error,result.far_error=path_errors(points,width,height,tuning)
    result.route_observed=observed_route(result,width,height,tuning)
    return result


def _white_sided_blocks(events,white,tuning,diagnostics=None):
    height,width=events.shape
    black=cv2.resize(events,(80,56),interpolation=cv2.INTER_AREA)>=96
    road=cv2.resize(white,(80,56),interpolation=cv2.INTER_AREA)>=160
    black[:int(56*tuning.vv_dash_roi_top_ratio)]=False
    black[int(56*tuning.vv_dash_roi_bottom_ratio):]=False
    count,_,stats,centers=cv2.connectedComponentsWithStats(black.astype(np.uint8),8)
    if diagnostics is not None:
        diagnostics.raw_blocks=count-1
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
    return height,width,candidates


def detect_vv_dashes(events,white,tuning,reference_x_ratio=None):
    """VV's small dark blocks + white sides + x spread + vertical gaps.

    Unlike the original global x-range check, score separate columns: a
    second curved branch must not invalidate a legitimate straight dash chain.
    Thresholds are in the reference grid, never reused as ROS speed units.
    """
    result=DashObservation()
    height,width,candidates=_white_sided_blocks(events,white,tuning,result)
    candidates=sorted(candidates,key=lambda c:c[1],reverse=True)[:40]
    result.white_blocks=len(candidates)
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
        result.column_blocks=max(result.column_blocks,len(rows))
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
    result.branch_blocks=len(branch)
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

def track_vv_curve(events,white,tuning,reference,reacquire=False):
    """Track current small blocks near an ALREADY selected curved guide.

    Never authorizes entry. Unlike fork detection, no straight main column is
    required. No observations means invalid, not an extrapolated old route.
    """
    height,width,candidates=_white_sided_blocks(events,white,tuning)
    ref=np.asarray(reference,float).reshape(-1,2)
    if len(ref)<2:
        return PathEstimate(False,reason="vv_curve_no_reference")
    ref=ref[np.argsort(ref[:,1])]
    nearby=[]
    for x,y,_ in candidates:
        px,py=x*width/80,y*height/56
        if ref[0,1]-.12*height<=py<=ref[-1,1]+.12*height:
            distance=abs(px-np.interp(py,ref[:,1],ref[:,0]))
            if distance<=.12*width:
                nearby.append((py,px,distance))
    chosen=[]
    for py,px,distance in sorted(nearby):
        if chosen and py-chosen[-1][0]<.04*height:
            if distance<chosen[-1][2]:
                chosen[-1]=(py,px,distance)
        else:
            chosen.append((py,px,distance))
    if len(chosen)<2 or chosen[-1][0]-chosen[0][0]<.10*height:
        if reacquire and tuning.ring_curve_reacquire_enabled:
            return _reacquire_owned_curve(candidates,ref,width,height,tuning)
        return PathEstimate(False,reason="vv_curve_missing_current_blocks")
    guide=_path([(x,y) for y,x,_ in chosen],width,height,tuning,"vv_owned_curve")
    guide.route_locked=True
    # Local multi-block evidence only; global black/road thresholds unchanged.
    guide.route_observed=(len(chosen)>=3 and chosen[-1][0]-chosen[0][0]>=.12*height
                          and chosen[-1][0]>=tuning.vv_dash_entry_near_ratio*height)
    return guide


def _reacquire_owned_curve(candidates,reference,width,height,tuning):
    """FOLLOW-only bounded translation consensus of CURRENT white-sided blocks.

    Never an entrance cue or a fabricated connector. Parallel unrelated
    alternatives with similar support are ambiguous and must not take over.
    """
    ref=reference[np.argsort(reference[:,1])]
    points=[(x*width/80,y*height/56) for x,y,_ in candidates
            if ref[0,1]-.12*height<=y*height/56<=ref[-1,1]+.12*height]
    offsets=sorted({round((x-np.interp(y,ref[:,1],ref[:,0]))/(.025*width))*.025*width
                    for x,y in points if abs(x-np.interp(y,ref[:,1],ref[:,0]))<=tuning.ring_curve_reacquire_max_shift_ratio*width})
    options=[]
    for shift in offsets:
        rows=[]
        for x,y in sorted(points,key=lambda p:p[1]):
            residual=abs(x-(np.interp(y,ref[:,1],ref[:,0])+shift))
            if residual>.04*width:continue
            point=(y,x,residual)
            if rows and y-rows[-1][0]<.04*height:
                if residual<rows[-1][2]:rows[-1]=point
            else:rows.append(point)
        if len(rows)<2 or rows[-1][0]-rows[0][0]<.14*height:continue
        expected=np.interp(rows[-1][0],ref[:,1],ref[:,0])-np.interp(rows[0][0],ref[:,1],ref[:,0])
        actual=rows[-1][1]-rows[0][1]
        if abs(expected-actual)>.045*width:continue
        score=len(rows)+(rows[-1][0]-rows[0][0])/height-sum(p[2] for p in rows)/width-abs(shift)/width
        options.append((score,shift,rows))
    if not options:return PathEstimate(False,reason="vv_curve_no_current_consensus")
    options.sort(key=lambda p:p[0],reverse=True)
    score,shift,rows=options[0]
    if any(score-other[0]<.1 and abs(shift-other[1])>.08*width for other in options[1:]):
        return PathEstimate(False,reason="vv_curve_reacquire_ambiguous")
    guide=_path([(x,y) for y,x,_ in rows],width,height,tuning,"vv_owned_curve_reacquired")
    guide.confidence=.65
    guide.route_locked=True
    guide.route_observed=(len(rows)>=3 and rows[-1][0]>=tuning.vv_dash_entry_near_ratio*height)
    return guide
