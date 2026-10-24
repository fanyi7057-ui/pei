"""Current-frame patrol masks; never paint old pixels into the image."""
import cv2
import numpy as np


def stabilize_masks(dark, events, hsv, tuning):
    dark, events = dark.copy(), events.copy()
    if not tuning.mask_stability_enabled:
        return dark, events
    if tuning.dark_blue_guard_enabled:
        bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR).astype(np.int16)
        blue = ((hsv[:,:,0] >= 76) & (hsv[:,:,0] <= 140)
                & (hsv[:,:,1] >= tuning.dark_blue_s_min)
                & (bgr[:,:,0]-bgr[:,:,2] >= tuning.dark_blue_br_min))
        dark[blue] = 0
        events[blue] = 0
    width = dark.shape[1]
    wide = []
    for y in range(int(dark.shape[0]*tuning.roi_top_ratio),dark.shape[0]):
        edges = np.diff(np.r_[False,dark[y]>0,False].astype(np.int8))
        for start,end in zip(np.flatnonzero(edges==1),np.flatnonzero(edges==-1)):
            if end-start > width*tuning.black_max_run_ratio:
                wide.append((y,start,end))
    # A sharp real curve can be almost horizontal for a few rows: do not
    # erase it merely for being wide. Require a persistent broad vertical blob.
    rows = sorted({y for y,_,_ in wide})
    bad_rows = set()
    start = 0
    for i in range(1,len(rows)+1):
        if i==len(rows) or rows[i] != rows[i-1]+1:
            if i-start >= max(8,round(dark.shape[0]*.12)):
                bad_rows.update(rows[start:i])
            start = i
    for y,start,end in wide:
        if y in bad_rows:
            dark[y,start:end] = 0
    # Broad zebra bars stay in the separate event mask.
    return dark, events
