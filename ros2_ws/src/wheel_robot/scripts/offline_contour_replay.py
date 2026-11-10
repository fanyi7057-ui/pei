#!/usr/bin/env python3
"""Offline white-road algorithm comparison. No ROS or motor commands."""
import argparse
import csv
from dataclasses import replace
from pathlib import Path
import cv2
import numpy as np
import yaml
from patrol_contour_runtime import PatrolTuning,crop_patrol_frame,road_masks,road_centerline


def replay(directory,output):
    directory=Path(directory);output=Path(output);output.mkdir(parents=True,exist_ok=True)
    records=list(csv.DictReader((directory/'frames.csv').open(encoding='utf-8')))
    cap=cv2.VideoCapture(str(directory/'raw.avi'))
    if not cap.isOpened():raise OSError('cannot open recorded raw.avi')
    histories=[None,None];widths=[None,None];last=[None,None];writer=None
    fields=['frame','capture_monotonic','legacy_valid','ordered_valid','legacy_reason','ordered_reason','legacy_error','ordered_error']
    count=[0,0];total=0;last_sample=-float('inf')
    configs={}
    try:
        with (output/'comparison.csv').open('w',newline='',encoding='utf-8') as f:
            table=csv.DictWriter(f,fieldnames=fields);table.writeheader()
            for index,row in enumerate(records):
                ok,frame=cap.read()
                if not ok:break
                digest=row.get('config_sha256','')
                if digest not in configs:
                    path=directory/f'config_{digest}.yaml'
                    if not path.exists():path=directory/'initial_config.yaml'
                    configs[digest]=PatrolTuning(**yaml.safe_load(path.read_text(encoding='utf-8'))['patrol_test']).validate()
                tuning=configs[digest];at=float(row['capture_monotonic'])
                work=cv2.resize(crop_patrol_frame(frame,tuning),(320,240));hsv=cv2.cvtColor(work,cv2.COLOR_BGR2HSV)
                masks=road_masks(hsv,tuning);panels=[];paths=[]
                for mode in (0,1):
                    if last[mode] is not None and at-last[mode]>tuning.loss_hold_seconds:histories[mode]=widths[mode]=None
                    path=road_centerline(hsv,replace(tuning,contour_order_enabled=mode),previous_points=histories[mode],previous_widths=widths[mode],masks=masks)
                    paths.append(path);count[mode]+=int(path.valid)
                    if path.valid:
                        histories[mode]=path.points;last[mode]=at
                        if not path.single_edge:widths[mode]=path.widths
                    panel=work.copy()
                    if path.points is not None and len(path.points)>1:cv2.polylines(panel,[np.rint(path.points).astype(np.int32)],False,(0,255,0),2)
                    cv2.putText(panel,('LEGACY' if mode==0 else 'ORDERED')+' '+path.reason,(2,16),0,.35,(0,0,255),1);panels.append(panel)
                joined=np.hstack(panels)
                if writer is None:
                    writer=cv2.VideoWriter(str(output/'comparison.avi'),cv2.VideoWriter_fourcc(*'MJPG'),30.,(640,240))
                    if not writer.isOpened():raise OSError('cannot open comparison writer')
                writer.write(joined)
                table.writerow(dict(zip(fields,[index,at,paths[0].valid,paths[1].valid,paths[0].reason,paths[1].reason,paths[0].error,paths[1].error])))
                if at-last_sample>=2:
                    encoded_ok,encoded=cv2.imencode('.png',joined)
                    if encoded_ok:encoded.tofile(str(output/f'frame_{index:06d}.png'))
                    last_sample=at
                total+=1
    finally:
        cap.release()
        if writer is not None:writer.release()
    (output/'summary.txt').write_text(f'frames={total}\nlegacy_valid={count[0]}\nordered_valid={count[1]}\nWhite-road replay only; output rate is NOT accuracy or closed-loop driving success. AVI uses nominal 30fps; CSV source timestamps are authoritative.\n',encoding='utf-8')
    return total

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('directory');parser.add_argument('--output');args=parser.parse_args()
    replay(args.directory,args.output or str(Path(args.directory)/'offline_comparison'))
