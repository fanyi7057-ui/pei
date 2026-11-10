#!/usr/bin/env python3
"""Stationary diagnostic capture. No race controller or entry state machine."""
import os
import select
import sys
import threading
import time
from pathlib import Path
import cv2
import numpy as np
import rclpy
import yaml
import icn as base
from stand_only import WheellegStandOnly
from patrol_contour_runtime import PatrolTuning, road_masks, crop_patrol_frame
from patrol_dash_entry import detect_vv_dashes_compatible


def build_capture(frame, tuning):
    cropped=crop_patrol_frame(frame,tuning)
    work=cv2.resize(cropped,(base.PROCESS_WIDTH,base.PROCESS_HEIGHT),interpolation=cv2.INTER_AREA)
    vision=base.CourseVision()
    dark,events,hsv,_=vision._make_dark_mask(work)
    blue,white=road_masks(hsv,tuning)
    guard=cv2.dilate(blue,np.ones((3,3),np.uint8))
    dark_before=dark.copy();events_before=events.copy()
    dark[guard>0]=0;events[guard>0]=0
    legacy=vision._legacy_event_dark.copy();legacy[guard>0]=0
    cue=detect_vv_dashes_compatible(events,white,tuning,legacy_events=legacy)
    # Independent COLOR proposal for offline comparison only, never controls motors.
    value=hsv[:,:,2];saturation=hsv[:,:,1];hue=hsv[:,:,0]
    local=cv2.GaussianBlur(value,(31,31),0).astype(np.int16)-value.astype(np.int16)
    known_blue=(hue>=tuning.blue_h_min)&(hue<=tuning.blue_h_max)&(saturation>=tuning.blue_s_min)&(value>=tuning.blue_v_min)
    proposal=(((value<85)|((value<145)&(local>22)))&~known_blue).astype(np.uint8)*255
    annotated=work.copy();count,labels,stats,_=cv2.connectedComponentsWithStats(proposal,8)
    rows=[]
    for i in range(1,count):
        x,y,w,h,area=map(int,stats[i]);reason='candidate'
        if area<8:reason='small'
        elif h>work.shape[0]*.28 or w>work.shape[1]*.22:reason='long_or_large'
        elif y+h<tuning.vv_dash_roi_top_ratio*work.shape[0] or y>tuning.vv_dash_roi_bottom_ratio*work.shape[0]:reason='outside_roi'
        rows.append(f'{i},{x},{y},{w},{h},{area},{reason}')
        if area>=8:
            color=(0,255,0) if reason=='candidate' else (0,0,255)
            cv2.rectangle(annotated,(x,y),(x+w,y+h),color,1)
            cv2.putText(annotated,f'{i}:{reason}',(x,max(10,y)),cv2.FONT_HERSHEY_SIMPLEX,.28,color,1)
    images={'raw':frame,'cropped_color':cropped,'processed_color':work,'black_before_guard':dark_before,'events_before_guard':events_before,
            'black_actual':dark,'events_actual':events,'blue_guard':guard,'white_road':white,
            'events_legacy':legacy,'color_proposal':proposal,'candidate_labels':annotated}
    panels=[]
    for label in ('processed_color','black_actual','events_actual','blue_guard','color_proposal','candidate_labels'):
        panel=images[label]
        if panel.ndim==2:panel=cv2.cvtColor(panel,cv2.COLOR_GRAY2BGR)
        panel=panel.copy();cv2.putText(panel,label,(4,16),0,.45,(0,0,255),1);panels.append(panel)
    images['comparison']=np.vstack((np.hstack(panels[:3]),np.hstack(panels[3:])))
    metadata=f'height_m=0.15 pitch_deg=25 crop_left={tuning.crop_left_ratio} crop_right={tuning.crop_right_ratio} crop_top={tuning.crop_top_ratio}\ndash_mask={cue.mask_source} dash_reason={cue.reason} raw={cue.raw_blocks} sided={cue.sided_blocks} accepted={cue.blocks} valid={cue.valid}\nCOLOR proposal is diagnostic only, not validated ring detection.\n'
    return images,metadata,'id,x,y,width,height,area,reason\n'+'\n'.join(rows)


def main():
    rclpy.init()
    node=WheellegStandOnly()
    package=Path(__file__).resolve().parents[1]
    tuning=PatrolTuning(**yaml.safe_load((package/'config/patrol_test.yaml').read_text())['patrol_test']).validate()
    output=package.parents[2]/'log'/'camera_capture'
    lock=threading.Lock();stop=threading.Event();latest=[None,0.];worker=None
    def capture():
        cap=cv2.VideoCapture(base.CAMERA_ID)
        try:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH,640);cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
            while not stop.is_set():
                ok,frame=cap.read()
                if not ok:stop.wait(.1);continue
                with lock:latest[:]=[frame,time.monotonic()]
        finally:cap.release()
    thread=threading.Thread(target=capture,daemon=True);thread.start()
    requests=[]
    from std_msgs.msg import Bool
    subscription=node.create_subscription(Bool,"/camera_capture/request",lambda msg: requests.append("1") if msg.data and not requests else None,1)
    topic_mode=os.environ.get("CAPTURE_TOPIC_MODE")=="1"
    started=time.monotonic();auto_done=False;preview=bool(os.environ.get('DISPLAY'))
    print('原地拍照：腿高0.15m，低头25度；持续零轮速，不巡线、不入环。5秒后自动拍一组；输入1再拍，0退出；Ctrl+C退出。',flush=True)
    def save(frame):
        try:
            images,metadata,csv=build_capture(frame,tuning)
            directory=output/time.strftime('%Y%m%d_%H%M%S')
            directory=directory.with_name(directory.name+f'_{time.time_ns()%1000000:06d}');directory.mkdir(parents=True)
            for name,img in images.items():
                if not cv2.imwrite(str(directory/(name+'.png')),img):raise OSError('image write failed: '+name)
            (directory/'details.txt').write_text(metadata,encoding='utf-8')
            (directory/'candidates.csv').write_text(csv,encoding='utf-8')
            print('照片已保存：'+str(directory),flush=True)
        except Exception as exc:print('拍照失败：'+str(exc),flush=True)
    try:
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.02)
            command=requests.pop(0) if requests else None
            if not topic_mode and select.select([sys.stdin],[],[],0)[0]:
                line=sys.stdin.readline()
                if not line:break
                command=line.strip()
            if command=='0':break
            with lock:frame,stamp=latest[0],latest[1]
            if frame is not None and preview:
                try:
                    cv2.imshow('Stationary capture - ZERO WHEEL SPEED',frame)
                    if cv2.waitKey(1)&255 in (27,ord('q')):break
                except cv2.error:preview=False
            if command=='1' or (not auto_done and time.monotonic()-started>=5):
                if frame is None or time.monotonic()-stamp>.35:
                    if command=='1':print('没有新鲜相机画面，未保存。',flush=True)
                    continue
                if worker is not None and worker.is_alive():
                    print('正在保存上一组，请稍候。',flush=True);continue
                if time.monotonic()-started<3:
                    print('等待姿态稳定3秒后再拍。',flush=True);continue
                auto_done=True;worker=threading.Thread(target=save,args=(frame.copy(),),daemon=True);worker.start()
    except KeyboardInterrupt:pass
    finally:
        stop.set()
        if rclpy.ok():node.close()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        cv2.destroyAllWindows()
        thread.join(timeout=1)
        if worker is not None:worker.join(timeout=5)

if __name__=='__main__':main()
