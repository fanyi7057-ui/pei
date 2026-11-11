"""Bounded background recording; acquisition never waits for storage."""
from dataclasses import asdict
import yaml
import csv
import hashlib
import json
import queue
import shutil
import threading
import time
from pathlib import Path
import cv2


class PatrolRecorder:
    def __init__(self,directory,package):
        self.directory=Path(directory);self.directory.mkdir(parents=True,exist_ok=False)
        self.queue=queue.Queue(maxsize=4);self.stop=threading.Event()
        self.submitted=self.dropped=self.saved=0;self.error=None
        provenance={"started_wall_time":time.time(),"video_nominal_fps":30,"timestamp_note":"CSV monotonic capture times are authoritative; AVI playback time is not elapsed time","source_sha256":{}}
        for name in ("patrol_contour_runtime.py","patrol_contour_test.py","icn.py","patrol_dash_entry.py","patrol_recorder.py"):
            path=Path(package)/'scripts'/name
            provenance['source_sha256'][name]=hashlib.sha256(path.read_bytes()).hexdigest()
        shutil.copy2(Path(package)/'config/patrol_test.yaml',self.directory/'initial_config.yaml')
        self.config_path=Path(package)/'config/patrol_test.yaml';self.config_digest=None
        (self.directory/'provenance.json').write_text(json.dumps(provenance,ensure_ascii=False,indent=2),encoding='utf-8')
        self.thread=threading.Thread(target=self._run,daemon=True);self.thread.start()

    def submit(self,frame,debug,row):
        self.submitted+=1
        if self.error or self.stop.is_set():self.dropped+=1;return
        if self.queue.full():self.dropped+=1;return
        item=(frame.copy(),None if debug is None else debug.copy(),dict(row))
        try:self.queue.put_nowait(item)
        except queue.Full:self.dropped+=1

    def _run(self):
        writer=None;last_sample=-float('inf');last_config=-float('inf');last_tuning=None
        try:
            with (self.directory/'frames.csv').open('w',newline='',encoding='utf-8') as f:
                fields=['video_frame','capture_monotonic','control_monotonic','sequence','source','mode','phase','reason','error','near','mid','far','far_bend','bend_slow','straight_limited','confidence','cmd_v','cmd_w','queue_dropped','config_sha256']
                rows=csv.DictWriter(f,fieldnames=fields);rows.writeheader()
                while not self.stop.is_set() or not self.queue.empty():
                    try:frame,debug,row=self.queue.get(timeout=.1)
                    except queue.Empty:continue
                    if writer is None:
                        h,w=frame.shape[:2];writer=cv2.VideoWriter(str(self.directory/'raw.avi'),cv2.VideoWriter_fourcc(*'MJPG'),30.,(w,h))
                        if not writer.isOpened():raise OSError('MJPG video writer could not open')
                    now=row['control_monotonic']
                    current_tuning=row.pop("_tuning",None)
                    data=None
                    if current_tuning is not None:
                        if current_tuning!=last_tuning:
                            data=yaml.safe_dump({"patrol_test":asdict(current_tuning)},allow_unicode=True).encode("utf-8")
                            last_tuning=current_tuning
                    elif now-last_config>=1.:
                        data=self.config_path.read_bytes();last_config=now
                    if data is not None:
                        digest=hashlib.sha256(data).hexdigest()
                        if digest!=self.config_digest:
                            (self.directory/f'config_{digest}.yaml').write_bytes(data);self.config_digest=digest
                    writer.write(frame)
                    row.update(video_frame=self.saved,queue_dropped=self.dropped,config_sha256=self.config_digest)
                    rows.writerow({key:row.get(key,'') for key in fields});self.saved+=1
                    if now-last_sample>=1.:
                        folder=self.directory/'samples';folder.mkdir(exist_ok=True)
                        prefix=f'{self.saved-1:06d}'
                        for label,img in [('raw',frame),('debug',debug)]:
                            if img is not None:
                                ok,encoded=cv2.imencode('.png',img)
                                if not ok:raise OSError('sample image encode failed')
                                encoded.tofile(str(folder/f'{prefix}_{label}.png'))
                        last_sample=now;f.flush()
        except Exception as exc:
            self.error=f'{type(exc).__name__}: {exc}'
        finally:
            if writer is not None:writer.release()
            self._summary()

    def _summary(self):
        data=dict(submitted=self.submitted,saved=self.saved,dropped=self.dropped,pending=self.queue.qsize(),error=self.error)
        try:(self.directory/'summary.json').write_text(json.dumps(data,indent=2),encoding='utf-8')
        except OSError:pass

    def close(self):
        self.stop.set();self.thread.join(timeout=5)
        self._summary()
