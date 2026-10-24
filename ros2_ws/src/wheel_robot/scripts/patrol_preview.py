"""Read-only loopback MJPEG preview, isolated from the control callback."""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import threading
import time
import cv2


class FirstPersonPreview:
    def __init__(self, get_frame, port=8080):
        self.get_frame = get_frame
        self.stop = threading.Event()
        self.condition = threading.Condition()
        self.jpeg = None
        self.stamp = 0.
        self.clients = 0
        self.slots = threading.BoundedSemaphore(4)
        owner = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_args):
                pass

            def do_GET(self):
                if self.path == '/':
                    body = (b'<!doctype html><meta charset="utf-8"><title>Robot FPV</title>'
                        b'<h3>Read-only camera; use robot remote emergency stop</h3>'
                        b'<p id="s">Connecting</p><img src="/stream" style="max-width:100%">'
                        b'<script>setInterval(async()=>{try{let r=await fetch("/status",{cache:"no-store"});'
                        b'let s=await r.json();document.getElementById("s").textContent='
                        b's.fresh?"LIVE camera age "+s.age.toFixed(2)+"s":"STALE / no fresh camera frame";'
                        b'}catch(e){document.getElementById("s").textContent="DISCONNECTED"}},1000)</script>')
                    self.send_response(200);self.send_header('Content-Type','text/html; charset=utf-8')
                    self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)
                    return
                if self.path == '/status':
                    with owner.condition:
                        age = time.monotonic()-owner.stamp if owner.stamp else None
                        fresh = owner.jpeg is not None and age is not None and age < .5
                    body = json.dumps({'fresh':fresh,'age':age}).encode()
                    self.send_response(200);self.send_header('Content-Type','application/json')
                    self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(body)))
                    self.end_headers();self.wfile.write(body);return
                if self.path != '/stream':
                    self.send_error(404);return
                if not owner.slots.acquire(blocking=False):
                    self.send_error(503);return
                with owner.condition:
                    owner.clients += 1
                try:
                    self.connection.settimeout(2.)
                    self.send_response(200)
                    self.send_header('Content-Type','multipart/x-mixed-replace; boundary=frame')
                    self.send_header('Cache-Control','no-store');self.end_headers()
                    previous = None
                    while not owner.stop.is_set():
                        with owner.condition:
                            owner.condition.wait_for(lambda:owner.stop.is_set() or
                                (owner.jpeg is not None and owner.stamp != previous),timeout=.5)
                            data,stamp = owner.jpeg,owner.stamp
                        if data is None or stamp == previous:
                            continue
                        previous = stamp
                        if time.monotonic()-stamp > .5:
                            continue
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+
                            str(len(data)).encode()+b'\r\n\r\n'+data+b'\r\n');self.wfile.flush()
                except (OSError,ValueError):
                    pass
                finally:
                    with owner.condition:
                        owner.clients -= 1
                    owner.slots.release()

        self.server = ThreadingHTTPServer(('127.0.0.1',port),Handler)
        self.server.daemon_threads = True
        self.port = self.server.server_address[1]
        self.http_thread = threading.Thread(target=self.server.serve_forever,kwargs={'poll_interval':.2},daemon=True)
        self.encoder = threading.Thread(target=self._encode,daemon=True)
        self.http_thread.start();self.encoder.start()

    def _encode(self):
        while not self.stop.wait(.125):
            with self.condition:
                watching = self.clients > 0
            if not watching:
                continue
            try:
                frame,stamp = self.get_frame()
                if frame is None or time.monotonic()-stamp > .35:
                    with self.condition:
                        self.jpeg = None
                    continue
                if stamp == self.stamp:
                    continue
                if frame.shape[1] > 640:
                    frame = cv2.resize(frame,(640,round(frame.shape[0]*640/frame.shape[1])))
                ok,jpeg = cv2.imencode('.jpg',frame,[cv2.IMWRITE_JPEG_QUALITY,70])
                if ok:
                    with self.condition:
                        self.jpeg,self.stamp = jpeg.tobytes(),stamp
                        self.condition.notify_all()
            except Exception:
                # Preview failure can never publish a motion command or take
                # down the control loop. Status becomes stale on failure.
                with self.condition:
                    self.jpeg = None

    def close(self):
        self.stop.set()
        with self.condition:
            self.condition.notify_all()
        self.server.shutdown();self.server.server_close()
        self.http_thread.join(timeout=1.);self.encoder.join(timeout=1.)
