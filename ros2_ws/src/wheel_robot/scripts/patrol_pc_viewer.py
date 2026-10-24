"""Auto-opened X11 FPV window: read-only, separate from camera/control."""
import http.client
import sys
import time
import cv2
import numpy as np


def read_jpeg(response):
    for _ in range(20):
        line=response.readline(2048)
        if not line:
            raise OSError('stream ended')
        if line.strip()==b'--frame':
            break
    else:
        raise ValueError('invalid boundary')
    size=0
    for _ in range(12):
        line=response.readline(2048)
        if line==b'\r\n':break
        if line.lower().startswith(b'content-length:'):
            size=int(line.split(b':',1)[1])
    if not 0<size<=1024*1024:
        raise ValueError('invalid JPEG length')
    data=response.read(size)
    if len(data)!=size:
        raise OSError('truncated JPEG')
    return data


def main(port):
    title='Robot First Person - read only (close window does NOT stop robot)'
    cv2.namedWindow(title,cv2.WINDOW_NORMAL)
    cv2.resizeWindow(title,640,480)
    connection=None
    try:
        while cv2.getWindowProperty(title,cv2.WND_PROP_VISIBLE)>=1:
            try:
                connection=http.client.HTTPConnection('127.0.0.1',port,timeout=1.)
                connection.request('GET','/stream');response=connection.getresponse()
                if response.status!=200:raise OSError('preview unavailable')
                while cv2.getWindowProperty(title,cv2.WND_PROP_VISIBLE)>=1:
                    frame=cv2.imdecode(np.frombuffer(read_jpeg(response),np.uint8),cv2.IMREAD_COLOR)
                    if frame is None:raise ValueError('invalid JPEG')
                    cv2.imshow(title,frame)
                    if cv2.waitKey(1)&0xff in (ord('q'),27):return
            except (OSError,ValueError,http.client.HTTPException):
                blank=np.zeros((360,640,3),np.uint8)
                cv2.putText(blank,'NO FRESH CAMERA - reconnecting',(20,170),cv2.FONT_HERSHEY_SIMPLEX,.7,(0,180,255),2)
                cv2.imshow(title,blank)
                if cv2.waitKey(100)&0xff in (ord('q'),27):return
            finally:
                if connection is not None:connection.close()
    finally:
        cv2.destroyAllWindows()


if __name__=='__main__':
    main(int(sys.argv[1]))
