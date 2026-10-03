"""Black single-line, roundabout and crosswalk diagnostic for task 3."""

from dataclasses import dataclass
from typing import Optional
import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


@dataclass
class Band:
    name: str
    valid: bool
    x: float = 0.0
    quality: float = 0.0
    box: Optional[tuple] = None


class Visual(Node):
    def __init__(self):
        super().__init__("hh_visual")
        defaults = {
            "camera_id": 0, "enable_motion": False, "black_v_max": 90,
            "black_s_max": 105, "roi_top": 0.58, "roi_bottom": 0.98,
            "min_component_area": 45,
            "max_component_area_ratio": 0.28, "line_target_offset": 0.0,
            "min_line_quality": 0.22, "min_valid_bands": 2,
            "line_speed": 0.10, "line_kp": 0.55,
            "max_angular_speed": 0.28, "crosswalk_top": 0.58,
            "crosswalk_bottom": 0.94, "crosswalk_row_fill": 0.24,
            "crosswalk_min_stripes": 5, "crosswalk_threshold": 0.75,
            "roundabout_top": 0.30, "roundabout_bottom": 0.70,
            "roundabout_min_parts": 4, "roundabout_threshold": 0.70,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.cap = cv2.VideoCapture(int(self.p("camera_id")))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        if not self.cap.isOpened():
            raise RuntimeError("无法打开RGB摄像头，请关闭占用 /dev/video0 的程序")
        self.timer = self.create_timer(0.05, self.tick)

    def p(self, name):
        return self.get_parameter(name).value

    @staticmethod
    def runs(bits):
        changes = np.flatnonzero(np.diff(np.pad(bits.astype(np.uint8), (1, 1)).astype(np.int16)))
        return [(int(changes[i]), int(changes[i + 1])) for i in range(0, len(changes), 2)]

    def band(self, mask, name, y0, y1):
        image = mask[y0:y1]
        contours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        limit = image.size * float(self.p("max_component_area_ratio"))
        choices = [c for c in contours if float(self.p("min_component_area")) <= cv2.contourArea(c) <= limit]
        if not choices:
            return Band(name, False)

        def score(c):
            _, _, width, height = cv2.boundingRect(c)
            return cv2.contourArea(c) * (1.0 + min(2.0, height / max(1, width)))

        contour = max(choices, key=score)
        moment = cv2.moments(contour)
        if moment["m00"] <= 0:
            return Band(name, False)
        x, y, width, height = cv2.boundingRect(contour)
        center = moment["m10"] / moment["m00"]
        quality = np.clip(0.65 * height / max(1, y1-y0) + 0.35 * min(1.0, height/max(1, width)), 0, 1)
        return Band(name, True, float(center), float(quality), (x, y+y0, width, height))

    def analyse(self, frame):
        height, width = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Real black tape/printing is both dark and weakly saturated.  Using V
        # alone incorrectly classifies the saturated blue track border as black.
        mask = cv2.inRange(
            hsv,
            np.array([0, 0, 0], dtype=np.uint8),
            np.array([179, int(self.p("black_s_max")),
                      int(self.p("black_v_max"))], dtype=np.uint8),
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        top, bottom = int(height*float(self.p("roi_top"))), int(height*float(self.p("roi_bottom")))
        span = bottom-top
        definitions = (("FAR", 0, .32, .15, (255,160,0)), ("MID", .32,.66,.30,(0,255,255)), ("NEAR",.66,1,.55,(0,255,0)))
        debug, bands, sx, sw, sq = frame.copy(), [], 0.0, 0.0, 0.0
        for name, a, b, weight, color in definitions:
            y0, y1 = top+int(span*a), top+int(span*b)
            item = self.band(mask, name, y0, y1)
            bands.append(item)
            cv2.rectangle(debug, (0,y0), (width-1,y1), color, 1)
            if item.valid:
                sx, sw, sq = sx+item.x*weight, sw+weight, sq+item.quality*weight
                x,y,bw,bh = item.box
                cv2.rectangle(debug, (x,y), (x+bw,y+bh), color, 2)
        quality = sq/sw if sw else 0.0
        target = width*(0.5+float(self.p("line_target_offset")))
        center = sx/sw if sw else target
        error = (center-target)/max(1, width*.5)
        valid_bands = sum(1 for item in bands if item.valid)
        line_ok = (valid_bands >= int(self.p("min_valid_bands"))
                   and quality >= float(self.p("min_line_quality")))
        cv2.line(debug, (int(target),top), (int(target),bottom), (255,0,255), 2)
        cv2.circle(debug, (int(center),bottom-8), 8, (0,0,255), -1)

        c0, c1 = int(height*float(self.p("crosswalk_top"))), int(height*float(self.p("crosswalk_bottom")))
        filled = np.mean(mask[c0:c1] > 0, axis=1) >= float(self.p("crosswalk_row_fill"))
        stripes = len([r for r in self.runs(filled) if 3 <= r[1]-r[0] <= 35])
        cross_score = min(1.0, stripes/max(1, int(self.p("crosswalk_min_stripes"))))
        cv2.rectangle(debug, (0,c0), (width-1,c1), (255,0,255), 2)

        r0, r1 = int(height*float(self.p("roundabout_top"))), int(height*float(self.p("roundabout_bottom")))
        contours, _ = cv2.findContours(mask[r0:r1], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        parts=[]
        for contour in contours:
            x,y,bw,bh=cv2.boundingRect(contour)
            if 5 <= bw <= width*.18 and 2 <= bh <= height*.10:
                parts.append((x,y+r0,bw,bh))
        align=max(0.0, 1.0-float(np.std([y+bh*.5 for _,y,_,bh in parts]))/(height*.08)) if parts else 0.0
        round_score=min(1.0, len(parts)/max(1,int(self.p("roundabout_min_parts"))))*align
        return mask, debug, bands, line_ok, error, quality, cross_score, stripes, round_score, len(parts)

    def tick(self):
        ok, frame = self.cap.read()
        if not ok:
            self.pub.publish(Twist()); return
        mask, debug, bands, line_ok, error, quality, cross, stripes, round_score, parts = self.analyse(frame)
        cross_hit = cross >= float(self.p("crosswalk_threshold"))
        round_hit = round_score >= float(self.p("roundabout_threshold"))
        labels=(f"BLACK LINE {'OK' if line_ok else 'LOST'} err={error:+.2f} q={quality:.2f}",
                " ".join(f"{b.name}:{b.quality:.2f}" if b.valid else f"{b.name}:--" for b in bands),
                f"ROUNDABOUT {'HIT' if round_hit else '---'} score={round_score:.2f} parts={parts}",
                f"CROSSWALK {'HIT' if cross_hit else '---'} score={cross:.2f} stripes={stripes}")
        for i, text in enumerate(labels):
            cv2.putText(debug,text,(10,26+i*26),cv2.FONT_HERSHEY_SIMPLEX,.55,(255,255,255),2)
        cmd=Twist()
        if bool(self.p("enable_motion")) and line_ok and not cross_hit and not round_hit:
            cmd.linear.x=float(self.p("line_speed"))
            limit=float(self.p("max_angular_speed"))
            cmd.angular.z=float(np.clip(-float(self.p("line_kp"))*error,-limit,limit))
        self.pub.publish(cmd)
        cv2.imshow("hh task3 black line | Q stop",np.hstack((debug,cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR))))
        if cv2.waitKey(1)&0xff in (ord('q'),27): rclpy.shutdown()

    def close(self):
        for _ in range(3): self.pub.publish(Twist())
        self.cap.release(); cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args); node=Visual()
    try: rclpy.spin(node)
    finally:
        node.close(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
