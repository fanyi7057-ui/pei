"""Measured, independently completed bends for posture only, never task gates."""
import math


class TurnPostureGate:
    def __init__(self):
        self.count = 0
        self.last_completed_deg = 0.
        self.pause()

    @property
    def complete(self):
        return self.count >= 2

    def pause(self):
        self.source = None
        self.last = self.reference = None
        self.last_stamp = None
        self.started_at = None
        self.turn = 0.
        self.travel = 0.
        self.last_pose = None
        self.center_hits = 0
        self.turning = False
        self.reason = 'waiting_heading'

    def reset(self):
        self.count = 0
        self.last_completed_deg = 0.
        self.pause()

    def update(self, heading, pose, now, linear, angular, path, tuning, active=True):
        if self.complete:
            return False
        if not active or linear <= .03 or heading is None:
            self.pause();return False
        source,yaw,stamp = heading
        if (not all(math.isfinite(v) for v in (yaw,stamp,now))
                or not 0 <= now-stamp <= tuning.roundabout_heading_timeout_seconds):
            self.pause();return False
        if self.source != source or self.last_stamp is None:
            self.pause();self.source=source;self.last=self.reference=yaw
            self.last_stamp=stamp;self.last_pose=pose;self.reason='ready';return False
        dt=stamp-self.last_stamp
        if dt <= 0:
            return False  # repeated camera callbacks cannot add gyro proof/center frames
        if dt > tuning.roundabout_heading_timeout_seconds:
            self.pause();return False
        delta=(yaw-self.last+math.pi)%(2*math.pi)-math.pi
        rate=delta/dt
        if abs(rate)>4.:
            self.pause();self.reason='heading_jump';return False
        self.last,self.last_stamp=yaw,stamp
        centered=(path.valid and not path.rescue and not path.handoff
                  and path.confidence>=tuning.rescue_corner_confidence
                  and abs(path.error)<=tuning.posture_turn_center_error)
        stable=centered and abs(rate)<.30 and abs(angular)<.35
        if not self.turning:
            change=(yaw-self.reference+math.pi)%(2*math.pi)-math.pi
            if abs(change)>=math.radians(12):
                self.turning=True;self.turn=change;self.started_at=now
                self.travel=0.;self.reason='turning'
            elif stable:
                # Rebase only in straight motion: never keep chasing a turning
                # yaw reference, and never sum absolute left/right wobble.
                if abs(rate)<.18 and abs(angular)<.15:
                    self.reference=yaw
        else:
            self.turn+=delta
            if pose is not None and self.last_pose is not None:
                step=math.hypot(pose[0]-self.last_pose[0],pose[1]-self.last_pose[1])
                if math.isfinite(step) and step<=.2:
                    self.travel+=step
            elif pose is None:
                self.travel+=linear*min(dt,.1)  # labeled command fallback, not measured travel
            angle=abs(math.degrees(self.turn))
            if now-self.started_at>tuning.posture_turn_timeout_seconds or angle>130:
                self.pause();self.reason='turn_rejected';return False
            confirmed=(angle>=tuning.posture_turn_target_deg and stable
                       and self.travel>=tuning.posture_turn_min_travel_m)
            self.center_hits=self.center_hits+1 if confirmed else 0
            if self.center_hits>=tuning.posture_turn_center_frames:
                self.last_completed_deg = math.degrees(self.turn)
                self.count+=1;self.pause();self.reason='completed' if self.complete else 'next_bend'
                return True
        self.last_pose=pose
        return False
