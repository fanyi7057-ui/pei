"""Course-order state machine; isolated tests do not use it."""
from dataclasses import dataclass
from enum import Enum, auto


class State(Enum):
    WAIT_START=auto(); FOLLOW_TO_STEP=auto(); STEP_ALIGN=auto(); STEP_APPROACH=auto()
    STEP_JUMP=auto(); STEP_LAND=auto(); REACQUIRE_LINE=auto()
    FOLLOW_TO_ROUNDABOUT=auto(); ROUNDABOUT_ENTER=auto(); ROUNDABOUT_CCW=auto()
    ROUNDABOUT_EXIT=auto(); FOLLOW_TO_CROSSWALK=auto(); CROSSWALK_APPROACH=auto()
    CROSSWALK_STOP=auto(); FOLLOW_TO_START=auto(); START_LINE_PASS=auto()
    FINISHED=auto(); FAULT=auto()


@dataclass
class Events:
    start:bool=False; stop:bool=False; fault:bool=False; step:bool=False
    aligned:bool=False; takeoff:bool=False; jump_sent:bool=False; landed:bool=False
    line:bool=False; roundabout:bool=False; entered:bool=False; round_done:bool=False
    exited:bool=False; crosswalk:bool=False; stop_point:bool=False
    wait_done:bool=False; start_line:bool=False; line_passed:bool=False


class Machine:
    def __init__(self,total_laps=2):
        self.state=State.WAIT_START; self.lap=0; self.total_laps=total_laps
    def update(self,e):
        if e.fault or e.stop: self.state=State.FAULT; return self.state
        table={
            State.WAIT_START:(e.start,State.FOLLOW_TO_STEP),
            State.FOLLOW_TO_STEP:(e.step,State.STEP_ALIGN),
            State.STEP_ALIGN:(e.aligned,State.STEP_APPROACH),
            State.STEP_APPROACH:(e.takeoff,State.STEP_JUMP),
            State.STEP_JUMP:(e.jump_sent,State.STEP_LAND),
            State.STEP_LAND:(e.landed,State.REACQUIRE_LINE),
            State.REACQUIRE_LINE:(e.line,State.FOLLOW_TO_ROUNDABOUT),
            State.FOLLOW_TO_ROUNDABOUT:(e.roundabout,State.ROUNDABOUT_ENTER),
            State.ROUNDABOUT_ENTER:(e.entered,State.ROUNDABOUT_CCW),
            State.ROUNDABOUT_CCW:(e.round_done,State.ROUNDABOUT_EXIT),
            State.ROUNDABOUT_EXIT:(e.exited,State.FOLLOW_TO_CROSSWALK),
            State.FOLLOW_TO_CROSSWALK:(e.crosswalk,State.CROSSWALK_APPROACH),
            State.CROSSWALK_APPROACH:(e.stop_point,State.CROSSWALK_STOP),
            State.CROSSWALK_STOP:(e.wait_done,State.FOLLOW_TO_START),
            State.FOLLOW_TO_START:(e.start_line,State.START_LINE_PASS),
        }
        if self.state in table and table[self.state][0]: self.state=table[self.state][1]
        elif self.state==State.START_LINE_PASS and e.line_passed:
            self.lap+=1
            self.state=State.FINISHED if self.lap>=self.total_laps else State.FOLLOW_TO_STEP
        return self.state


def main():
    machine=Machine(); print(f"state={machine.state.name}, lap={machine.lap}")
