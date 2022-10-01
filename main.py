from miro_lib import Robot
from state_machine import *
import time


class StateAlways(State):
    def on_enter(self):
        pass
    def on_exit(self):
        pass
    def on_run(self):
        pass




class StateSleep(State):
    def __init__(self, robot: Robot) -> None:
        super().__init__()
        self.robot = robot
        self.last_call = 0

    def on_enter(self):
        self.last_call = time.time()

        # head down, turn right, lie on ground
        self.robot.set_neck(0, 90, 90)

    def on_exit(self):
        pass

    def on_run(self):
        current_time = time.time()
        delta_time = current_time - self.last_call
        self.last_call = current_time

        self.robot.energy += delta_time
        if self.robot.energy > 20:  # sleep 20 seconds
            # awake by it self
            self.switch_to('wake')
            return
        
        is_touched = any(self.robot.get_body_touches()) or any(self.robot.get_head_touches())
        if is_touched:
            self.switch_to('scared')


class StateLieDown(State):
    def on_enter(self):
        pass
    def on_exit(self):
        pass
    def on_run(self):
        pass



class StateWake(State):
    def on_enter(self):
        pass
    def on_exit(self):
        pass
    def on_run(self):
        pass



class StateWondering(State):
    def on_enter(self):
        pass
    def on_exit(self):
        pass
    def on_run(self):
        pass



class StateCurious(State):
    def on_enter(self):
        pass
    def on_exit(self):
        pass
    def on_run(self):
        pass



class StateInteractive(State):
    def on_enter(self):
        pass
    def on_exit(self):
        pass
    def on_run(self):
        pass




# main function

if __name__ == '__main__':
    pass