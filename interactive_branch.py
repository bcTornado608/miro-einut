from operator import truediv
from state_machine import *
from miro_lib import *
import random

class StateFishingGame(State):
    def __init__(self, robot:Robot) -> None:
        super().__init__()
        self.robot = robot

        self.process = 50
        self.last_update1 = 0
        self.last_update2 = 0
        self.touch_count = 0
        self.timer = Timer()

    def on_enter(self):
        self.timer.reset()
        self.timer.start()
        self.last_update1 = 0
        self.last_update2 = 0
        self.touch_count = 0
        self.process = 50


    def on_run(self):
        if self.touch_count >= 3:
            if random.random() > 0.5:
                self.switch_to('dance')

        if self.process > 0 and self.process < 100:
            self.timer.update()
            self.last_update1 = self.timer.get_time

            if self.timer.get_time - self.last_update1 > 0.1:
                self.process -= 1
                self.last_update1 = self.timer.get_time

            if self.timer.get_time - self.last_update2 > 2:
                self.last_update2 = self.timer.get_time
                if self.robot.get_body_touches() != False or self.robot.get_head_touches() != False:
                    self.process += 40
                    self.touch_count += 1

        elif self.process <= 0:
            self.switch_to('move_away')
        else:
            self.switch_to('run_away')

            

            
    def on_exit(self):
        pass

class StateDance(State):
    def __init__(self, robot:Robot) -> None:
        super().__init__()
        self.robot = robot

        self.last_update = 0
        self.timer = Timer()

    def on_enter(self):
        self.timer.reset()
        self.timer.start()
        self.last_update = 0

    def on_run(self):
        self.timer.update()

        if self.timer.get_time - self.last_update > 0.1:
            self.robot.energy -= 1
            self.last_update = self.timer.get_time
        
        # dance
        if self.timer.get_time > 0.6:
            self.robot.drive(0, 30)
        if self.timer.get_time > 1.6:
            self.robot.drive(0,0)
            self.robot.set_neck(yaw = 30)
        if self.timer.get_time > 2.6:
            self.robot.set_neck(yaw = -30)
        if self.timer.get_time > 3.6:
            self.robot.set_neck(yaw = 0)
            self.robot.drive(0, -30)
        if self.timer.get_time > 5.6:
            self.robot.set_neck(yaw = -30)
        if self.timer.get_time > 6.6:
            self.robot.set_neck(yaw = 30)
        if self.timer.get_time > 7.6:
            self.robot.drive(0, 30)
        if self.timer.get_time > 19.6:
            self.robot.drive(0, 0)

            done = True

        if done == True:
            if self.robot.energy <= 0:
                self.switch_to('sleep')
            else:
                self.switch_to('wake')


    def on_exit(self):
        pass


class StateRunaway(State):
    def __init__(self, robot:Robot) -> None:
        super().__init__()
        self.robot = robot
        self.last_update = 0
        self.timer = Timer()

    def on_enter(self):
        self.timer.reset()
        self.timer.start()
        self.last_update = 0
        self.robot.drive(-0.5)

    def on_run(self):
        self.timer.update()
        if self.timer.get_time - self.last_update >= 1:
            self.robot.drive(0)
            self.switch_to('wake')

    def on_exit(self):
        pass

class StateMoveaway(State):
    def __init__(self, robot:Robot) -> None:
        super().__init__()
        self.robot = robot
        self.last_update = 0
        self.timer = Timer()

    def on_enter(self):
        self.timer.reset()
        self.timer.start()
        self.last_update = 0
        self.robot.drive(turn= 90)


    def on_run(self):
        if self.timer.get_time - self.last_update > 1:
            self.robot.drive(turn = 0)
            self.robot.drive(0.4)
        if self.timer.get_time - self.last_update > 2:
            self.robot.drive(0)
            self.switch_to('wake')
            

    def on_exit(self):
        pass