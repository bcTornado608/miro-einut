from state_machine import *
from miro_lib import *

class StateFishingGame(State):
    def __init__(self, robot:Robot) -> None:
        super().__init__()
        self.robot = robot

        self.process = 50
        self.last_update = 0
        self.delta = 0
        self.touch_count = 0
        self.timer = Timer()

    def on_enter(self):
        self.timer.reset()
        self.timer.start()

    def on_run(self):
        if self.touch_count >= 3:
            self.switch_to('dance')

        if self.process > 0 and self.process < 100:
            self.timer.update()
            self.delta = self.timer.get_time - self.last_update
            self.last_update = self.timer.get_time

            if self.delta > 0.1:
                self.process -= 1
            if self.delta > 2:
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

    def on_enter(self):
        pass

    def on_run(self):
        pass

    def on_exit(self):
        pass

class StateChase(State):
    def __init__(self, robot:Robot) -> None:
        super().__init__()
        self.robot = robot

    def on_enter(self):
        pass

    def on_run(self):
        pass

    def on_exit(self):
        pass

class StateRunaway(State):
    def __init__(self, robot:Robot) -> None:
        super().__init__()
        self.robot = robot

    def on_enter(self):
        pass

    def on_run(self):
        pass

    def on_exit(self):
        pass

class StateMoveaway(State):
    def __init__(self, robot:Robot) -> None:
        super().__init__()
        self.robot = robot

    def on_enter(self):
        pass

    def on_run(self):
        pass

    def on_exit(self):
        pass