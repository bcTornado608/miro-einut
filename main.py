from miro_lib import Robot
from state_machine import *
import time
import random

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




class StateScared(State):
    def __init__(self, robot: Robot) -> None:
        super().__init__()
        self.robot = robot
        self.state = 0
        self.timer = Timer()
    
    def on_enter(self):
        self.state = 0
        self.timer.reset()
        self.timer.start()

    def on_run(self):
        self.timer.update()

        if self.state == 0:
            if self.timer.get_time() < 1.5:
                self.robot.set_neck(90, 0, 0)
            else:
                self.timer.reset()
                self.state = 1

        elif self.state == 1:
            if self.robot.energy < 10:
                self.switch_to('walk_away')
            else:
                self.switch_to('interactive')
        
    def on_exit(self):
        pass


class StateWalkAway(State):
    def __init__(self, robot: Robot) -> None:
        super().__init__()
        self.robot = robot
        self.timer = Timer()
        self.state = 0

        self.random_rotate = 0

    def on_enter(self):
        # set normal pose
        self.robot.set_neck(90, 0, 0)
        self.timer.reset()
        self.timer.start()
        self.state = 0

        self.random_rotate = random.random() * 0.5 + 0.5

    def on_run(self):
        # rotate to random direction
        self.timer.update()

        if self.state == 0:
            if self.timer.get_time() < self.random_rotate:
                self.robot.drive(0, 90)
            else:
                self.robot.drive(0, 0)
                self.state += 1
                self.timer.reset()

        elif self.state == 1:
            if self.timer.get_time() < 4:
                self.robot.drive(0.3, 0)
            else:
                self.robot.drive(0, 0)
                self.switch_to('sleep')
        
    def on_exit(self):
        pass


class StateLieDown(State):
    def on_enter(self):
        pass
    def on_exit(self):
        pass
    def on_run(self):
        pass



class StateWake(State):
    def __init__(self, robot: Robot) -> None:
        super().__init__()
        self.robot = robot
        self.last_call = 0
    def on_enter(self):
        self.last_call = time.time()
    def on_exit(self):
        pass
    def on_run(self):
        current_time = time.time()
        delta_time = current_time - self.last_call
        self.last_call = current_time

        if delta_time >= 10:
            self.switch_to('lie_down')
            return
        
        is_touched = any(self.robot.get_body_touches()) or any(self.robot.get_head_touches())
        if is_touched:
            self.switch_to('interactive')
            return
        
        is_sounded =self.robot.get_sonar()
        if (is_sounded < 1) and random.randint(0,1):
            self.switch_to('curious')
            return

        wander = random.randint(0,1):
        if wander == 0:
            self.switch_to('wandering')
            return
        



class StateWondering(State):
    def __init__(self, robot: Robot) -> None:
        super().__init__()
        self.robot = robot
        self.last_call = 0
    def on_enter(self):
        self.last_call = time.time()
        stop = random.randint(0,3)
        # head down, turn right, lie on ground
        self.robot.set_neck(45, 45, 0)
        
        while stop != 0:
            self.robot.drive(0.2, 0)
            turnhead = random.randint(0,1)
            if turnhead == 0:
                self.robot.set_neck(75, 75, 45)
            self.robot.drive(0.2, 45)

    def on_exit(self):
        pass
    def on_run(self):
        current_time = time.time()
        delta_time = current_time - self.last_call
        self.last_call = current_time

        self.robot.energy -= delta_time
        if self.robot.energy <= 0:  # sleep 20 seconds
            # awake by it self
            self.switch_to('sleep')
            return
        
        if self.robot.energy <= 5:
            self.switch_to('lie_down')
            return

        is_touched = any(self.robot.get_body_touches()) or any(self.robot.get_head_touches())
        if is_touched:
            self.switch_to('wake')
            return


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
    robot = Robot(fake=True)
    machine = StateMachine()

    machine.add_state('sleep', StateSleep(robot))
    machine.add_state('walk_away', StateWalkAway(robot))
    machine.add_state('scared', StateScared(robot))
    machine.add_state('lie_down', StateLieDown(robot))
    machine.add_state('wondering', StateWondering(robot))
    machine.add_state('interactive', StateInteractive(robot))
    machine.add_state('curious', StateCurious(robot))

    # set start state
    machine.set_state('sleep')

    while True:
        machine.run()


