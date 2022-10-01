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

        self.robot.energy += delta_time*0.5

        is_touched = any(self.robot.get_body_touches()) or any(self.robot.get_head_touches())
        if is_touched:
            self.switch_to('interactive')

        if delta_time >= 5:
            self.switch_to('sleep')
        



class StateWake(State):
    def __init__(self, robot: Robot) -> None:
        super().__init__()
        self.robot = robot
        self.last_call = 0
        self.timer = Timer()
        self.wondering_cd = Timer()

    def on_enter(self):
        self.last_call = time.time()
        self.timer.start()
        self.timer.reset()
        self.wondering_cd.reset()
        self.wondering_cd.start()
    def on_exit(self):
        pass
    def on_run(self):
        self.timer.update()
        current_time = time.time()
        delta_time = current_time - self.last_call
        self.last_call = current_time

        if self.timer.get_time() >= 10:
            self.switch_to('lie_down')
            return
        
        is_touched = any(self.robot.get_body_touches()) or any(self.robot.get_head_touches())
        if is_touched:
            self.switch_to('interactive')
            return
        
        is_sounded = self.robot.get_last_clap() < 0.5
        if (is_sounded) and random.randint(0,1):
            self.switch_to('curious')
            return

        if self.wondering_cd.get_time() > 3:
            self.wondering_cd.reset()

            wonder = random.randint(0,1)
            if wonder == 0:
                self.switch_to('wondering')
                return
        



class StateWondering(State):
    def __init__(self, robot: Robot) -> None:
        super().__init__()
        self.robot = robot
        self.last_call = 0

        self.wondering_timer = Timer()
        self.wondering_length = 0
        self.wondering_angle = 0

    def on_enter(self):
        self.last_call = time.time()
        self.wondering_length = random.randrange(3, 5)
        self.wondering_angle = random.random() * 90 - 45
        self.wondering_timer.reset()
        self.wondering_timer.start()

        # head down, turn right, lie on ground
        self.robot.set_neck(45, 45, self.wondering_angle)
        self.robot.drive(0.2, self.wondering_angle)

    def on_exit(self):
        pass

    def on_run(self):
        current_time = time.time()
        delta_time = current_time - self.last_call
        self.last_call = current_time

        self.robot.energy -= delta_time
        
        if self.robot.energy <= 5:
            self.switch_to('lie_down')
            return

        is_touched = any(self.robot.get_body_touches()) or any(self.robot.get_head_touches())
        if is_touched:
            self.switch_to('wake')
            return

        if self.wondering_timer.get_time() >= self.wondering_length:
            self.robot.drive(0, 0)
            self.switch_to('wake')


class StateCurious(State):
    def __init__(self, robot: Robot) -> None:
        super().__init__()
        self.robot = robot
        self.last_call = 0

    def on_enter(self):
        self.robot.set_neck(45, 45, 0)
        self.robot.drive(0.5, 45)
    def on_exit(self):
        pass
    def on_run(self):
        current_time = time.time()
        delta_time = current_time - self.last_call
        self.last_call = current_time

        self.robot.energy -= delta_time
        if self.robot.energy <= 0:
            self.switch_of('sleep')
            return
        
        if self.robot.energy <= 5:
            self.switch_to('lie_down')
            return
        
        self.switch_to('wake')



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
    machine.add_state('wake', StateWake(robot))
    machine.add_state('walk_away', StateWalkAway(robot))
    machine.add_state('scared', StateScared(robot))
    machine.add_state('lie_down', StateLieDown())
    machine.add_state('wondering', StateWondering(robot))
    machine.add_state('interactive', StateInteractive())
    machine.add_state('curious', StateCurious())

    # set start state
    machine.set_state('sleep')

    while True:
        print(machine.current_state)
        machine.run()
        time.sleep(0.2)


