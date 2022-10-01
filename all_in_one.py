



import time
import random



import time
from typing import List, Tuple

try:
    import miro2 as miro
except:
    pass

try:
    from MirocodeInterface import *
except:
    pass


__all__ = ['Robot']

class Robot:
    def __init__(self, fake=False) -> None:
        self.fake = fake

        self.energy = 0.0

        # connect to robot
        if not fake:
            self.robot = MirocodeInterface(pose_ctrl=False, cliff_reflex=False)


    # chassis
    def drive(self, forward: float = None, turn: float = None):
        '''set forward and turn speed in m/s and degrees/s'''
        print('driving: {}, {}'.format(forward, turn))

        if not self.fake:
            if forward is not None:
                self.robot.speed(forward)
            if turn is not None:
                self.robot.turn_speed(turn)
    
    # body
    def set_neck(self, lift: float = None, pitch: float = None, yaw: float = None):
        '''set neck position, pass none to keep current
        lift = head up/down, pitch = look up/down, yaw = rotate left/right'''
        print('set neck: {}, {}, {}'.format(lift, pitch, yaw))

        if not self.fake:
            if lift is not None:
                self.robot.neck_angle(miro.constants.JOINT_LIFT, lift)
            if pitch is not None:
                self.robot.neck_angle(miro.constants.JOINT_PITCH, pitch)
            if yaw is not None:
                self.robot.neck_angle(miro.constants.JOINT_YAW, yaw)

    def set_ears(self, left: float = None, right: float = None):
        '''rotate ears to the direction.
        1.0 is front, 0.0 is outward'''
        print('ears: {}, {}'.format(left, right))

        if not self.fake:
            if left is not None:
                self.robot.joint_position(miro.constants.JOINT_EAR_L, left)
            if right is not None:
                self.robot.joint_position(miro.constants.JOINT_EAR_R, right)


    def set_eyelids(self, left: float = None, right: float = None):
        '''set eyelids positions, 0 for open, 1.0 for close'''
        print('eyes: {}, {}'.format(left, right))

        if not self.fake:
            if left is not None:
                self.robot.joint_position(miro.constants.JOINT_EYE_L, left)
            if right is not None:
                self.robot.joint_position(miro.constants.JOINT_EYE_R, right)

    
    def set_tail(self, vertical: float = None, horizontal: float = None):
        '''point tail. vertical: 1 is up, 0 is down, horizontal: 1 is right, 0 is left'''
        print('eyes: {}, {}'.format(vertical, horizontal))

        if not self.fake:
            if horizontal is not None:
                self.robot.joint_position(miro.constants.JOINT_WAG, horizontal)
            if vertical is not None:
                self.robot.joint_position(miro.constants.JOINT_DROOP, vertical)

    def set_tail_frequency(self, freq: float):
        if not self.fake:
            self.robot.wag_frequency(freq)
    
    def get_sonar(self) -> float:
        '''get sonar distance from 0.03 to 1, in meters.'''
        if not self.fake:
            return self.robot.read_sonar_range()
        else:
            return 1.0

    def get_cliff(self) -> Tuple[bool, bool]:
        '''get left and right cliff sensor. True means there is a cliff'''
        
        if not self.fake:
            left = self.robot.read_cliff_sensor(miro.constants.CLIFF_L)
            right = self.robot.read_cliff_sensor(miro.constants.CLIFF_R)
            return (left, right)
        else:
            return (False, False)

    def get_light(self) -> float:
        if not self.fake:
            return self.robot.read_light_sensor(miro.constants.LIGHT_LF)
        return 0.5
    
    def get_last_clap(self) -> float:
        '''get seconds since last detected clap'''
        if not self.fake:
            ret = self.robot.time_since_clap()
            if ret is None:
                return 9999
            else:
                return ret
        return 999
    
    def get_body_touches(self) -> List[bool]:
        '''true if being touched'''
        if not self.fake:
            return self.robot.read_body_touch_sensor_list()
        return [False for i in range(14)]
    
    def get_head_touches(self) -> List[bool]:
        '''true if being touched'''
        if not self.fake:
            return self.robot.read_head_touch_sensor_list()
        return [False for i in range(14)]
        

    def get_last_shake(self):
        if not self.fake:
            return self.robot.time_since_shake(miro.constants.IMU_B)
        return 999
    
    def start_sound(self, pitch, volume, length):
        '''pitch: hz, volume: 0-100, length: seconds'''
        if not self.fake:
            self.robot.play_tone(pitch, length, volume)

    def set_leds(self, idx, rgb):
        pass






from typing import Callable, Union, List
from abc import ABC, abstractmethod
import time




class Timer:
    def __init__(self) -> None:
        self.last_update = 0
        self.accumulate_time = 0
        self.running = False
        
    def reset(self):
        self.accumulate_time = 0

    def start(self):
        self.running = True
        self.last_update = time.time()

    def stop(self):
        self.running = False

    def update(self):
        if self.running:
            current_time = time.time()
            delta = current_time - self.last_update
            self.accumulate_time += delta
            self.last_update = current_time
    
    def get_time(self) -> float:
        return self.accumulate_time



class Sequential:
    def __init__(self, functions: List[Callable]) -> None:
        '''functions is a list of functions that are called repeatedly and return true once finished'''
        assert len(functions) > 0
        self.functions = functions
        self.index = 0

    def run(self, *args, **kwargs) -> bool:
        '''return true once finished'''
        finished = self.functions[self.index](*args, **kwargs)
        if finished:
            self.index += 1
        
        if self.index >= len(self.functions):
            return True
        else:
            return False
    
    def __call__(self, *args, **kwargs) -> bool:
        return self.run(*args, **kwargs)



class State(ABC):
    def __init__(self) -> None:
        self._machine: 'StateMachine' = None
    
    def switch_to(self, state_name: Union[str, None]):
        assert self._machine is not None
        self._machine.set_state(state_name)
    
    def get_current_state(self):
        assert self._machine is not None
        return self._machine.current_state

    @abstractmethod
    def on_enter(self):
        pass

    @abstractmethod
    def on_run(self):
        pass

    @abstractmethod
    def on_exit(self):
        pass


class StateMachine:
    def __init__(self, states: dict = None, bg_state: State = None) -> None:
        self.states = states
        if self.states is None:
            self.states: dict[str, State] = {}
        self.current_state: str = None
        self.bg_state: State = None
    
    def __del__(self):
        if self.current_state is not None:
            self.states[self.current_state].on_exit()
        if self.bg_state is not None:
            self.bg_state.on_exit()

    def add_state(self, name: str, state: State):
        self.states[name] = state
        state._machine = self
    
    def set_bg_state(self, state: State):
        self.bg_state = state
        if state is not None:
            state._machine = self
        self.bg_state.on_enter()
    
    def set_state(self, name: Union[str, None]):
        # call exit
        if self.current_state is not None:
            self.states[self.current_state].on_exit()
        
        if name in self.states:
            self.current_state = name
            # call enter
            self.states[self.current_state].on_enter()
        else:
            self.current_state = None
        
    def run(self):
        if self.current_state is not None:
            self.states[self.current_state].on_run()
        if self.bg_state is not None:
            self.bg_state.on_run()














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
        self.robot.set_neck(90, 90, 90)
        self.robot.set_eyelids(1.0, 1.0)

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
        self.robot.set_eyelids(0, 0)

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
        self.robot.set_eyelids(0, 0)

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
        self.timer = Timer()
    def on_enter(self):
        self.last_call = time.time()
        self.timer.reset()
        self.timer.start()

        self.robot.set_eyelids(0.5, 0.5)

    def on_exit(self):
        pass
    def on_run(self):
        self.timer.update()
        current_time = time.time()
        delta_time = current_time - self.last_call
        self.last_call = current_time

        self.robot.energy += delta_time*0.5

        is_touched = any(self.robot.get_body_touches()) or any(self.robot.get_head_touches())
        if is_touched:
            self.switch_to('interactive')

        if self.timer.get_time() >= 5:
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
        self.robot.set_eyelids(0, 0)
    def on_exit(self):
        self.robot.drive(0, 0)
    def on_run(self):
        self.timer.update()
        self.wondering_cd.update()

        current_time = time.time()
        delta_time = current_time - self.last_call
        self.last_call = current_time

        if self.timer.get_time() >= 10 and self.robot.energy < 10:
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

        if self.wondering_cd.get_time() > 1:
            self.wondering_cd.reset()

            wonder = random.randint(0,1)
            # wonder = 0
            if wonder == 0:
                self.switch_to('wondering')
                return
        
        if any(self.robot.get_cliff()):
            self.robot.drive(-0.5, 0)



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
        self.robot.drive(0, 0)

    def on_run(self):
        self.wondering_timer.update()
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
    robot = Robot(fake=False)
    robot.energy = 10
    machine = StateMachine()

    machine.add_state('sleep', StateSleep(robot))
    machine.add_state('wake', StateWake(robot))
    machine.add_state('walk_away', StateWalkAway(robot))
    machine.add_state('scared', StateScared(robot))
    machine.add_state('lie_down', StateLieDown(robot))
    machine.add_state('wondering', StateWondering(robot))
    machine.add_state('interactive', StateInteractive())
    machine.add_state('curious', StateCurious(robot))

    # set start state
    machine.set_state('sleep')

    while True:
        print(machine.current_state)
        machine.run()
        time.sleep(0.2)


