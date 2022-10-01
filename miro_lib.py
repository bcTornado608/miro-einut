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
                self.robot.joint_position(miro.constants.JOINT_EYE_L, right)

    
    def set_tail(self, vertical: float = None, horizontal: float = None):
        '''point tail. vertical: 1 is up, 0 is down, horizontal: 1 is right, 0 is left'''
        print('eyes: {}, {}'.format(vertical, horizontal))

        if not self.fake:
            if horizontal is not None:
                self.robot.joint_position(miro.constants.JOINT_WAG, 0.0)
            if vertical is not None:
                self.robot.joint_position(miro.constants.JOINT_DROOP, 0.0)

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
            return self.robot.time_since_clap()
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


# robot.find_ball([255, 0, 0], miro.constants.CAM_L, prop=robot.vision.loc_x)
