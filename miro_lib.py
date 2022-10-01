

class Robot:
    def __init__(self) -> None:
        pass

    # chassis
    def drive(self, forward: float = None, turn: float = None):
        '''set forward and turn speed in m/s and degrees/s'''
        print('driving: {}, {}'.format(forward, turn))
    
    def get_speed(self) -> tuple[float, float]:
        '''get speed (forward, turn) in m/s and degrees/s'''
        return (0, 0)

    # body
    def set_neck(self, lift: float = None, pitch: float = None, yaw: float = None):
        '''set neck position, pass none to keep current
        lift = up/down, pitch = left/right, yaw = axial rotate'''
        print('set neck: {}, {}, {}'.format(lift, pitch, yaw))

    def get_neck(self) -> tuple[float, float, float]:
        '''get (lift, pitch, yaw) angles in degrees'''
        return (0, 0, 0)

    def set_ears(self, left: float = None, right: float = None):
        '''rotate ears to the direction'''
        print('ears: {}, {}'.format(left, right))

    def set_eyelids(self, left: float = None, right: float = None):
        '''set eyelids positions, 0 for close, 1.0 for full open'''
        print('eyes: {}, {}'.format(left, right))
    
    def set_tail(self, vertical: float = None, horizontal: float = None):
        '''point tail. vertical: 1 is up, 0 is down, horizontal: 1 is right, 0 is left'''
        print('eyes: {}, {}'.format(vertical, horizontal))
    
    def get_sonar(self) -> float:
        '''get sonar distance from 0.03 to 1, in meters.'''
        return 1.0

    def get_cliff(self) -> tuple[bool, bool]:
        '''get left and right cliff sensor. True means there is a cliff'''
        return (False, False)
    
    def get_last_clap(self) -> float:
        '''get seconds since last detected clap'''
        return -1
    
    def get_touch(self) -> tuple[bool, bool]:
        '''get (head, body), true if being touched'''
        return (False, False)
    
    def start_sound(self, pitch, volume):
        pass
    def stop_sound(self):
        pass

    def set_leds(self, idx, rgb):
        pass
