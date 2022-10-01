from state_machine import *


class S1(State):
    def __init__(self) -> None:
        super().__init__()
        self.counter = 2

    def on_enter(self):
        print('S1 entered')

    def on_exit(self):
        print('S1 exited')

    def on_run(self):
        print('S1 runs', self.counter)
        self.counter -= 1
        if self.counter <= 0:
            self.switch_to('s2')


class S2(State):
    def __init__(self) -> None:
        super().__init__()
        self.counter = 5

    def on_enter(self):
        print('S2 entered')

    def on_exit(self):
        print('S2 exited')

    def on_run(self):
        print('S2 runs', self.counter)
        self.counter -= 1
        if self.counter <= 0:
            self.switch_to('s3')


class S3(State):
    def __init__(self) -> None:
        super().__init__()
        self.counter = 3

    def on_enter(self):
        print('S3 entered')

    def on_exit(self):
        print('S3 exited')

    def on_run(self):
        print('S3 runs', self.counter)
        self.counter -= 1
        if self.counter <= 0:
            self.switch_to(None)



class BGState(State):
    def on_enter(self):
        pass
    def on_exit(self):
        pass
    def on_run(self):
        print('bg state ran')



if __name__ == '__main__':
    machine = StateMachine()

    machine.add_state('s1', S1())
    machine.add_state('s2', S2())
    machine.add_state('s3', S3())

    machine.set_state('s1')

    machine.set_bg_state(BGState())

    for i in range(20):
        machine.run()