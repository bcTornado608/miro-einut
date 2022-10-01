from typing import Callable, Union, List
from abc import ABC, abstractmethod


__all__ = ['Sequential', 'State', 'StateMachine']


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

