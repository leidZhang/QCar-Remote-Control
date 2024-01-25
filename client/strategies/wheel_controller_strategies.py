import sys 
import time 
from abc import ABC, abstractmethod 

sys.path.append('src/')
from common.constants import BUTTON_UP_INDEX
from common.constants import BUTTON_DOWN_INDEX 
from common.constants import BUTTON_A_INDEX
from common.constants import BUTTON_XBOX_INDEX 

class WheelControllerStrategy(ABC): 
    def __init__(self) -> None:
        self.last_press = time.time() # for cooldown 

    @abstractmethod
    def execute(self, controller, state, index) -> None: 
        pass 

class WheelReverseFlagStrategy(WheelControllerStrategy): 
    def execute(self, controller, state, index) -> None:
        current_time = time.time() 

        if controller.button_is_pressed(index, BUTTON_UP_INDEX) and current_time - self.last_press > 1: 
            state['control_flags']['reverse'] = not state['control_flags']['reverse'] 
            self.last_press = current_time

class WheelCruiseFlagStrategy(WheelControllerStrategy): 
    def execute(self, controller, state, index) -> None:
        current_time = time.time() 

        if controller.button_is_pressed(index, BUTTON_DOWN_INDEX) and current_time - self.last_press > 1: 
            state['control_flags']['cruise'] = not state['control_flags']['cruise']
            state['cruise_throttle']= state['throttle']
            self.last_press = current_time
              
class WheelLightFlagStrategy(WheelControllerStrategy): 
    def execute(self, controller, state, index) -> None:
        current_time = time.time() 

        if controller.button_is_pressed(index, BUTTON_A_INDEX) and current_time - self.last_press > 1: 
            state['control_flags']['light'] = not state['control_flags']['light'] 
            self.last_press = current_time

class WheelSafeFlagStrategy(WheelControllerStrategy): 
    def execute(self, controller, state, index) -> None:
        current_time = time.time() 

        if controller.button_is_pressed(index, BUTTON_XBOX_INDEX) and current_time - self.last_press > 1: 
            state['control_flags']['safe'] = not state['control_flags']['safe']
            self.last_press = current_time