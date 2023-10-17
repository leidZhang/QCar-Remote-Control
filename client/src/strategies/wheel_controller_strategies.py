import sys 
from abc import ABC, abstractmethod 

sys.path.append('src/')
from common.constants import BUTTON_UP_INDEX
from common.constants import BUTTON_DOWN_INDEX 
from common.constants import BUTTON_A_INDEX
from common.constants import BUTTON_XBOX_INDEX 

class WheelControllerStrategy(ABC): 
    @abstractmethod
    def execute(self, controller, state) -> None: 
        pass 

class WheelReverseFlagStrategy(WheelControllerStrategy): 
    def execute(self, controller, state) -> None:
        if controller.button_is_pressed(controller.index, BUTTON_UP_INDEX): 
            state['reverse'] = not state['reverse'] 

class WheelCruiseFlagStrategy(WheelControllerStrategy): 
    def execute(self, controller, state) -> None:
        if controller.button_is_pressed(controller.index, BUTTON_DOWN_INDEX): 
            state['cruise'] = not state['cruise']
            state['cruise_throttle']= state['throttle']
              
class WheelLightFlagStrategy(WheelControllerStrategy): 
    def execute(self, controller, state) -> None:
        if controller.button_is_pressed(controller.index, BUTTON_A_INDEX): 
            state['light'] = not state['light'] 

class WheelSafeFlagStrategy(WheelControllerStrategy): 
    def execute(self, controller, state) -> None:
        if controller.button_is_pressed(controller.index, BUTTON_XBOX_INDEX): 
            state['safe'] = not state['safe']