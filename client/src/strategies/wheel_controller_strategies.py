import sys 
from abc import ABC, abstractmethod 

sys.path.append('src/')
from common.constants import BUTTON_UP_INDEX
from common.constants import BUTTON_DOWN_INDEX 
from common.constants import BUTTON_A_INDEX
from common.constants import BUTTON_XBOX_INDEX 

class WheelControllerStrategy(ABC): 
    @abstractmethod
    def execute(self, *args) -> None: 
        pass 

class WheelReverseFlagStrategy(WheelControllerStrategy): 
    def execute(self, controller, flags) -> None:
        if controller.button_is_pressed(controller.index, BUTTON_UP_INDEX): 
            flags['reverse'] = not flags['reverse'] 

class WheelCruiseFlagStrategy(WheelControllerStrategy): 
    def __init__(self) -> None:
        self.cruise_throttle = 0 

    def execute(self, controller, flags) -> None:
        if controller.button_is_pressed(controller.index, BUTTON_DOWN_INDEX): 
            flags['cruise'] = not flags['cruise']
            self.cruise_throttle = controller.throttle
              
class WheelLightFlagStrategy(WheelControllerStrategy): 
    def execute(self, controller, flags) -> None:
        if controller.button_is_pressed(controller.index, BUTTON_A_INDEX): 
            flags['light'] = not flags['light'] 

class WheelSafeFlagStrategy(WheelControllerStrategy): 
    def execute(self, controller, flags) -> None:
        if controller.button_is_pressed(controller.index, BUTTON_XBOX_INDEX): 
            flags['safe'] = not flags['safe']