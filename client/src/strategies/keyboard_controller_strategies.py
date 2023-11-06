from abc import ABC, abstractmethod 

class KeyboardControllerStrategy(ABC): 
    def __init__(self, controller, key) -> None:
        self.controller = controller 
        self.key = key 

    @abstractmethod 
    def execute(self, keyboard_event) -> None: 
        pass 

class KeyboardReverseFlagStrategy(KeyboardControllerStrategy): 
    def execute(self, keyboard_event) -> None:
        if keyboard_event.event_type == 'down': 
            self.controller.state['control_flags']['reverse'] = not self.controller.state['control_flags']['reverse'] 

class KeyboardCruiseFlagStrategy(KeyboardControllerStrategy): 
    def execute(self, keyboard_event) -> None:
        if keyboard_event.event_type == 'down': 
            self.controller.state['control_flags']['cruise'] = not self.controller.state['control_flags']['cruise'] 
            self.controller.state['cruise_throttle'] = self.controller.state['throttle']

class KeyboardLightFlagStrategy(KeyboardControllerStrategy): 
    def execute(self, keyboard_event) -> None:
        if keyboard_event.event_type == 'down': 
            self.controller.state['control_flags']['light'] = not self.controller.state['control_flags']['light']

class KeyboardSafeFlagStrategy(KeyboardControllerStrategy):  
    def execute(self, keyboard_event) -> None:
        if keyboard_event.event_type == 'down': 
            self.controller.state['control_flags']['safe'] = not self.controller.state['control_flags']['safe'] 