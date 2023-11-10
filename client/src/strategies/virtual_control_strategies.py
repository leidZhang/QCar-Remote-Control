from abc import ABC, abstractmethod 

class VirtualControlStrategy(ABC): 
    @abstractmethod 
    def execute(self, control) -> None: 
        pass 

class VirtualSafeStrategy(VirtualControlStrategy): 
    def execute(self, control) -> None: 
        flag = control.state['control_flags']['safe']

        if flag == True: 
            control.state['throttle'] *= 0 
        else: 
            control.state['throttle'] *= 1

class VirtualReverseStrategy(VirtualControlStrategy): 
    def execute(self, control) -> None:
        if control.state['control_flags']['reverse']: 
            control.state['throttle'] *= -1 
            control.state['cruise_throttle'] *= -1
        else: 
            control.state['throttle'] *= 1
            control.state['cruise_throttle'] *= 1
    
class VirtualCruiseStrategy(VirtualControlStrategy): 
    def execute(self, control) -> None:
        flag = control.state['control_flags']['cruise']

        if flag == True: 
            control.state['throttle'] = control.state['cruise_throttle']

class VirtualLightStrategy(VirtualControlStrategy): 
    def execute(self, control) -> None: 
        flag = control.state['control_flags']['light']

        if flag == True: 
            control.LEDs[6] = 1 
            control.LEDs[7] = 1 
        else: 
            control.LEDs[6] = 0
            control.LEDs[7] = 0 