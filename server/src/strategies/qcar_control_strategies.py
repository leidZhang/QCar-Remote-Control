from abc import ABC, abstractmethod 

class QCarControlStrategy(ABC): 
    @abstractmethod 
    def execute(self, control) -> None: 
        pass 

class SafeStrategy(QCarControlStrategy): 
    def execute(self, control) -> None: 
        flag = control.state['control_flags']['safe']

        if flag == True: 
            control.state['throttle'] *= 0 
        else: 
            control.state['throttle'] *= 1

class ReverseStrategy(QCarControlStrategy): 
    def execute(self, control) -> None:
        if control.state['control_flags']['reverse']: 
            control.state['throttle'] *= -1 
            control.state['cruise_throttle'] *= -1
        else: 
            control.state['throttle'] *= 1
            control.state['cruise_throttle'] *= 1 
    
class CruiseStrategy(QCarControlStrategy): 
    def execute(self, control) -> None:
        flag = control.state['control_flags']['cruise']

        if flag == True: 
            control.state['throttle'] = control.state['cruise_throttle']

class LightStrategy(QCarControlStrategy): 
    def execute(self, control) -> None: 
        flag = control.state['control_flags']['light']

        if flag == True: 
            control.LEDs[6] = 1 
            control.LEDs[7] = 1 
        else: 
            control.LEDs[6] = 0
            control.LEDs[7] = 0 
