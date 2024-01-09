import os   
import keyboard 

from common.service_module import ServiceModule  # abstract class
from common.common_controller import Controller 
from common.utils import handle_full_queue
from core.init_settings import INIT_STATE
from strategies import KeyboardCruiseFlagStrategy 
from strategies import KeyboardReverseFlagStrategy 
from strategies import KeyboardLightFlagStrategy 
from strategies import KeyboardSafeFlagStrategy 

class KeyboardController(ServiceModule, Controller):  
    def __init__(self, mode) -> None: 
        self.mode = mode 
        self.done = False
        self.state = INIT_STATE 

        self.control_strategies = [ # add more strategies here if needed 
            KeyboardSafeFlagStrategy(self, '/'), 
            KeyboardReverseFlagStrategy(self, 'e'), 
            KeyboardLightFlagStrategy(self, 'l'), 
            KeyboardCruiseFlagStrategy(self, 'q'), 
        ]

    def is_valid(self) -> bool:
        if self.mode != "keyboard": 
            return False
        return True 
    
    def terminate(self) -> None:
        self.done = True 
        os._exit(0)  

    def to_zero(self, val) -> int: 
        if val > 0: 
            return val - 2 
        elif val < 0: 
            return val + 2 
        else: 
            return 0 

    def normalize_steering(self, y_axis_signal) -> float:
        return y_axis_signal / 500 
    
    def normalize_throttle(self, x_axis_signal) -> float:
        return x_axis_signal / 1000 

    def run(self, queue_lock, remote_queue, local_queue) -> None:  
        print('activating keyboard controller') 

        for strategy in self.control_strategies: 
            keyboard.on_press_key(strategy.key, strategy.execute)

        throttle, steering = 0, 0 
        # last_state = copy_state(self.state) 
        while not self.done:  
            if keyboard.is_pressed('w'): 
                throttle += 2 
                if throttle > 1000: 
                    throttle = 1000  
            elif keyboard.is_pressed('s'): 
                throttle = 0 
            else: 
                throttle = self.to_zero(throttle)  

            if keyboard.is_pressed('a'): 
                steering += 2 
                if steering > 500: 
                    steering = 500 
            elif keyboard.is_pressed('d'): 
                steering -= 2 
                if steering < -500: 
                    steering = -500
            else: 
                steering = self.to_zero(steering) 

            if keyboard.is_pressed('`'): 
                self.terminate() 

            self.state['throttle'] = self.normalize_throttle(throttle)
            self.state['steering'] = self.normalize_steering(steering) 
            
            queue_lock.acquire()
            handle_full_queue(remote_queue, self.state)
            handle_full_queue(local_queue, self.state)
            queue_lock.release() 