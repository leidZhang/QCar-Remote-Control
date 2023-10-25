import os 
import sys 
import queue 
import threading 
import keyboard 

sys.path.append('src/')
from service.service_module import ServiceModule  # abstract class
from service.controller.controller import Controller 
from common.utils import handle_full_queue
from strategies.keyboard_controller_strategies import KeyboardBrakeStrategy 
from strategies.keyboard_controller_strategies import KeyboardCruiseFlagStrategy 
from strategies.keyboard_controller_strategies import KeyboardReverseFlagStrategy 
from strategies.keyboard_controller_strategies import KeyboardLightFlagStrategy 
from strategies.keyboard_controller_strategies import KeyboardSafeFlagStrategy 

class KeyboardController(ServiceModule, Controller):  
    def __init__(self, mode) -> None: 
        self.mode = mode 
        self.done = False
        self.state = {
            'throttle': 0, 
            'steering': 0, 
            'cruise_throttle': 0, 
            'control_flags': {
                'safe': True, 
                'reverse': False, 
                'light': False, 
                'cruise': False, 
            }
        }

        self.control_strategies = [ # add more strategies here if needed 
            KeyboardSafeFlagStrategy(self, '/'), 
            KeyboardReverseFlagStrategy(self, 'e'), 
            KeyboardBrakeStrategy(self, 's'), 
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
            return val - 1 
        elif val < 0: 
            return val + 1 
        else: 
            return 0 

    def normalize_steering(self, y_axis_signal) -> float:
        return y_axis_signal / 5000 
    
    def normalize_throttle(self, x_axis_signal) -> float:
        return x_axis_signal / 3000 

    def run(self, queue_lock, remote_queue, local_queue) -> None:  
        print('activating keyboard controller') 

        for strategy in self.control_strategies: 
            keyboard.on_press_key(strategy.key, strategy.execute)

        throttle, steering = 0, 0 
        while not self.done:  
            if keyboard.is_pressed('w'): 
                throttle += 2 
                if throttle > 3000: 
                    throttle = 3000  
            else: 
                throttle = self.to_zero(throttle)  

            if keyboard.is_pressed('a'): 
                steering += 1 
                if steering > 5000: 
                    steering = 5000 
            elif keyboard.is_pressed('d'): 
                steering -= 1 
                if steering < -5000: 
                    steering = -5000
            else: 
                steering = self.to_zero(steering) 

            if keyboard.is_pressed('`'): 
                self.terminate() 

            self.state['throttle'] = self.normalize_throttle(throttle)
            self.state['steering'] = self.normalize_steering(steering) 
            # print('steering', self.state['steering'])
            # print('throttle', self.state['throttle'])
            
            queue_lock.acquire()
            handle_full_queue(remote_queue, self.state)
            handle_full_queue(local_queue, self.state)
            queue_lock.release()
            
if __name__ == "__main__": 
    q1 = queue.Queue(10)
    q2 = queue.Queue(10)
    k = KeyboardController('keyboard') 
    l = threading.Lock() 
    k.run(l, q1, q2) 