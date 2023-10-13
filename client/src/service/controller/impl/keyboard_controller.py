import os 
import sys 
import queue 
import threading 
import keyboard 

sys.path.append('src/')
from service.service_module import ServiceModule  # abstract class
from service.controller.controller import Controller 
from common.utils import handle_full_queue

class KeyboardController(ServiceModule, Controller): # this is a prototype class, may violate SOLID principle
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
         
        self.reverse_flag = False 

    def is_valid(self) -> bool:
        if self.mode != "keyboard": 
            return False
        return True 
    
    def terminate(self) -> None:
        self.done = True 

    def brake(self, keyborad_event) -> None: 
        if keyborad_event.event_type == 'down': 
            self.state['throttle'] = 0 

    def reverse(self, keyborad_event) -> None: 
        if keyborad_event.event_type == 'down':
            self.state['control_flags']['reverse'] = not self.state['control_flags']['reverse']
    
    def cruise(self, keyborad_event) -> None: 
        if keyborad_event.event_type == 'down':
            self.state['control_flags']['cruise'] = not self.state['control_flags']['cruise'] 
            self.state['cruise_throttle'] = self.state['throttle'] 

    def light_toggle(self, keyborad_event) -> None: 
        if keyborad_event.event_type == 'down':
             self.state['control_flags']['light'] = not self.state['control_flags']['light']
        
    def safe_toggle(self, keyborad_event) -> None: 
        if keyborad_event.event_type == 'down':
            self.state['control_flags']['safe'] = not self.state['control_flags']['safe'] 

    def normalize_steering(self, y_axis_signal) -> float:
        return y_axis_signal / 6.25 
    
    def normalize_throttle(self, x_axis_signal) -> float:
        return x_axis_signal / 6.25 

    def run(self, queue_lock, remote_queue, local_queue) -> None: 
        keyboard.on_press_key('s', self.brake)  
        keyboard.on_press_key('e', self.reverse) 
        keyboard.on_press_key('q', self.cruise) 
        keyboard.on_press_key('l', self.light_toggle)
        keyboard.on_press_key('/', self.safe_toggle) 

        while not self.done: 
            if keyboard.is_pressed('w'): 
                self.state['throttle'] += 5 
                if self.state['throttle'] > 1000: 
                    self.state['throttle'] = 1000   
                if self.state['throttle'] < -1000:  
                    self.state['throttle'] = -1000 
            elif not self.reverse_flag and self.state['throttle'] > 0: 
                self.state['throttle'] -= 5 
                if self.state['throttle'] < 0: 
                    self.state['throttle'] = 0  
            elif self.reverse_flag and self.state['throttle'] < 0: 
                self.state['throttle'] += 5 
                if self.state['throttle'] > 0: 
                    self.state['throttle'] = 0 

            if keyboard.is_pressed('a') and self.state['steering'] >= -100: 
                self.state['steering'] += 5 
                if self.state['steering'] < -100: 
                    self.state['steering'] = -100 
            elif keyboard.is_pressed('d') and self.state['steering'] <= 100: 
                self.state['steering'] -= 5 
                if self.state['steering'] > 100: 
                    self.state['steering'] = 100 

            if keyboard.is_pressed('`'): 
                break 

            self.state['throttle'] = self.normalize_throttle(self.state['throttle'])
            self.state['steering'] = self.normalize_steering(self.state['steering'])

            queue_lock.acquire()
            handle_full_queue(remote_queue, self.state)
            handle_full_queue(local_queue, self.state)
            queue_lock.release()
            
# if __name__ == "__main__": 
#     q1 = queue.Queue(10)
#     q2 = queue.Queue(10)
#     k = KeyboardController() 
#     l = threading.Lock() 
#     k.run(l, q1, q2) 