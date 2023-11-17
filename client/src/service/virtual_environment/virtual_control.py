import os 
import sys
import time
import numpy as np  
import queue  
# quanser packages 
sys.path.append('dependencies/q_libs') # start from project root 
from lib_qcar import QCar
from lib_utilities import GPS, Calculus 
# custom scripts 
sys.path.append('src/') 
from common.service_module import ServiceModule 
from strategies.virtual_control_strategies import VirtualReverseStrategy
from strategies.virtual_control_strategies import VirtualSafeStrategy 
from strategies.virtual_control_strategies import VirtualCruiseStrategy 
from strategies.virtual_control_strategies import VirtualLightStrategy 

class VirtualControl(ServiceModule): 
    def __init__(self, mode) -> None:
        self.mode = mode  
        self.rate = 50 # placeholder rate 
        self.done = False 
        self.start_time = time.time() 

        self.LEDs = np.array([0, 0, 0, 0, 0, 0, 0, 0])
        self.my_car = None # QCarTask(frequency=int(self.rate), hardware=0)
        self.state = None 

        self.virtual_qcar_strategies = [
            VirtualCruiseStrategy(), 
            VirtualReverseStrategy(), 
            VirtualSafeStrategy(), 
            VirtualLightStrategy(),
        ]

    def elapsed_time(self) -> None:
        return time.time() - self.start_time

    def handle_LEDs(self) -> None: 
        if self.state['steering'] > 0.3: 
            self.LEDs[0] = 1
            self.LEDs[2] = 1
        elif self.state['steering'] < -0.3:
            self.LEDs[1] = 1
            self.LEDs[3] = 1
        else: 
            self.LEDs = np.array([0, 0, 0, 0, 0, 0, self.LEDs[6], self.LEDs[7]])

    def terminate(self) -> None:
        self.done = True 
        print("Virtual Control terminated") 

    def is_valid(self) -> bool:
        return self.mode != "remote"
    
    def run(self, local_queue) -> None:
        print("activating virtual environment control...") 
        
        try:  
            self.my_car = QCar(hardware=0) 

            # diff = Calculus().differentiator_variable(1 / self.rate) 
            # _ = next(diff)

            while not self.done: 
                if not local_queue.empty(): 
                    self.state = local_queue.get() # get controller state 
                    
                    # execute strategies 
                    for strategy in self.virtual_qcar_strategies: 
                        strategy.execute(self) 
                    # handle control
                    throttle = 0.3 * 0.15 * self.state['throttle'] # config here 
                    steering = 0.5 * self.state['steering']
                    # handle LEDs 
                    self.handle_LEDs() 
                    
                    # apply state and get readings 
                    self.my_car.read_write_std(np.array([throttle, steering]), self.LEDs)
                    # encoder_speed = diff.send((encoder_counts, time_step))
                    # speed = self.my_car.estimate_speed(encoder_speed, steering)    
 
        except Exception as e: 
            print(e) 
        finally: 
            self.my_car.terminate() 
            os._exit(0) 

# if __name__ == "__main__": 
#     q = queue.Queue(10)
#     state = {
#             'throttle': 0.1, 
#             'steering': 0, 
#             'cruise_throttle': 0, 
#             'control_flags': {
#                 'safe': False, 
#                 'reverse': False, 
#                 'light': False, 
#                 'cruise': False, 
#             }
#         }
#     q.put(state)  
#     v = VirtualControl("10") 
#     v.run(q) 