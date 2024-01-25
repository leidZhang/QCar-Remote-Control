import os 
import sys 
import time 
import queue 
import threading 
import multiprocessing
from PyQt5.QtWidgets import QApplication, QWidget
import eel

from logidrivepy import LogitechController

sys.path.append('src/') 
from ui.init_ui import InitUI
from common.utils import handle_full_queue
from common.constants import WHEEL_CONTROLLER_STEERING_MIN
from common.constants import WHEEL_CONTROLLER_ACCELERATOR_MID
from common.constants import WHEEL_CONTROLLER_ACCELERATOR_MAX 
from service.controller.controller import Controller  
from common.service_module import ServiceModule  # abstract class 
from strategies.wheel_controller_strategies import WheelReverseFlagStrategy 
from strategies.wheel_controller_strategies import WheelLightFlagStrategy 
from strategies.wheel_controller_strategies import WheelSafeFlagStrategy 
from strategies.wheel_controller_strategies import WheelCruiseFlagStrategy 

class WheelController(ServiceModule, Controller): 
    def __init__(self, mode, index) -> None:
        self.mode = mode 
        self.index = index 
        self.controller = LogitechController() 
        self.done = False 

        self.state = {
            'throttle': 0, 
            'steering': 0,  
            'cruise_throttle': 0,
            'control_flags': { # add more flags if needed
                'safe': True,
                'reverse': False, 
                'light': False, 
                'cruise': False, 
            }
        }

        self.control_strategies = [ # add more strategies if needed
            WheelSafeFlagStrategy(), 
            WheelReverseFlagStrategy(), 
            WheelLightFlagStrategy(), 
            WheelCruiseFlagStrategy(),
        ]  

    def terminate(self) -> None:
        self.done = True 
        self.controller.steering_shutdown()
        print("Wheel controller module stopped")  

    def check_device(self) -> None: 
        try: 
            if not self.controller.is_connected(int(self.index)):
                self.terminate()
            
            print(f"Device connection: {self.controller.is_connected(int(self.index))}")
        except: 
            print('Device not connected!')
            self.terminate() 
            os._exit(0)

    def normalize_steering(self, y_axis_signal) -> float:
        return y_axis_signal / WHEEL_CONTROLLER_STEERING_MIN 
    
    def normalize_throttle(self, x_axis_signal) -> float:
        return (WHEEL_CONTROLLER_ACCELERATOR_MID - x_axis_signal) / WHEEL_CONTROLLER_ACCELERATOR_MAX 
    
    def is_valid(self) -> bool:
        if self.mode != "wheel" or self.index is None: 
            print('Wheel Controller will not activate')
            return False 
        return True 

    def run(self, queue_lock, remote_queue, local_queue) -> None:
        print('activating wheel controller...')
        self.controller.steering_initialize(False) 
        # self.check_device() # make sure the correct controller is listened 
        print(f"Device connection: {self.controller.is_connected(int(self.index))}")

        while not self.done: 
            if self.controller.logi_update(): # update every frame 
                state_engines = self.controller.get_state_engines(int(self.index)) # get input from the wheel controller
                steering = self.normalize_steering(state_engines.contents.lX)                
                throttle = self.normalize_throttle(state_engines.contents.lY) 

                self.state['throttle'] = throttle 
                self.state['steering'] = steering 

                for strategy in self.control_strategies: 
                    strategy.execute(self.controller, self.state, int(self.index)) 

                queue_lock.acquire()
                handle_full_queue(remote_queue, self.state)
                handle_full_queue(local_queue, self.state)
                queue_lock.release()

                os.system("cls")
                print(self.state) 

                time.sleep(0.01)

            # print(remote_queue.qsize())

if __name__ == "__main__": 
    try: 
        ui = InitUI() 
        p = multiprocessing.Process(target=ui.initialize)
        p.start() 

        q1 = queue.Queue(10)
        q2 = queue.Queue(10)
        k = WheelController("wheel", "0") 
        l = threading.Lock() 
        t = threading.Thread(target=k.run, args=(l, q1, q2))
        t.start() 

        # Entering the main event loop

        # k.run(l, q1, q2)
        while True: 
            time.sleep(100) 
    except KeyboardInterrupt: 
        ui.api.quit()
        print("ui terminated")
        k.terminate() 
        print("terminated!")
        

                
    