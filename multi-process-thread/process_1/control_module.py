import sys 
import time 

sys.path.append('process_1/') 
from control_thread_manager import ControlThreadManager  

c = ControlThreadManager() 

class ControlModule: 
    def __init__(self) -> None:
        self.done = False 

    def terminate(self) -> None: 
        self.done = True 

    def run(self) -> None: 
        # use try ... finally .. if want to maintain the status 
        try: 
            c.run() 
            while not self.done: 
                time.sleep(100) 
        finally: 
            print('terminating')
            c.terminate() 