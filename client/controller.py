import time 
from logidrivepy import LogitechController

from utils import stateToDict
from utils import handleFullQueue 
from cosntants import DEFALUT_DATA 

class Controller: 
    def __init__(self) -> None:
        self.controller = LogitechController()
        self.controllerIndex = 0 
        self.done = False 

    def terminate(self) -> None: 
        self.done = True 
        self.controller.LogiSteeringShutdown()
        print("Wheel controller stopped")

    def run(self, queueLock, dataQueue) -> None: 
        self.controller.LogiSteeringInitialize(True) 
        
        while not self.done: 
            self.controller.LogiStopSpringForce(self.controllerIndex)
            
            if self.controller.LogiUpdate(): # update every frame 
                state = self.controller.LogiGetStateENGINES(self.controllerIndex) # get input from the wheel controller
                data = stateToDict(state) # convert DIJOYSTATE2ENGINES object to python dict object

                if data == DEFALUT_DATA: continue # filt default value 

                queueLock.acquire() 
                handleFullQueue(dataQueue, data)
                queueLock.release()

                time.sleep(0.01)