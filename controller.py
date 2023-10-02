import time 
from logidrivepy import LogitechController

from utils import stateToDict
from utils import handleFullQueue 
from constants import INITIAL_DATA

class Controller: 
    def __init__(self) -> None:
        self.controller = LogitechController()
        self.controllerIndex = 0 
        self.done = False 
        self.state = None 

    def terminate(self) -> None: 
        self.done = True 
        self.controller.LogiSteeringShutdown()
        print("Wheel controller stopped") 
    
    def checkController(self) -> None: 
        for i in range(2): # update 2 times to capture the wrong data
            self.controller.LogiUpdate(self.controllerIndex) 

        self.state = self.controller.LogiGetStateENGINES(self.controllerIndex) 
        data = stateToDict(self.state) 
        
        if data != INITIAL_DATA: 
            print(f"Cannot get input from the device {self.controllerIndex}") 
            self.terminate() 

    def run(self, queueLock, dataQueue) -> None: 
        self.controller.LogiSteeringInitialize(True) 
        self.checkController() 
        
        print("Accepting controller input now...")
        while self.controller.LogiIsConnected(self.controllerIndex) and not self.done: 
            if self.controller.LogiUpdate(): # update every frame 
                self.state = self.controller.LogiGetStateENGINES(self.controllerIndex) # get input from the wheel controller
                data = stateToDict(self.state) # convert DIJOYSTATE2ENGINES object to python dict object

                if data == INITIAL_DATA: continue # filt default value 

                queueLock.acquire() 
                handleFullQueue(dataQueue, data)
                queueLock.release()

                time.sleep(0.01)