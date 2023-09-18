import time 

from utils import stateToDict
from utils import handleFullQueue
from logidrivepy import LogitechController

class Controller: 
    def __init__(self) -> None:
        self.controller = LogitechController()
        self.controllerIndex = 0 
        self.done = False 

    def terminate(self) -> None: 
        self.controller.LogiSteeringShutdown()
        print("Wheel controller stopped")

    def run(self, queueLock, dataQueue) -> None: 
        self.controller.LogiSteeringInitialize(True) 
        self.controller.LogiStopSpringForce(self.controllerIndex)

        while not self.done: 
            self.controller.LogiUpdate() # update every frame 
            state = self.controller.LogiGetStateENGINES(self.controllerIndex) # get input from the wheel controller
            data = stateToDict(state) # convert DIJOYSTATE2ENGINES object to python dict object

            queueLock.acquire() 
            handleFullQueue(dataQueue, data)
            queueLock.release()

            time.sleep(0.01)