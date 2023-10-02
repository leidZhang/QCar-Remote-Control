import os 
import time 

from constants import DISPLAY_THRESHOLD 

class UI: 
    def __init__(self) -> None: 
        self.data = None 
        self.done = False 
        self.lastPlayTime = 0 

    def terminate(self) -> None: 
        self.done = True 
        print("UI stopped") 

    def run(self, responseQueue) -> None: 
        while not self.done: 
            responseData = responseQueue.get() 
            if self.data != responseData: 
                self.data = responseData 
                currentTime = time.time() 
            
                if currentTime - self.lastPlayTime >= DISPLAY_THRESHOLD and self.data != None: 
                    os.system("cls") 

                    print("Linear Speed: {}m/s".format(round(self.data['linearSpeed'], 2))) 
                    print("Remaining battery Capacity: {}%".format(round(self.data['batteryCapacity'], 2)))
                    print("Motor Throttle: {}% PWM".format(round(self.data['motorThrottle'], 2))) 
                    print("Steering: {} rad".format(round(self.data['steering'], 2))) 

                    self.lastPlayTime = currentTime 

