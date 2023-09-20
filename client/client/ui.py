import os 
import time 

from cosntants import DISPLAY_THRESHOLD 

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

                    print(f"Linear Speed: {round(self.data['linearSpeed'], 2)}m/s") 
                    print(f"Remaining battery Capacity: {round(self.data['batteryCapacity'], 2)}%")
                    print(f"Motor Throttle: {round(self.data['motorThrottle'], 2)}% PWM") 
                    print(f"Steering: {round(self.data['steering'], 2)} rad") 

                    self.lastPlayTime = currentTime 

