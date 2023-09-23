import os
import time
import struct
import numpy as np 

from utils import handleFullQueue 
from utils import statusToDict
from constants import STEERING_MIN 
from constants import ACCELERATOR_MAX
from constants import BUTTON_A_INDEX 
from constants import BUTTON_UP_INDEX 
from constants import BUTTON_DOWN_INDEX 

from Quanser.product_QCar import QCar
from Quanser.q_ui import gamepadViaTarget
from Quanser.q_misc import Calculus
from Quanser.q_interpretation import basic_speed_estimation

class QcarControl:  
    def __init__(self) -> None: 
        self.myCar = QCar()
        self.startTime = time.time() 
        self.sampleRate = 50
        self.sampleTime = 1 / self.sampleRate 
        self.counter = 0

        self.motorCommand = np.array([0.0, 0.0]) 
        self.LEDs = np.array([0, 0, 0, 0, 0, 0, 0, 0])
        self.buttonTimes = [time.time()] * 10
         
        self.done = False 
        self.reverseFlag = False 
        self.cuirseFlag = False 
        self.cuirseThrottle = 0 
        self.lightFlag = False 

    def elapsedTime(self) -> float: 
        return time.time() - self.startTime 
    
    def terminate(self) -> None: 
        print("Stopping QCar...")
        self.done = True 
        self.myCar.terminate() 

    def buttonIsPressed(self, data, index) -> bool: 
        currentTime = time.time() 
        buttonStatus = data['buttons'][index] # get button reading 

        if buttonStatus != 0 and currentTime - self.buttonTimes[index] > 1: # cooldown
            self.buttonTimes[index] = currentTime 
            return True 

        return False 
    
    def buttonIsTriggered(self, data, index) -> bool: 
        buttonStatus = data['buttons']['index'] # get button reading 
        
        if buttonStatus != 0: 
            return True 
        return False 
    
    def handleButton(self, data) -> None: 
        if self.buttonIsPressed(data, BUTTON_UP_INDEX): 
            self.reverseFlag = not self.reverseFlag 

        if self.buttonIsPressed(data, BUTTON_DOWN_INDEX): 
            self.cuirseFlag = not self.cuirseFlag 
            self.cuirseThrottle = 0.3 * (-1 if self.reverseFlag else 1) * (32767 - int(data['y'])) / ACCELERATOR_MAX  
        if self.buttonIsPressed(data, BUTTON_A_INDEX): 
            self.lightFlag = not self.lightFlag 
 
    def handleLEDs(self) -> None: 
        # turn on/off light 
        if self.lightFlag: 
            self.LEDs[6] = 1 
            self.LEDs[7] = 1 
        else: 
            self.LEDs[6] = 0
            self.LEDs[7] = 0                        
        # Adjust LED indicators based on steering and reverse indicators based on reverse gear       
        if self.motorCommand[1] > 0.3:
            self.LEDs[0] = 1
            self.LEDs[2] = 1
        elif self.motorCommand[1] < -0.3:
            self.LEDs[1] = 1
            self.LEDs[3] = 1
        else: 
            self.LEDs = np.array([0, 0, 0, 0, 0, 0, self.LEDs[6], self.LEDs[7]])

        if self.motorCommand[0] < 0:
            self.LEDs[5] = 1

    def run(self, queueLock, dataQueue, responseQueue) -> None: 
        print("Starting Controller...")
        # Set up a differentiator to get encoderSpeed from encoderCounts
        diff = Calculus().differentiator_variable(self.sampleTime)
        _ = next(diff)
        timeStep = self.sampleTime 

        # Reset startTime before Main Loop
        self.startTime = time.time()

        while not self.done: 
            start = self.elapsedTime()

            queueLock.acquire() 
            if not dataQueue.empty(): 
                new = dataQueue.get()
                if new: 
                    self.handleButton(new) 

                    steering = 0.5 * new['x'] / STEERING_MIN                     
                    if not self.cuirseFlag: 
                        motorThrottle = 0.3 * (-1 if self.reverseFlag else 1) * (32767 - int(new['y'])) / ACCELERATOR_MAX 
                    else: 
                        motorThrottle = self.cuirseThrottle 

                    self.motorCommand = np.array([motorThrottle, steering])  
                
                # Adjust LED
                self.handleLEDs()

                # Perform I/O
                current, batteryVoltage, encoderCounts = self.myCar.read_write_std(self.motorCommand, self.LEDs)        
        
                # Differentiate encoder counts and then estimate linear speed in m/s
                encoderSpeed = diff.send((encoderCounts, timeStep))
                linearSpeed = basic_speed_estimation(encoderSpeed)

                # End timing this iteration
                end = self.elapsedTime()

                # Calculate computation time, and the time that the thread should pause/sleep for
                computationTime = end - start
                sleepTime = self.sampleTime - computationTime % self.sampleTime

                # Calculate the remaining battery capacity 
                batteryCapacity = 100 - (batteryVoltage - 10.5) * 100 / (12.6 - 10.5)

                # Pause/sleep and print out the current timestamp
                # time.sleep(sleepTime)

                # responseQueue.put(str(velocity))
                responseData = statusToDict(linearSpeed, batteryCapacity, self.motorCommand[0], self.motorCommand[1]) 
                handleFullQueue(responseQueue, responseData)
                   
            queueLock.release()
            
            timeAfterSleep = self.elapsedTime()
            timeStep = timeAfterSleep - start
            self.counter += 1
