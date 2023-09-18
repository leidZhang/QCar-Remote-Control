import os
import time
import struct
import numpy as np 

from utils import handleFullQueue 
from utils import statusToDict
from constants import STEERING_FACTOR 
from constants import VELOCITY_FACTOR

from Quanser.product_QCar import QCar
from Quanser.q_ui import gamepadViaTarget
from Quanser.q_misc import Calculus
from Quanser.q_interpretation import basic_speed_estimation

class QcarControl:  
    def __init__(self) -> None: 
        self.startTime = time.time() 
        self.sampleRate = 50
        self.sampleTime = 1/self.sampleRate 
        self.myCar = QCar() 
        self.counter = 0 
        self.motorCommand = np.array([0,0]) 

    def elapsedTime(self):
        return time.time() - self.startTime 
    
    def terminate(self): 
        self.myCar.terminate() 
        print("QCar stopped")

    def run(self, queueLock, dataQueue, responseQueue) -> None: 
        print("Controller starting...")

        # Set up a differentiator to get encoderSpeed from encoderCounts
        diff = Calculus().differentiator_variable(self.sampleTime)
        _ = next(diff)
        timeStep = self.sampleTime 

        # Reset startTime before Main Loop
        self.startTime = time.time()

        while True: 
            start = self.elapsedTime()

            queueLock.acquire() 
            if not dataQueue.empty(): 
                new = dataQueue.get()
                if new: 
                     steering = 0.5 * new['x'] / STEERING_FACTOR 
                     motorThrottle = 0.1 * (32767 - int(new['y'])) / VELOCITY_FACTOR 
                     self.motorCommand = np.array([motorThrottle, steering]) 

                LEDs = np.array([0, 0, 0, 0, 0, 0, 0, 0])
                
                # Perform I/O
                current, batteryVoltage, encoderCounts = self.myCar.read_write_std(self.motorCommand, LEDs)        
        
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
