from Quanser.product_QCar import QCar
from Quanser.q_ui import gamepadViaTarget
from Quanser.q_misc import Calculus
from Quanser.q_interpretation import basic_speed_estimation
import os
import time
import struct
import numpy as np 

class QcarControl:  
    def __init__(self) -> None: 
        self.startTime = time.time() 
        self.sampleRate = 50
        self.sampleTime = 1/self.sampleRate 
        self.myCar = QCar() 
        self.counter = 0 
        self.mtr_cmd = np.array([0,0]) 

    def elapsed_time(self):
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
            start = self.elapsed_time()

            queueLock.acquire() 
            if not dataQueue.empty(): 
                new = dataQueue.get()
                if new: 
                     steering = new['x'] / -10000 
                     velocity = (32767 - new['y']) / 30000 
                     self.mtr_cmd = np.array([0, steering]) 

                LEDs = np.array([0, 0, 0, 0, 0, 0, 0, 0])
                
                # Perform I/O
                current, batteryVoltage, encoderCounts = self.myCar.read_write_std(self.mtr_cmd, LEDs)        
        
                # Differentiate encoder counts and then estimate linear speed in m/s
                encoderSpeed = diff.send((encoderCounts, timeStep))
                linearSpeed = basic_speed_estimation(encoderSpeed)

                # End timing this iteration
                end = self.elapsed_time()

                # Calculate computation time, and the time that the thread should pause/sleep for
                computation_time = end - start
                sleep_time = self.sampleTime - computation_time % self.sampleTime

                # Pause/sleep and print out the current timestamp
                # time.sleep(sleep_time)

                responseQueue.put(str(velocity))
                   
            queueLock.release()
            
            timeAfterSleep = self.elapsed_time()
            timeStep = timeAfterSleep - start
            self.counter += 1
