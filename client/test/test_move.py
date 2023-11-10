import sys
import numpy as np
import time
import os
import cv2
import math 

sys.path.append('C:/Users/zphwi/OneDrive/Desktop/HaoZhang/QCarControl/client/dependencies/quanser/')
from numpy.core.numeric import zeros_like
from lib_qcar import QCarTask
from lib_utilities import GPS, Controllers, Camera2D, LaneDetector, RoadMap, QLabsWorkspace
from lib_utilities import Other
from test_util import init_qcar

# spawn qcar 
init_qcar(10)

# Timing Parameters and methods 
start_time = time.time()
def elapsed_time():
    return time.time() - start_time

sample_rate = 100.0
sample_time = 1/sample_rate
simulationTime = 100000 # some large number
print('Simulation rate is set to', sample_rate, '...')

# Qcar control 
my_car = QCarTask(frequency=int(sample_rate), hardware=0) 
LEDs = np.array([0, 0, 0, 0, 0, 0, 1, 1])

try: 
    while True: 
        my_car.read_write_std(np.array([0.1, 0]), LEDs) 

except KeyboardInterrupt: 
    print("User interrupted!")

except Exception as e: 
    print(e) 

finally: 
    my_car.terminate()