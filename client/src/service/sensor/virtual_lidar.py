import os  
import sys  
import matplotlib.pyplot as plt

sys.path.append('dependencies/q_libs/') 
from lib_utilities import Lidar
sys.path.append('src/') 
from common.service_module import ServiceModule 

class VirtualLidar(ServiceModule): 
    def __init__(self, mode) -> None:
        self.mode = mode 
        self.counter = -1 
        self.done = False
        self.lidar = None 

    def is_valid(self) -> bool:
        return self.mode == 1 
    
    def terminate(self) -> None:
        self.done = True
        self.lidar.terminate() 
        print("Lidar terminated") 

    def init(self) -> None: 
        self.lidar = Lidar() 
        print("Lidar activated")

    def run(self) -> None: 
        try: 
            self.init() 
            ax = plt.subplot(111, projection='polar')
            plt.show(block=False)

            while not self.done: 
                plt.cla() 

                self.lidar.read()

                ax.scatter(self.lidar.angles, self.lidar.distances, marker='.')
                ax.set_theta_zero_location("W")
                ax.set_theta_direction(-1)

                plt.pause(0.1)
        except Exception as e: 
            print(e) 

if __name__ == "__main__": 
    vl = VirtualLidar(1) 
    vl.run() 