import os 
import sys
import time
import numpy as np   
# quanser packages 
sys.path.append('dependencies/q_libs') # start from project root 
from lib_utilities import GPS, Calculus 
# custom scripts 
sys.path.append('src/') 
from common.service_module import ServiceModule 

class VirtualGPS(ServiceModule): 
    def __init__(self, mode) -> None:
        self.mode = mode 
        self.done = False 
        self.count = 0 

        self.gps = None 
        self.start_time = None
        self.last_state = None 
        self.current_state = None  
        self.sleep_time = 0.18 

    def is_valid(self) -> bool:
        # return self.mode != 'remote' 
        return False
    
    def get_gps_state(self) -> tuple:  
        position_x = self.gps.position[0] 
        position_y = self.gps.position[1]
        orientation = self.gps.orientation[2] 

        return position_x, position_y, orientation 
    
    def calculate_speed(self, current, last, time) -> float: 
        return (current - last) / time 
    
    def terminate(self) -> None:
        self.done = True 

    def calcualte_speed_vector(self, delta_t) -> tuple: 
            speed_x = self.calculate_speed(self.current_state[0], self.last_state[0], delta_t) 
            speed_y = self.calculate_speed(self.current_state[1], self.last_state[1], delta_t) 
            angular_speed = self.calculate_speed(self.current_state[2], self.last_state[2], delta_t)

            return speed_x, speed_y, angular_speed 
    
    def run(self) -> None:
        print("Activating GPS")
        self.gps = GPS('tcpip://localhost:18967') 

        previous_time = time.time() 
        self.gps.read() # read gps info
        self.last_state = self.get_gps_state() 
        while not self.done: 
            if self.count > 30:   
                current_time = time.time()
                # read current position  
                self.gps.read() # read gps info 
                self.current_state = self.get_gps_state() 
                # cal absolute speed 
                delta_t = current_time - previous_time
                speed_vecotr = self.calcualte_speed_vector(delta_t)

                os.system("cls") 
                print(f"delta_t: {delta_t:.4f}s") 
                print(f"last_x: {self.last_state[0]:.2f}, last_y: {self.last_state[1]:.2f},  last_orientation: {((180 / np.pi) * self.last_state[2]):.2f}°")    
                print(f"x: {self.current_state[0]:.2f}, y: {self.current_state[1]:.2f},  orientation: {((180 / np.pi) * self.current_state[2]):.2f}°") 
                print(f"speed x: {speed_vecotr[0]:.2f} m/s, speed y: {speed_vecotr[1]:.2f} m/s, angular speed: {speed_vecotr[2]:.2f} rad/s")            

                # update position and time for next iteration 
                self.last_state = self.current_state   
                previous_time = current_time 

                time.sleep(self.sleep_time) 
            self.count += 1 

if __name__ == "__main__": 
    try: 
        v = VirtualGPS("local") 
        v.run() 
    except KeyboardInterrupt: 
        os._exit(0) 