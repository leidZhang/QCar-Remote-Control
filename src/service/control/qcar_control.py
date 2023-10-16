import sys 
import time
import numpy as np 

sys.path.append('dependencies/')
from Quanser.product_QCar import QCar
from Quanser.q_misc import Calculus
from Quanser.q_interpretation import basic_speed_estimation
sys.path.append('src/') 
from service.service_module import ServiceModule
from common.utils import handle_full_queue
from common.utils import status_to_dict 
from strategies.qcar_control_strategies import LightStrategy 
from strategies.qcar_control_strategies import ReverseStrategy 
from strategies.qcar_control_strategies import CruiseStrategy 
from strategies.qcar_control_strategies import SafeStrategy 

class QCarControl(ServiceModule):  
    def __init__(self) -> None: 
        self.my_car = QCar()
        self.start_time = time.time() 
        self.sample_rate = 50
        self.sample_time = 1 / self.sample_rate 
        self.state = None 

        self.motor_command = np.array([0.0, 0.0]) 
        self.LEDs = np.array([0, 0, 0, 0, 0, 0, 0, 0])
        self.control_strategies = [
            LightStrategy(), 
            ReverseStrategy(), 
            CruiseStrategy(), 
            SafeStrategy(), 
        ]
    
    def terminate(self) -> None: 
        print("Stopping QCar...")
        self.done = True 
        self.my_car.terminate() 
    
    def handle_LEDs(self) -> None:                      
        # Adjust LED indicators based on steering        
        if self.motor_command[1] > 0.3:
            self.LEDs[0] = 1
            self.LEDs[2] = 1
        elif self.motor_command[1] < -0.3:
            self.LEDs[1] = 1
            self.LEDs[3] = 1
        else: 
            self.LEDs = np.array([0, 0, 0, 0, 0, 0, self.LEDs[6], self.LEDs[7]])
        # reverse indicators based on reverse gear
        if self.motor_command[0] < 0:
            self.LEDs[5] = 1 
    
    def elapsed_time(self) -> float: 
        return time.time() - self.start_time 

    def run(self, queue_lock, control_queue, response_queue) -> None: 
        print("Activating QCar control...")
        # Set up a differentiator to get encoderSpeed from encoderCounts
        diff = Calculus().differentiator_variable(self.sample_time)
        _ = next(diff)
        time_step = self.sample_time 

        # Reset start_time before Main Loop
        self.start_time = time.time()

        try: 
            while not self.done: 
                start = self.elapsed_time()

                queue_lock.acquire() 
                if not control_queue.empty(): 
                    self.state = control_queue.get()  

                    # execute strategies 
                    for strategy in self.control_strategies: 
                        strategy.execute(self) 

                    # handle control 
                    throttle = 0.3 * self.state['throttle'] 
                    steering = 0.5 * self.state['steering'] 
                    self.motor_command = np.array([throttle, steering]) 
                    # handle LEDs 
                    self.handle_LEDs()

                    # Perform I/O
                    current, battery_voltage, encoder_counts = self.myCar.read_write_std(self.motor_command, self.LEDs)
                    
                    # Differentiate encoder counts and then estimate linear speed in m/s
                    encoder_speed = diff.send((encoder_counts, time_step))
                    linear_speed = basic_speed_estimation(encoder_speed)

                    # End timing this iteration
                    end = self.elapsed_time()

                    # Calculate computation time, and the time that the thread should pause/sleep for
                    computation_time = end - start
                    sleep_time = self.sample_time - computation_time % self.sample_time

                    # Calculate the remaining battery capacity 
                    battery_capacity = (battery_voltage - 10.5) * 100 / (12.6 - 10.5)

                    # send response data to control_socket 
                    response_data = status_to_dict(linear_speed, battery_capacity, self.motor_command[0], self.motor_command[1]) 
                    handle_full_queue(response_queue, response_data)

                    queue_lock.release()
                time_after_sleep = self.elapsed_time()
                time_step = time_after_sleep - start   
        except Exception as e: 
            print(e) 
        finally: 
            self.my_car.terminate()
            
            
