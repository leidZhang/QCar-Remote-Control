import os 
import sys
import time
import numpy as np  
import queue  
# quanser packages 
sys.path.append('dependencies/q_libs') # start from project root 
from numpy.core.numeric import zeros_like 
from lib_qcar import QCar
from lib_utilities import RoadMap, QLabsWorkspace, GPS, Calculus 
# custom scripts 
sys.path.append('src/') 
from common.service_module import ServiceModule 
from strategies.virtual_control_strategies import VirtualReverseStrategy
from strategies.virtual_control_strategies import VirtualSafeStrategy 
from strategies.virtual_control_strategies import VirtualCruiseStrategy 
from strategies.virtual_control_strategies import VirtualLightStrategy 

class VirtualControl(ServiceModule): 
    def __init__(self, mode, traffic, start_node, end_node) -> None:
        self.style = traffic
        self.mode = mode 
        self.status = False 
        self.rate = 50 # placeholder rate 
        self.done = False 
        self.start_time = time.time() 
        self.count = 0 

        self.LEDs = np.array([0, 0, 0, 0, 0, 0, 0, 0])
        self.my_car = None # QCarTask(frequency=int(self.rate), hardware=0)
        self.gps = None 
        self.qlabs_workspace = None 
        self.road_map = None 
        self.state = None 
        self.start_node = int(start_node) 
        self.end_node = int(end_node) # place holder attribute setting 

        self.virtual_qcar_strategies = [
            VirtualCruiseStrategy(), 
            VirtualReverseStrategy(), 
            VirtualSafeStrategy(), 
            VirtualLightStrategy(),
        ]

    def elapsed_time(self) -> None:
        return time.time() - self.start_time
            
    def init_virtual_environment(self): 
        desired_nodes = [self.start_node, self.end_node] # [start_node, end_node] 

        # generate pathway to get the correct direction 
        self.road_map = RoadMap(self.style) 
        closed_circuit = self.road_map.generate_waypoints(desired_nodes, factor = 10)
        pathway = self.road_map.pathway
        waypoint_list = self.road_map.waypoint_list
        # connect to QLab
        self.qlabs_workspace = QLabsWorkspace(self.road_map) 
        position, orientation = self.qlabs_workspace.spawnVehicle(self.start_node)
        # qlabs_workspace.spawnRoadPoints() # used for generate path 
        # Launch the spawn models for the QCar
        relative_path = "dependencies/rt-win64/QCar_Workspace_5.rt-win64" 
        file_path = os.path.abspath(relative_path) 
        os.startfile(file_path)
        time.sleep(2)
        print("QCar spawned!")

    def handle_LEDs(self) -> None: 
        if self.state['steering'] > 0.3: 
            self.LEDs[0] = 1
            self.LEDs[2] = 1
        elif self.state['steering'] < -0.3:
            self.LEDs[1] = 1
            self.LEDs[3] = 1
        else: 
            self.LEDs = np.array([0, 0, 0, 0, 0, 0, self.LEDs[6], self.LEDs[7]])

    def terminate(self) -> None:
        self.done = True 
        print("Virtual Control terminated") 

    def is_valid(self) -> bool:
        if self.mode == "remote": 
            return False 
        
        if self.start_node is None or self.end_node is None: 
            return False 
        
        return True 
    
    def run(self, local_queue) -> None:
        print("activating virtual environment control...") 
        
        try: 
            self.init_virtual_environment() 
            self.my_car = QCar(hardware=0) 
            self.gps = GPS('tcpip://localhost:18967') 
            self.status = True 

            diff = Calculus().differentiator_variable(1 / self.rate) 
            _ = next(diff)

            start = time.time() 
            time_step = start 
            while not self.done: 
                start = self.elapsed_time()

                if not local_queue.empty(): 
                    self.state = local_queue.get() # get controller state 
                    
                    # execute strategies 
                    for strategy in self.virtual_qcar_strategies: 
                        strategy.execute(self) 
                    # handle control
                    throttle = 0.3 * 0.15 * self.state['throttle'] # config here 
                    steering = 0.5 * self.state['steering']
                    # handle LEDs 
                    self.handle_LEDs() 
                    
                    # apply state and get readings 
                    current, battery_voltage, encoder_counts = self.my_car.read_write_std(np.array([throttle, steering]), self.LEDs)
                    encoder_speed = diff.send((encoder_counts, time_step))
                    speed = self.my_car.estimate_speed(encoder_speed, steering) 
                    
                    # read current pos 
                    self.gps.read() # read gps info 
                    if self.count % 3 == 0: # only for decrease flash on the terminal 
                        os.system("cls") 
                        print("flags:", self.state['control_flags'])
                        print(f"linear speed: {speed[0]:.2f} m/s, angular speed: {speed[1]:.2f} rad/s")   
                        print(f"x: {self.gps.position[0]:.2f}, y: {self.gps.position[1]:.2f},  orientation: {((180 / np.pi) * self.gps.orientation[2]):.2f}°") 
                        
                    self.count += 1 
                time_step = self.elapsed_time() - start  
        except Exception as e: 
            print(e) 
        finally: 
            self.my_car.terminate() 
            os._exit(0) 

if __name__ == "__main__": 
    q = queue.Queue(10)
    state = {
            'throttle': 0.1, 
            'steering': 0, 
            'cruise_throttle': 0, 
            'control_flags': {
                'safe': False, 
                'reverse': False, 
                'light': False, 
                'cruise': False, 
            }
        }
    q.put(state)  
    v = VirtualControl("10") 
    v.run(q) 