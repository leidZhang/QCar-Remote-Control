import os 
import sys 
import time 
# quanser packages 
sys.path.append('dependencies/q_libs') # start from project root 
from lib_utilities import RoadMap, QLabsWorkspace 
# custom scripts 
sys.path.append('src/')
from common.service_module import ServiceModule 

class VirtualSpawn(ServiceModule): 
    def __init__(self, mode, traffic, start_node, end_node) -> None:
        self.mode = mode 
        self.traffic = traffic 
        self.start_node = int(start_node) 
        self.end_node = int(end_node) # place holder attribute setting 

        self.done = False
        self.qlabs_workspace = None 
        self.road_map = None 

    def is_valid(self) -> bool:
        if self.mode == 'remote': 
            return False 
                
        if self.start_node is None or self.end_node is None: 
            return False

        return True 
    
    def init_virtual_environment(self): 
        desired_nodes = [self.start_node, self.end_node] # [start_node, end_node] 

        # generate pathway to get the correct direction 
        self.road_map = RoadMap(self.traffic) 
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
    
    def terminate(self) -> None:
        self.done = True  
    
    def run(self) -> None:
        self.init_virtual_environment() 
        self.terminate() 

# if __name__ == "__main__": 
#     mode = 'local'
#     traffic = "right" 
#     start_node = '9' 
#     end_node = '16' 
#     v = VirtualSpawn(mode, traffic, start_node, end_node) 
#     v.run() 