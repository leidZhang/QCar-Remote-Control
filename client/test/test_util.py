import os 
import sys
import time  

sys.path.append('C:/Users/zphwi/OneDrive/Desktop/HaoZhang/QCarControl/client/dependencies/q_libs/')
from numpy.core.numeric import zeros_like 
from lib_qcar import QCarTask
from lib_utilities import GPS, Controllers, Camera2D, LaneDetector, RoadMap, QLabsWorkspace, Other

def init_qcar(start_node): 
        desired_nodes = [10, 27] # [start_node, end_node]
        desired_speed = 0.5 
        style = 'right' 
        # generate pathway
        road_map = RoadMap(style) 
        closed_circuit = road_map.generate_waypoints(desired_nodes, factor = 10)
        pathway = road_map.pathway
        waypoint_list = road_map.waypoint_list
        # connect to QLab
        qlabs_workspace = QLabsWorkspace(road_map) 
        position, orientation = qlabs_workspace.spawnVehicle(start_node)
        # qlabs_workspace.spawnRoadPoints() # used for generate path 
        # Launch the spawn models for the QCar
        relative_path = "dependencies/rt-win64/QCar_Workspace_5.rt-win64" 
        file_path = os.path.abspath(relative_path)
        os.startfile(file_path)
        time.sleep(2)