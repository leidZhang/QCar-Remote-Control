import os 
import sys 
import numpy as np  
from threading import Thread
# quanser's package
sys.path.append('dependencies/q_libs')
from library_qlabs_qcar import QLabsQCar 
from lib_utilities import RoadMap, QLabsWorkspace
from lib_qcar import QCar, QCarTask

def start_qcar(qcar, val): 
    # qcar_0 = QCarTask(100.0, hardware=0) # hardware 0 
    LEDs = np.array([0, 0, 0, 0, 0, 0, 0, 0]) 
    throttle = 0.3 * 0.15 * val # config here 
    steering = 0.5 * 0
    
    while True: 
        qcar.read_write_std(np.array([throttle, steering]), LEDs)

# init settings 
traffic = "right" 
road_map = RoadMap(traffic)
workspace = QLabsWorkspace(road_map)
desired_nodes = [[1, 5], [23, 9]]

# spawn qcar 
try: 
    for i in range(len(desired_nodes)): 
        _ = road_map.generate_waypoints(desired_nodes[i], factor = 10)
        workspace.spawnVehicle(desired_nodes[i][0], i)
        workspace.spawnRoadPoints() 

    relative_path = "dependencies/rt-win64/QCar_Workspace_5.rt-win64" 
    file_path = os.path.abspath(relative_path) 
    os.startfile(file_path)

except: 
    print("Some thing is wrong in spawn")

    







