import sys 
import numpy as np 

sys.path.append('dependencies/q_libs') 
from lib_utilities import RoadMap
from library_qlabs_qcar import QLabsQCar 
from library_qlabs import QuanserInteractiveLabs 

class VirtualEnvironment: 
    def __init__(self) -> None:
        self.road_map = None 
        self.qlabs = QuanserInteractiveLabs() 
        self.qlabs.open("tcpip://localhost:18000")

    def destroy_all_actors(self) -> None: 
        print("Deleting all existing actors...")
        self.qlabs.destroyAllSpawnedActors()
        print("Actors deleted")

    def terminate(self) -> None: 
        self.qlabs.close() 

    def update_road_map(self, road_map) -> None: 
        self.road_map = road_map 

    def spawn_vehicle(self, node_id, qcar_id) -> None: 
        position = self.road_map.nodeList[node_id-1][0].position 
        orientation = [0, 0, (180/np.pi)*np.math.atan2(self.road_map.waypoint_list[1][1] - self.road_map.waypoint_list[0][1], self.road_map.waypoint_list[1][0] - self.road_map.waypoint_list[0][0])] 
        QLabsQCar().spawnDegrees(self.qlabs, qcar_id, position, orientation) 
        print(f"QCar {qcar_id} spawned") 

    def mount(self, traffic, desired_nodes) -> None: 
        try: 
            self.destroy_all_actors() 
            self.road_map = RoadMap(traffic) 
            
            for i in range(len(desired_nodes)): 
                _ = self.road_map.generate_waypoints(desired_nodes[i], factor = 10)
                self.spawn_vehicle(desired_nodes[i][0], i) 
        except Exception as e: 
            print(e) 
            self.terminate()    

# if __name__ == "__main__": 
#     traffic = "right" 
#     venv = VirtualEnvironment() 
#     desired_nodes = [[1, 5], [8, 9]] 

#     venv.mount(traffic, desired_nodes)