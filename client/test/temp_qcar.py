import sys 
import time 
import numpy as np
sys.path.append('dependencies/q_libs')
from library_qlabs_qcar import QLabsQCar
from library_qlabs_basic_shape import QLabsBasicShape

class TempQCar: 
    def __init__(self, qlabs, road_map, qcar_id) -> None:
        self.qlabs = qlabs 
        self.road_map = road_map 
        self.qcar_id = qcar_id

    def spawn(self, node_id) -> None:
        position = self.road_map.nodeList[node_id-1][0].position
        orientation = [0, 0, (180/np.pi)*np.math.atan2(self.road_map.waypoint_list[1][1] - self.road_map.waypoint_list[0][1], self.road_map.waypoint_list[1][0] - self.road_map.waypoint_list[0][0])]
        QLabsQCar().spawnDegrees(self.qlabs, self.qcar_id, position, orientation)

        print(f"Spawned the QCar {self.qcar_id}")

        return position, orientation 
    
    def spawn_road_points(self):
		# Draw out pathway w/ waypoints in QLabs
        for i in range(len(self.road_map.waypoint_list)):
            QLabsBasicShape().spawn(self.qlabs, i, self.road_map.waypoint_list[i]*10, [0, 0, 0], [0.25, 0.25, 0.02], 1, False)
            time.sleep(0.01) 

        print(f"QCar {self.qcar_id} waypoints complete")
        