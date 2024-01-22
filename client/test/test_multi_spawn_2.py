import sys 
import cv2
import time 

sys.path.append('dependencies/q_libs')
from lib_utilities import RoadMap
from library_qlabs_qcar import QLabsQCar 
from library_qlabs import QuanserInteractiveLabs 
from library_qlabs_trafficlight_single import QLabsTrafficLightSingle 


'''
Validation code to test multi spawn and multi control 
'''

def spawn_qcar(qlabs, qcar_id, position, orientation): 
    QLabsQCar().spawnDegrees(qlabs, qcar_id, position, orientation)
    print(f"QCar {qcar_id} spawned") 

# Connecting to QLab   
url="tcpip://localhost:18000"
qlabs = QuanserInteractiveLabs()
qlabs.open(url)
# Initialize virtual envirnment 
print("Deleting all existing actors")
qlabs.destroyAllSpawnedActors()

positions = [
    [13.85, 1.9, 0], 
    [13.85, 25.5, 0], 
    [-5.7, 22.0, 0], 
    [1.4, 46.75, 0], 
    [-20.094, -3.833, 0], 
    [1.69, -8.183, 0], 
    [-22.67, 35.20, 0]
]

orientations = [
    [0, 0, 90], 
    [0, 0, 90], 
    [0, 0, 270], 
    [0, 0, 0], 
    [0, 0, 318], 
    [0, 0, 180], 
    [0, 0, 90]
]

for i in range(len(orientations)): 
    spawn_qcar(qlabs, i, positions[i], orientations[i]) 

# spawn traffic light
traffic_light = QLabsTrafficLightSingle()
traffic_light.spawn(qlabs, 0, [11, 1.9, 0],[0, 0, 90], [0, 0, 0])
traffic_light.setState(qlabs, 0, 0)

_ = QLabsQCar().setVelocityAndRequestState(qlabs, 0, 2, 0, 0, 1, 0, 0, 0, 0)
_ = QLabsQCar().setVelocityAndRequestState(qlabs, 1, 2, 0, 1, 1, 0, 0, 0, 0)
# QLabsQCar().setVelocityAndRequestState(qlabs, 2, 1, 0, 0, 0, 0, 0, 0, 0)
# QLabsQCar().setVelocityAndRequestState(qlabs, 3, 1, 0, 0, 0, 0, 0, 0, 0)

try: 
    while True: 
        _, image_0 = QLabsQCar().getCameraData(qlabs, 0, 4)
        _, image_1 = QLabsQCar().getCameraData(qlabs, 1, 4)
        cv2.imshow("RGBD Raw 0", image_0)
        cv2.imshow("RGBD Raw 1", image_1)

        if cv2.waitKey(int(1)) & 0xFF == ord('m'):
            break 
except Exception as e: 
    print(e) 
    qlabs.close()