import sys 
import time 
# quanser's package
sys.path.append('dependencies/q_libs')
from library_qlabs import QuanserInteractiveLabs
from library_qlabs_basic_shape import QLabsBasicShape
from library_qlabs_qcar import QLabsQCar  

def spawn_qcar(qlabs, qcar_id, position, orientation): 
    QLabsQCar().spawnDegrees(qlabs, qcar_id, position, orientation)
    time.sleep(1) # delay time 1s

# Connecting to QLab   
url="tcpip://localhost:18000"
qlabs = QuanserInteractiveLabs()
qlabs.open(url)
# Initialize virtual envirnment 
time.sleep(2)
qlabs.destroyAllSpawnedActors()
time.sleep(2)

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

for i in range(7): 
    spawn_qcar(qlabs, i, positions[i], orientations[i]) 

print("Spawn complete")
qlabs.close()