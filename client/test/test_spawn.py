import sys 
import time 
# quanser's package
sys.path.append('../common/')
from library_qlabs import QuanserInteractiveLabs
from library_qlabs_basic_shape import QLabsBasicShape
from library_qlabs_qcar import QLabsQCar  

# Connecting to QLab 
url="tcpip://localhost:18000"
qlabs = QuanserInteractiveLabs()
qlabs.open(url)
# Initialize virtual envirnment 
time.sleep(2)
qlabs.destroyAllSpawnedActors()
time.sleep(2)
# spawn QCar 1 
delay = 1
position_0 = [13.85, 1.9, 0]
orientation_0 = [0, 0, 90]
QLabsQCar().spawnDegrees(qlabs, 0, position_0, orientation_0)
time.sleep(delay)