import sys 
import time 

sys.path.append('dependencies/q_libs') 
from library_qlabs import QuanserInteractiveLabs 
from library_qlabs_trafficlight_single import QLabsTrafficLightSingle 

# Connect to QLab 
url="tcpip://localhost:18000"
qlabs = QuanserInteractiveLabs()
qlabs.open(url)
qlabs.destroyAllSpawnedActors() 
# spawn traffic light 
traffic_light = QLabsTrafficLightSingle() 
position_0 = [24.85, 15.5, 0] # temp position 
# orientation_0 = [0, 0, 3.15]
orientation_0 = [0, 0, 6.3] # temp orientation 
scale_0 = [1.0, 1.0, 1.0]
traffic_light.spawn(qlabs, 0, position_0, orientation_0, scale_0) 
print("spawn complete")

# set to green 
print("change to green")
traffic_light.setState(qlabs, 0, 1)
time.sleep(5) 
# set to yellow 
print("change to yellow")
traffic_light.setState(qlabs, 0, 2)
time.sleep(5) 
# set to red 
print("change to red")
traffic_light.setState(qlabs, 0, 0)
time.sleep(5) 

# disconnect 
print("test complete") 
qlabs.close() 