import sys 

sys.path.append('dependencies/q_libs') 
from library_qlabs import QuanserInteractiveLabs 
from library_qlabs_trafficlight_single import QLabsTrafficLightSingle 

'''
May have to upgrade the package
'''

# Connecting to QLab   
url="tcpip://localhost:18000"
qlabs = QuanserInteractiveLabs()
qlabs.open(url)
qlabs.destroyAllSpawnedActors()
# spawn traffic light
traffic_light = QLabsTrafficLightSingle()
traffic_light.spawn(qlabs, 0, [11, 1.9, 0],[0, 0, 90], [0, 0, 0])
traffic_light.setState(qlabs, 0, 0)
# disconnect 
qlabs.close() 