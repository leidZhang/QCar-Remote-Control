import sys 
from multiprocessing import Process

sys.path.append('src/') 
from service.sensor.virtual_csi_camera import VirtualCSICamera 

v1 = VirtualCSICamera(1, 0) 
v2 = VirtualCSICamera(1, 1) 
v3 = VirtualCSICamera(1, 2) 

process_1 = Process(target=v1.run) 
process_2 = Process(target=v2.run) 
process_3 = Process(target=v3.run) 

process_1.start() 
process_2.start() 
process_3.start() 