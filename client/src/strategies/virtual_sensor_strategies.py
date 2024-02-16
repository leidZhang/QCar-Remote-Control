import sys 
from multiprocessing import Process 
from abc import ABC, abstractmethod 

sys.path.append('src/')
from service.sensor.virtual_csi_camera import VirtualCSICamera 
from service.sensor.virtual_rgbd_camera import VirtualRGBDCamera 
from service.sensor.virtual_lidar import VirtualLidar
from service.sensor.virtual_gps import VirtualGPS 

class VirtualSensorStrategy(ABC): 
    @abstractmethod 
    def __init__(self, mode) -> None:
        pass 

    def register(self) -> Process:
        try: 
            if self.target.is_valid(): 
                process = Process(target=self.target.run, 
                                  name=self.name) 
                return process 
            else: 
                print(self.name, "will not activate")
        except Exception: 
            print("Error happened on", self.name)

class VirtualCSICameraStrategy(VirtualSensorStrategy): 
    def __init__(self, mode) -> None:
        self.name = "CSI-Camera" 
        self.target = VirtualCSICamera(mode) 

class VirtualRGBDCameraStrategy(VirtualSensorStrategy): 
    def __init__(self, mode) -> None:
        self.name = "RGBD-Camera" 
        self.target = VirtualRGBDCamera(mode) 

class VirtualLidarStrategy(VirtualSensorStrategy): 
    def __init__(self, mode) -> None:
        self.name = "Lidar" 
        self.target = VirtualLidar(mode) 

class VirtualGPSStrategy(VirtualSensorStrategy): 
    def __init__(self, mode) -> None:
        self.name = "GPS" 
        self.target = VirtualGPS(mode) 
