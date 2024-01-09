import sys 
import time 
import queue
import threading 
from multiprocessing import Process 

from common.service_module import ServiceModule 
from core.settings import INIT_QUEUES
from core.settings import INIT_LOCKS
# control thread strategies 
from common.thread_strategies import ControlSocketStrategy 
from common.thread_strategies import WheelControllerStrategy 
from common.thread_strategies import VirtualControlStrategy 
from common.thread_strategies import VirtualSpawnStrategy 
from common.thread_strategies import KeyboardControllerStrategy 
# sensor process strategies 
from common.process_strategies import VirtualCSICameraStrategy
from common.process_strategies import VirtualRGBDCameraStrategy
from common.process_strategies import VirtualLidarStrategy 
from common.process_strategies import VirtualGPSStrategy 

class ServiceManager: 
    def __init__(self, settings) -> None:  
        self.threads = [] 
        self.processes = [] 

        self.queues = INIT_QUEUES 
        self.locks = INIT_LOCKS 
        
        self.init_strategies = { # add more strategies here if needed 
            'control_socket': ControlSocketStrategy(
                mode=settings['operation_mode'],
                ip=settings['ip'], 
                port=settings['port'], 
                args=(self.locks['control'], self.queues['remote'], self.queues['response'])
            ), 
            'wheel_controller': WheelControllerStrategy(
                mode=settings['controller'],
                index=settings['device'], 
                args=(self.locks['control'], self.queues['remote'], self.queues['local'])
            ), 
            'keyboard_controller': KeyboardControllerStrategy(
                mode=settings['controller'],
                args=(self.locks['control'], self.queues['remote'], self.queues['local'])
            ), 
            'virtual_spawn': VirtualSpawnStrategy(
                mode=settings['operation_mode'],
                traffic=settings['traffic'], 
                start_node=settings['spawn_node'],  
                end_node=settings['destination_node'],
            ), 
            'virtual_gps': VirtualGPSStrategy(
                mode=settings['operation_mode']
            ), 
            'virtual_control': VirtualControlStrategy(
                mode=settings['operation_mode'],
                args=(self.queues['local'], )
            ),
        } 

        self.sensors = {
            'csi_camera': VirtualCSICameraStrategy(settings['csi_camera']), 
            'rgbd_camera': VirtualRGBDCameraStrategy(settings['rgbd_camera']), 
            'lidar': VirtualLidarStrategy(settings['lidar']), 
            'gps': VirtualGPSStrategy(settings['operation_mode']),
        }

    def terminate(self) -> None:
        for strategy in self.init_strategies.values(): 
            if strategy is not None: 
                strategy.target.terminate() 
        for thread in self.threads: 
            thread.join() 
        print('control threads stopped')

        for sensor in self.sensors.values(): 
            if sensor is not None: 
                sensor.target.terminate() 
        for process in self.processes: 
            process.join() 
        print('sensors stopped')
         
    