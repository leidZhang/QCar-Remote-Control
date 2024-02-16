import sys 
import time 
import queue
import threading 
from multiprocessing import Process 

sys.path.append('src/') 
from common.service_module import ServiceModule 
# control thread strategies 
from strategies.thread_strategies import ControlSocketStrategy 
from strategies.thread_strategies import WheelControllerStrategy 
from strategies.thread_strategies import VirtualControlStrategy 
from strategies.thread_strategies import VirtualSpawnStrategy 
from strategies.thread_strategies import KeyboardControllerStrategy 
# sensor process strategies 
from strategies.virtual_sensor_strategies import VirtualCSICameraStrategy
from strategies.virtual_sensor_strategies import VirtualRGBDCameraStrategy
from strategies.virtual_sensor_strategies import VirtualLidarStrategy 
from strategies.virtual_sensor_strategies import VirtualGPSStrategy 

class ServiceManager(ServiceModule): 
    def __init__(self, settings) -> None:  
        self.threads = [] 
        self.processes = [] 

        self.queues = { # add more queues if needed 
            'remote': queue.Queue(10), 
            'local': queue.Queue(10), 
            'response': queue.Queue(10), 
        }

        self.locks = { # add more locks if needed 
            'control': threading.Lock()
        }
        
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
            # 'gps': VirtualGPSStrategy(settings['operation_mode']),
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
         
    def is_valid(self) -> bool:
        return True 
    
    def run(self) -> None:
        print("settings applied!")

        for key, strategy in self.init_strategies.items(): 
            thread = strategy.register()
            if thread != None: 
                self.threads.append(thread) 
            else: 
                self.init_strategies[key] = None 

        virtual_spawn = self.init_strategies['virtual_spawn'].target
        for thread in self.threads:  
            thread.start() 
            while thread.name == 'Virtual-Spawn' and not virtual_spawn.done: 
                time.sleep(5) 

        print("activated threads:", len(self.threads))

        for key, strategy in self.sensors.items(): 
            process = strategy.register() 
            if process != None: 
                self.processes.append(process) 
            else: 
                self.sensors[key] = None 
        for process in self.processes: 
            process.start() 
        
        print('sensor processes have activated') 
                 
# if __name__ == "__main__": 
#     t = ThreadManager() 
#     t.run() 