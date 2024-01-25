import sys 
import time 
import json
import queue
import threading 
from multiprocessing import Process 

sys.path.append('src/') 
from common.service_module import ServiceModule 
from strategies.thread_strategies import ControlSocketStrategy 
from strategies.thread_strategies import WheelControllerStrategy, WheelController 
from strategies.thread_strategies import VirtualControlStrategy 
from strategies.thread_strategies import KeyboardControllerStrategy 
from strategies.virtual_sensor_strategies import VirtualCSICameraStrategy
from strategies.virtual_sensor_strategies import VirtualRGBDCameraStrategy
from strategies.virtual_sensor_strategies import VirtualLidarStrategy 

class ServiceManager(ServiceModule): 
    def __init__(self, settings) -> None:  
        self.threads = [] 
        self.processes = [] 
        self.wheel = WheelController(mode=settings['controller'], index=settings['device'])

        self.queues = { # add more queues if needed 
            'remote': queue.Queue(10), 
            'local': queue.Queue(10), 
            'response': queue.Queue(10), 
        }

        self.locks = { # add more locks if needed 
            'control': threading.Lock()
        }
        
        self.init_strategies = { # add more strategies here if needed 
            'wheel_controller': WheelControllerStrategy(
                mode=settings['controller'],
                index=settings['device'], 
                args=(self.locks['control'], self.queues['remote'], self.queues['local'])
            ), 
            'keyboard_controller': KeyboardControllerStrategy(
                mode=settings['controller'],
                args=(self.locks['control'], self.queues['remote'], self.queues['local'])
            ), 
            'control_socket': ControlSocketStrategy(
                ip=settings['ip'], 
                port=settings['port'], 
                args=(self.locks['control'], self.queues['remote'], self.queues['response'])
            ), 
            'virtual_control': VirtualControlStrategy(
                traffic=settings['traffic'], 
                start_node=settings['spawn_node'],  
                end_node=settings['destination_node'], 
                args=(self.queues['local'], )
            ),
        } 

        self.sensors = {
            'csi_camera': VirtualCSICameraStrategy(settings['csi_camera']), 
            'rgbd_camera': VirtualRGBDCameraStrategy(settings['rgbd_camera']), 
            'lidar': VirtualLidarStrategy(settings['lidar']), 
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
        virtual_control = self.init_strategies['virtual_control'].target
        return virtual_control.status 
    
    def run(self) -> None:
        print("settings applied!")

        for key, strategy in self.init_strategies.items(): 
            thread = strategy.register()
            if thread != None: 
                self.threads.append(thread) 
            else: 
                self.init_strategies[key] = None 
        for thread in self.threads:  
            thread.start() 

        print("activated threads:", len(self.threads))

        if self.init_strategies['virtual_control'] is not None: 
            while not self.init_strategies['virtual_control'].target.status: 
                time.sleep(5) # wait until virtual environment initialized 
            print("control threads have activated")

            for key, strategy in self.sensors.items(): 
                process = strategy.register() 
                if process != None: 
                    self.processes.append(process) 
                else: 
                    self.sensors[key] = None 
            for process in self.processes: 
                process.start() 
        
            print('sensor processes have activated') 
            pass  
        else: 
            # implement when stakeholders decide to read vedio stream from the real QCar 
            pass 
                 
# if __name__ == "__main__": 
#     try: 
#         file_path = "src/ui/json/setting.json"
#         with open(file_path, "r") as json_file: 
#             data = json.load(json_file)
#         print("The following is your initial setting: ")
#         for key, val in data.items(): 
#             print(key + ":", val) 

#         t = ServiceManager(data) 
#         t.run() 

#         while True: 
#             time.sleep(100) 
#     except KeyboardInterrupt: 
#         t.terminate() 
#         print("program stopped") 