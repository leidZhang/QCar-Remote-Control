import sys 
from multiprocessing import Process 

sys.path.append('process_2/') 
from mock_sensor import MockSensor 

class SensorProcessManager: 
    _instance = None 

    def __new__(cls, *args, **kwargs) -> _instance:
        if cls._instance is None: 
            cls._instance = super().__new__(cls, *args, **kwargs)
        return cls._instance 
    
    def __init__(self) -> None:
        self.sensors = [
            MockSensor('a'), 
            MockSensor('b'), 
            MockSensor('c'),  
        ]
        self.sub_processes = [] 

    def termiante(self) -> None: 
        for sensor in self.sensors: 
            sensor.terminate() 
        for sub_process in self.sub_processes: 
            sub_process.join() 

    def run(self) -> None: 
        for sensor in self.sensors: 
            process = Process(target=sensor.run) 
            self.sub_processes.append(process) 

        for process in self.sub_processes: 
            process.start()  
