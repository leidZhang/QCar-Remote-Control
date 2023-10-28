import time 
from multiprocessing import Process 

from process_1.control_module import ControlModule
from process_2.sensor_module import SensorModule

class ProcessManager: 
    def __init__(self) -> None:
        self.modules = [
            ControlModule(), 
            SensorModule(), 
        ] 
        self.pool = [] 

    def termiante(self) -> None: 
        for module in self.modules: 
            module.terminate() 
        for process in self.pool: 
            process.join() 

    def run(self) -> None: 
        for module in self.modules: 
            process = Process(target=module.run) 
            self.pool.append(process) 
        for process in self.pool: 
            process.start() 