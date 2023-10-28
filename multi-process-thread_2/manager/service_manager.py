import sys  
from threading import Thread, Lock
from multiprocessing import Process 

sys.path.append('modules/')
from mock_thread import MockThread 
from mock_sensor import MockSensor 

class ServiceManager: 
    def __init__(self) -> None:
        self.lock = Lock() 
        self.control_services = [
            MockThread(1), 
            MockThread(2), 
            MockThread(3),  
        ]
        self.sensors_services = [
            MockSensor('a'), 
            MockSensor('b'), 
            MockSensor('c'), 
        ]
        self.threads = [] 
        self.processes = []

    def terminate(self) -> None: 
        for service in self.control_services: 
            service.terminate() 
        for thread in self.threads: 
            thread.join() 
        print('control modules terminated')

        for service in self.sensors_services:
            service.terminate() 
        for process in self.processes: 
            process.join() 
        print('sensor modules terminated')

    def run(self) -> None: 
        for service in self.control_services: 
            thread = Thread(target=service.run) 
            self.threads.append(thread) 
        for thread in self.threads: 
            thread.start() 
        print('control module activated') 

        for service in self.sensors_services: 
            service = Process(target=service.run)
            self.processes.append(service) 
        for process in self.processes: 
            process.start() 
        print('sensor module activated')
