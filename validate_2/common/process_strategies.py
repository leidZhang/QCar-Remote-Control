from multiprocessing import Process 
from abc import ABC, abstractmethod 

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