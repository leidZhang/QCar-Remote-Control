from abc import ABC, abstractmethod

class ServiceModule(ABC): 
    @abstractmethod 
    def terminate(self) -> None: 
        pass 

    @abstractmethod 
    def is_valid(self) -> bool: 
        pass 

    @abstractmethod
    def run(self, *args) -> None: 
        pass 