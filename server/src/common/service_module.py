from abc import ABC, abstractmethod 


class ServiceModule(ABC): 
    @abstractmethod 
    def __init__(self, *args) -> None:
        pass 

    @abstractmethod 
    def terminate(self, *args) -> None: 
        pass 

    @abstractmethod
    def run(self, *args) -> None: 
        pass 