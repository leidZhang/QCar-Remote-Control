from abc import ABC, abstractmethod 

class Controller(ABC): 
    @abstractmethod 
    def normalize_throttle(self, x_axis_signal) -> float: 
        pass 

    @abstractmethod 
    def normalize_steering(self, y_axis_signal) -> float: 
        pass 