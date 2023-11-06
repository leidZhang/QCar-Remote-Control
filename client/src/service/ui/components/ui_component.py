from abc import ABC, abstractmethod 

class UIComponent(ABC): 
    def __init__(self, root, values, pos_y) -> None:
        self.root = root 
        self.values = values  
        self.pos_y = pos_y 

    @abstractmethod 
    def create_widgets(self) -> None: 
        pass 

