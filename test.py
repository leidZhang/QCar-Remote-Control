from logidrivepy import LogitechController 
from utils import stateToDict

c = LogitechController()
c.LogiSteeringInitialize(True)  


for i in range(10): 
    c.LogiUpdate(1)
    c.LogiUpdate(1)
    state = c.LogiGetStateENGINES(1) 
    data = stateToDict(state) 
    print(data)
