import sys 

sys.path.append('process_2/') 
from senseor_process_manager import SensorProcessManager

s = SensorProcessManager() 

class SensorModule: 
    def terminate(self) -> None: 
        s.termiante() 

    def run(self) -> None: 
        s.run() 