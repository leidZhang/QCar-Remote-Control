import sys 
import time 
from threading import Thread 

sys.path.append('dependencies/q_libs') 
from library_qlabs import QuanserInteractiveLabs 
from library_qlabs_trafficlight_single import QLabsTrafficLightSingle 

class TrafficLightDecorator: 
    def __init__(self, qlabs, id) -> None: 
        # common attributes
        self.qlabs = qlabs 
        self.id = id 
        # traffic light attributes 
        self.signals = [1, 2, 0] # green, yellow, red
        self.signal_duration_map = {1: 5, 2: 1, 0: 7} 
        self.signal_time_stamp = time.time() 
        self.signal_pointer = None 

    def set_state(self, pointer) -> None: 
        # update the signal_pointer 
        self.signal_pointer = pointer % 3 # avoid overflow 
        # update the signal 
        QLabsTrafficLightSingle().setState(self.qlabs, self.id, self.signals[self.signal_pointer]) 
    
    def spawn(self, position, orientation, init_pointer=1) -> None: 
        # spawn the traffic light at designated position 
        QLabsTrafficLightSingle().spawn(self.qlabs, self.id, position, orientation, [1.0, 1.0, 1.0]) 
        # set the initial signal 
        self.set_state(init_pointer) 

    def execute(self) -> None: 
        current_time = time.time() 
        # get current signal, duration and next signal  
        current_signal = self.signals[self.signal_pointer] 
        duration = self.signal_duration_map[current_signal] 
        next_pointer = (self.signal_pointer + 1) % 3
        # check if the current time exceeds the duration of the current signal 
        if current_time - self.signal_time_stamp > duration: 
            # upadate the signal pointer to the next signal 
            self.signal_pointer = next_pointer 
            # set the state of the traffic light to the next signal 
            QLabsTrafficLightSingle().setState(self.qlabs, self.id, self.signals[self.signal_pointer]) 
            # update the signal time stamp to the current time 
            self.signal_time_stamp = current_time 
        
if __name__ == "__main__": 
    # Connect to QLab 
    url="tcpip://localhost:18000"
    qlabs = QuanserInteractiveLabs()
    qlabs.open(url)
    qlabs.destroyAllSpawnedActors()

    gps_state = [
        ([24.25, 16.5, 0], [0, 0, 6.25]), # pair 1
        ([17.85, 2.5, 0], [0, 0, 3.15]), # pair 1
        # sec 0 ends here 
        ([-6, 12.8, 0], [0, 0, 7.85]), # pair2
        ([8.5, 6.5, 0], [0, 0, 4.7]), # pair2
        ([4.85, 16.5, 0], [0, 0, 6.25]), # pair3
        ([-1.85, 2.5, 0], [0, 0, 3.15]), # pair3
        # sec2 ends here 
    ]    

    traffic_lights = [] 
    for i in range(len(gps_state)): 
        traffic_light = TrafficLightDecorator(qlabs=qlabs, id=i)
        if i == 2 or i == 3: 
            traffic_light.spawn(gps_state[i][0], gps_state[i][1], init_pointer=0) 
        else: 
            traffic_light.spawn(gps_state[i][0], gps_state[i][1], init_pointer=2)  
        traffic_lights.append(traffic_light) 
        print(f"Traffic Light {i} spawned")

    counter = 100 
    while counter >= 0: 
        for i in range(len(traffic_lights)): 
            traffic_light = traffic_lights[i] 
            traffic_light.execute() 
        counter -= 1 
        time.sleep(1) 

    qlabs.close() 
    print("Traffic light demo stop")