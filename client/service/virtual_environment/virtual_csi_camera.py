import os 
import sys 
import cv2 
import time 
import numpy as np 

sys.path.append('dependencies/q_libs/') 
from numpy.core.numeric import zeros_like 
from lib_qcar import QCarTask
from lib_utilities import GPS, Controllers, Camera2D, LaneDetector, RoadMap, QLabsWorkspace
from lib_utilities import Other
sys.path.append('src/service') 
from service_module import ServiceModule

class VirtualCSICamera(ServiceModule): 
    def __init__(self) -> None:
        self.counter = -1 
        self.done = False 
        # set up camera 
        self.id = "3@tcpip://localhost:18964"
        self.front_csi = None 
        self.detector = None 
        self.rgb_placeholder = None 
        self.rgb_segment = None  

    def is_valid(self) -> bool:
        return True 

    def terminate(self) -> None: 
        self.done = True 
        self.front_csi.terminate() 
        print("front csi terminated") 

    def init(self) -> None: 
        self.front_csi = Camera2D(
            camera_id=self.id, # currently, id of other cameras are unknown, f: 3
            frame_width=760, 
            frame_height=380, 
            frame_rate=33.3, 
            focal_length=np.array([[157.9], [161.7]], dtype = np.float64), 
            principle_point=np.array([[168.5], [123.6]], dtype = np.float64), 
            position=np.array([[0], [0], [0.14]], dtype = np.float64), 
            orientation=np.array([[ 0, 0, 1], [ 1, 0, 0], [ 0, -1, 0]], dtype = np.float64), 
            skew=0
        )
        # line detect 
        self.detector = LaneDetector(self.front_csi)
        self.rgb_placeholder = zeros_like(self.front_csi.image_data)
        self.rgb_segment = zeros_like(self.front_csi.image_data)        

    def run(self, camera_queue) -> None: 
        try: 
            self.init() # init camera 

            while not self.done: 
                t1 = time.time()
                self.counter += 1 
                if self.counter < 30: 
                    continue # skip the first 10 frames 

                if self.counter % 3 == 0 and not camera_queue.empty(): # 33 Hz 
                    self.front_csi.read() 
                    image = self.front_csi.image_data
                    # steering = camera_queue.get() 
                    # canny = self.detector.doCanny(self.front_csi.image_data)
                    # segment, mask = self.detector.doSegment(canny, steering)
                    # left_lines, right_lines = self.detector.calculateLines(segment)
                    # line_parameters, image_with_lines = self.detector.averageLines(self.front_csi.image_data, left_lines, right_lines)
                    # self.rgb_segment[:, :, 1] = mask
                    # rgb_placeholder[:, :, 2] = canny
                    # final = cv2.addWeighted(self.rgb_segment, 0.1, image_with_lines, 0.9, 0.0)
                    # final = cv2.addWeighted(self.rgb_placeholder, 0.2, final, 0.8, 0.0)
                    # cv2.imshow('ImageCanny', canny)    
                    cv2.imshow('ImageWithLines', image)
                    
                    if cv2.waitKey(int(1)) & 0xFF == ord('m'):
                        break 
                    print(time.time()-t1)
        except Exception as e: 
            print(e)
        
        