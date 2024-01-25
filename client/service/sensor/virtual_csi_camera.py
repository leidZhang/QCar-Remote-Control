import sys 
import cv2  
import queue 
import numpy as np 

sys.path.append('dependencies/q_libs/') 
from lib_utilities import Camera2D
sys.path.append('src/') 
from common.service_module import ServiceModule

class VirtualCSICamera(ServiceModule): 
    def __init__(self, mode) -> None:
        self.mode = mode 
        self.counter = -1 
        self.done = False 
        self.cameras = [] # 0: right, 1: rear, 2: left, 3: front 
        # set up camera 
        self.focal_length = np.array([[157.9], [161.7]], dtype = np.float64) 
        self.principle_point = np.array([[168.5], [123.6]], dtype = np.float64) 
        self.position = np.array([[0], [0], [0.14]], dtype = np.float64)
        self.orientation = np.array([[ 0, 0, 1], [ 1, 0, 0], [ 0, -1, 0]], dtype = np.float64)
        
    def is_valid(self) -> bool:
        return self.mode 

    def terminate(self) -> None: 
        self.done = True 
        for i in range(len(self.cameras)): 
            self.cameras[i].terminate() 
        print("CSI cameras terminated") 

    def init_camera(self, id): 
        camera = Camera2D(
            camera_id=str(id) + "@tcpip://localhost:"  + str(18961+id), # currently, id of other cameras are unknown, f: 3
            frame_width=760, 
            frame_height=380, 
            frame_rate=33.3, 
            focal_length=self.focal_length, 
            principle_point=self.principle_point, 
            position=self.position, 
            orientation=self.orientation, 
            skew=0
        )
        self.cameras.append(camera) 

    def init_csi(self) -> None: 
        for i in range(4): 
            self.init_camera(i) 

        print('CSI Camera Activated')        

    def run(self) -> None: 
        try: 
            self.init_csi() # init cameras 

            while not self.done: 
                self.counter += 1 
                if self.counter < 30: 
                    continue # skip the first 10 frames 

                if self.counter % 3 == 0: # 33 Hz 
                    # capture image from csi 
                    images = [] 
                    image_width = 300 
                    image_height = 150 
                    for i in range(4): 
                        self.cameras[i].read() 
                        image_data = self.cameras[i].image_data
                        image_data = cv2.resize(image_data, (image_width, image_height))
                        images.append(image_data)
                    
                    final_image = cv2.hconcat([
                        images[1][:, image_width // 2:], # rear camera right 
                        images[2],                       # left camera 
                        images[3],                       # front camera 
                        images[0],                       # right camera 
                        images[1][:, :image_width // 2], # rear camera left
                    ]) 

                    # display the image 
                    cv2.imshow('CSI Camera Image', final_image) 

                    if cv2.waitKey(int(1)) & 0xFF == ord('m'):
                        break 
                    
        except Exception as e: 
            print(e)
        
if __name__ == "__main__": 
    vc = VirtualCSICamera(1) 
    q1 = queue.Queue(10)
    vc.run() 