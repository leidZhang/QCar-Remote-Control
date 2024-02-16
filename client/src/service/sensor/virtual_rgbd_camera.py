import sys 
import cv2  

sys.path.append('dependencies/q_libs/')
from lib_utilities import Camera3D
sys.path.append('src/') 
from common.service_module import ServiceModule 

class VirtualRGBDCamera(ServiceModule): 
    def __init__(self, mode) -> None:
        self.mode = mode 
        self.done = False
        self.id = "0@tcpip://localhost:18965"
        self.rgbd_camera = None 
        self.counter = -1 

    def is_valid(self) -> bool:
        return self.mode == 1 
    
    def terminate(self) -> None:
        self.done = True 
        self.rgbd_camera.terminate() 
        print("RGBD Camera terminated") 

    def init(self) -> None: 
        self.rgbd_camera = Camera3D(
            mode='RGB&DEPTH', 
            frame_width_rgb=640, 
            frame_height_rgb=480, 
            frame_rate_rgb=30.0, 
            frame_width_depth=640, 
            frame_height_depth=480, 
			frame_rate_depth=15.0, 
            device_id="0@tcpip://localhost:18965", 
        )

        print("RGBD Camera activated")

    def run(self) -> None:
        try: 
            self.init() 

            while not self.done: 
                self.counter += 1 
                if self.counter < 30: 
                    continue # skip the first 10 frames 

                if self.counter % 3 == 0: # 33 Hz 
                    self.rgbd_camera.read_RGB() 
                    cv2.imshow("RGBD Raw", self.rgbd_camera.image_buffer_rgb) 

                    # self.rgbd_camera.read_depth() 
                    # cv2.imshow("Depth", self.rgbd_camera.image_buffer_depth_px)

                if cv2.waitKey(int(1)) & 0xFF == ord('m'):
                    break 
        except Exception as e: 
            print(e) 

if __name__ == "__main__": 
    vrc = VirtualRGBDCamera(1)
    vrc.run() 
