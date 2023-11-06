import sys
import numpy as np
import time
import os
import cv2
import math 

sys.path.append('dependencies/q_libs')
from numpy.core.numeric import zeros_like
from lib_qcar import QCarTask
from lib_utilities import GPS, Controllers, Camera2D, LaneDetector, RoadMap, QLabsWorkspace
from lib_utilities import Other
sys.path.append('src/test/')
from test_util import init_qcar

init_qcar(10) 

# image parameters
image_width = 820
image_height = 410
# may have problems if not on the road
focal_length = np.array([[157.9], [161.7]], dtype = np.float64)
principle_point = np.array([[168.5], [123.6]], dtype = np.float64)
c_position = np.array([[0], [0], [0.14]], dtype = np.float64)
c_orientation = np.array([[ 0, 0, 1], [ 1, 0, 0], [ 0, -1, 0]], dtype = np.float64)

# do they have a manual??? 
front_csi = Camera2D(camera_id="3@tcpip://localhost:18964", # currently, id of other cameras are unknown, f: 3
                     frame_width=image_width, 
                     frame_height=image_height, 
                     frame_rate=33.3, 
                     focal_length=focal_length, 
                     principle_point=principle_point, 
                     position=c_position, 
                     orientation=c_orientation, 
                     skew=0)
detector = LaneDetector(front_csi)
rgb_placeholder = zeros_like(front_csi.image_data)
rgb_segment = zeros_like(front_csi.image_data)

print(front_csi.url)

sample_rate = 100.0
sample_time = 1/sample_rate
my_car = QCarTask(frequency=int(sample_rate), hardware=0) 
LEDs = np.array([0, 0, 0, 0, 0, 0, 1, 1])
# v = VirtualCSICamera()
# v.init_camera()  
counter = 0
steering = 0 # need this in line detection 
while True: 
    try: 
        # Do imaging stuff here at 33 Hz (every third sample)
        if counter % 3 == 0:
            front_csi.read()
            # skip the first 10 frames
            if counter > 30:
                canny = detector.doCanny(front_csi.image_data)
                segment, mask = detector.doSegment(canny, steering)
                left_lines, right_lines = detector.calculateLines(segment)
                lineParameters, imageWithLines = detector.averageLines(front_csi.image_data, left_lines, right_lines)
                rgb_segment[:, :, 1] = mask
                rgb_placeholder[:, :, 2] = canny
                final = cv2.addWeighted(rgb_segment, 0.1, imageWithLines, 0.9, 0.0)
                final = cv2.addWeighted(rgb_placeholder, 0.2, final, 0.8, 0.0)
                cv2.imshow('ImageCanny', canny)    
                cv2.imshow('ImageWithLines', final)

            if cv2.waitKey(int(1)) & 0xFF == ord('q'):
                break
        # v.get_image(counter, steering)
        my_car.read_write_std(np.array([0.1, steering]), LEDs)

        counter += 1
    except KeyboardInterrupt as e: 
        print(e) 
        os._exit(0) 