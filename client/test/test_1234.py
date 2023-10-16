# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
#region: Imports
import sys
import numpy as np
import time
import os
import cv2
import math 

sys.path.append('../common/')
from numpy.core.numeric import zeros_like
from lib_qcar import QCarTask
from lib_utilities import GPS, Controllers, Camera2D, LaneDetector, RoadMap, QLabsWorkspace
from lib_utilities import Other
saturate = Other.saturate
#endregion
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# User Interface... 
# style = input('Are you driving on the (left) side or the (right)? :')
style = 'right' 
if style=='right':
    print('Selected right-sided driving...')
    node_map = cv2.imread('../images/Nodes_Right.png')
else:
    print('Selected left-sided driving...')
    node_map = cv2.imread('../images/Nodes_Left.png')
cv2.imshow('Roadmap: Pick a starting node and ending node index (map is not clickable)', node_map)  #  Generate GUI 
print('press any key to continue') 
cv2.waitKey(0)  # press enter on the GUI 
# stringCmd = input('Provide waypoints you wish to navigate, seperated by commas "," :')
# desiredNodes = stringCmd.split(',')  # command array [start, end]
# desiredNodes = [int(x) for x in desiredNodes]  # convert string to int 
desiredNodes = [10, 27]
start_index = desiredNodes[0]
end_index = desiredNodes[-1]
# desired_speed = float(input('Enter the desired speed (between 0.1 to 0.75 m/s):'))  # enter speed 
desired_speed = 0.5 

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
#region: HMI / Road-map 
roadMap = RoadMap(style=style)
# Generate shortest pathway
closedCircuit = roadMap.generate_waypoints(desiredNodes, factor=10)
pathway = roadMap.pathway
waypoint_list = roadMap.waypoint_list
print('Pathway from', start_index, 'to', end_index, 'is:', pathway, '...')
time.sleep(2)
#endregion
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
#region: Connect to QLabs
qlabsWorkspace = QLabsWorkspace(roadMap) # if you generate a new pathway using the roadMap.generate_waypoints() method, don't forget
                                         # to update the roadMap object in QLabsWorkspace by using the update_roadmap() method
position, orientation = qlabsWorkspace.spawnVehicle(start_index)
qlabsWorkspace.spawnRoadPoints()

#endregion
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
#region: Spawn Models
# Launch the spawn models for a joystick and QCar
relative_path = "../rt-win64/QCar_Workspace_5.rt-win64" 
file_path = os.path.abspath(relative_path)
os.startfile(file_path)
# os.startfile("QCar_Workspace_5.rt-win64")   # this has 100 Hz IO & 5 Hz non-blocking GPS
time.sleep(2)
# os.startfile("joystick.rt-win64")

#endregion
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
#region: Timing Parameters and methods 
startTime = time.time()
def elapsed_time():
    return time.time() - startTime

sampleRate = 100.0
sampleTime = 1/sampleRate
simulationTime = 100000 # some large number
print('Simulation rate is set to', sampleRate, '...')

#endregion
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
#region Image Parameters
image_width = 820
image_height = 410
focal_length = np.array([[157.9], [161.7]], dtype = np.float64)
principle_point = np.array([[168.5], [123.6]], dtype = np.float64)
c_position = np.array([[0], [0], [0.14]], dtype = np.float64)
c_orientation = np.array([[ 0, 0, 1], [ 1, 0, 0], [ 0, -1, 0]], dtype = np.float64)
frontCSI = Camera2D(camera_id="3@tcpip://localhost:18964", frame_width=image_width, frame_height=image_height, frame_rate=33.3, focal_length=focal_length, principle_point=principle_point, position=c_position, orientation=c_orientation, skew=0)
detector = LaneDetector(frontCSI)
rgb_placeholder = zeros_like(frontCSI.image_data)
rgb_segment = zeros_like(frontCSI.image_data)

#endregion
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
#region: All the objects, filters, control parameters
# myCar = QCar(hardware=0)
# _, _, _ = myCar.read_write_std(np.array([0, 0]), np.array([0, 0, 0, 0, 0, 0, 1, 1]))
# myCar.terminate()
myCar       = QCarTask(frequency=int(sampleRate), hardware=0)  # "QCar configured successfully..."
print("pass myCar")
gps         = GPS('tcpip://localhost:18967')  
print("pass gps") 
print(gps.read())
# joystick    = GamePad('tcpip://localhost:18872')
flag = False
while not flag: # a dead loop here 
    flag = gps.read()
x_0     = gps.position[0]
y_0     = gps.position[1]
theta_0 = (np.pi/180)*orientation[2] # decide orientation? 
index = 1
next_waypoint = waypoint_list[1]
prev_waypoint = waypoint_list[0]

# Set up integrators, complementary filters, estimators, etc. and increment them (they are generators)
comp_filter_x = myCar.complementary_filter(sampleTime, 0.02, estimate=x_0)
comp_filter_y = myCar.complementary_filter(sampleTime, 0.02, estimate=y_0)
comp_filter_theta = myCar.complementary_filter(sampleTime, 0.02, estimate=theta_0)
heading_est = myCar.heading_from_gps(sampleTime, prev_position = gps.position, prev_heading=theta_0)
myController = Controllers(myCar, kp_f=0.35, ki_f=0.0, kd_f=0, kf_f=0.273, k_l=2.0)
lateral_controller = myController.lateral_controller()
longitudonal_controller = myController.longitudonal_controller(sampleTime)
pedal_model = myController.pedal_model

_ = next(heading_est)
x_est = next(comp_filter_x)
y_est = next(comp_filter_y)
theta_est = next(comp_filter_theta)
_ = next(lateral_controller)
_ = next(longitudonal_controller)
 
startTime = time.time()
if desired_speed > 0.75:
    print('Is that you Paul? This is not a Ferrari! Max speed limited to 0.75 m/s...')
    desired_speed = saturate(desired_speed, 0.75, -0.75)

forward_speed = desired_speed
counter = 0

np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})
#endregion
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
#region: Main Loop

# mapping state to QCar instance as well as camera data 
qlabsWorkspace.setCameraToVehicle()

try:
    prev = elapsed_time() - sampleTime

    # while elapsed_time() < simulationTime:
    while True:
        # Start timing this iteration
        start = elapsed_time()
        delta = start - prev # delta = true timestep
        
        # # Do imaging stuff here at 33 Hz (every third sample)
        if counter % 3 == 0:
            frontCSI.read()
            # skip the first 10 frames
            if counter > 30:
                canny = detector.doCanny(frontCSI.image_data)
                segment, mask = detector.doSegment(canny, steering)
                left_lines, right_lines = detector.calculateLines(segment)
                lineParameters, imageWithLines = detector.averageLines(frontCSI.image_data, left_lines, right_lines)
                rgb_segment[:, :, 1] = mask
                rgb_placeholder[:, :, 2] = canny
                final = cv2.addWeighted(rgb_segment, 0.1, imageWithLines, 0.9, 0.0)
                final = cv2.addWeighted(rgb_placeholder, 0.2, final, 0.8, 0.0)
                cv2.imshow('ImageCanny', canny)    
                cv2.imshow('ImageWithLines', final)

            if cv2.waitKey(int(1)) & 0xFF == ord('q'):
                break

        # If delta is zero, you never iterated, so skip the iteration manually
        if delta == 0.0:
            continue        

        # check if you are close enough to the next waypoint, and if so, increment
        # print(x_est, myCar.wheel_track, theta_est)
        front_x = x_est + myCar.wheel_track * math.cos(theta_est)
        front_y = y_est + myCar.wheel_track * math.sin(theta_est)
        if np.linalg.norm(next_waypoint[0:2] - np.array([front_x, front_y]), 2) < 0.1: # this should be 0.1
            if index == len(waypoint_list) - 1: # if you hit the last waypoint, quit the loop.
                if closedCircuit == True:
                    index = 0
                else:
                    print('Reached the end')
                    break            
            next_waypoint = waypoint_list[index + 1]
            prev_waypoint = waypoint_list[index]            
            index = index + 1            

        # Lateral Stanley Controller
        steering_delta = lateral_controller.send((next_waypoint, prev_waypoint, np.array([x_est, y_est, theta_est]), forward_speed)) # expect 4 variables  
        # Longitudonal PID+FF controller
        commanded_speed = ((math.cos(np.sqrt(abs(steering_delta))))**4)*desired_speed
        acceleration, braking = longitudonal_controller.send((commanded_speed, forward_speed))
        # Grab commands from pedal values
        # mtr_cmd, steering = pedal_model(acceleration, braking, steering_delta)
        throttle, steering = pedal_model(acceleration, braking, steering_delta)

        # Adjust LED indicators based on steering and reverse indicators based on reverse gear
        LEDs = np.array([0, 0, 0, 0, 0, 0, 1, 1])
        if steering > 0.3:
            LEDs[0] = 1
            LEDs[2] = 1
        elif steering < -0.3:
            LEDs[1] = 1
            LEDs[3] = 1
        if throttle < 0:
            LEDs[5] = 1
    
        status = myCar.read_write_std(np.array([0, 0]), LEDs)     
        mtr_encoder = status[2] 
        
        # estimate vehicle speed, and then fuse with GPS data using complementary filters
        speed = myCar.estimate_speed(mtr_encoder, steering)
        forward_speed = speed[0]
        angular_speed = speed[1]
        print("dir: ", steering, "spd: ", forward_speed)
        # GPS data is incoming at 100 Hz only (every 5 samples at 500Hz)
        # if counter % 10 == 0: 
        flag = gps.read()
        
        if flag:
            print(gps.position, gps.orientation)
            theta = heading_est.send((gps.position, forward_speed))
            theta_est = comp_filter_theta.send((theta, angular_speed))
            x_est = comp_filter_x.send((gps.position[0], forward_speed * math.cos(theta_est)))
            y_est = comp_filter_y.send((gps.position[1], forward_speed * math.sin(theta_est)))
        else:
            theta_est = comp_filter_theta.send((theta_est, angular_speed))
            x_est = comp_filter_x.send((x_est, forward_speed * math.cos(theta_est)))
            y_est = comp_filter_y.send((y_est, forward_speed * math.sin(theta_est)))
        end = elapsed_time()
        prev = start
        counter = counter + 1
        
# except KeyboardInterrupt:
#     print("User interrupted!")

except Exception as e: 
    print(e) 

# finally:    
#     myCar.terminate()
#     gps.terminate()
#     qlabsWorkspace.terminate()
#endregion
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
