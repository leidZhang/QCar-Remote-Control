import numpy as np

from turtle import forward
from quanser.hardware import HIL, HILError, PWMMode, MAX_STRING_LENGTH, Clock
from lib_utilities import Filter, Other 

saturate = Other.saturate

class QCar():
    #region: Buffers
    # Throttle Write Only - PWM channel 0 is mtr cmd
    write_pwm_channel_throttle = np.array([0], dtype=np.int32)
    write_pwm_buffer_throttle = np.array([0], dtype=np.float64)

    # Steering Write Only - Other channel 1000 is steering cmd
    write_other_channel_steering = np.array([1000], dtype=np.int32)
    write_other_buffer_steering = np.array([0], dtype=np.float64)

    # LEDs Write Only - Other channel 11000:11003 + 11008:11011 are LEDs
    write_other_channels_LEDs = np.array([11008, 11009, 11010, 11011, 11000, 11001, 11002, 11003], dtype=np.int32)
    write_other_buffer_LEDs = np.zeros(8, dtype=np.float64)
    
    # User LEDs Write Only - Other channel 11004:11007 are User LEDs
    write_other_channels_usr_LEDs = np.array([11004, 11005, 11006, 11007], dtype=np.int32)
    write_other_buffer_usr_LEDs = np.zeros(4, dtype=np.float64)

    # Steering and LEDs Write - Other channel 1000 is steering cmd and 11000:11003 + 11008:11011 are LEDs
    write_other_channels_str_LEDs = np.array([1000, 11008, 11009, 11010, 11011, 11000, 11001, 11002, 11003], dtype=np.int32)
    write_other_buffer_str_LEDs = np.append(np.array([0], dtype=np.float64), np.zeros(8, dtype=np.float64))

    # Initialize return arrays
    mtr_current, bat_voltage = np.zeros(2, dtype=np.float64)
    mtr_encoder = np.zeros(1, dtype=np.int32)
    accelerometer = np.zeros(3, dtype=np.float64)
    gyroscope = np.zeros(3, dtype=np.float64)

    # Battery Read Only - Analog channel 6 is battery voltage
    read_analog_channels_battery = np.array([6], dtype=np.int32)
    read_analog_buffer_battery = np.zeros(1, dtype=np.float64)

    # Encoder Read Only - Encoder channel 0 is throttle motor encoder
    read_encoder_channels_throttle = np.array([0], dtype=np.int32)
    read_encoder_buffer_throttle = np.zeros(1, dtype=np.int32)

    # Gyroscope Read Only - Other channels 3000:3002 are for gyroscope 
    read_other_channels_gyroscope = np.array([3000, 3001, 3002, 14000], dtype=np.int32)
    read_other_buffer_gyroscope = np.zeros(4, dtype=np.float64)

    # Accelerometer Read Only - Other channels 4000:4002 are for accelerometer 
    read_other_channels_accelerometer = np.array([4000, 4001, 4002], dtype=np.int32)
    read_other_buffer_accelerometer = np.zeros(3, dtype=np.float64)

    # IMU Read - Other channels 3000:3002 + 4000:4002 are for IMU 
    read_other_channels_IMU = np.array([3000, 3001, 3002, 4000, 4001, 4002], dtype=np.int32)
    read_other_buffer_IMU = np.zeros(6, dtype=np.float64)

    # Power Read - Analog channels 5, 6 are motor current and battery voltage
    read_analog_channels_power = np.array([5, 6], dtype=np.int32)
    read_analog_buffer_power = np.zeros(2, dtype=np.float64)
    #endregion

    def __init__(self, hardware=0):
        ''' This function configures and initializes the QCar. Set hardware = 1 if not using the Virtual QCar.'''

        self.card = HIL()
        self.hardware = hardware

        self.wheel_radius = 0.0342
        self.pin_to_spur_ratio = ( (13.0*19.0) / (70.0*37.0) ) # ( diff pinion * pinion )  /  ( spur * diff spur )
        self.encoder_counts = 720.0
        self.wheel_base = 0.256 # front to rear wheel distance
        self.wheel_track = 0.17 # left to right wheel distance

        try:
            if self.hardware:
                board_identifier = "0"
            else:
                board_identifier = "0@tcpip://localhost:18960?nagle='off'"

            # Open the Card
            self.card.open("qcar", board_identifier)


            if self.card.is_valid():
                # Set PWM mode (duty cycle) and frequency
                self.card.set_pwm_mode(np.array([0], dtype=np.uint32), len(np.array([0], dtype=np.uint32)), np.array([PWMMode.DUTY_CYCLE], dtype=np.int32))
                self.card.set_pwm_frequency(np.array([0], dtype=np.uint32), len(np.array([0], dtype=np.uint32)), np.array([60e6/4096], dtype=np.float64))
                board_specific_options = "steer_bias=0;motor_limit=0.3;"
                
                # Set Motor coast to 0            
                self.card.write_digital(np.array([40], dtype=np.uint32), len(np.array([40], dtype=np.uint32)), np.zeros(len(np.array([0], dtype=np.uint32)), dtype=np.float64))
                self.card.set_card_specific_options(board_specific_options, MAX_STRING_LENGTH)
                
                # Set Encoder Quadrature 
                encoder_channels = np.array([0], dtype=np.uint32)
                num_encoder_channels = len(encoder_channels)                     
                self.card.set_encoder_quadrature_mode(encoder_channels, num_encoder_channels, np.array([4], dtype=np.uint32))
                self.card.set_encoder_filter_frequency(encoder_channels, num_encoder_channels, np.array([60e6/1], dtype=np.uint32))         
                self.card.set_encoder_counts(encoder_channels, num_encoder_channels, np.zeros(1, dtype=np.int32))
                    
                print('QCar configured successfully.')
        
        except HILError as h:
            print(h.get_error_message())  

    def terminate(self):
        ''' This function terminates the QCar card after setting final values for throttle, steering and LEDs.'''
        
        # PWM channel 0 is mtr cmd
        pwm_channels = np.array([0], dtype=np.int32)
        pwm_buffer = np.zeros(1, dtype=np.float64)

        # Other channel 1000 is steering, 11008:11011 are 4x indicators, and 11000:11003 are 4x lamps  
        other_channels = np.array([1000, 11000, 11001, 11002, 11003, 11008, 11009, 11010, 11011], dtype=np.int32)
        other_buffer = np.zeros(9, dtype=np.float64)

        try:    
            self.card.write(None, 0, pwm_channels, len(pwm_channels), None, 0, other_channels, len(other_channels), None, pwm_buffer, None, other_buffer)    
            self.card.close()
            
        except HILError as h:
            print(h.get_error_message())

    def read_encoder(self):
        '''Use this to read encoder counts \n

        OUTPUTS:
        mtr_encoder - throttle motor encoder measurement'''
    
        try:
            if True:
                #Reads: Analog Channel, Num Analog Channel, PWM Channel, Num PWM Channel, Digital Channel, Num Digital Channel, Other Channel, Num Other Channel, Analog Buffer, PWM Buffer, Digital Buffer, Other Buffer
                self.card.read(None, 0, self.read_encoder_channels_throttle, 1, None, 0, None, 0, 
                                None, self.read_encoder_buffer_throttle, None, None)

        except HILError as h:
            print(h.get_error_message())

        finally:
            return self.read_encoder_buffer_throttle[0]

    def read_gyroscope(self):
        '''Use this to read the gyroscope \n
        
        OUTPUTS:
        gyroscope - gyroscopic measurement'''
    
        try:
            if True:
                #Reads: Analog Channel, Num Analog Channel, PWM Channel, Num PWM Channel, Digital Channel, Num Digital Channel, Other Channel, Num Other Channel, Analog Buffer, PWM Buffer, Digital Buffer, Other Buffer
                self.card.read(None, 0, None, 0, None, 0, self.read_other_channels_gyroscope, 3, 
                                None, None, None, self.read_other_buffer_gyroscope)

        except HILError as h:
            print(h.get_error_message())

        finally:
            self.read_other_buffer_IMU[0:3] = self.read_other_buffer_gyroscope[0:3]
            return self.read_other_buffer_gyroscope

    def read_accelerometer(self):
        '''Use this to read the accelerometer \n

        OUTPUTS:
        accelerometer - accelerometer measurement'''
    
        try:
            if True:
                #Reads: Analog Channel, Num Analog Channel, PWM Channel, Num PWM Channel, Digital Channel, Num Digital Channel, Other Channel, Num Other Channel, Analog Buffer, PWM Buffer, Digital Buffer, Other Buffer
                self.card.read(None, 0, None, 0, None, 0, self.read_other_channels_accelerometer, 3, 
                                None, None, None, self.read_other_buffer_accelerometer)

        except HILError as h:
            print(h.get_error_message())

        finally:
            self.read_other_buffer_IMU[3:6] = self.read_other_buffer_accelerometer
            return self.read_other_buffer_accelerometer

    def read_IMU(self):
        '''Use this to read the IMU (gyroscope and accelerometer) \n

        OUTPUTS:
        gyroscope - gyroscopic measurement
        accelerometer - accelerometer measurement'''
    
        try:
            if True:
                #Reads: Analog Channel, Num Analog Channel, PWM Channel, Num PWM Channel, Digital Channel, Num Digital Channel, Other Channel, Num Other Channel, Analog Buffer, PWM Buffer, Digital Buffer, Other Buffer
                self.card.read(None, 0, None, 0, None, 0, self.read_other_channels_IMU, 6, 
                                None, None, None, self.read_other_buffer_IMU)

        except HILError as h:
            print(h.get_error_message())

        finally:
            self.read_other_buffer_gyroscope = self.read_other_buffer_IMU[0:3]
            self.read_other_buffer_accelerometer = self.read_other_buffer_IMU[3:6]
            return self.read_other_buffer_gyroscope, self.read_other_buffer_gyroscope 

    def read_power(self):
        '''Use this to read the motor current and battery voltage \n

        OUTPUTS:
        mtr_current - throttle motor current measurement
        bat_voltage - battery voltage measurement'''
    
        try:
            if True:
                #Reads: Analog Channel, Num Analog Channel, PWM Channel, Num PWM Channel, Digital Channel, Num Digital Channel, Other Channel, Num Other Channel, Analog Buffer, PWM Buffer, Digital Buffer, Other Buffer
                self.card.read(self.read_analog_channels_power, 2, None, 0, None, 0, None, 0, 
                                self.read_analog_buffer_power, None, None, None)

        except HILError as h:
            print(h.get_error_message())

        finally:
            self.read_analog_buffer_battery = self.read_analog_buffer_power[1]
            return self.read_analog_buffer_power[0], self.read_analog_buffer_battery

    def read_std(self):
        '''Use this to read the motor current, battery voltage and encoder counts \n
        
        OUTPUTS:
        mtr_current - throttle motor current measurement
        bat_voltage - battery voltage measurement
        mtr_encoder - throttle motor encoder measurement'''
        
        # IO
        try:
            if True:
                #Reads: Analog Channel, Num Analog Channel, PWM Channel, Num PWM Channel, Digital Channel, Num Digital Channel, Other Channel, Num Other Channel, Analog Buffer, PWM Buffer, Digital Buffer, Other Buffer
                self.card.read(self.read_analog_channels_power, 2, self.read_encoder_channels_throttle, 1, None, 0, None, 0, 
                                self.read_analog_buffer_power, self.read_encoder_buffer_throttle, None, None)

        except HILError as h:
            print(h.get_error_message())

        finally:
            self.read_analog_buffer_battery = self.read_analog_buffer_power[1]
            return self.read_analog_buffer_power[0], self.read_analog_buffer_power[1], self.read_encoder_buffer_throttle[0]

    def write_mtrs(self, mtr_cmd):
        '''Use this to write motor commands\n
        INPUTS:
        mtr_cmd - numpy 1x2 array of throttle (%) and steering (rad) motor commands. '''
    
        self.write_pwm_buffer_throttle[0] = -saturate(mtr_cmd[0], 0.2, -0.2)
        self.write_other_buffer_steering[0] = -saturate(mtr_cmd[1], 0.5, -0.5)
        
        try:
            if True:
                #Writes: Analog Channel, Num Analog Channel, PWM Channel, Num PWM Channel, Digital Channel, Num Digital Channel, Other Channel, Num Other Channel, Analog Buffer, PWM Buffer, Digital Buffer, Other Buffer           
                self.card.write(None, 0, self.write_pwm_channel_throttle, 1, None, 0, self.write_other_channel_steering, 1, 
                                None, self.write_pwm_buffer_throttle, None, self.write_other_buffer_steering)

        except HILError as h:
            print(h.get_error_message())

    def write_LEDs(self, LEDs):
        '''Use this to write LED commands\n
        INPUTS:
        LEDs - numpy 1x8 array of 4x indicators (0, 1) and 4x lamps (0, 1)'''

        self.write_other_buffer_LEDs = LEDs

        try:
            if True:
                #Writes: Analog Channel, Num Analog Channel, PWM Channel, Num PWM Channel, Digital Channel, Num Digital Channel, Other Channel, Num Other Channel, Analog Buffer, PWM Buffer, Digital Buffer, Other Buffer           
                self.card.write(None, 0, None, 0, None, 0, self.write_other_channels_LEDs, 8, 
                                None, None, None, self.write_other_buffer_LEDs)
                
        except HILError as h:
            print(h.get_error_message())

    def write_usr_LEDs(self, LEDs):
        '''Use this to write user LED commands\n
        INPUTS:
        LEDs - numpy 1x4 array of 4x LEDs'''

        self.write_other_buffer_usr_LEDs = LEDs

        try:
            if True:
                #Writes: Analog Channel, Num Analog Channel, PWM Channel, Num PWM Channel, Digital Channel, Num Digital Channel, Other Channel, Num Other Channel, Analog Buffer, PWM Buffer, Digital Buffer, Other Buffer           
                self.card.write(None, 0, None, 0, None, 0, self.write_other_channels_usr_LEDs, 4, 
                                None, None, None, self.write_other_buffer_usr_LEDs)
                
        except HILError as h:
            print(h.get_error_message())

    def write_std(self, mtr_cmd, LEDs):
        '''Use this to write motor and LED commands\n
        INPUTS:
        mtr_cmd - numpy 1x2 array of throttle (%) and steering (rad) motor commands. 
        LEDs - numpy 1x8 array of 4x indicators (0, 1) and 4x lamps (0, 1)'''
    
        self.write_pwm_buffer_throttle[0] = -saturate(mtr_cmd[0], 0.2, -0.2)
        self.write_other_buffer_str_LEDs[0] = -saturate(mtr_cmd[1], 0.5, -0.5)
        self.write_other_buffer_str_LEDs[1:9] = LEDs
        
        try:
            if True:
                #Writes: Analog Channel, Num Analog Channel, PWM Channel, Num PWM Channel, Digital Channel, Num Digital Channel, Other Channel, Num Other Channel, Analog Buffer, PWM Buffer, Digital Buffer, Other Buffer           
                self.card.write(None, 0, self.write_pwm_channel_throttle, 1, None, 0, self.write_other_channels_str_LEDs, 9, 
                                None, self.write_pwm_buffer_throttle, None, self.write_other_buffer_str_LEDs)

        except HILError as h:
            print(h.get_error_message())

    def read_write_std(self, mtr_cmd, LEDs):
        '''Use this to write motor and LED commands, and read the battery voltage, motor current and encoder counts \n

        INPUTS:
        mtr_cmd - numpy 1x2 array of throttle (%) and steering (rad) motor commands. 
        LEDs - numpy 1x8 array of 4x indicators (0, 1) and 4x lamps (0, 1)

        OUTPUTS:
        mtr_current - throttle motor current measurement
        bat_voltage - battery voltage measurement
        mtr_encoder - throttle motor encoder measurement'''
        
        self.write_pwm_buffer_throttle[0] = -saturate(mtr_cmd[0], 0.2, -0.2)
        self.write_other_buffer_str_LEDs[0] = -saturate(mtr_cmd[1], 0.5, -0.5)
        self.write_other_buffer_str_LEDs[1:9] = LEDs
        
        # print(self.write_pwm_channel_throttle)
        print(mtr_cmd) 
        # IO
        try:
            if True:
                #Writes: Analog Channel, Num Analog Channel, PWM Channel, Num PWM Channel, Digital Channel, Num Digital Channel, Other Channel, Num Other Channel, Analog Buffer, PWM Buffer, Digital Buffer, Other Buffer           
                self.card.write(None, 0, self.write_pwm_channel_throttle, 1, None, 0, self.write_other_channels_str_LEDs, 9, 
                                None, self.write_pwm_buffer_throttle, None, self.write_other_buffer_str_LEDs)

                #Reads: Analog Channel, Num Analog Channel, PWM Channel, Num PWM Channel, Digital Channel, Num Digital Channel, Other Channel, Num Other Channel, Analog Buffer, PWM Buffer, Digital Buffer, Other Buffer
                self.card.read(self.read_analog_channels_power, 2, self.read_encoder_channels_throttle, 1, None, 0, None, 0, 
                                self.read_analog_buffer_power, self.read_encoder_buffer_throttle, None, None)

        except HILError as h:
            print(h.get_error_message())

        finally:
            self.read_analog_buffer_battery = self.read_analog_buffer_power[1]
            return self.read_analog_buffer_power[0], self.read_analog_buffer_battery, self.read_encoder_buffer_throttle[0]

    def estimate_speed(self, enc_speed, steering):
        '''This function contains the out-of-the-box mapping from encoder speed (counts/s) to the longitudonal 
        speed and turning rate of the QCar. 

        Inputs:
        enc_speed - encoder speed in counts/s
        steering - current steering value

        Outputs:
        speed - numpy vector of longitudonal car speed in m/s and turning rate in rad/s'''

        # Calculate motor and axle speeds (rad/s)
        motor_speed = 2*np.pi * (1/self.encoder_counts/4) * enc_speed
        axle_speed = self.pin_to_spur_ratio * motor_speed
        
        # Calculate front and rear wheel speeds (m/s)
        rear_wheel_speed = self.wheel_radius * 2 * axle_speed / (1 + np.math.cos(steering))
        forward_wheel_speed = self.wheel_radius * 2 * axle_speed * np.math.cos(steering) / (1 + np.math.cos(steering))
        
        # Calculate the inverse turning radius, as well as angular speed of the vehicle
        inverse_turning_radius = np.math.tan(steering) / self.wheel_base # 1 / R
        angular_speed = inverse_turning_radius * rear_wheel_speed

        # Return the vehicle forward speed (rear_wheel_speed) as well as angular turning rate (angular_speed)
        speed = np.array([rear_wheel_speed, angular_speed]) # [m/s, rad/s]

        return speed

    def update_position(self, dt, x=0, y=0, heading=0):
        while True:
            forward_speed, angular_speed, dt = yield x, y, heading
            heading = heading + angular_speed * dt
            x = x + forward_speed * np.math.cos(heading) * dt
            y = y + forward_speed * np.math.sin(heading) * dt

    def complementary_filter(self, dt, alpha, estimate=0):
        estimate = 0

        while True:
            correction, rate = yield estimate
            estimate = (1-alpha)*(correction + rate * dt) + alpha * correction   

    def heading_from_gps(self, dt, prev_position=np.zeros((3)), prev_heading=0):
        heading = 0

        while True:
            position, forward_speed = yield heading 
            if (abs(position[0] - prev_position[0]) < 0.01) and (abs(position[1] - prev_position[1]) < 0.01):
                heading = prev_heading
            else:
                if forward_speed >= 0:
                    heading = np.math.atan2((position[1] - prev_position[1]), (position[0] - prev_position[0]))
                else:
                    heading = np.math.atan2(-(position[1] - prev_position[1]), -(position[0] - prev_position[0]))
            prev_position = position
            prev_heading = heading

    def pose_estimator(self, stop_threshold, dt, pose_0=np.array([0, 0, 0], dtype=np.float64)):
        ''' stop_threshold = encoder speed below which the car can be considered stationary
            dt = timestep'''

        # initialize the pose output yield to the initial condition provided
        heading_est = self.heading_from_gps(dt, prev_position = np.array([pose_0[0], pose_0[1], 0]), prev_heading=pose_0[2])
        _ = next(heading_est)
        pose = pose_0
        prev_gps = np.array([pose_0[0], pose_0[1], 0])
        while True:
            gps, compass, steering, gyroscope, enc_speed, forward_speed = yield pose
            # Calculate motor and axle speeds (rad/s)
            motor_speed = 2*np.pi * (1/(self.encoder_counts*4)) * enc_speed
            axle_speed = self.pin_to_spur_ratio * motor_speed
            
            # Calculate front and rear wheel speeds (m/s)
            car_speed = self.wheel_radius * 2 * axle_speed / (1 + np.math.cos(steering))

            # If the LIDAR correction is not different, or if the car has stopped moving, then use bicycle model
            if prev_gps[0] == gps[0]: # or enc_speed < stop_threshold:
                new = False
                # print('No data available')
            else:
                new = True
                # print('Correction available')
            
            if new:
                # a new pose correction is available, just use it.
                alpha = 0.35
                pose[0] = alpha * pose[0] + (1-alpha) * gps[0]
                pose[1] = alpha * pose[1] + (1-alpha) * gps[1]
                pose[2] = heading_est.send((np.array([pose[0], pose[1], gps[2]]), forward_speed))
                alpha = 0
                pose[2] = alpha * pose[2] + (1-alpha) * compass[2]
            else:
                # a new pose correction is not available, build upon the previous one using the bicycle model
                
                # update the pose_yaw using gyroscopic rate correction
                pose[2] = pose[2] + gyroscope[2] * dt

                # use the bicycle model to project the car speed to pose rate correction in x and y
                net_angle = np.math.atan( np.math.tan(steering) * 0.5 * self.wheel_base / self.wheel_base ) + pose[2]

                # update the pose_x and pose_y using pose_rate correction
                pose[0] = pose[0] + car_speed * np.math.cos(net_angle) * dt
                pose[1] = pose[1] + car_speed * np.math.sin(net_angle) * dt

            # store the previous pose correction
            prev_gps = gps

    def calculate_front_axle_position(self, pose_est, factor=1):
        '''Calculate the front axle centre position from pose_est. Factor is a percentage (0 to 1). Use it to determine how forward you want to go w.r.t. the back axle.'''
        front_x = pose_est[0] + factor * self.wheel_track * np.math.cos(pose_est[2])
        front_y = pose_est[1] + factor * self.wheel_track * np.math.sin(pose_est[2])

        return np.array([front_x, front_y])

    def indicate(self, steering, mtr_cmd):
        LEDs = np.array([0, 0, 0, 0, 0, 0, 1, 1])
        if steering > 0.3:
            LEDs[0] = 1
            LEDs[2] = 1
        elif steering < -0.3:
            LEDs[1] = 1
            LEDs[3] = 1
        if mtr_cmd < 0:
            LEDs[5] = 1
        
        return LEDs

class QCarTask():
    #region: Buffers
    # Throttle Write Only - PWM channel 0 is mtr cmd
    write_pwm_channel_throttle = np.array([0], dtype=np.int32)
    write_pwm_buffer_throttle = np.array([0], dtype=np.float64)

    # Steering and LEDs Write - Other channel 1000 is steering cmd and 11000:11003 + 11008:11011 are LEDs
    write_other_channels_str_LEDs = np.array([1000, 11008, 11009, 11010, 11011, 11000, 11001, 11002, 11003], dtype=np.int32)
    write_other_buffer_str_LEDs = np.append(np.array([0], dtype=np.float64), np.zeros(8, dtype=np.float64))

    # Initialize return arrays
    mtr_current, bat_voltage = np.zeros(2, dtype=np.float64)
    mtr_encoder = np.zeros(1, dtype=np.int32)
    accelerometer = np.zeros(3, dtype=np.float64)
    gyroscope = np.zeros(3, dtype=np.float64)

    # Encoder Read Only - Encoder channel 0 is throttle motor encoder
    read_encoder_channels_throttle = np.array([0], dtype=np.int32)
    read_encoder_buffer_throttle = np.zeros(1, dtype=np.int32)

    # Power Read - Analog channels 5, 6 are motor current and battery voltage
    read_analog_channels_power = np.array([5, 6], dtype=np.int32)
    read_analog_buffer_power = np.zeros(2, dtype=np.float64)
    #endregion

    # Gyroscope & Tachometer Read Only - Other channel 14000 is tachometer on main encoder
    read_other_channels = np.array([3000, 3001, 3002, 14000], dtype=np.int32)
    read_other_buffer = np.zeros(4, dtype=np.float64)

    def __init__(self, frequency=1000, hardware=0):
        ''' This function configures the QCar and returns a handle to the QCar card. Use the handle for other methods such as qcar_io or terminate_qcar. This class uses Task Based IO at 1000Hz by default.'''
        
        self.card = HIL()
        self.hardware = hardware
        self.wheel_radius = 0.0315
        self.pin_to_spur_ratio = ( (13.0*19.0) / (70.0*37.0) ) # ( diff pinion * pinion )  /  ( spur * diff spur )
        self.encoder_counts = 720.0
        self.wheel_base = 0.256 # front to rear wheel distance
        self.wheel_track = 0.17 # left to right wheel distance
        try:
            if self.hardware:
                board_identifier = "0"
            else:
                board_identifier = "0@tcpip://localhost:18960" #?nagle='off'
            
            # Open the Card
            self.card.open("qcar", board_identifier)
            if self.card.is_valid():
                # Set PWM mode (duty cycle) and frequency
                self.card.set_pwm_mode(np.array([0], dtype=np.uint32), len(np.array([0], dtype=np.uint32)), np.array([PWMMode.DUTY_CYCLE], dtype=np.int32))
                self.card.set_pwm_frequency(np.array([0], dtype=np.uint32), len(np.array([0], dtype=np.uint32)), np.array([60e6/4096], dtype=np.float64))
                board_specific_options = "steer_bias=0;motor_limit=0.3;"
                
                # Set Motor coast to 0            
                self.card.write_digital(np.array([40], dtype=np.uint32), len(np.array([40], dtype=np.uint32)), np.zeros(len(np.array([0], dtype=np.uint32)), dtype=np.float64))
                self.card.set_card_specific_options(board_specific_options, MAX_STRING_LENGTH)
            
                # Set Encoder Quadrature 
                encoder_channels = np.array([0], dtype=np.uint32)
                num_encoder_channels = len(encoder_channels)                     
                self.card.set_encoder_quadrature_mode(encoder_channels, num_encoder_channels, np.array([4], dtype=np.uint32))
                self.card.set_encoder_filter_frequency(encoder_channels, num_encoder_channels, np.array([60e6/1], dtype=np.uint32))         
                self.card.set_encoder_counts(encoder_channels, num_encoder_channels, np.zeros(1, dtype=np.int32))
                self.frequency = frequency
                samples_in_buffer = int(self.frequency*2)
                self.read_task = self.card.task_create_reader(samples_in_buffer, 
                        self.read_analog_channels_power, len(self.read_analog_channels_power), 
                        self.read_encoder_channels_throttle, len(self.read_encoder_channels_throttle), 
                        None, 0, 
                        self.read_other_channels, len(self.read_other_channels))
                print('QCar configured successfully...')
                self.card.task_start(self.read_task, Clock.HARDWARE_CLOCK_0, self.frequency, 2**32-1)

        except HILError as h:
            print(h.get_error_message())  

    def terminate(self):
        ''' This function terminates the QCar card after setting final values for throttle, steering and LEDs. Also terminates the task reader.'''
        
        # PWM channel 0 is mtr cmd
        pwm_channels = np.array([0], dtype=np.int32)
        pwm_buffer = np.zeros(1, dtype=np.float64)

        # Other channel 1000 is steering, 11008:11011 are 4x indicators, and 11000:11003 are 4x lamps  
        other_channels = np.array([1000, 11000, 11001, 11002, 11003, 11008, 11009, 11010, 11011], dtype=np.int32)
        other_buffer = np.zeros(9, dtype=np.float64)

        try:    
            self.card.write(None, 0, pwm_channels, len(pwm_channels), None, 0, other_channels, len(other_channels), None, pwm_buffer, None, other_buffer)    
            self.card.task_stop(self.read_task)
            self.card.close()
            
        except HILError as h:
            print(h.get_error_message())

    def read_write_std(self, mtr_cmd, LEDs):
        '''Use this to write motor and LED commands, and read the battery voltage, motor current and encoder counts \n

        INPUTS:
        mtr_cmd - numpy 1x2 array of throttle (%) and steering (rad) motor commands. 
        LEDs - numpy 1x8 array of 4x indicators (0, 1) and 4x lamps (0, 1)

        OUTPUTS:
        mtr_current - throttle motor current measurement
        bat_voltage - battery voltage measurement
        mtr_encoder - throttle motor encoder measurement'''
    
        self.write_pwm_buffer_throttle[0] = -saturate(mtr_cmd[0], 0.2, -0.2)
        self.write_other_buffer_str_LEDs[0] = -saturate(mtr_cmd[1], 0.5, -0.5)
        self.write_other_buffer_str_LEDs[1:9] = LEDs

        # IO
        try:
            if True:
                #Writes: Analog Channel, Num Analog Channel, PWM Channel, Num PWM Channel, Digital Channel, Num Digital Channel, Other Channel, Num Other Channel, Analog Buffer, PWM Buffer, Digital Buffer, Other Buffer           
                self.card.write(None, 0, self.write_pwm_channel_throttle, 1, None, 0, self.write_other_channels_str_LEDs, 9, 
                                None, self.write_pwm_buffer_throttle, None, self.write_other_buffer_str_LEDs)

                #Reads: Analog Channel, Num Analog Channel, PWM Channel, Num PWM Channel, Digital Channel, Num Digital Channel, Other Channel, Num Other Channel, Analog Buffer, PWM Buffer, Digital Buffer, Other Buffer
                #self.card.read(self.read_analog_channels_power, 2, self.read_encoder_channels_throttle, 1, None, 0, None, 0, 
                #                self.read_analog_buffer_power, self.read_encoder_buffer_throttle, None, None)
                self.card.task_read(self.read_task, 1, self.read_analog_buffer_power, self.read_encoder_buffer_throttle, None, self.read_other_buffer)
                # print('got here')
        except HILError as h:
            print(h.get_error_message())

        finally:
            self.read_analog_buffer_battery = self.read_analog_buffer_power[1]
            return self.read_analog_buffer_power[0], self.read_analog_buffer_battery, self.read_encoder_buffer_throttle[0], self.read_other_buffer[0:3], self.read_other_buffer[3]

    def estimate_speed(self, enc_speed, steering):
        '''This function contains the out-of-the-box mapping from encoder speed (counts/s) to the longitudonal 
        speed and turning rate of the QCar. 

        Inputs:
        enc_speed - encoder speed in counts/s
        steering - current steering value

        Outputs:
        speed - numpy vector of longitudonal car speed in m/s and turning rate in rad/s'''

        # Calculate motor and axle speeds (rad/s)
        motor_speed = 2*np.pi * (1/(self.encoder_counts*4)) * enc_speed
        axle_speed = self.pin_to_spur_ratio * motor_speed
        
        # Calculate front and rear wheel speeds (m/s)
        rear_wheel_speed = self.wheel_radius * 2 * axle_speed / (1 + np.math.cos(steering))
        forward_wheel_speed = self.wheel_radius * 2 * axle_speed * np.math.cos(steering) / (1 + np.math.cos(steering))
        
        # Calculate the inverse turning radius, as well as angular speed of the vehicle
        inverse_turning_radius = np.math.tan(steering) / self.wheel_base # 1 / R
        angular_speed = inverse_turning_radius * rear_wheel_speed

        # Return the vehicle forward speed (rear_wheel_speed) as well as angular turning rate (angular_speed)
        speed = np.array([rear_wheel_speed, angular_speed]) # [m/s, rad/s]

        return speed

    def pose_estimator(self, stop_threshold, dt, pose_0=np.array([0, 0, 0], dtype=np.float64)):
        ''' stop_threshold = encoder speed below which the car can be considered stationary
            dt = timestep'''

        # initialize the pose output yield to the initial condition provided
        heading_est = self.heading_from_gps(dt, prev_position = np.array([pose_0[0], pose_0[1], 0]), prev_heading=pose_0[2])
        _ = next(heading_est)
        pose = pose_0
        prev_gps = np.array([pose_0[0], pose_0[1], 0])
        while True:
            gps, compass, steering, gyroscope, enc_speed, forward_speed = yield pose
            # Calculate motor and axle speeds (rad/s)
            motor_speed = 2*np.pi * (1/(self.encoder_counts*4)) * enc_speed
            axle_speed = self.pin_to_spur_ratio * motor_speed
            
            # Calculate front and rear wheel speeds (m/s)
            car_speed = self.wheel_radius * 2 * axle_speed / (1 + np.math.cos(steering))

            # If the LIDAR correction is not different, or if the car has stopped moving, then use bicycle model
            if prev_gps[0] == gps[0]: # or enc_speed < stop_threshold:
                new = False
                # print('No data available')
            else:
                new = True
                # print('Correction available')
            
            if new:
                # a new pose correction is available, just use it.
                alpha = 0.35
                pose[0] = alpha * pose[0] + (1-alpha) * gps[0]
                pose[1] = alpha * pose[1] + (1-alpha) * gps[1]
                pose[2] = heading_est.send((np.array([pose[0], pose[1], gps[2]]), forward_speed))
                alpha = 0
                pose[2] = alpha * pose[2] + (1-alpha) * compass[2]
            else:
                # a new pose correction is not available, build upon the previous one using the bicycle model
                
                # update the pose_yaw using gyroscopic rate correction
                pose[2] = pose[2] + gyroscope[2] * dt

                # use the bicycle model to project the car speed to pose rate correction in x and y
                net_angle = np.math.atan( np.math.tan(steering) * 0.5 * self.wheel_base / self.wheel_base ) + pose[2]

                # update the pose_x and pose_y using pose_rate correction
                pose[0] = pose[0] + car_speed * np.math.cos(net_angle) * dt
                pose[1] = pose[1] + car_speed * np.math.sin(net_angle) * dt

            # store the previous pose correction
            prev_gps = gps

    def update_position(self, dt, x=0, y=0, heading=0):
        while True:
            forward_speed, angular_speed, dt = yield x, y, heading
            heading = heading + angular_speed * dt
            x = x + forward_speed * np.math.cos(heading) * dt
            y = y + forward_speed * np.math.sin(heading) * dt

    def complementary_filter(self, dt, alpha, estimate=0):

        while True:
            correction, rate = yield estimate
            estimate = (1-alpha)*(correction + rate * dt) + alpha * correction   

    def heading_from_gps(self, dt, prev_position=np.zeros((3)), prev_heading=0):
        heading = prev_heading
        counter = 0
        while True:
            position, forward_speed = yield heading 
            if np.linalg.norm(position[0:2] - prev_position[0:2], 2) < 0.0001:
                heading = prev_heading
            else:
                if forward_speed >= 0:
                    heading = np.math.atan2((position[1] - prev_position[1]), (position[0] - prev_position[0]))
                else:
                    heading = np.math.atan2(-(position[1] - prev_position[1]), -(position[0] - prev_position[0]))
            counter = counter + 1
            prev_position = position
            prev_heading = heading

    def calculate_front_axle_position(self, pose_est, factor=1):
        '''Calculate the front axle centre position from pose_est. Factor is a percentage (0 to 1). Use it to determine how forward you want to go w.r.t. the back axle.'''
        front_x = pose_est[0] + factor * self.wheel_track * np.math.cos(pose_est[2])
        front_y = pose_est[1] + factor * self.wheel_track * np.math.sin(pose_est[2])

        return np.array([front_x, front_y])

    def indicate(self, steering, mtr_cmd):
        LEDs = np.array([0, 0, 0, 0, 0, 0, 1, 1])
        if steering > 0.3:
            LEDs[0] = 1
            LEDs[2] = 1
        elif steering < -0.3:
            LEDs[1] = 1
            LEDs[3] = 1
        if mtr_cmd < 0:
            LEDs[5] = 1
        
        return LEDs