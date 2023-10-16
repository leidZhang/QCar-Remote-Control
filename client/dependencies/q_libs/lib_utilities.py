
from library_qlabs_trafficlight_single import QLabsTrafficLightSingle
from library_qlabs_silhouette_person import QLabsSilhouettePerson
from library_qlabs_crosswalk import QLabsCrosswalk
from library_qlabs import QuanserInteractiveLabs
from library_qlabs_basic_shape import QLabsBasicShape
from library_qlabs_qcar import QLabsQCar
from quanser.communications import Stream, StreamError, PollFlag, Timeout
from quanser.multimedia import Video3D, VideoCapture, Video3DStreamType, MediaError, ImageFormat, ImageDataType
from quanser.devices import RPLIDAR, RangingMeasurements, RangingMeasurementMode, DeviceError, RangingDistance, GameController
import cv2
import numpy as np
from scipy.sparse import csr, csr_matrix
from scipy.sparse.csgraph import shortest_path
import time
import os


class BasicStream:
	'''Class object consisting of basic stream server/client functionality'''
	def __init__(self, uri, agent='s', send_buffer_size=2048, recv_buffer_size=2048, non_blocking=False):
		"""
		This functions simplifies functionality of the quanser_stream module to provide a 
		simple server or client. \n \n

		INPUTS: \n
		uri - IP server and port in one string, eg. 'tcpip://IP_ADDRESS:PORT' \n
		agent - 's' or 'c' string representing server or client respectively
		send_buffer_size - (optional) size of send buffer, default is 2048 \n
		recv_buffer_size - (optional) size of recv buffer, default is 2048 \n

		"""
		self.agent = agent
		self.send_buffer_size = send_buffer_size
		self.recv_buffer_size = recv_buffer_size
		self.uri = uri
		
		# If the agent is a Client, then Server isn't needed. 
		# If the agent is a Server, a Client will also be needed. The server can start listening immediately.
		
		self.clientStream = Stream()
		if agent=='s':
			self.serverStream = Stream()
			
		# Set polling timeout to 1 second, and initialize counter for polling connections         
		self.t_out = Timeout(seconds=0, nanoseconds=10000000)
		# counter = 0

		# connected flag initialized to False
		self.connected = False

		try:
			if agent == 'c':
				self.connected = self.clientStream.connect(uri, non_blocking, self.send_buffer_size, self.recv_buffer_size)
	   
			elif agent == 's':
				self.serverStream.listen(self.uri, non_blocking)
			pass

		except StreamError as e:
			if self.agent == 's':
				print('Server initialization failed.')
			elif self.agent == 'c':
				print('Client initialization failed.')
			print(e.get_error_message())

	def checkConnection(self, t_out_local=Timeout(seconds=0, nanoseconds=10000000)):

		if self.agent == 'c' and not self.connected:
			try:

				poll_result = self.clientStream.poll(t_out_local, PollFlag.CONNECT)
								
				if (poll_result & PollFlag.CONNECT) == PollFlag.CONNECT:
					self.connected = True
					print('Connected to the Server successfully.')            

			except StreamError as e:
				if e.error_code == -33:
					self.connected = self.clientStream.connect(self.uri, True, self.send_buffer_size, self.recv_buffer_size)
				else:
					print('Client initialization failed.')
					print(e.get_error_message())

		if self.agent == 's' and not self.connected:
			try:
				poll_result = self.serverStream.poll(self.t_out, PollFlag.ACCEPT)
				if (poll_result & PollFlag.ACCEPT) == PollFlag.ACCEPT:
					self.connected = True
					print('Found a Client successfully...')
					self.clientStream = self.serverStream.accept(self.send_buffer_size, self.recv_buffer_size)

			except StreamError as e:
				print('Server initialization failed...')
				print(e.get_error_message())

	def terminate(self):
		if self.connected:
			self.clientStream.shutdown()
			self.clientStream.close()
			print('Successfully terminated clients...')

		if self.agent == 's':
			self.serverStream.shutdown()
			self.serverStream.close()
			print('Successfully terminated servers...')

	def receive(self, buffer, iterations=1000, timeout=Timeout(seconds=1, nanoseconds=0)):
		"""
		This functions receives a numpy buffer object that it will fill with bytes if available. \n \n

		INPUTS: \n
		buffer -  numpy float32 array  \n
		iterations - (optional) number of times to poll for incoming data before terminating, default is 1000 \n 

		OUTPUTS: \n
		buffer - data received \n
		bytes_received - number of bytes received \n
		"""
		
		self.t_out = timeout
		counter = 0
		dataShape = buffer.shape

		# Find number of bytes per array cell based on type
		numBytesBasedOnType = len(np.array([0], dtype=buffer.dtype).tobytes())

		# Calculate total dimensions
		dim = 1
		for i in range(len(dataShape)):
			dim = dim*dataShape[i]
		
		# Calculate total number of bytes needed and set up the bytearray to receive that
		totalNumBytes = dim*numBytesBasedOnType
		self.data = bytearray(totalNumBytes)
		self.bytes_received = 0        

		# Poll to see if data is incoming, and if so, receive it. Poll a max of 'iteration' times
		try:
			while True:

				# See if data is available
				poll_result = self.clientStream.poll(self.t_out, PollFlag.RECEIVE)
				counter += 1
				if not (iterations == 'Inf'):
					if counter >= iterations:
						break        
				if not ((poll_result & PollFlag.RECEIVE) == PollFlag.RECEIVE):
					continue # Data not available, skip receiving

				# Receive data
				self.bytes_received = self.clientStream.receive(self.data, totalNumBytes)
				
				# data received, so break this loop
				break 

			#  convert byte array back into numpy array and reshape.
			buffer = np.reshape(np.frombuffer(self.data, dtype=buffer.dtype), dataShape) 

		except StreamError as e:
			print(e.get_error_message())
		finally:
			return buffer, self.bytes_received

	def send(self, buffer):
		"""
		This functions sends the data in the numpy array buffer
		(server or client). \n \n

		INPUTS: \n
		buffer - numpy array of data to be sent \n

		OUTPUTS: \n
		bytesSent - number of bytes actually sent (-1 if send failed) \n
		"""

		# Set up array to hold bytes to be sent
		byteArray = buffer.tobytes()
		self.bytesSent = 0
		
		# Send bytes and flush immediately after
		try:
			self.bytesSent = self.clientStream.send(byteArray, len(byteArray))
			self.clientStream.flush()
		except StreamError as e:
			print(e.get_error_message())
			self.bytesSent = -1 # If an error occurs, set bytesSent to -1 for user to check
		finally:
			return self.bytesSent

class Calculus:
	'''Class object consisting of basic differentiation and integration functions'''

	def differentiator(self, dt, x_prev=0):
		'''Standard derivative. Provide the sample time (s), and use the .send(value) method to differentiate.
		
		For example, 
		>>> diff_1 = Calculus().differentiator(0.01) 
		>>> while True:
		>>>     value = some_random_function()
		>>>     value_derivative = diff_1.send(value)

		Multiple differentiators can be defined for different signals. Do not use the same handle to differentiate different value signals.
		'''
		derivative = 0
		while True:
			x = yield derivative
			derivative = (x - x_prev)/dt
			x_prev = x
	
	def differentiator_variable(self, dt, x_prev=0):
		'''Standard derivative. Provide the sample time (s), and use the .send(value) method to differentiate.
		
		For example, 
		>>> diff_1 = Calculus().differentiator_variable(0.01) 
		>>> while True:
		>>>     value = some_random_function()
		>>>     time_step = some_time_taken
		>>>     value_derivative = diff_1.send((value, time_step))

		Multiple differentiators can be defined for different signals. Do not use the same handle to differentiate different value signals.
		'''
		derivative = 0
		while True:
			x, dt = yield derivative
			derivative = (x - x_prev)/dt
			x_prev = x
	
	def integrator(self, dt, integrand=0):
		'''Standard integral. Provide the sample time (s), and use the .send(value) method to integrate.
		
		For example, 
		>>> intg_1 = Calculus().integrator(0.01)
		>>> while True:
		>>>     value = some_random_function()
		>>>     value_integral = intg_1.send(value)

		Multiple integrators can be defined for different signals. Do not use the same handle to integrate different value signals.
		'''
		while True:
			x = yield integrand
			integrand = integrand + x * dt

	def integrator_variable(self, dt, integrand=0):
		'''Standard integral. Provide the sample time (s), and use the .send(value) method to integrate.
		
		For example, 
		>>> intg_1 = Calculus().integrator_variable(0.01)
		>>> while True:
		>>>     value = some_random_function()
		>>>     time_step = some_time_taken
		>>>     value_integral = intg_1.send((value, time_step)))

		Multiple integrators can be defined for different signals. Do not use the same handle to integrate different value signals.
		'''
		while True:
			x, dt = yield integrand
			integrand = integrand + x * dt

class Filter:
	'''Class object consisting of different filter functions'''

	def low_pass_first_order(self, wn, dt, x0=0):
		'''Standard first order low pass filter. Provide the filter frequency (rad/s), sample time (s), and use the .send(value) method to filter.
		
		For example, 
		>>> filter_1 = filter().low_pass_first_order(20, 0.01)
		>>> value_filtered = next(filter_1)
		>>> while True:
		>>>     value = some_random_function()
		>>>     value_filtered = filter_1.send(value)

		Multiple filters can be defined for different signals. Do not use the same handle to filter different signals.'''

		output = 0
		integrator_1 = Calculus().integrator(dt, integrand=x0)
		next(integrator_1)
		while True:
			x = yield output
			output = integrator_1.send(wn * (x - output))

	def low_pass_first_order_variable(self, wn, dt, x0=0):
		'''Standard first order low pass filter. Provide the filter frequency (rad/s), sample time (s), and use the .send(value) method to filter.
		
		For example, 
		>>> filter_1 = filter().low_pass_first_order(20, 0.01)
		>>> value_filtered = next(filter_1)
		>>> while True:
		>>>     value = some_random_function()
		>>>     value_filtered = filter_1.send(value)

		Multiple filters can be defined for different signals. Do not use the same handle to filter different signals.'''

		output = 0
		integrator_1 = Calculus().integrator_variable(dt, integrand=x0)
		next(integrator_1)
		while True:
			x, dt = yield output
			output = integrator_1.send((wn * (x - output), dt))

	def low_pass_second_order(self, wn, dt, zeta=1, x0=0):
		'''Standard second order low pass filter. Provide the filter frequency (rad/s), sample time (s), and use the .send(value) method to filter.
		
		For example, 
		>>> filter_2 = filter().low_pass_second_order(20, 0.01)
		>>> value_filtered = next(filter_2)
		>>> while True:
		>>>     value = some_random_function()
		>>>     value_filtered = filter_2.send(value)

		Multiple filters can be defined for different signals. Do not use the same handle to filter different signals.'''
		output = 0
		temp = 0        
		integrator_1 = Calculus().integrator(dt, integrand=0)
		integrator_2 = Calculus().integrator(dt, integrand=x0)
		next(integrator_1)
		next(integrator_2)
		while True:
			x = yield output
			temp = integrator_1.send(wn * ( x - output - 2*zeta*temp ) )
			output = integrator_2.send(wn * temp)

	def complimentary_filter(self, kp, ki, dt, x0=0):
		output = 0
		temp = 0
		integrator_rate = Calculus().integrator(dt, integrand=0)
		integrator_tuner = Calculus().integrator(dt, integrand=x0)
		next(integrator_rate)
		next(integrator_tuner)
		while True:
			rate, correction = yield output
			temp = integrator_rate.send(rate)
			error = output - correction
			output = temp - (kp*error) - (ki*integrator_tuner.send(error))

	def moving_average(self, samples, x_0=0):
		'''Standard moving average filter. Provide the number of samples to average, and use the .send(value) method to filter.
		
		For example, 
		>>> filter_3 = filter().moving_average(20)
		>>> value_filtered = next(filter_3)
		>>> while True:
		>>>     value = some_random_function()
		>>>     value_filtered = filter_3.send(value)

		Multiple filters can be defined for different signals. Do not use the same handle to filter different signals.'''
		window = x_0*np.ones(samples)
		average = x_0
		while True:
			new_value = yield average
			window = np.append(new_value, window[0:samples-1])
			average = window.mean()

class Signal_Generator:
	'''Class object consisting of common signal generators'''
	def sine(self, amplitude, ang_freq, phase=0, mean=0):
		'''This function outputs a sinusoid wave based on the provided timestamp. 
		
		For example:
		>>> generator_sine = Signal_Generator().sine(2, pi/2)
		>>> initial_output = next(generator_sine)
		>>> while True:
		>>>     timestamp = your_timing_function()
		>>>     output = generator_sine.send(timestamp) '''

		output = amplitude*np.sin(phase) + mean
		while True:
			timestamp = yield output
			output = amplitude*np.sin(ang_freq*timestamp + phase) + mean

	def cosine(self, amplitude, ang_freq, phase=0, mean=0):
		'''This function outputs a cosinusoid wave based on the provided timestamp. 
		
		For example:
		>>> generator_cosine = Signal_Generator().cosine(2, pi/2)
		>>> initial_output = next(generator_sine)
		>>> while True:
		>>>     timestamp = your_timing_function()
		>>>     output = generator_cosine.send(timestamp) '''

		output = amplitude*np.sin(phase + np.pi/2) + mean
		while True:
			timestamp = yield output
			output = amplitude*np.sin(ang_freq*timestamp + phase + np.pi/2) + mean

	def PWM(self, freq, width, phase=0):
		'''This function outputs a PWM wave based on the provided timestamp. 
		
		For example:
		>>> generator_PWM = Signal_Generator().PWM(2, 0.5)
		>>> initial_output = next(generator_PWM)
		>>> while True:
		>>>     timestamp = your_timing_function()
		>>>     output = generator_PWM.send(timestamp) '''

		period = 1/freq
		if phase%1 >= width:
			output = 0
		else:
			output = 1
		while True:
			timestamp = yield output
			marker = ( ( (timestamp % period) / period ) + phase ) % 1
			if marker > width:
				output = 0
			else:
				output = 1

class Other:
	'''Class object consisting of common utilities such as saturation methods'''
	@staticmethod
	def saturate(value, upper, lower):
		'''Saturate the input value based on the upper and lower thresholds provided.
			
			For example, 
			>>> saturate(0.1, 0.2, -0.2) # will yeild 0.1
			>>> saturate(0.3, 0.2, -0.2) # will yeild 0.2
			>>> saturate(-0.3, 0.2, -0.2) # will yeild -0.2
			'''

		value_sat = value
		if value > upper:
			value_sat = upper
		if value < lower:
			value_sat = lower   

		return value_sat

	@staticmethod
	# Click Event (to return image coordinates at click point)
	def click_event(event, x, y, flags, params):
		
		# checking for left mouse clicks
		if event == cv2.EVENT_LBUTTONDOWN:
	
			# displaying the coordinates
			# on the Shell
			print(x, ' ', y)

		# checking for right mouse clicks    
		if event==cv2.EVENT_RBUTTONDOWN:

			params.append((x, y))
			
saturate = Other.saturate

class Camera3D():
	def __init__(self, mode='RGB&DEPTH', frame_width_rgb=1920, frame_height_rgb=1080, frame_rate_rgb=30.0, frame_width_depth=1280, frame_height_depth=720, 
				frame_rate_depth=15.0, device_id='0', 
				focal_length_rgb=np.array([[None], [None]], dtype=np.float64), principle_point_rgb=np.array([[None], [None]], dtype=np.float64), skew_rgb=None, 
				position_rgb=np.array([[None], [None], [None]], dtype=np.float64), orientation_rgb=np.array([[None, None, None], [None, None, None], [None, None, None]], dtype=np.float64),
				focal_length_depth=np.array([[None], [None]], dtype=np.float64), principle_point_depth=np.array([[None], [None]], dtype=np.float64), skew_depth=None, 
				position_depth=np.array([[None], [None], [None]], dtype=np.float64), orientation_depth=np.array([[None, None, None], [None, None, None], [None, None, None]], dtype=np.float64)):
		'''This function configures the Intel Realsense RGB and depth cameras for use.
		
		Outputs:
		video3d - video3d object, you must call video3d.start_streaming() before your main loop
		stream_RGB - stream object to be passed to the read method
		image_buffer_RGB - buffer array that will be updated by the read method
		stream_depth - stream object to be passed to the read method
		image_buffer_depth - buffer array that will be updated by the read method'''
		self.mode = mode
		self.stream_index = 0
		self.image_buffer_rgb = np.zeros((frame_height_rgb, frame_width_rgb, 3), dtype=np.uint8)
		self.image_buffer_depth_px = np.zeros((frame_height_depth, frame_width_depth, 1), dtype=np.uint8)
		self.image_buffer_depth_m = np.zeros((frame_height_depth, frame_width_depth, 1), dtype=np.float32)
		self.frame_width_rgb = frame_width_rgb
		self.frame_height_rgb = frame_height_rgb
		self.frame_width_depth = frame_width_depth
		self.frame_height_depth = frame_height_depth

		self.focal_length_rgb = 2*focal_length_rgb
		self.focal_length_rgb[0, 0] = -self.focal_length_rgb[0, 0]
		self.principle_point_rgb = principle_point_rgb
		self.skew_rgb = skew_rgb
		self.position_rgb = position_rgb
		self.orientation_rgb = orientation_rgb

		self.focal_length_depth = 2*focal_length_depth
		self.focal_length_depth[0, 0] = -self.focal_length_depth[0, 0]
		self.principle_point_depth = principle_point_depth
		self.skew_depth = skew_depth
		self.position_depth = position_depth
		self.orientation_depth = orientation_depth

		try:
			self.video3d = Video3D(device_id)
			if mode == 'RGB':
				self.stream_RGB = self.video3d.stream_open(Video3DStreamType.COLOR, self.stream_index, frame_rate_rgb, frame_width_rgb, frame_height_rgb, ImageFormat.ROW_MAJOR_INTERLEAVED_BGR, ImageDataType.UINT8)
			elif mode == 'DEPTH':
				self.stream_depth = self.video3d.stream_open(Video3DStreamType.DEPTH, self.stream_index, frame_rate_depth, frame_width_depth, frame_height_depth, ImageFormat.ROW_MAJOR_GREYSCALE, ImageDataType.UINT8)        
			else:
				self.stream_RGB = self.video3d.stream_open(Video3DStreamType.COLOR, self.stream_index, frame_rate_rgb, frame_width_rgb, frame_height_rgb, ImageFormat.ROW_MAJOR_INTERLEAVED_BGR, ImageDataType.UINT8)
				self.stream_depth = self.video3d.stream_open(Video3DStreamType.DEPTH, self.stream_index, frame_rate_depth, frame_width_depth, frame_height_depth, ImageFormat.ROW_MAJOR_GREYSCALE, ImageDataType.UINT8)
			self.video3d.start_streaming()                
		except MediaError as me:
			print(me.get_error_message())
			
	def terminate(self):
		'''This function terminates the RGB and depth video and stream objects correctly.
		
		Inputs:
		video3d - video object from the configure method
		stream_RGB - RGB stream object from the configure method
		stream_depth - depth stream object from the configure method '''

		try:
			self.video3d.stop_streaming()
			if self.mode == 'RGB':
				self.stream_RGB.close()
			elif self.mode == 'DEPTH':
				self.stream_depth.close()
			else:
				self.stream_RGB.close()
				self.stream_depth.close()

			self.video3d.close()

		except MediaError as me:
			print(me.get_error_message())
	
	def read_RGB(self):
		'''This function reads an image from the RGB camera for use.
		
		Outputs:
		timestamp - timestamp corresponding to the frame read '''
		timestamp = -1
		try:
			frame = self.stream_RGB.get_frame()
			while not frame:
				frame = self.stream_RGB.get_frame() 
			frame.get_data(self.image_buffer_rgb)
			timestamp = frame.get_timestamp()
			frame.release()
		except KeyboardInterrupt:
			pass
		except MediaError as me:
			print(me.get_error_message())
		finally:
			return timestamp

	def read_depth(self, dataMode='px'):
		'''This function reads an image from the depth camera for use.
		dataMode is 'px' for pixels or 'm' for meters. Use corresponding image buffer.
		
		Outputs:
		timestamp - timestamp corresponding to the frame read '''
		timestamp = -1
		try:
			frame = self.stream_depth.get_frame()
			while not frame:
				frame = self.stream_depth.get_frame()
			if dataMode == 'px':
				frame.get_data(self.image_buffer_depth_px)
			elif dataMode == 'm':
				frame.get_meters(self.image_buffer_depth_m)
			timestamp = frame.get_timestamp()
			frame.release()
		except KeyboardInterrupt:
			pass
		except MediaError as me:
			print(me.get_error_message())
		finally:
			return timestamp

	def extrinsics_rgb(self):
		# define rotation matrix from camera frame into the body frame
		b_T_c = np.concatenate((np.concatenate((self.orientation_rgb, self.position_rgb), axis=1), [[0, 0, 0, 1]]), axis=0)

		# invert the matrix to get transform from body frame to camera frame, aka, Extrinsic Matrix
		c_T_b = np.linalg.inv(b_T_c)

		return c_T_b

	def intrinsics_rgb(self):
		# construct the intrinsic matrix
		intrinsic = np.array([[self.focal_length_rgb[0,0], self.skew_rgb, self.principle_point_rgb[0,0]], [0, self.focal_length_rgb[1,0], self.principle_point_rgb[1,0]], [0, 0, 1]], dtype = np.float64)

		return intrinsic

	def extrinsics_depth(self):
		# define rotation matrix from camera frame into the body frame
		b_T_c = np.concatenate((np.concatenate((self.orientation_depth, self.position_depth), axis=1), [[0, 0, 0, 1]]), axis=0)

		# invert the matrix to get transform from body frame to camera frame, aka, Extrinsic Matrix
		c_T_b = np.linalg.inv(b_T_c)

		return c_T_b

	def intrinsics_depth(self):
		# construct the intrinsic matrix
		intrinsic = np.array([[self.focal_length_depth[0,0], self.skew_depth, self.principle_point_depth[0,0]], [0, self.focal_length_depth[1,0], self.principle_point_depth[1,0]], [0, 0, 1]], dtype = np.float64)

		return intrinsic

	def RGB2DepthRegionCalibrator(self, img_rgb, img_depth, detector_rgb, detector_depth, points_list, color):
		# This method transforms the points extracted from the RGB image into those in the Depth image
		
		click_event = Other.click_event
		# Display the RGB image at half the resolution
		cv2.imshow('rgb_img', img_rgb)
		# setting mouse handler for the image and calling the click_event() function
		cv2.setMouseCallback('rgb_img', click_event, points_list)
		# wait for a key to be pressed to exit
		cv2.waitKey(0)
		
		# Create 2D numpy array for the points (polylines function likes np array, drawMarker likes tuples :( )
		pts_2d_rgb = np.zeros((len(points_list),2))
		pts_2d_depth = np.zeros((len(points_list),2))
		counter = 0
		max_height = 2*detector_depth.height/5
		# Convert from tuples to numpy arrays
		for rgb_px_point in points_list:
			unused, body_point_r = detector_rgb.imageToBody(rgb_px_point)
			unused, depth_px_point = detector_depth.bodyToImage(body_point_r)
			# cv2.drawMarker(img_rgb, (int(rgb_px_point[0]), int(rgb_px_point[1])), color)    
			# cv2.drawMarker(img_depth, (int(depth_px_point[0]), int(depth_px_point[1])), color)    
			pts_2d_rgb[counter, 0] =  int(rgb_px_point[0])
			pts_2d_rgb[counter, 1] =  int(rgb_px_point[1])
			pts_2d_depth[counter, 0] =  int(depth_px_point[0])
			pts_2d_depth[counter, 1] =  int(depth_px_point[1])
			counter += 1
		
		# Figure out the left view from the first two points in pts_2d_depth
		pts_2d_depth_left_view = np.copy(pts_2d_depth)
		slope = (pts_2d_depth_left_view[0, 0] - pts_2d_depth_left_view[1, 0])/(pts_2d_depth_left_view[0, 1] - pts_2d_depth_left_view[1, 1])
		intercept = pts_2d_depth_left_view[0, 0] - slope*pts_2d_depth_left_view[0, 1]
		
		if -intercept/slope > detector_depth.height:
			pts_2d_depth_left_view[0, 0] = slope*detector_depth.height + intercept
			pts_2d_depth_left_view[0, 1] = detector_depth.height
			
		else:
			pts_2d_depth_left_view[0, 0] = 0
			pts_2d_depth_left_view[0, 1] = -intercept/slope
			
		pts_2d_depth_left_view[2, 0] = pts_2d_depth_left_view[1, 0]
		pts_2d_depth_left_view[3, 0] = pts_2d_depth_left_view[0, 0]
		pts_2d_depth_left_view[2, 1] = max_height
		pts_2d_depth_left_view[3, 1] = max_height
		
		# Figure out the right view from the last two points in pts_2d_depth
		pts_2d_depth_right_view = np.copy(pts_2d_depth)
		slope = (pts_2d_depth_right_view[3, 0] - pts_2d_depth_right_view[2, 0])/(pts_2d_depth_right_view[3, 1] - pts_2d_depth_right_view[2, 1])
		intercept = pts_2d_depth_right_view[3, 0] - slope*pts_2d_depth_right_view[3, 1]
		if (detector_depth.width - intercept)/slope > detector_depth.height:
			pts_2d_depth_right_view[3, 0] = slope*detector_depth.height + intercept
			pts_2d_depth_right_view[3, 1] = detector_depth.height
			
		else:
			pts_2d_depth_right_view[3, 0] = detector_depth.width
			pts_2d_depth_right_view[3, 1] = (detector_depth.width - intercept)/slope

		pts_2d_depth_right_view[0, 0] = pts_2d_depth_right_view[3, 0]
		pts_2d_depth_right_view[1, 0] = pts_2d_depth_right_view[2, 0]
		pts_2d_depth_right_view[0, 1] = max_height
		pts_2d_depth_right_view[1, 1] = max_height

		pts_2d_center = np.copy(pts_2d_depth)
		pts_2d_center[0, 0] = pts_2d_center[1, 0] + 5
		pts_2d_center[1, 0] = pts_2d_center[1, 0] + 5
		pts_2d_center[0, 1] = pts_2d_center[1, 1]
		pts_2d_center[2, 0] = pts_2d_center[2, 0] - 5
		pts_2d_center[3, 0] = pts_2d_center[2, 0] - 5
		pts_2d_center[3, 1] = pts_2d_center[2, 1]
		pts_2d_center[1, 1] = max_height
		pts_2d_center[2, 1] = max_height

		pts_2d_depth_visual = np.copy(pts_2d_depth)
		pts_2d_depth_visual[0, 1] = pts_2d_depth_visual[0, 1]+5
		pts_2d_depth_visual[3, 1] = pts_2d_depth_visual[3, 1]+5
		pts_2d_depth_visual[1, 1] = pts_2d_depth_visual[1, 1]+5
		pts_2d_depth_visual[2, 1] = pts_2d_depth_visual[2, 1]+5
		pts_2d_depth_visual[0, 0] = pts_2d_depth_visual[0, 0]+5
		pts_2d_depth_visual[1, 0] = pts_2d_depth_visual[1, 0]+5
		pts_2d_depth_visual[2, 0] = pts_2d_depth_visual[2, 0]-5
		pts_2d_depth_visual[3, 0] = pts_2d_depth_visual[3, 0]-5

		# Reshape the arrays for polylines to work properly
		pts_2d_rgb = pts_2d_rgb.reshape((-1, 1, 2))
		pts_2d_depth = pts_2d_depth.reshape((-1, 1, 2))
		pts_2d_depth_visual = pts_2d_depth_visual.reshape((-1, 1, 2))
		pts_2d_center = pts_2d_center.reshape((-1, 1, 2))
		pts_2d_depth_left_view = pts_2d_depth_left_view.reshape((-1, 1, 2))
		pts_2d_depth_right_view = pts_2d_depth_right_view.reshape((-1, 1, 2))		

		# Draw the polylines onto the depth and RGB images    
		img_depth = cv2.polylines(img_depth, np.int32([pts_2d_depth_visual]), True, (255,0,0), 2)
		img_depth = cv2.polylines(img_depth, np.int32([pts_2d_depth_left_view]), True, (0,255,0), 2)
		img_depth = cv2.polylines(img_depth, np.int32([pts_2d_depth_right_view]), True, (0,0,255), 2)
		img_depth = cv2.polylines(img_depth, np.int32([pts_2d_center]), True, (0,255,255), 2)
		img_rgb = cv2.polylines(img_rgb, np.int32([pts_2d_rgb]), True, color, 2)    

		return img_rgb, img_depth, pts_2d_depth, pts_2d_depth_left_view, pts_2d_depth_right_view, pts_2d_center

class Camera2D():
	def __init__(self, camera_id="0", frame_width=820, frame_height=410, frame_rate=30.0, focal_length=np.array([[None], [None]], dtype=np.float64), principle_point=np.array([[None], [None]], dtype=np.float64), skew=None, position=np.array([[None], [None], [None]], dtype=np.float64), orientation=np.array([[None, None, None], [None, None, None], [None, None, None]], dtype=np.float64)):
		'''This function configures the 2D camera for use based on the camera_id provided.'''

		self.url = "video://localhost:"+camera_id
		self.image_data = np.zeros((frame_height, frame_width, 3), dtype=np.uint8)
		self.frame_width = frame_width
		self.frame_height = frame_height

		self.focal_length = 2*focal_length
		self.focal_length[0, 0] = -self.focal_length[0, 0]
		self.principle_point = principle_point
		self.skew = skew
		self.position = position
		self.orientation = orientation

		try:
			self.capture = VideoCapture(self.url, frame_rate, frame_width, frame_height, ImageFormat.ROW_MAJOR_INTERLEAVED_BGR, ImageDataType.UINT8, None, 0)
			self.capture.start()
		except MediaError as me:
			print(me.get_error_message())
		
	def read(self):
		'''This function reads a frame, updating the corresponding image buffer.'''
		try:
			# self.capture.read()
			self.capture.read(self.image_data)
		except MediaError as me:
			print(me.get_error_message())
		except KeyboardInterrupt:
			print('User Interupted')

	def reset(self):
		'''This function resets the 2D camera stream by stopping and starting the capture service.'''

		try:
			self.capture.stop()
			self.capture.start()
		except MediaError as me:
			print(me.get_error_message())

	def terminate(self):
		'''This function terminates the 2D camera operation.'''
		try:
			self.capture.stop()
			self.capture.close()
		except MediaError as me:
			print(me.get_error_message())

	def extrinsics(self):
		# define rotation matrix from camera frame into the body frame
		b_T_c = np.concatenate((np.concatenate((self.orientation, self.position), axis=1), [[0, 0, 0, 1]]), axis=0)

		# invert the matrix to get transform from body frame to camera frame, aka, Extrinsic Matrix
		c_T_b = np.linalg.inv(b_T_c)

		return c_T_b

	def intrinsics(self):
		# construct the intrinsic matrix
		intrinsic = np.array([[self.focal_length[0,0], self.skew, self.principle_point[0,0]], [0, self.focal_length[1,0], self.principle_point[1,0]], [0, 0, 1]], dtype = np.float64)

		return intrinsic

class Lidar():
	def __init__(self, num_measurements=720):
		# 
		self.num_measurements = num_measurements
		# self.measurements = [RangingMeasurement() for x in range(self.num_measurements)]
		# self.measurements = RangingMeasurements(num_measurements)
		self.measurements = RangingMeasurements(num_measurements)
		self.distances = np.zeros((num_measurements,1), dtype=np.float64)
		self.angles = np.zeros((num_measurements,1), dtype=np.float64)
		# self.angles = np.linspace(0, 2*np.pi-(2*np.pi/num_measurements), num_measurements, dtype=np.float64)
		self.lidar = RPLIDAR()
		# self.maxDistance = 18.0
		try:
			self.lidar.open("serial-cpu://localhost:2?baud='115200',word='8',parity='none',stop='1',flow='none',dsr='on'", RangingDistance.LONG)
		except DeviceError as de:
			if de.error_code == -34:
				pass
			else:
				print(de.get_error_message())

	def terminate(self):
		try:
			self.lidar.close()
		except DeviceError as de:
			if de.error_code == -34:
				pass
			else:
				print(de.get_error_message())

	def read(self):
		try:
			self.lidar.read(RangingMeasurementMode.NORMAL, 0, 0, self.measurements)
			self.distances = np.array(self.measurements.distance)
			# self.distances = np.append(  np.flip( self.distances[0:int(self.num_measurements/4)] ) , 
			#                              np.flip( self.distances[int(self.num_measurements/4):]) )
			# self.distances[self.distances > self.maxDistance] = self.maxDistance
			# self.distances[self.distances > self.maxDistance] = 0
			self.angles = np.array(self.measurements.heading)
			
		except DeviceError as de:
			if de.error_code == -34:
				pass
			else:
				print(de.get_error_message())

class LaneDetector():
	# This example is derived from the repository ["Tutorial: Build a lane detector"](https://towardsdatascience.com/tutorial-build-a-lane-detector-679fd8953132) published on Medium.

	def __init__(self, camera):

		self.blur = (5, 5)
		self.canny_lower_threshold = 100
		self.canny_upper_threshold = 200
		self.camera = camera
		self.DEPTH_CAMERA_AVAILABLE = False

		if type(self.camera) == Camera2D:
			self.width = self.camera.frame_width
			self.height = self.camera.frame_height
			self.principle_point = self.camera.principle_point
			self.focal_length = self.camera.focal_length
			self.extrinsic_matrix = self.camera.extrinsics()
			self.intrinsic_matrix = self.camera.intrinsics()
			
		elif type(camera) == Camera3D:
			self.width = self.camera.frame_width_rgb
			self.height = self.camera.frame_height_rgb
			self.principle_point = self.camera.principle_point_rgb
			self.focal_length = self.camera.focal_length_rgb
			self.extrinsic_matrix = self.camera.extrinsics_rgb()
			self.intrinsic_matrix = self.camera.intrinsics_rgb()
	
	def doCanny(self, frame):
		# Converts frame to grayscale because we only need the luminance channel for detecting edges - less computationally expensive
		gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
		# Applies a 5x5 gaussian blur with deviation of 0 to frame - not mandatory since Canny will do this for us
		blur = cv2.GaussianBlur(gray, self.blur, 0)
		# Applies Canny edge detector with minVal of 50 and maxVal of 150
		canny = cv2.Canny(blur, self.canny_lower_threshold, self.canny_upper_threshold)
		return canny

	def doSegment(self, frame, steering = 0):
		
		# Creates a polygon for the mask defined by multiple coordinates
		polygons = np.array([
								[(0, int(0.9*self.height - 0.4*self.height*steering)), 
								(0, self.height), 
								(self.width, self.height), 
								(self.width, int(0.9*self.height + 0.4*self.height*steering)), 
								(int(self.width/2 + int(self.width/6) - int(self.width/3)*3*steering), int(3.5*self.height/5)), 
								(int(self.width/2 - int(self.width/6) - int(self.width/3)*3*steering), int(3.5*self.height/5))]
							])

		# Creates an image filled with zero intensities with the same dimensions as the frame
		mask = np.zeros_like(frame)

		# Allows the mask to be filled with values of 1 and the other areas to be filled with values of 0
		cv2.fillPoly(mask, polygons, 255)
		# A bitwise and operation between the mask and frame keeps only the triangular area of the frame
		segment = cv2.bitwise_and(frame, mask)

		return segment, mask

	def calculateLines(self, segment):
		# +--------> x-axis
		# |               /          |               \
		# |             /            | Inf slope      \                     0 slope
		# v           / -ve slope    |                 \ +ve slope      _______________
		# y-axis    /                |                  \               
		#
		# Use hough function to get the lines in the image. vertical line has Infinite slope. Horizontal line has slope 0.
		lines = cv2.HoughLinesP(segment, 2, np.pi/180, 100, np.array([]), minLineLength = 150, maxLineGap = 100)
		
		# Empty arrays to store the coordinates of the left and right lines
		left = []
		right = []

		# Loops through every detected line
		if lines is None:
			# No lines were found, womp womp
			return [], []
		else:
			# Group the lines into a left/right category
			for line in lines:
				# Reshapes line from 2D array to 1D array
				x1, y1, x2, y2 = line.reshape(4)
				# Fit a linear polynomial to the x and y coordinates and returns a vector of coefficients which describe the slope and y-intercept
				parameters = np.polyfit((x1, x2), (y1, y2), 1)
				slope = parameters[0]
				y_intercept = parameters[1]

				# Avoid lines that have slopes close to 0, as these might be crosswalks, 
				# or parking lanes, etc. Thus, anything with a slope higher than 0.1, will
				# be the right lane, and less than -0.1 will be the left lane. 
				if slope < -0.1:
					left.append([slope, y_intercept])
				elif slope > 0.1:
					right.append([slope, y_intercept])
			return left, right

	def averageLines(self, frame, left, right):    
		# Averages out all the values for left and right into a single slope and y-intercept value for each line
		
		if len(left) > 0:
			left_param = np.average(left, axis = 0)
			left_line = self.calculateCoordinates(left_param)
		else:
			left_param = None
			left_line = None
		if len(right) > 0:
			right_param = np.average(right, axis = 0)
			right_line = self.calculateCoordinates(right_param)
		else:
			right_param = None
			right_line = None
		
		return [left_param, right_param], self.visualizeLines(frame, np.array([left_line, right_line]))

	def calculateCoordinates(self, parameters):

		slope, intercept = parameters
		# Sets initial y-coordinate as height from top down (bottom of the frame)
		y1 = self.height
		# Sets final y-coordinate as 150 above the bottom of the frame
		y2 = int(y1 - 1.5*self.height/5)
		# Sets initial x-coordinate as (y1 - b) / m since y1 = mx1 + b
		x1 = int((y1 - intercept) / slope)
		# Sets final x-coordinate as (y2 - b) / m since y2 = mx2 + b
		x2 = int((y2 - intercept) / slope)

		return np.array([x1, y1, x2, y2])

	def drivingParameters(self, line_parameters):
		if line_parameters[0] is not None:
			m, b = line_parameters[0]
			actual_l  = int((255 - b)/m)
			desired_l = int((255 - 423)/(-0.53))
			mtr_spd = 0.05
			steering = (desired_l - actual_l)/600 + 0.5*(m + 0.53)
			print('Using Left Lane:', line_parameters[0], steering)

		elif line_parameters[1] is not None:
			m, b = line_parameters[1]
			actual_r  = int((255 - b)/m)
			desired_r = int((255 - 30)/(0.65))
			mtr_spd = 0.05
			steering = (desired_r - actual_r)/600 + 0.5*(m - 0.65)
			print('Using Right Lane:', line_parameters[1], steering)

		else:
			steering = 0
			mtr_spd = 0 
			print('No Driving Lane Available')

		return mtr_spd, steering

	def visualizeLines(self, frame, lines):
		# Checks if any lines are detected
		
		for line in lines:
			if line is not None:
				x1, y1, x2, y2 = line
				# Draws lines between two coordinates with green color and 5 thickness
				cv2.line(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)

		return frame

	def detectYellowLane(self, frame):
		lowerBounds = (40, 200, 200)
		upperBounds = (120, 255, 255)
		mask = cv2.inRange(frame, lowerBounds, upperBounds)
		mask = cv2.dilate(mask, np.ones((5, 5), np.uint8), iterations=1)
		return mask

	def extractPointGivenRow(self, row):
		counter = len(row)
		solution = -1
		for i in range(len(row)):
			if row[counter - 1 - i] > 0:
				solution = counter - 1 - i
				break

		return solution

class CameraCalibration():
	def __init__(self, camera, lens='RGB'):

		self.camera = camera
		if type(self.camera) == Camera2D:
			self.width = self.camera.frame_width
			self.height = self.camera.frame_height
			self.principle_point = self.camera.principle_point
			self.focal_length = self.camera.focal_length
			self.extrinsic_matrix = self.camera.extrinsics()
			self.intrinsic_matrix = self.camera.intrinsics()

		elif type(camera) == Camera3D:
			if lens == 'RGB':
				self.width = self.camera.frame_width_rgb
				self.height = self.camera.frame_height_rgb
				self.principle_point = self.camera.principle_point_rgb
				self.focal_length = self.camera.focal_length_rgb
				self.extrinsic_matrix = self.camera.extrinsics_rgb()
				self.intrinsic_matrix = self.camera.intrinsics_rgb()
			elif lens == 'DEPTH':
				self.width = self.camera.frame_width_depth
				self.height = self.camera.frame_height_depth
				self.principle_point = self.camera.principle_point_depth
				self.focal_length = self.camera.focal_length_depth
				self.extrinsic_matrix = self.camera.extrinsics_depth()
				self.intrinsic_matrix = self.camera.intrinsics_depth()

	def bodyToImage(self, body_point):
		# convert from tuple to 2D numpy array
		body_point = np.array([[body_point[0]], [body_point[1]], [body_point[2]]])

		# convert body point to camera point using Extrinsic Matrix
		camera_point = self.extrinsic_matrix@np.concatenate((body_point, np.array([[1]])))        
		
		# convert camera point to image point - the camera point is an [cx, cy, cz, 1] vector. Ignore the 1, and 
		# premultiply it to get the image point, of form, [ix, iy, iz] 
		image_point = self.intrinsic_matrix@camera_point[0:3]
		
		# convert camera point and image point back to tuple for return
		camera_point = (camera_point[0, 0], camera_point[1, 0], camera_point[2, 0])
		image_point = (image_point[0, 0] / image_point[2, 0], image_point[1, 0] / image_point[2, 0])

		return camera_point, image_point

	def imageToBody(self, image_point):
		image_point = np.array([[image_point[0]], [image_point[1]]])
		camera_point_ratio = (image_point - self.principle_point)/self.focal_length

		# camera_point_ratio is (Cx/Cz, Cy/Cz). The Cy value for a road is fixed, which is the extrinsic_matrix(1,3), that is, 2nd row 4th col
		Cy = self.extrinsic_matrix[1,3]
		Cz = Cy / camera_point_ratio[1, 0]
		Cx = Cz * camera_point_ratio[0, 0]
		
		camera_point = np.array([[Cx], [Cy], [Cz], [1]], dtype = np.float64)

		# map the point back to body frame
		body_point = np.linalg.inv(self.extrinsic_matrix)@camera_point

		camera_point = (camera_point[0, 0], camera_point[1, 0], camera_point[2, 0])
		body_point = (body_point[0, 0], body_point[1, 0], body_point[2, 0])

		return camera_point, body_point

class GPS():
	def __init__(self, uri):
		self.gps_client = BasicStream(uri, agent='c', non_blocking=True, recv_buffer_size=32*6, send_buffer_size=1)
		self.gps_data = np.zeros((6), dtype=np.float32) 
		self.gps_client.checkConnection(t_out_local=Timeout(seconds=10, nanoseconds=0))
		self.position = np.zeros((3))
		self.orientation = np.zeros((3))

	def read(self):
		self.gps_data, bytes_received = self.gps_client.receive(self.gps_data, iterations=2, timeout=Timeout(seconds=0, nanoseconds=0))
		flag = True
		if bytes_received < len(self.gps_data.tobytes()):
			flag = False						
		else:
			self.position = self.gps_data[0:3]
			self.orientation = self.gps_data[3:6]
		return flag

	def terminate(self):
		self.gps_client.terminate()

class GamePad():
	'''This method opens a GameController device and opens a connection to it. This is set up for the Logitech Gamepad F710. \n
	Use the read() method to update joystick states and terminate() method to close the connection. '''
	def __init__(self, uri):
		self.joystick_client = BasicStream(uri, agent='c', recv_buffer_size=64*8, non_blocking=True, send_buffer_size=1)
		self.joystick_data = np.zeros((8), dtype=np.float64)
		print('Connecting to Joystick Server...')
		self.joystick_client.checkConnection(t_out_local=Timeout(seconds=0, nanoseconds=100000000))
		self.throttle = 0
		self.steering = 0
		self.buttons = np.zeros((6))

	def read(self):
		self.joystick_data, bytes_received = self.joystick_client.receive(self.joystick_data, iterations=2, timeout=Timeout(seconds=1, nanoseconds=0))
		flag = True
		if bytes_received < len(self.joystick_data.tobytes()):
			# print('Joystick Server not broadcasting.')
			flag = False
		else:
			self.throttle = self.joystick_data[0]
			self.steering = self.joystick_data[1]
			self.buttons = self.joystick_data[2:8]

		return flag

	def terminate(self):
		self.joystick_client.terminate()

class Controllers():
	def __init__(self, qcar, kp_f=1, ki_f=0, kd_f=0, kf_f=0.0794, k_l=1):
		'''Provide up to 5 gains on initialization. The gains are
		kp_f = proportional gain for forward speed controller 
		ki_f = integral gain for forward speed controller 
		kd_f = derivative gain for forward speed controller 

		kf_f = feed-forward gain for forward speed controller
				(initial value = 0.0794 = 0.0025 / 0.033 / (13*19/70/37) / 10)
		k_l  = proportional gain for lateral angular speed controller '''
		self.car = qcar
		self.kp_f = kp_f
		self.ki_f = ki_f
		self.kd_f = kd_f
		self.kf_f = kf_f
		self.k_l = k_l
		self.desired_speed = 0

	def lateral_controller(self):
		'''Stanley controller for Lateral Angular Speed control via steering.
		Use the next method to pass in the following parameters, 
		next_waypoint => X-Y or X-Y-Z numpy array
		prev_waypoint => X-Y or X-Y-Z numpy array
		position      => X-Y or X-Y-Z numpy array
		heading       => heading angle in radians
		forward_speed => longitudonal car speed 

		This functions yields the steering output delta
		'''
		front_wheel_center_position = np.array([0.0, 0.0])
		counter = 0
		delta = 0
		road_heading = None
		prev_heading = None
		while True:
			next_waypoint, prev_waypoint, pose, forward_speed = yield delta 
			if road_heading == None:
				road_heading = np.math.atan2(next_waypoint[1]-prev_waypoint[1], next_waypoint[0]-prev_waypoint[0])
				prev_heading = pose[2]
			front_wheel_center_position = self.car.calculate_front_axle_position(pose, factor=0.75) 

			# Get the slope of the line connecting the waypoints... 
			# this is the desired trajectory line => ax + by + c = 0
			raw_road_heading = np.math.atan2(next_waypoint[1]-prev_waypoint[1], next_waypoint[0]-prev_waypoint[0])
			if (raw_road_heading < -3*np.pi/4) or (raw_road_heading > 3*np.pi/4) or (raw_road_heading >= -np.pi/4 and raw_road_heading < np.pi/4):
				# slope is greater than -1 but smaller than 1, where the eqn -mx + y -b = 0 can be used
				slope = (next_waypoint[1] - prev_waypoint[1])/(next_waypoint[0] - prev_waypoint[0])
				a = -slope
				b = 1
				c = slope*next_waypoint[0] - next_waypoint[1]
			else:
				# slope is smaller than -1 or greater than 1, where the eqn -x + y/m -b/m = 0 can be used
				inv_slope = (next_waypoint[0] - prev_waypoint[0])/(next_waypoint[1] - prev_waypoint[1])
				a = -1
				b = inv_slope
				c = next_waypoint[0] - inv_slope*next_waypoint[1]

			# Find the cross-track error
			closest_x = ((b**2)*front_wheel_center_position[0] - a*b*front_wheel_center_position[1] - a*c)/(a**2 + b**2)
			closest_y = ((a**2)*front_wheel_center_position[1] - a*b*front_wheel_center_position[0] - b*c)/(a**2 + b**2)
			
			closest = np.array([closest_x, closest_y], dtype=front_wheel_center_position.dtype)
			road_vector = next_waypoint - prev_waypoint
			car_vector = front_wheel_center_position - closest
			relative_heading = (np.math.atan2(road_vector[1], road_vector[0]) - np.math.atan2(car_vector[1], car_vector[0])) % (2*np.pi)
			if (relative_heading - np.pi/2) < 0.1:
				sign = 1
			else:
				sign = -1
			e = sign*np.linalg.norm(car_vector, 2)

			# Find the steering's cross-track-term
			softening_constant = 0.5
			e_d = 0.3 * delta # crosstrack setpoint to understeer so that the back of the car does not ride the curb
			steering_cross_track = np.math.atan2(self.k_l * (e - e_d), forward_speed+softening_constant)

			# Find the steering's heading-correction-term			
			road_heading = np.unwrap([road_heading, raw_road_heading])[1]
			heading = np.unwrap([prev_heading, pose[2]])[1]
			steering_heading = road_heading - heading

			# Calculate overall steering 
			delta = (steering_cross_track + steering_heading)
			counter = counter + 1
			prev_heading = heading

	def longitudonal_controller(self, sampleTime, integrand=0):
		
		acceleration = 0.0
		braking = 0.0

		while True:
			desired_speed, current_speed = yield acceleration, braking
			error = desired_speed - current_speed

			# Generate controller output based on PID controller
			control_output = self.kp_f * (error) + self.ki_f*(integrand + (error * sampleTime)) + self.kd_f * error / sampleTime
			if control_output >= 0:
				acceleration = control_output
				braking = 0
			else:
				acceleration = 0
				braking = -control_output

			# Update self desired_speed target (for feedforward controller under the pedal_model method)
			self.desired_speed = desired_speed

	def pedal_model(self, acceleration, braking, delta):
		'''Simple pedal model and saturation on cmd values'''
		term = self.kf_f * self.desired_speed
		
		mtr_cmd = saturate(term + acceleration - braking, 0.5, -0.5)
		# print('Internal Speed Controller:', term, acceleration, braking, mtr_cmd)
		steering = saturate(delta, 0.6, -0.6)

		return mtr_cmd, steering

class Node():
	def __init__(self, id, style, position, orientation_l, orientation_r):
		self.id = id
		self.position = position
		self.style = style
		if style == 'left':
			self.orientation = orientation_l
		else:
			self.orientation = orientation_r

class Roadway():
	def __init__(self, startNode, endNode, points=np.array([[]])):
		self.start_node = startNode[0]
		self.end_node = endNode[0]
		self.roadpoints = points

	def create_road_turn_with_center(self, center=np.empty(2), direction='+'):
		# use this function when center is known

		# generate the road pattern from the center and start/end node positions
		angle_s = (np.math.atan2(self.start_node.position[1] - center[1], self.start_node.position[0] - center[0]))%(2*np.pi) # angular position of start point
		angle_e = (np.math.atan2(self.end_node.position[1] - center[1], self.end_node.position[0] - center[0]))%(2*np.pi) # angular position of end point
		
		# adjust start and end angles based on direction
		if direction == '+':
			if angle_e < angle_s:
				angle_e = angle_e + 2*np.pi
		else:
			if angle_e > angle_s:
				angle_e = angle_e - 2*np.pi

		radius_s = np.linalg.norm(self.start_node.position[0:2] - center, 2) # radius at start point
		radius_e = np.linalg.norm(self.end_node.position[0:2] - center, 2) # radius at end point
		numpts = int(round( abs(angle_e-angle_s)*(radius_s + radius_e)/2 ) - 1) # number of points based on arc length of the curve
		# print('Angle_s', angle_s, 'Angle_e', angle_e, 'S_radius', radius_s, 'E_radius', radius_e, 'Numpts', numpts)
		radii = np.zeros(numpts-1) 
		angles = np.zeros(numpts-1)
		solution = np.zeros((numpts-1, 3)) 
		for i in range(numpts-1):
			radii[i] = radius_s + (radius_e - radius_s)*(i+1)/numpts
			angles[i] = angle_s + (angle_e - angle_s)*(i+1)/numpts
			solution[i] = np.array([center[0] + radii[i]*np.math.cos(angles[i]), center[1] + radii[i]*np.math.sin(angles[i]), self.start_node.position[2]])

		self.roadpoints = solution

	def create_road_turn_no_center(self, direction='+'):
		# direction helps figure out the center of turning when center isn't available
		# direction='+' is counter-clockwise turns
		# direction='-' is clockwise turns

		# find alpha...
		difference = self.end_node.position[0:2] - self.start_node.position[0:2]
		beta = np.math.atan2(difference[1], difference[0])
		if direction == '+':
			alpha = beta + np.pi/4
		else:
			alpha = beta - np.pi/4
		alpha = np.pi/2 * round(alpha/(np.pi/2)) # round to closest multiple of pi/2

		# find center
		center=np.empty(2)
		center[0] = self.start_node.position[0] + abs(difference[0])*np.math.cos(alpha)
		center[1] = self.start_node.position[1] + abs(difference[1])*np.math.sin(alpha)
		
		self.create_road_turn_with_center(center=center, direction=direction)

	def create_road_straight(self, points=np.array([[]])):
		delta = self.end_node.position - self.start_node.position
		length = np.linalg.norm(delta, 2)

		numpts = int(round(length)-1)
		solution = np.zeros((numpts, 3)) 
		for i in range(numpts):                    
			solution[i] = np.array([self.start_node.position[0] + (i+1)*(delta[0])/round(length), self.start_node.position[1] + (i+1)*(delta[1])/round(length), self.start_node.position[2]])
		# else:
			# numpts = int(round(abs(delta[0]))-1)
			# solution = np.zeros((numpts, 3))
			# for i in range(numpts):
			#     solution[i] = np.array([self.start_node.position[0] + (i+1)*(delta[0])/round(abs(delta[0])), self.start_node.position[1], self.start_node.position[2]])
		self.roadpoints = solution

class RoadMap():
	def __init__(self, style='left'):
		# This class instance holds the definition of all nodes and roads in Workspace 4A.
		# It contains a total of 50 nodes
		# It contains a total of 43 roads
		#       33 straight roads
		#       24 90-degree curved roads
		#       13 custom roads
		self.num_nodes = 50
		self.style = style
								#  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48 49 50
		self.G_dense = np.array([[ 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # 1
								 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # 2
								 [ 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # 3
								 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # 4
								 [ 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # 5
								 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # 6
								 [ 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # 7
								 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # 8
								 [ 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # 9
								 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # 10
								 [ 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # 11
								 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # 12
								 [ 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # 13
								 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],  # 14
								 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # 15
								 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # 16
								 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],  # 17
								 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # 18
								 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # 19
								 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],  # 20
								 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],  # 21
								 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],  # 22
								 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # 23
								 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # 24 
								 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # 25
								 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # 26
								 [ 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # 27
								 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # 28
								 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # 29
								 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # 30
								 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # 31
								 [ 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # 32
								 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # 33
								 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # 34
								 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # 35
								 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # 36
								 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # 37
								 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # 38
								 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # 39
								 [ 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # 40
								 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],  # 41
								 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # 42
								 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # 43
								 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],  # 44
								 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # 45
								 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # 46
								 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],  # 47
								 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # 48
								 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],  # 49
								 [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]) # 50
		
		self.waypoint_list = []
		self.pathway = []
		self.closedCircuit = False
		# Create 1-D numpy array of nodes and populate them
		pi = np.pi
		self.nodeList = np.empty((self.num_nodes, 1), dtype=Node)
		self.nodeList[0]  = Node( 1, self.style, np.array([-12.00, 10.69, 0.00]),       0,      pi)
		self.nodeList[1]  = Node( 2, self.style, np.array([-12.00, 13.36, 0.00]),      pi,       0)
		self.nodeList[2]  = Node( 3, self.style, np.array([- 5.75, 19.40, 0.00]),   -pi/2,    pi/2)
		self.nodeList[3]  = Node( 4, self.style, np.array([- 3.16, 19.40, 0.00]),    pi/2,   -pi/2)
		self.nodeList[4]  = Node( 5, self.style, np.array([  3.20, 13.36, 0.00]),      pi,       0)
		self.nodeList[5]  = Node( 6, self.style, np.array([  3.20, 10.69, 0.00]),       0,      pi)
		self.nodeList[6]  = Node( 7, self.style, np.array([- 3.16,  4.68, 0.00]),    pi/2,   -pi/2)
		self.nodeList[7]  = Node( 8, self.style, np.array([- 5.75,  4.68, 0.00]),   -pi/2,    pi/2)
		self.nodeList[8]  = Node( 9, self.style, np.array([  7.70, 13.36, 0.00]),      pi,       0)
		self.nodeList[9]  = Node(10, self.style, np.array([  7.70, 10.69, 0.00]),       0,      pi)
		self.nodeList[10] = Node(11, self.style, np.array([ 16.35,  4.68, 0.00]),    pi/2,   -pi/2)
		self.nodeList[11] = Node(12, self.style, np.array([ 13.70,  4.68, 0.00]),   -pi/2,    pi/2)
		self.nodeList[12] = Node(13, self.style, np.array([ 13.70, 19.40, 0.00]),   -pi/2,    pi/2)
		self.nodeList[13] = Node(14, self.style, np.array([ 16.35, 19.40, 0.00]),    pi/2,   -pi/2)
		self.nodeList[14] = Node(15, self.style, np.array([-22.50, 25.00, 0.00]),    pi/2,   -pi/2)
		self.nodeList[15] = Node(16, self.style, np.array([-25.22, 25.00, 0.00]),   -pi/2,    pi/2)
		self.nodeList[16] = Node(17, self.style, np.array([-22.50, 38.20, 0.00]),    pi/2,   -pi/2)
		self.nodeList[17] = Node(18, self.style, np.array([-25.22, 38.20, 0.00]),   -pi/2,    pi/2)
		self.nodeList[18] = Node(19, self.style, np.array([-15.67, 31.75, 0.00]),      pi,       0)
		self.nodeList[19] = Node(20, self.style, np.array([  8.57, 33.76, 0.00]),       0,      pi)
		self.nodeList[20] = Node(21, self.style, np.array([  3.34, 38.92, 0.00]),   -pi/2,    pi/2)
		self.nodeList[21] = Node(22, self.style, np.array([ 14.42, 44.82, 0.00]),  3*pi/4,   -pi/4)
		self.nodeList[22] = Node(23, self.style, np.array([-23.72,  8.00, 0.00]),   -pi/2,    pi/2)
		self.nodeList[23] = Node(24, self.style, np.array([- 8.00, -8.00, 0.00]),       0,      pi)
		self.nodeList[24] = Node(25, self.style, np.array([-13.13, -8.00, 0.00]),   -pi/4,  3*pi/4)
		self.nodeList[25] = Node(26, self.style, np.array([-23.72,  2.31, 0.00]),   -pi/2,    pi/2)
		self.nodeList[26] = Node(27, self.style, np.array([-17.00, 10.69, 0.00]),       0,      pi)
		self.nodeList[27] = Node(28, self.style, np.array([-17.00, 13.36, 0.00]),      pi,       0)
		self.nodeList[28] = Node(29, self.style, np.array([-22.50, 19.40, 0.00]),    pi/2,   -pi/2)
		self.nodeList[29] = Node(30, self.style, np.array([-25.22, 19.40, 0.00]),   -pi/2,    pi/2)
		self.nodeList[30] = Node(31, self.style, np.array([- 5.75,  0.00, 0.00]),   -pi/2,    pi/2)
		self.nodeList[31] = Node(32, self.style, np.array([- 3.16,  0.00, 0.00]),    pi/2,   -pi/2)
		self.nodeList[32] = Node(33, self.style, np.array([ 13.70,  0.00, 0.00]),   -pi/2,    pi/2)
		self.nodeList[33] = Node(34, self.style, np.array([ 16.35,  0.00, 0.00]),    pi/2,   -pi/2)
		self.nodeList[34] = Node(35, self.style, np.array([  7.70,- 8.00, 0.00]),       0,      pi)
		self.nodeList[35] = Node(36, self.style, np.array([  7.70,- 5.63, 0.00]),      pi,       0)
		self.nodeList[36] = Node(37, self.style, np.array([  3.20,- 5.63, 0.00]),      pi,       0)
		self.nodeList[37] = Node(38, self.style, np.array([  3.20,- 8.00, 0.00]),       0,      pi)
		self.nodeList[38] = Node(39, self.style, np.array([- 3.16, 23.80, 0.00]),    pi/2,   -pi/2)
		self.nodeList[39] = Node(40, self.style, np.array([- 5.75, 23.80, 0.00]),   -pi/2,    pi/2)
		self.nodeList[40] = Node(41, self.style, np.array([- 1.64, 27.25, 0.00]),    pi/4, -3*pi/4)
		self.nodeList[41] = Node(42, self.style, np.array([- 3.62, 28.82, 0.00]), -3*pi/4,    pi/4)
		self.nodeList[42] = Node(43, self.style, np.array([  3.28, 31.74, 0.00]),    pi/4, -3*pi/4)
		self.nodeList[43] = Node(44, self.style, np.array([  1.69, 33.76, 0.00]), -3*pi/4,    pi/4)
		self.nodeList[44] = Node(45, self.style, np.array([ 13.81, 27.93, 0.00]),   -pi/2,    pi/2)
		self.nodeList[45] = Node(46, self.style, np.array([ 16.37, 33.22, 0.00]),    pi/2,   -pi/2)
		self.nodeList[46] = Node(47, self.style, np.array([  2.90, 46.70, 0.00]),      pi,       0)
		self.nodeList[47] = Node(48, self.style, np.array([- 2.52, 44.11, 0.00]),       0,      pi)
		self.nodeList[48] = Node(49, self.style, np.array([-17.00, 44.11, 0.00]),       0,      pi)
		self.nodeList[49] = Node(50, self.style, np.array([-17.00, 46.77, 0.00]),      pi,       0)

		# Create 2-D numpy array of roads
		self.roadList = np.empty((self.num_nodes, self.num_nodes), dtype=Roadway)
		for i in range(self.num_nodes):
			for j in range(self.num_nodes):
				if self.G_dense[i, j] > 0:
					self.roadList[i, j] = Roadway(self.nodeList[i], self.nodeList[j])

		# Populate the 33 straight roads
		self.roadList[ 1-1][ 6-1].create_road_straight()
		self.roadList[ 2-1][28-1].create_road_straight()
		self.roadList[ 3-1][ 8-1].create_road_straight()
		self.roadList[ 4-1][39-1].create_road_straight()
		self.roadList[ 5-1][ 2-1].create_road_straight()
		self.roadList[ 6-1][10-1].create_road_straight()
		self.roadList[ 7-1][ 4-1].create_road_straight()
		self.roadList[ 8-1][31-1].create_road_straight()
		self.roadList[ 9-1][ 5-1].create_road_straight()
		self.roadList[11-1][14-1].create_road_straight()
		self.roadList[12-1][33-1].create_road_straight()
		self.roadList[13-1][12-1].create_road_straight()
		self.roadList[14-1][46-1].create_road_straight()
		self.roadList[15-1][17-1].create_road_straight()
		self.roadList[16-1][30-1].create_road_straight()
		self.roadList[18-1][16-1].create_road_straight()
		self.roadList[23-1][26-1].create_road_straight()
		self.roadList[24-1][38-1].create_road_straight()
		self.roadList[25-1][24-1].create_road_straight()
		self.roadList[26-1][25-1].create_road_straight()
		self.roadList[27-1][ 1-1].create_road_straight()
		self.roadList[29-1][15-1].create_road_straight()
		self.roadList[30-1][23-1].create_road_straight()
		self.roadList[32-1][ 7-1].create_road_straight()
		self.roadList[34-1][11-1].create_road_straight()
		self.roadList[36-1][37-1].create_road_straight()
		self.roadList[38-1][35-1].create_road_straight()
		self.roadList[40-1][ 3-1].create_road_straight()
		self.roadList[41-1][43-1].create_road_straight()
		self.roadList[44-1][42-1].create_road_straight()
		self.roadList[45-1][13-1].create_road_straight()
		self.roadList[47-1][50-1].create_road_straight()
		self.roadList[49-1][48-1].create_road_straight()

		# Populate the 25 90-degree curved roads
		self.roadList[ 1-1][ 4-1].create_road_turn_no_center(direction='+')
		self.roadList[ 1-1][ 8-1].create_road_turn_no_center(direction='-')
		self.roadList[ 3-1][ 2-1].create_road_turn_no_center(direction='-')
		self.roadList[ 3-1][ 6-1].create_road_turn_no_center(direction='+')
		self.roadList[ 5-1][ 4-1].create_road_turn_no_center(direction='-')
		self.roadList[ 5-1][ 8-1].create_road_turn_no_center(direction='+')
		self.roadList[ 7-1][ 2-1].create_road_turn_no_center(direction='+')
		self.roadList[ 7-1][ 6-1].create_road_turn_no_center(direction='-')
		self.roadList[10-1][12-1].create_road_turn_no_center(direction='-')
		self.roadList[10-1][14-1].create_road_turn_no_center(direction='+')
		self.roadList[11-1][ 9-1].create_road_turn_no_center(direction='+')
		self.roadList[13-1][ 9-1].create_road_turn_no_center(direction='-')
		self.roadList[15-1][19-1].create_road_turn_no_center(direction='-')
		self.roadList[17-1][49-1].create_road_turn_no_center(direction='-')
		self.roadList[18-1][19-1].create_road_turn_no_center(direction='+')
		# self.roadList[19-1][16-1].create_road_turn_no_center(direction='+')
		# self.roadList[19-1][17-1].create_road_turn_no_center(direction='-')
		self.roadList[28-1][23-1].create_road_turn_no_center(direction='+')
		self.roadList[28-1][29-1].create_road_turn_no_center(direction='-')
		self.roadList[30-1][27-1].create_road_turn_no_center(direction='+')
		self.roadList[31-1][38-1].create_road_turn_no_center(direction='+')
		self.roadList[33-1][36-1].create_road_turn_no_center(direction='-')
		self.roadList[35-1][34-1].create_road_turn_no_center(direction='+')
		self.roadList[37-1][32-1].create_road_turn_no_center(direction='-')
		self.roadList[50-1][18-1].create_road_turn_no_center(direction='+')

		# Populate the 12 custom roads
		self.roadList[20-1][22-1].create_road_turn_with_center(np.array([  9.86, 40.07]), direction='+')
		self.roadList[22-1][21-1].create_road_turn_with_center(np.array([  9.86, 40.07]), direction='+')
		self.roadList[21-1][20-1].create_road_turn_with_center(np.array([  9.86, 40.07]), direction='+')
		self.roadList[24-1][32-1].create_road_turn_with_center(np.array([-10.50,- 1.32]), direction='+')
		self.roadList[21-1][44-1].create_road_turn_with_center(np.array([- 1.81, 37.89]), direction='-')
		self.roadList[43-1][20-1].create_road_turn_with_center(np.array([  8.08, 27.04]), direction='-')
		self.roadList[39-1][41-1].create_road_turn_with_center(np.array([  2.59, 24.70]), direction='-')
		self.roadList[42-1][40-1].create_road_turn_with_center(np.array([  2.59, 24.70]), direction='+')
		self.roadList[20-1][45-1].roadpoints = np.array([[  9.69, 33.57, 0.00],
														  [10.74, 33.07, 0.00],
														  [11.69, 32.39, 0.00],
														  [12.42, 31.64, 0.00],
														  [13.06, 30.66, 0.00],
														  [13.54, 29.79, 0.00],
														  [13.78, 28.86, 0.00]])
		self.roadList[48-1][21-1].roadpoints = np.array([[ -1.49, 43.90, 0.00],
														  [-0.46, 43.67, 0.00],
														  [ 0.50, 43.24, 0.00],
														  [ 1.38, 42.72, 0.00],
														  [ 2.14, 42.03, 0.00],
														  [ 2.73, 41.09, 0.00],
														  [ 3.14, 40.04, 0.00]])
		self.roadList[22-1][47-1].roadpoints = np.array([[13.62, 45.47, 0.00],
														 [12.73, 45.99, 0.00],
														 [11.77, 46.37, 0.00],
														 [10.76, 46.60, 0.00],
														 [ 9.73, 46.70, 0.00],
														 [ 8.73, 46.70, 0.00],
														 [ 7.77, 46.70, 0.00],
														 [ 6.81, 46.70, 0.00],
														 [ 5.85, 46.70, 0.00],
														 [ 4.89, 46.70, 0.00],
														 [ 3.93, 46.70, 0.00]])
		self.roadList[46-1][22-1].roadpoints = np.array([[16.37, 34.20, 0.00],
														 [16.37, 35.18, 0.00],
														 [16.37, 36.16, 0.00],
														 [16.37, 37.14, 0.00],
														 [16.37, 38.12, 0.00],
														 [16.37, 39.10, 0.00],
														 [16.37, 40.08, 0.00],
														 [16.37, 41.06, 0.00],
														 [16.07, 42.14, 0.00],
														 [15.67, 43.12, 0.00],
														 [15.11, 44.02, 0.00]])
		
		if self.style=='right':
			G_dense_r = np.transpose(self.G_dense)
			roadList_r = np.empty((self.num_nodes, self.num_nodes), dtype=Roadway)
			for i in range(self.num_nodes):
				for j in range(self.num_nodes):
					if G_dense_r[i, j] > 0:
						roadList_r[i, j] = Roadway(self.nodeList[i], self.nodeList[j])
						roadList_r[i, j].roadpoints = np.flip(self.roadList[j, i].roadpoints, 0)
			self.G_dense = G_dense_r
			self.roadList = roadList_r

		self.update_connectivity_with_weights()

	def shortest_pathway(self, start, goal):
		graph = csr_matrix(self.G_dense)

		# Find the predecessors to all possible goals from the starting location
		d, predecessors = shortest_path(csgraph=graph, directed=True, indices=start, unweighted=True, return_predecessors=True)   

		# The goal index is your last item, store it in the solution
		solution = [goal+1]

		# Build the pathway in a loop
		while True:
			# if the predecessor to your goal is the starting position, this loop has achieved the task, termiante...
			if start == predecessors[goal]:
				break
			else:
				# the predecessor to the current goal is an intermediary, so update your goal and add it to the solution list
				goal = predecessors[goal]
				solution.insert(0, goal+1)
				
				# repeat...
				
		# insert the starting position into the solution and return
		solution.insert(0, start+1)

		return solution

	def update_connectivity_with_weights(self):
		for i in range(self.num_nodes):
			for j in range(self.num_nodes):
				if self.G_dense[i, j] > 0:
					self.G_dense[i, j] = self.roadList[i, j].roadpoints.shape[0]

	def generate_waypoints(self, nodes, factor=10):
		if nodes[0] == nodes[-1]:
			self.closedCircuit = True

		pathway = [nodes[0]]
		for index in range(len(nodes)-1):
			next_pathway = self.shortest_pathway(nodes[index] - 1, nodes[index+1] - 1)
			pathway = pathway + next_pathway[1:]

		currentNode = self.nodeList[pathway[0]-1]
		waypoint_list = []
		waypoint_list.append(currentNode[0].position/int(factor)) # append starting node
		for i in range(len(pathway)-1):
			roadpoints = self.roadList[pathway[i]-1][pathway[i+1]-1].roadpoints
			numpts, _ = roadpoints.shape
			for j in range(numpts):
				waypoint_list.append(roadpoints[j]/int(factor)) # append waypoints
			nextNode = self.nodeList[pathway[i+1]-1]
			waypoint_list.append(nextNode[0].position/int(factor)) # append interim node (including final)

		self.waypoint_list = waypoint_list
		self.pathway = pathway

	def generate_desired_nodes(self, nodeString):
		splitNodes = nodeString.split(',')
		desiredNodes = [int(x) for x in splitNodes]
		return desiredNodes[0], desiredNodes[-1], desiredNodes

	def human_machine_interface(self, distance_threshold=0, lookahead_additive=0):
		prev_waypoint = self.waypoint_list[0]
		next_waypoint = self.waypoint_list[1 + lookahead_additive]
		index = 1
		while True:
			car_front_axle_position = yield next_waypoint, prev_waypoint
			# print("front axle : ", car_front_axle_position)
			# print("next waypoints : ", next_waypoint[0:2])
			condition = np.linalg.norm(next_waypoint[0:2] - car_front_axle_position, 2)
			print("condition", condition)
			if np.linalg.norm(next_waypoint[0:2] - car_front_axle_position, 2) < distance_threshold:
				print("here!")
				if index == len(self.waypoint_list) - 1 - lookahead_additive: # if you hit the last waypoint, quit the loop.
					if self.closedCircuit == True:
						index = 0
					else:
						print('Reached the end')
						break            
				next_waypoint = self.waypoint_list[index + 1 + lookahead_additive]
				prev_waypoint = self.waypoint_list[index]            
				index = index + 1 

class QLabsWorkspace():
	def __init__(self, roadMap, uri="tcpip://localhost:18000"):
		# Initiate and open a session with Qlabs
		self.roadMap = roadMap
		self.qlabs = QuanserInteractiveLabs()
		print("Connecting to QLabs...")
		self.qlabs.open(uri)
		time.sleep(2)

		# destroy all spawned actors to reset the scenes
		print("Deleting current spawned actors...")
		self.qlabs.destroyAllSpawnedActors()
		time.sleep(2)

		# stop all quarc executables running for the .rt-win64 target
		os.system('quarc_run -q -t shmem://quarc-target:1 *.rt-win64')

	def terminate(self):
		self.qlabs.close()

	def updateRoadMap(self, roadMap):
		self.roadMap = roadMap

	def spawnVehicle(self, nodeID, carID=0):
		position = self.roadMap.nodeList[nodeID-1][0].position
		orientation = [0, 0, (180/np.pi)*np.math.atan2(self.roadMap.waypoint_list[1][1] - self.roadMap.waypoint_list[0][1], self.roadMap.waypoint_list[1][0] - self.roadMap.waypoint_list[0][0])]
		self.carID = carID
		QLabsQCar().spawnDegrees(self.qlabs, carID, position, orientation)

		return position, orientation

	def spawnVehicleByPose(self, position, orientation, carID=0):
		QLabsQCar().spawn(self.qlabs, carID, position, orientation)

	def spawnRoadPoints(self):
		# # Draw out pathway w/ waypoints in QLabs
		for i in range(len(self.roadMap.waypoint_list)):
			QLabsBasicShape().spawn(self.qlabs, i, self.roadMap.waypoint_list[i]*10, [0, 0, 0], [0.25, 0.25, 0.02], 1, False)
			time.sleep(.01)
		# self.qlabs.close()
		print("Spawned the QCar and all waypoints...")

	def setCameraToVehicle(self):
		QLabsQCar().possess(self.qlabs, self.carID, 7)

	def spawnCalibrationCheckerboard(self):
		shape_index = 0
		for v in range(9):
			for h in range(9):
				QLabsBasicShape().spawn(self.qlabs, shape_index, [-18 + h, 30, 0.5 + v], [0,0,0], [1, 0.1, 1], QLabsBasicShape().SHAPE_CUBE, False)
				
				if shape_index % 2 == 0:
					QLabsBasicShape().setMaterialProperties(self.qlabs, shape_index, [0.7, 0, 0], 0.0, metallic=False, waitForConfirmation=False)
				
				shape_index = shape_index + 1


	# creating different scenarios for behavior module
	myTrafficLight = QLabsTrafficLightSingle()
	myPedestrian = QLabsSilhouettePerson()
	crosswalk = QLabsCrosswalk()

	def spawnScenarioOne(self, trafficLightID):

		print("Scenerio one : ")
		
		self.myTrafficLight.spawn(self.qlabs, trafficLightID, [-0.056, 16.819, 3], [0, 0, -np.pi/2], [1, 1, 1])

		self.crosswalk.spawn(self.qlabs, 110110, [-4, 5.75, 0], [0, 0, 0], [1, 1, 1])
		self.crosswalk.spawn(self.qlabs, 110111, [-4, 18.25, 0], [0, 0, 0], [1, 1, 1])
		self.crosswalk.spawn(self.qlabs, 110112, [-10.6, 11.25, 0], [0, 0, np.pi/2], [1, 1, 1])
		self.crosswalk.spawn(self.qlabs, 110113, [1.75, 13.25, 0], [0, 0, np.pi/2], [1, 1, 1])

	def spawnScenarioTwo(self, trafficLightID):
		self.myTrafficLight.spawn(self.qlabs, trafficLightID, [-9.012, 16.184, 3], [0, 0, 0], [1, 1, 1])
		self.myPedestrian.spawnDegrees(self.qlabs, 0, [-9.012, 16.184, 0.215], [0, 0, 0], [1.0, 1.0, 1.0], 0)
		self.crosswalk.spawn(self.qlabs, 110110, [-4, 5.75, 0], [0, 0, 0], [1, 1, 1])
		self.crosswalk.spawn(self.qlabs, 110111, [-4, 18.25, 0], [0, 0, 0], [1, 1, 1])
		self.crosswalk.spawn(self.qlabs, 110112, [-10.6, 11.25, 0], [0, 0, np.pi/2], [1, 1, 1])
		self.crosswalk.spawn(self.qlabs, 110113, [1.75, 13.25, 0], [0, 0, np.pi/2], [1, 1, 1])
		
	def spawnScenarioThree(self, trafficLightID):
		self.myTrafficLight.spawn(self.qlabs, trafficLightID, [-9.012, 8.8, 3], [0, 0, 0], [1, 1, 1])
		self.myPedestrian.spawnDegrees(self.qlabs, 0, [-9.012, 6.184, 0.215], [0, 0, 0], [1.0, 1.0, 1.0], 0)
		
		self.crosswalk.spawn(self.qlabs, 110110, [-4, 5.75, 0], [0, 0, 0], [1, 1, 1])
		self.crosswalk.spawn(self.qlabs, 110111, [-4, 18.25, 0], [0, 0, 0], [1, 1, 1])
		self.crosswalk.spawn(self.qlabs, 110112, [-10.6, 11.25, 0], [0, 0, np.pi/2], [1, 1, 1])
		self.crosswalk.spawn(self.qlabs, 110113, [1.75, 13.25, 0], [0, 0, np.pi/2], [1, 1, 1])

	def setTrafficLightState(self, id, state):
		self.myTrafficLight.setState(self.qlabs, id, state)

	def movePedestrian(self, id, location):
		self.myPedestrian.moveTo(self.qlabs, id, location, 1)