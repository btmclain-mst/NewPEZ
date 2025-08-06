import cv2
from multiprocessing import Process, Queue, Value, Manager,Lock
import time
import cherrypy
import numpy as np
from pynput import keyboard
import neoapi
import os

class USBVideoDisplay:
	##### Video States
	### 0 is idle
	### 1 is recording
	### 2 is stop recording
	### 3 is close program
	def __init__(self, video_source=0):
		self.display_queue = Queue(maxsize=10)  # Queue to hold frames
		self.record_queue = Queue(maxsize=600)
		self.close_video = Value('b', False)  # Shared flag to stop processes
		self.state_flag = Value('i', 0)
		self.wiggleometer_flag = Value('i', 0)
		self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')
		self.video_source = video_source
		self.record_state_flag = Value('i',0)

		self.fps = 30
		self.height = 1080
		self.width = 1920


	def capture_frames(self):
		self.capture = cv2.VideoCapture(self.video_source)
		self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
		self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
		self.capture.set(cv2.CAP_PROP_FPS, 30)  
		
		self.state, self.frame = self.capture.read()
		self.height, self.width, rgb= self.frame.shape
		self.fps = self.capture.get(cv2.CAP_PROP_FPS)

		print(f'process camera height is {self.height}')
		print(f'process camera width is {self.width}')
		print(f'process camera fps is {self.fps}')

		while True:
			start_time = time.time()
			ret, frame = self.capture.read()
			if ret:
				if self.state_flag.value == 1:
					self.record_queue.put(frame)
				if self.state_flag.value == 2:
					self.close_video.value = True
				self.display_queue.put(frame)
			else:
				empty_frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
				frame = cv2.putText(empty_frame,'RECIEVED BAD FRAME',(10,60), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255), 2, cv2.LINE_AA)
				self.display_queue.put(frame)

			if self.state_flag.value == 3:
				break

		self.capture.release()
		self.state_flag.value = 4

	def display_frames(self):
		crop = False
		roi_height = 1080
		roi_width = 1440
		cv2.namedWindow('Meltpool',cv2.WINDOW_NORMAL)
		while True:
			if not self.display_queue.empty():
				frame = self.display_queue.get()
				if crop:
					cx, cy = self.width//2,self.height//2
					x1 = max(cx-roi_width //2,0)
					x2 = min(cx+roi_width//2,self.width)
					y1 = max(cy - roi_height //2,0)
					y2 = min(cy+roi_height//2,self.height)
					roi = frame[y1:y2,x1:x2]
					cv2.imshow('Meltpool, cropped',roi)
					cv2.waitKey(1)
				else:
					cv2.imshow("Meltpool", frame)
					cv2.waitKey(1)
			if self.state_flag.value == 4:
				break

		while not self.display_queue.empty():
			frame = self.display_queue.get()
			cv2.imshow("Video", frame)
			cv2.waitKey(1)
		cv2.destroyAllWindows() 

class CavitarVideoDisplay:
	##### Video States
	### 0 is idle
	### 1 is recording
	### 2 is stop recording
	### 3 is close program
	def __init__(self,):
		self.display_queue = Queue(maxsize=10)  # Queue to hold frames
		self.record_queue = Queue(maxsize=600)
		self.state_flag = Value('i', 0)
		self.close_video = Value('b', False)  # Shared flag to stop processes
		self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')
		self.width=1440
		self.height=1080
		self.fps = 30


	def capture_frames(self):
		self.camera = neoapi.Cam()
		self.camera.Connect()
		
		print(f'camera connected')
		
		if self.camera.f.ExposureMode.IsWritable():
			self.camera.f.ExposureMode.SetString('Timed')
		else:
			print('could not set timed exposure mode')
			
		if self.camera.f.TriggerMode.IsWritable():
			self.camera.f.TriggerMode.SetString("Off")  # internal/free-run
		else:
			print('could not set internal free run mode')
			

		self.camera.f.ExposureTime.Set(20.0)
		self.camera.f.Gain.Set(11)

		self.camera.f.LineSelector.SetString('Line2')
		self.camera.f.LineMode.SetString('Output')
		self.camera.f.LineSource.SetString('ExposureActive')


		if self.camera.f.Width.IsWritable():
			self.camera.f.Width.Set(self.width)
		else:
			print('could not set width')
		if self.camera.f.Height.IsWritable():
			self.camera.f.Height.Set(self.height)
		else:
			print('could not set height')

		print(f' the line mode is set to an {self.camera.f.LineMode.GetString()}')
		print(f' the device temperature is {self.camera.f.DeviceTemperature.Get()}')
		
		while True:
			start_time = time.time()
			img = self.camera.GetImage()
			if img:
				frame = img.GetNPArray()
				if self.state_flag.value == 1:
					self.record_queue.put(frame)
				self.display_queue.put(frame)
			else:
				empty_frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
				frame = cv2.putText(empty_frame,'RECIEVED BAD FRAME',(10,60), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255), 2, cv2.LINE_AA)
				self.display_queue.put(frame)

			if self.state_flag.value == 3:
				break
		self.camera.f.LineSource.SetString('Off')
		self.camera.Disconnect()
		
		self.state_flag.value = 4

	def display_frames(self):
		cv2.namedWindow('Cavitar',cv2.WINDOW_NORMAL)
		while True:
			if not self.display_queue.empty():
				frame = self.display_queue.get()
				#frame = cv2.putText(frame,f'Temp: {self.camera.f.DeviceTemperature.Get()}',(10,130), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255), 2, cv2.LINE_AA)
				if self.state_flag.value == 1:
					frame = cv2.putText(frame,'Recording',(10,130), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255), 2, cv2.LINE_AA)
				cv2.imshow("Cavitar", frame)
				cv2.waitKey(1)
			
			if self.state_flag.value == 4:
				break

		while not self.display_queue.empty():
			frame = self.display_queue.get()
			cv2.imshow("Cavitar", frame)
			cv2.waitKey(1)
		cv2.destroyAllWindows() 

	def record_frames(self):
		name = str(time.time())
		out = cv2.VideoWriter(name+'.mp4', self.fourcc, self.fps, (self.width, self.height))
		while True:
			if not self.record_queue.empty():
				record_frame = self.record_queue.get()
				if record_frame is not None: 
					out.write(record_frame)
				else:
					print('empty frame recieved while recording')
			
			if self.close_video.value == True:
				while not self.record_queue.empty():
					record_frame = self.record_queue.get()
					if record_frame is not None: 
						out.write(record_frame)
						print('dumping last frames')
					else:
						print('empty frame recieved while closing')

				out.release()
					
				out = cv2.VideoWriter(name+'.mp4', self.fourcc, self.fps, (self.width, self.height))
				self.close_video.value = False

			if self.state_flag.value == 3:
				if not self.record_queue.empty():
					record_frame = self.record_queue.get()
					out.write(record_frame)
					print(f'dumping last frames')
				out.release()
				break

			time.sleep(.01)

def on_press(key):
	global pressed_p
	global pressed_r
	global pressed_s
	try:
		if key.char == 'p':
			pressed_p = True
		if key.char == 'r':
			pressed_r = True
		if key.char == 'r':
			pressed_s = True
	except AttributeError:
		if key == keyboard.Key.esc:
			return False  # allow Esc to stop listener


# Usage example
if __name__ == "__main__":

	meltpool_camera = USBVideoDisplay(video_source=0)
	meltpool_capture_process = Process(target=meltpool_camera.capture_frames)
	meltpool_display_process = Process(target=meltpool_camera.display_frames)

	meltpool_capture_process.start()
	meltpool_display_process.start()

	cavitar_camera = CavitarVideoDisplay()
	cavitar_capture_process = Process(target=cavitar_camera.capture_frames)
	cavitar_display_process = Process(target=cavitar_camera.display_frames)
	cavitar_record_process =  Process(target=cavitar_camera.record_frames)

	cavitar_capture_process.start()
	cavitar_display_process.start()
	cavitar_record_process.start()
	
	listener = keyboard.Listener(on_press=on_press)
	listener.start()

	pressed_p = False

	#CHECK FOR USER INPUTS
	while True:
		if pressed_r:
			print(f'\nSTARTED RECORDING')
			cavitar_camera.state_flag.value = 1
		if pressed_s:
			print(f'\nSTOPPED RECORDING')
			cavitar_camera.state_flag.value = 0
			cavitar_camera.close_video.value = True
		if pressed_p:
			print('\nCLOSING')
			meltpool_camera.state_flag.value = 3
			cavitar_camera.state_flag.value = 3
			
			break
		time.sleep(.01)



