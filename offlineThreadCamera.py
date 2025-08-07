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
		#crop display if desired
		fx, fy = 0.5, 1.0  
		cx, cy = self.width // 2, self.height // 2
		rw, rh = int(self.width * fx / 2), int(self.height * fy / 2)
		cv2.namedWindow('Meltpool',cv2.WINDOW_NORMAL)

		while True:
			if not self.display_queue.empty():
				frame = self.display_queue.get()
				cx, cy = self.width//2,self.height//2
				frame = frame[cy - rh:cy + rh, cx - rw:cx + rw]
				cv2.imshow('Meltpool, cropped',frame)
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
		self.temperature = Value('i',0)
		self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')
		self.width=1440
		self.height=1080
		self.fps = 30


	def capture_frames(self):
		camera = neoapi.Cam()
		camera.Connect()

		# --- Basic image settings ---
		camera.f.ExposureMode.SetString('Timed')
		camera.f.TriggerMode.SetString('Off')
		camera.f.ExposureAuto.SetString('Off')
		camera.f.GainAuto.SetString('Off')
		camera.f.PixelFormat.SetString('Mono8')

		# Set short exposure and gain
		camera.f.ExposureTime.Set(4.0)  # in µs
		camera.f.Gain.Set(11)

		# Set resolution
		camera.f.Width.Set(self.width)
		camera.f.Height.Set(self.height)

		# --- Set fixed frame rate ---
		camera.f.AcquisitionFrameRateEnable.Set(True)
		camera.f.AcquisitionFrameRate.Set(self.fps)

		# --- Timer1: delay laser pulse after exposure start ---
		camera.f.TimerSelector.SetString("Timer1")
		camera.f.TimerTriggerSource.SetString("ExposureStart")
		camera.f.TimerDelay.Set(1.0)      # Delay laser pulse 1 µs after exposure
		camera.f.TimerDuration.Set(3.0)   # Pulse duration ~3 µs

		# --- Line2: fire laser during Timer1Active ---
		camera.f.LineSelector.SetString("Line2")
		camera.f.LineMode.SetString("Output")
		camera.f.LineSource.SetString("Timer1Active")
		camera.f.LineInverter.Set(True)

		camera.f.ExposureTime.Set(4.0)  # in µs

		# Confirm camera is ready
		print(f'FPS: {camera.f.AcquisitionFrameRate.Get()}')
		print(f'ExposureTime: {camera.f.ExposureTime.Get()}')
		print(f'Gain: {camera.f.Gain.Get()}')

		while True:
			#SEE HOW LONG THIS TAKES BEFORE IMPLEMENTING
			self.temperature.value = self.camera.f.DeviceTemperature.Get()
			img = self.camera.GetImage()
			if img:
				frame = img.GetNPArray()
				if self.state_flag.value == 1:
					height, width = frame.shape[:2]
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
				if self.temperature.value != 0:
					frame = cv2.putText(frame,f'Temp is {self.temperature.value}',(10,130), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255), 2, cv2.LINE_AA)
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
		out = cv2.VideoWriter(str(time.time())+'.mp4', self.fourcc, self.fps, (self.width, self.height))
		while True:
			if not self.record_queue.empty():
				record_frame = self.record_queue.get()
				record_frame = cv2.cvtColor(record_frame, cv2.COLOR_GRAY2BGR)
				if record_frame is not None: 
					out.write(record_frame)
				else:
					print('empty frame recieved while recording')
			
			if self.close_video.value == True:
				while not self.record_queue.empty():
					record_frame = self.record_queue.get()
					record_frame = cv2.cvtColor(record_frame, cv2.COLOR_GRAY2BGR)
					if record_frame is not None:
						out.write(record_frame)
						print('dumping last frames')
					else:
						print('empty frame recieved while closing')

				out.release()
					
				out = cv2.VideoWriter(str(time.time())+'.mp4', self.fourcc, self.fps, (self.width, self.height))
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
		if key.char == 's':
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
	pressed_s = False
	pressed_r = False

	#CHECK FOR USER INPUTS
	while True:
		if pressed_r:
			print(f'\nSTARTED RECORDING')
			cavitar_camera.state_flag.value = 1
			pressed_r = False
		if pressed_s:
			print(f'\nSTOPPED RECORDING')
			cavitar_camera.state_flag.value = 0
			cavitar_camera.close_video.value = True
			pressed_s = False
		if pressed_p:
			print('\nCLOSING')
			meltpool_camera.state_flag.value = 3
			cavitar_camera.state_flag.value = 3
			meltpool_camera.display_process.join()
			cavitar_camera.display_process.join()
			print('display processes are dead')
			meltpool_camera.capture_frames.join()
			cavitar_camera.capture_frames.join()
			print('capture processes are dead')
			cavitar_camera.record_frames.join()
			print('record process is dead')
			print('program closed gracefully')
			break
		time.sleep(.01)



