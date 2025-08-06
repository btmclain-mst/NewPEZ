import neoapi
import numpy as np
import cv2
import time

#connect to camera
camera = neoapi.Cam()
camera.Connect()

#set exposure mode
camera.f.ExposureMode.SetString('Timed')
camera.f.TriggerMode.SetString("Off") 

#set camera settings
camera.f.ExposureTime.Set(20.0)
camera.f.Gain.Set(11)

#set resolution
w=1440
h=1080
camera.f.Width.Set(w)
camera.f.Height.Set(h)

#double check settings are correct
print(f' the camera exposure is set to {camera.f.ExposureTime.Get()}')
print(f' the camera gain is set to {camera.f.Gain.Get()}')
print(f' the camera exposure mode is {camera.f.ExposureMode.GetString()}')
print(f' the camera trigger mode is {camera.f.TriggerMode.GetString()}')
print(f' the camera auto exposure is set to {camera.f.ExposureAuto.GetString()}')
print(f' the camera auto gain is set to {camera.f.GainAuto.GetString()}')
print(f' the device temperature is {camera.f.DeviceTemperature.Get()}')

#turn on lasers
camera.f.LineSelector.SetString('Line2')
camera.f.LineMode.SetString('Output')
camera.f.LineSource.SetString('ExposureActive')
camera.f.LineInverter.Set(True)

#capture images as fast as possible
while True:
	img = camera.GetImage()
	if img:
		frame = img.GetNPArray()
		cv2.imshow("Cavitar Camera", frame)
		print(frame.shape)

	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

#turn off lasers and stop recording
camera.f.LineSource.SetString('Off')
camera.Disconnect()

