import neoapi
import numpy as np
import cv2
import time

camera = neoapi.Cam()
camera.Connect()

if camera.f.ExposureMode.IsWritable():
	camera.f.ExposureMode.SetString('Timed')
else:
	print('could not set timed exposure mode')
	
if camera.f.TriggerMode.IsWritable():
    camera.f.TriggerMode.SetString("Off")  # internal/free-run
else:
	print('could not set internal free run mode')
	

camera.f.ExposureTime.Set(30.0)
camera.f.Gain.Set(11)
TARGET_FPS = 30
w=1440
h=1080
if camera.f.Width.IsWritable():
	camera.f.Width.Set(w)
else:
	print('could not set width')
if camera.f.Height.IsWritable():
	camera.f.Height.Set(h)
else:
	print('could not set height')


print(f' the camera exposure is set to {camera.f.ExposureTime.Get()}')
print(f' the camera gain is set to {camera.f.Gain.Get()}')
print(f' the camera exposure mode is {camera.f.ExposureMode.GetString()}')
print(f' the camera trigger mode is {camera.f.TriggerMode.GetString()}')
print(f' the camera auto exposure is set to {camera.f.ExposureAuto.GetString()}')
print(f' the camera auto gain is set to {camera.f.GainAuto.GetString()}')

camera.f.LineSelector.SetString('Line2')
camera.f.LineMode.SetString('Output')
camera.f.LineSource.SetString('ExposureActive')

print(f' the line mode is set to an {camera.f.LineMode.GetString()}')
print(f' the device temperature is {camera.f.DeviceTemperature.Get()}')

camera.f.LineInverter.Set(True)

while True:
	img = camera.GetImage()
	if img:
		frame = img.GetNPArray()
		cv2.imshow("Cavitar Camera", frame)


	if cv2.waitKey(1) & 0xFF == ord('q'):
		break


camera.f.LineSource.SetString('Off')
camera.Disconnect()

