import neoapi
import numpy as np
import cv2
import time

# Connect to camera
camera = neoapi.Cam()
camera.Connect()

# Set exposure mode
if camera.f.ExposureMode.IsWritable():
	camera.f.ExposureMode.SetString('Timed')
else:
	print('Could not set timed exposure mode')



# Set trigger mode
if camera.f.TriggerMode.IsWritable():
	camera.f.TriggerMode.SetString("Off")  # internal/free-run
else:
	print('Could not set internal free run mode')

# Check and set exposure time
min_exp = camera.f.ExposureTime.GetMin()
print(f"Minimum allowed exposure time: {min_exp} µs")
camera.f.ExposureTime.Set(30.0)

# Set gain
camera.f.Gain.Set(11)

# Set resolution
w = 1440
h = 1080
if camera.f.Width.IsWritable():
	camera.f.Width.Set(w)
else:
	print('Could not set width')
if camera.f.Height.IsWritable():
	camera.f.Height.Set(h)
else:
	print('Could not set height')

# Print camera settings
print(f'Camera exposure: {camera.f.ExposureTime.Get()} µs')
print(f'Camera gain: {camera.f.Gain.Get()}')
print(f'Exposure mode: {camera.f.ExposureMode.GetString()}')
print(f'Trigger mode: {camera.f.TriggerMode.GetString()}')
print(f'Auto exposure: {camera.f.ExposureAuto.GetString()}')
print(f'Auto gain: {camera.f.GainAuto.GetString()}')

# Get camera temperature
start = time.time()
temp = camera.f.DeviceTemperature.Get()
print(f'Getting camera temp took {time.time() - start:.4f} s')
print(f'Device temperature: {temp} °C')

# Configure Line2 to toggle laser
camera.f.LineSelector.SetString('Line2')
camera.f.LineMode.SetString('Output')
camera.f.LineSource.SetString('ExposureActive')
camera.f.LineInverter.Set(True)
print(f'Line mode: {camera.f.LineMode.GetString()}')

# Start capturing images
while True:
	img = camera.GetImage()
	if img:
		frame = img.GetNPArray()
		cv2.imshow("Cavitar Camera", frame)

	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

# Cleanup
camera.f.LineSource.SetString('Off')
camera.Disconnect()

