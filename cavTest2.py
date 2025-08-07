import neoapi
import numpy as np
import cv2
import time

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
camera.f.Width.Set(1440)
camera.f.Height.Set(1080)

# --- Set fixed frame rate ---
camera.f.AcquisitionFrameRateEnable.Set(True)
camera.f.AcquisitionFrameRate.Set(50.0)

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

# --- Image capture loop ---
while True:
    start = time.time()
    time.sleep(0.01)
    img = camera.GetImage(timeout=1000)
    print(f'capture fps is {1/(time.time()-start)}')
    if img:
        frame = img.GetNPArray()
        cv2.imshow("Cavitar Video @30FPS", frame)


    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# --- Cleanup ---
camera.f.LineSource.SetString("Off")
camera.Disconnect()

