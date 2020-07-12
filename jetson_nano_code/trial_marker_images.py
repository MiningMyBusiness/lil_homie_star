import cv2
import time
import glob

# GStreamer Pipeline to access the Raspberry Pi camera
GSTREAMER_PIPELINE = 'nvarguscamerasrc ! video/x-raw(memory:NVMM), width=3280, height=2464, format=(string)NV12, framerate=21/1 ! nvvidconv flip-method=0 ! video/x-raw, width=960, height=616, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink wait-on-eos=false max-buffers=1 drop=True'

# Video capturing from OpenCV
video_capture = cv2.VideoCapture(GSTREAMER_PIPELINE, cv2.CAP_GSTREAMER)
time.sleep(2)

# grab image
return_key, frame = video_capture.read()

# close video stream
video_capture.release()
        
# save image
this_img_name = 'images/marker_1_3_BedroomDoorPrinter.jpg'
cv2.imwrite(this_img_name, frame)