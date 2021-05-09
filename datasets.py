import cv2
import numpy as np
import os
from picamera.array import PiRGBArray
from picamera import PiCamera
from threading import Thread
import time

FPS = 40
ROTATION = 0
HFLIP = False
VFLIP = True
RESOLUTION = (640, 480)

class PiStream:
    ''' Pi Camera Setup '''
    def __init__(self, resolution = RESOLUTION, framerate = FPS, rotation = ROTATION, hflip = HFLIP, vflip = VFLIP):
        ''' Class initialization '''
        self.camera = PiCamera()                
        self.camera.resolution = resolution
        self.camera.framerate = framerate
        self.camera.hflip = hflip
        self.camera.vflip = vflip
        self.rawCapture = PiRGBArray(self.camera, size = resolution)
        self.stream = self.camera.capture_continuous(self.rawCapture, format = "bgr", use_video_port = True)

        self.frame = None
        self.stopped = False

    def start(self):
        '''  Starting thread to read frames '''
        t = Thread(target = self.update)
        t.daemon = True
        t.start()
        return self

    def update(self):
        ''' Updating frames '''
        for frame in self.stream:
            self.frame = frame.array
            self.rawCapture.truncate(0)

            if self.stopped:
                self.stream.close()
                self.rawCapture.close()
                self.camera.close()
                return

    def read(self):
        ''' Reading most recent frame '''
        return self.frame

    def stop(self):
        ''' Stopping thread '''
        self.stopped = True
        
def check_path(path):
    dir = os.path.dirname(path)
    if not os.path.exists(dir):
        os.makedirs(dir)

cap = PiStream().start()
cap.camera.rotation = ROTATION
cap.camera.hflip = HFLIP
cap.camera.vflip = VFLIP

time.sleep(2)

face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

face_id = 1
count = 0

check_path("dataset/")

while True:
    img = cap.read()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)    
    faces = face_cascade.detectMultiScale(gray, 1.4, 5)

    for (x,y,w,h) in faces:
        cv2.rectangle(img, (x,y), (x+w,y+h), (255,0,0), 2)

        count += 1
        cv2.imwrite("dataset/User." + str(face_id) + "." + str(count) + ".jpg",gray[y:y+h,x:x+w])

    cv2.imshow("Frame", img)

    key = cv2.waitKey(20) & 0xFF

    if key == 27 or count == 400:
        break

cap.stop()
cv2.destroyAllWindows()
