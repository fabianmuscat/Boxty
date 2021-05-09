#-----------------------------------------------------------------------------------------------------------------------------------------------
# PiCamera modules
from picamera.array import PiRGBArray
from picamera import PiCamera
from threading import Thread
# -------------------------------------

# GPIO modules
import RPi.GPIO as GPIO
from time import sleep, time
# -------------------------------------

import cv2 #OpenCV
import numpy as np

import pigpio
import sys
import os

# Global Variables ----------------------------------------------------------------------------------------
face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
RESOLUTION = (640, 480)

H_WIDTH = int(RESOLUTION[0]/2) #Half screen width
H_HEIGHT = int(RESOLUTION[1]/2) #half screen height
img_center = (H_WIDTH, H_HEIGHT)

# Pi Camera Settings
FPS = 40
ROTATION = 0
HFLIP = False
VFLIP = True
##

SERVO = 5

MOTION = 9
BUZZER = 19
LEFT_LED = 6
RIGHT_LED = 13

LEDS = [LEFT_LED,RIGHT_LED]


pi = pigpio.pi()

#Timer
global start
start = time()
timeout = 60

#----------------------------------------------------------------------------------------------------------

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

def setup():
    ''' GPIO Components Setup '''
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(MOTION, GPIO.IN)

    pi.set_mode(SERVO, pigpio.OUTPUT) #setup using the pigpio library
    GPIO.setup(BUZZER, GPIO.OUT)
    GPIO.setup(LEDS, GPIO.OUT)

    global l_led
    global r_led
    l_led = GPIO.PWM(LEFT_LED, 2)
    r_led = GPIO.PWM(RIGHT_LED, 2)
    
    #Starting LEDs at 50% of their full brightness
    l_led.start(50) 
    sleep(0.25)
    r_led.start(50)
    
    #Checking for motion
    channel = GPIO.add_event_detect(MOTION, GPIO.RISING, callback=motion)


def destroy():
    ''' Ensuring a clean exit '''
    l_led.stop()
    r_led.stop()
    final_position()
    GPIO.cleanup()
    
def init_servo_position():
    ''' Setting the sero to its initial position '''
    global pw

    '''1500 is the center (90 degrees) of the servo, 
    500 (0 degrees) is the minimum and 2500 is the maximum (180 degrees)'''
    pw = 1500 
    pi.set_servo_pulsewidth(SERVO, pw)

def final_position():
    ''' Setiing the servo to its final position when program is terminated '''
    pi.set_servo_pulsewidth(SERVO, 1500)
    sleep(0.3)

def set_pw(num, RIGHT_BOOL, LEFT_BOOL):
    '''Setting the pulsewidth for the servo.

    num - the number which will increment the pulsewidth
    LEFT_BOOL - boolean for left LED
    RIGHT_BOOL - boolean for right LED'''
    
    global pw
    pw += (num)

    led_on_off(RIGHT_LED,RIGHT_BOOL) #turning the right led on/off
    led_on_off(LEFT_LED,LEFT_BOOL) #turning the left led on/off

    return pw

def check_limits_right(OFFSET, MAX, MAX_PW=2400):
    '''Checking the offset's limits when the servo is panning to the right.

    OFFSET - the difference from the center of a detected face and the center of the screen.
    MAX - the maximum offset on the right of the screen.
    MAX_PW - the maximum pulsewidth.'''
    
    return OFFSET > MAX and pw <= MAX_PW

def check_limits_left(OFFSET, MIN, MIN_PW=501):
    '''Checking the offset's limits when the servo is panning to the left.

    OFFSET - the difference from the center of a detected face and the center of the screen.
    MIN - the minimum offset on the left of the screen.
    MIN_PW - the minimum pulsewidth.'''
    
    return OFFSET < MIN and pw >= MIN_PW
    
def pan_left_right(x):
    '''Panning the servo left and right to track a face.'''
    offset = x - H_WIDTH
    max_right = 30
    min_left = -max_right
    
    if check_limits_right(offset, max_right): #checking if the servo reached the maximum angle (180 degrees)
        set_pw(20, 1, 0) #setting the pulsewidth and turning on/off the LEDs

    elif check_limits_right(offset, 80):
        set_pw(40, 1, 0)

    elif check_limits_right(offset, 160):
        set_pw(60, 1, 0)

    elif check_limits_right(offset, 80):
        set_pw(80, 1, 0)

    #############################################

    elif check_limits_left(offset, min_left): #checking if the servo reached the minimum angle (0 degrees)
        set_pw(-20, 0, 1)
        
    elif check_limits_left(offset, -80):
        set_pw(-40, 0, 1)

    elif check_limits_left(offset, -160):
        set_pw(-60, 0, 1)

    elif check_limits_left(offset, -240):
        set_pw(-80, 0, 1)
        

    if pw > 2350:
        # Beeping the buzzer if the servo is almost at 180 degrees.
        GPIO.output(BUZZER, GPIO.HIGH)
        sleep(0.2)
        GPIO.output(BUZZER, GPIO.LOW)
        sleep(0.2)

    elif pw < 550:
        # Beeping the buzzer if the servo is almost at 0 degrees.
        GPIO.output(BUZZER, GPIO.HIGH)
        sleep(0.2)
        GPIO.output(BUZZER, GPIO.LOW)
        sleep(0.2)

    if offset < max_right and offset > min_left:
        #lighting both LEDs if the face is centered.
        led_on_off(RIGHT_LED,True)
        led_on_off(LEFT_LED,True)
        
    pi.set_servo_pulsewidth(SERVO, pw)
    sleep(0.03)

def buzzer(loop, loop_time, time):
    ''' Playing some beeps with the buzzer '''
    for i in range(loop):
        GPIO.output(BUZZER, GPIO.HIGH)
        sleep(loop_time)
        GPIO.output(BUZZER, GPIO.LOW)
        sleep(loop_time)

    GPIO.output(BUZZER, GPIO.HIGH)
    sleep(time)
    GPIO.output(BUZZER, GPIO.LOW)
    sleep(time)

def scanning_leds():
    ''' LEDs will turn on and off '''
    l_led.ChangeDutyCycle(20)
    sleep(0.2)
    l_led.ChangeDutyCycle(0)

    r_led.ChangeDutyCycle(20)
    sleep(0.2)
    r_led.ChangeDutyCycle(0)

def stop_scan():
    '''Stops leds'''
    l_led.stop()
    r_led.stop()

def led_on_off(LED,BOOL):
    ''' Turning the led on or off '''
    GPIO.output(LED,BOOL)
    
def motion(MOTION):
    global start
    
    start = time()
    #print(start)
    #print('Resetting timer')
#-----------------------------------------------------------------------------------------------------------------------------------------------

def face_track():
    ''' Recognizing a specific face and tracking it '''
    recognizer = cv2.face.LBPHFaceRecognizer_create() #opencv recognizer
    recognizer.read('trainer.yml') #reading face data
    font = cv2.FONT_HERSHEY_COMPLEX

    id = 0
    global cap
    global start
    
    cap = PiStream().start()
    cap.camera.rotation = ROTATION
    cap.camera.hflip = HFLIP
    cap.camera.vflip = VFLIP

    sleep(2)
    buzzer(3,0.3,0.7)
    stop_scan()
    scanning = True

    init_servo_position() #Setting the servo to its initial position.
    while scanning:
        if time() - start < timeout: #checking if time is less then the timeout (60)
            #print(time() - start)
            img = cap.read() #reading the data from the camera
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) #converting the image to grayscale for better contrast

            faces = face_cascade.detectMultiScale(gray, 1.2, 8) #checking for faces using the face cascade.
                  
            for (x,y,w,h) in faces:
                center = ((int(x + w/2)), (int(y + h/2)))
                cv2.rectangle(img, (x,y), (x+w, y+h), (0,255,0), 2)

                roi_gray = gray[y:y+h, x:x+w]
                id, confidence = recognizer.predict(roi_gray)

                fc = x+(w/2) #face centre
                fc = int(fc)

                #pan_left_right(fc)

                if confidence > 80:
                    continue

                elif confidence <= 80:
                    id = "Fabian + {0:.2f}%".format(round(confidence, 2))
                    cv2.putText(img, str(id), (x+20, y), font, 1, (255,255,255), 2)
                    pan_left_right(fc)

            cv2.imshow('Boxty', img)
        
            key = cv2.waitKey(0.3) & 0xFF #Escape key

            #The program will be terminated if 'escape' is pressed
            if key == 27:
                final_position()
                scanning = False
            
        else:
            scanning = False
            
#-----------------------------------------------------------------------------------------------------------------------------------------------
if __name__ == '__main__':
    setup()
    try:
        face_track()
            
        #print(time()-start)
        final_position()
        cap.stop()
        cv2.destroyAllWindows()
        destroy()

    except KeyboardInterrupt:
        destroy()