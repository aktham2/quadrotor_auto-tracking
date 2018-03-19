import dlib # dlib for accurate face detection
import cv2 # opencv
import imutils # helper functions from pyimagesearch.com
import time
import RPi.GPIO as GPIO

# Grab video from your webcam
stream = cv2.VideoCapture(0)
# Face detector
detector = dlib.get_frontal_face_detector()
# Screen width for processing
SCREENWIDTH = 400
# Desired face location and width
DES_LOCATION = (SCREENWIDTH/2,SCREENWIDTH/2)
DES_WIDTH = SCREENWIDTH/20
tol = 10

# Initialize GPIO Pins for PWM output: roll,pitch,yaw,throttle
GPIO.setmode(GPIO.BOARD)
# Roll
GPIO.setup(5,GPIO.OUT)
roll = GPIO.PWM(5,1000)
roll.start(50)
# Pitch
GPIO.setup(18,GPIO.OUT)
pitch = GPIO.PWM(18,1000)
pitch.start(50)
# Yaw
GPIO.setup(13,GPIO.OUT)
yaw = GPIO.PWM(13,1000)
yaw.start(50)
# Throttle
GPIO.setup(19,GPIO.OUT)
throttle = GPIO.PWM(19,1000)
throttle.start(0)

def face():
    # read frames from live web cam stream
    (grabbed, frame) = stream.read()
 
    # resize the frames to be smaller and switch to gray scale
    frame = imutils.resize(frame, width=SCREENWIDTH)
    frame = cv2.flip(frame,0)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
 
 
    # detect faces in the gray scale frame
    face_rects = detector(gray, 0)
    # loop over the face detections
    for i, d in enumerate(face_rects):
        x1, y1, x2, y2, w, h = d.left(), d.top(), d.right() + 1, d.bottom() + 1, d.width(), d.height()
    if len(face_rects)>0:
        faceLocation = ((x1+x2)/2, (y1+y2)/2)
        faceWidth = x2-x1
        return faceLocation,faceWidth
    else:
        return (0,0),0

# On Start-Up: Takeoff
# pick it up

while True:
    faceLocation,faceWidth = face()
    if faceWidth != 0:
        xDiff = faceLocation[0] - DES_LOCATION[0]
        yDiff = faceLocation[1] - DES_LOCATION[1]
        widthDiff = faceWidth - DES_WIDTH
        if xDiff>tol:
            roll.ChangeDutyCycle(45)
            print "roll 45"
        elif xDiff<-tol:
            roll.ChangeDutyCycle(55)
            print "roll 55"
        else:
            roll.ChangeDutyCycle(50)
            
        if yDiff>tol:
            throttle.ChangeDutyCycle(45)
        elif yDiff<-tol:
            throttle.ChangeDutyCycle(55)
        else:    
            throttle.ChangeDutyCycle(50)
            
        if widthDiff>tol:
            pitch.ChangeDutyCycle(45)
        elif widthDiff<-tol:
            pitch.ChangeDutyCycle(55)
        else:    
            pitch.ChangeDutyCycle(50)
    else:
        roll.ChangeDutyCycle(50)
        pitch.ChangeDutyCycle(50)
        throttle.ChangeDutyCycle(50)
        print "no face"
