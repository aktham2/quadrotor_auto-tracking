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
SCREENWIDTH = 600
# Desired face location and width
DES_LOCATION = (SCREENWIDTH/2,SCREENWIDTH/2)
DES_WIDTH = SCREENWIDTH/6
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

# calculate duty cycle for PWM based upon gradient descent
# for control of desired roll,pitch,yaw,throttle
# input: err, [-200,200]
# output: duty cycle, about [40, 60] 
def quadControl(err):
    # threshold for linear -> quadratic gradient descent
    b = 100
    # descent coefficient
    k = 0.0005
    # positive error means we need to decrease, and vice versa
    negFlag = -1
    if err<0:
        err = err*-1
        negFlag = 1
    if err<=b:
        f = 0.5*k*err**2
    else:
        f = k*b*err - 0.5*k*b**2
    return (50 + negFlag*f)


    
# On Start-Up: Takeoff
# pick it up

while True:
    faceLocation,faceWidth = face()
    if faceWidth != 0:
        xErr = faceLocation[0] - DES_LOCATION[0]
        yErr = -faceLocation[1] + DES_LOCATION[1]
        widthErr = faceWidth - DES_WIDTH
        print quadControl(xErr)
        roll.ChangeDutyCycle(quadControl(xErr))
        pitch.ChangeDutyCycle(quadControl(widthErr))
        throttle.ChangeDutyCycle(quadControl(yErr))
    else:
        roll.ChangeDutyCycle(50)
        pitch.ChangeDutyCycle(50)
        throttle.ChangeDutyCycle(50)
        print "no face"
