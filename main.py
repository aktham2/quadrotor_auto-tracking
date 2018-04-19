import dlib # dlib for accurate face detection
import cv2 # opencv
import imutils # helper functions from pyimagesearch.com
import time
import pigpio
from pynput import keyboard
import threading
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

        
# Radio control values duty cycle, out of 10000
LOW = 1150
MED = 1200
HI = 1250

# Throttle values
LOW_THR = 1250
MED_THR = 1275
HI_THR = 1300

# Initialize GPIO Pins for PWM output: roll,pitch,yaw,throttle
pi = pigpio.pi()

# pins numbering
global R, P, T, Y
R = 3
P = 18
T = 19
Y = 13

# DC frequency
f = 73.53

# set range of duty cycle from 0-100
pi.set_PWM_range(R,10000)
pi.set_PWM_range(P,10000)
pi.set_PWM_range(T,10000)
pi.set_PWM_range(Y,10000)
# initialize frequency to 73.5 Hz
pi.set_PWM_frequency(R,f)
pi.set_PWM_frequency(P,f)
pi.set_PWM_frequency(T,f)
pi.set_PWM_frequency(Y,f)

# initialize duty cycles
pi.set_PWM_dutycycle(R,MED)
pi.set_PWM_dutycycle(P,MED)
pi.set_PWM_dutycycle(T,LOW_THR)
pi.set_PWM_dutycycle(Y,MED)

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
    return (negFlag*f)

def land():
    pi.set_PWM_dutycycle(T,LOW_THR)
    time.sleep(3)
    arm(False)

# Arm or disarm (AUX1)
def arm(arming):
    if arming:
        pi.set_PWM_dutycycle(17,800)
    else:
        pi.set_PWM_dutycycle(17,1200)

######################
def on_press(key):
    try:
        print('alphanumeric key {0} pressed'.format(
            key.char))
        #bump(key.char)            
    except AttributeError:
        print 'wrong key'

def on_release(key):
    print('{0} released'.format(
        key))
    if key == keyboard.Key.esc:
        # Stop listener
        return False

def bump(key):
    if key == 'w':
        pi.set_PWM_dutycycle(T,HI_THR)
    
#########################

##keythread = threading.Thread(target=keyMonitor)
##keythread.daemon = True
##keythread.start()





# arm
arm(True)
loop = True
# Takeoff
pi.set_PWM_dutycycle(T,HI_THR)
time.sleep(2)
pi.set_PWM_dutycycle(T,MED_THR)

nofaceCount = 0
while loop:
    try:
        faceLocation,faceWidth = face()
        if faceWidth != 0:
            xErr = faceLocation[0] - DES_LOCATION[0]
            yErr = -faceLocation[1] + DES_LOCATION[1]
            widthErr = faceWidth - DES_WIDTH
            print quadControl(xErr)
            pi.set_PWM_dutycycle(R,MED + quadControl(xErr))
            pi.set_PWM_dutycycle(P,MED + quadControl(widthErr))
            pi.set_PWM_dutycycle(T,MED_THR + quadControl(yErr))
            nofaceCount = 0
        else:
            pi.set_PWM_dutycycle(R,MED)
            pi.set_PWM_dutycycle(P,MED)
            pi.set_PWM_dutycycle(T,MED_THR)
            print "no face"
            nofaceCount = nofaceCount + 1
        if nofaceCount>7:
            land()
            loop = False
        
    except KeyboardInterrupt:
        arm(False)
        loop = False
