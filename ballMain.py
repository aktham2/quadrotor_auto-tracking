import dlib # dlib for accurate face detection
import cv2 # opencv
import imutils # helper functions from pyimagesearch.com
import time
import pigpio
from pynput import keyboard
import threading
import numpy as np

class Pin:
    def __init__(self,pinNumber,rng,frq,cycle):
        self.pin = pinNumber
        self.pwmRange = rng
        self.pwmFrequency = frq
        self.pwmDutyCycle = cycle

        pi.set_PWM_range(pinNumber,rng)
        pi.set_PWM_frequency(pinNumber,frq)
        pi.set_PWM_dutycycle(pinNumber,cycle)

    def setDutyCycle(self,newCycle):
        self.pwmDutyCycle = newCycle
        pi.set_PWM_dutycycle(self.pin,newCycle)

    def getDutyCycle(self):
        return self.pwmDutyCycle

    def setFrequency(self,frq):
        self.pwmFrequency = frq
        pi.set_PWM_frequency(self.pin,frq)

    def getFrequency(self):
        return self.pwmFrequency

    def setRange(self,rng):
        self.pwmRange = rng
        pi.set_PWM_range(self.pin,rng)

    def getRange(self):
        return self.pwmRange

    def trimUp(self):
        self.pwmDutyCycle += 4
        self.setDutyCycle(self.pwmDutyCycle)

    def trimDown(self):
        self.pwmDutyCycle -= 4
        self.setDutyCycle(self.pwmDutyCycle)
        

class ArmPin(Pin):
    def isArmed(self):
        return self.pwmDutyCycle <= 900

def ball():
    # read frames from live web cam stream
    (grabbed, frame) = stream.read()
 
    # resize the frames to be smaller and switch to HSV colorspace
    frame = imutils.resize(frame, width=SCREENWIDTH)
    frame = cv2.flip(frame,0)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Red colorspace (HSV)
    redLowerLOW = np.array([0,70,70])
    redUpperLOW = np.array([10,255,255])
    redLowerHIGH = np.array([170,70,70])
    redUpperHIGH = np.array([180,255,255])

    # Create mask for red
    mask0 = cv2.inRange(hsv,redLowerLOW,redUpperLOW)
    mask1 = cv2.inRange(hsv,redLowerHIGH, redUpperHIGH)
    mask = mask0+mask1

    # Erode small blobs, dilate prominate features
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None

    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # only proceed if the radius meets a minimum size
        if radius > 10:
            return (x,y),radius
    return (0,0),0


# calculate duty cycle for PWM based upon gradient descent
# for control of desired roll,pitch,yaw,throttle
# input: err, [-200,200]
# output: duty cycle
##def quadControl(err):
##    # threshold for linear -> quadratic gradient descent
##    b = 1
##    # descent coefficient
##    k = 0.15
##    # positive error means we need to decrease, and vice versa
##    negFlag = -1
##    if err<0:
##        err *= -1
##        negFlag = 1
##    if err<=b:
##        f = 0.5*k*err**2
##    else:
##        f = k*b*err - 0.5*k*b**2
##    if f > 100:
##        f = 100
##    return (negFlag*f)

# PD control
def quadControl(err, oldErr):
    # gains
    kP = 0.09
    kD = 100
    # PD control
    output = kP*abs(err) - kD*(abs(oldErr-err))
    return output*np.sign(-err)



def land():
    T.setDutyCycle(LOW_THR-100)
    print("Landing...")
    time.sleep(10)

def on_press(key):
    try:
        bump(key.char)            
    except AttributeError:
        print("Arm: M   Track: Y   Pitch: WS   Roll: AD   Yaw: QE   Throttle: PL")

def on_release(key):
    if key == keyboard.Key.esc:
        # Stop listener
        return False

def bump(key):
    global MED_THR, LOW_THR, HI_THR, trackBall, YMED, RMED

    # Check if quad is armed before changing values
##    if not ARM.isArmed() and key != 'm':
##        print("Please arm first.")
##        print("Arm: M   Track: Y   Pitch: WS   Roll: AD   Yaw: QE   Throttle: PL")
##        return

    if key == 'p':
        T.trimUp()
    elif key == 'l':
        T.trimDown()
    elif key == 'w':
        P.trimUp()
    elif key == 's':
        P.trimDown()
    elif key == 'a':
        R.trimDown()
    elif key == 'd':
        R.trimUp()
    elif key == 'q':
        Y.trimDown()
    elif key == 'e':
       Y.trimUp()
    elif key == 'm':
        toggleArm()
    elif key == 'y':
        trackBall = not trackBall
    else:
        print("Arm: M   Track: Y   Pitch: WS   Roll: AD   Yaw: QE   Throttle: PL")

def keyListener():
    # Collect events until released
    with keyboard.Listener(
        on_press=on_press,
        on_release=on_release) as listener:
        listener.join()

def toggleArm():
    if ARM.getDutyCycle() <= 900:
        ARM.setDutyCycle(1200)
        print("DISARMED")
    else:
        ARM.setDutyCycle(800)
        print("ARMED")

def arm(a):
    if a:
        ARM.setDutyCycle(800)
        print("ARMED")
    else:
        ARM.setDutyCycle(1200)
        print("DISARMED")

def printState():
    print("\n*** Armed: "+str(ARM.isArmed())+" *** Roll: "+str(R.getDutyCycle())+" *** Pitch: "+str(P.getDutyCycle())+" *** Yaw: "+str(Y.getDutyCycle())+" *** Throttle: "+str(T.getDutyCycle())+" ***")


# Grab video from your webcam
stream = cv2.VideoCapture(0)
# Face detector
#detector = dlib.get_frontal_face_detector()
# Screen width for processing
SCREENWIDTH = 600
# Desired face location and width
DES_LOCATION = (SCREENWIDTH/2,SCREENWIDTH/2)
DES_RADIUS = 40

# Default values for PWM
global pwmRange, f, armed
pwmRange = 10000
f = 73.53

# HI for arming, LOW_THR for landing
global HI, LOW_THR
HI = 1250
LOW_THR = 1250

# Initialize GPIO Pins for PWM output: roll,pitch,yaw,throttle
global pi
pi = pigpio.pi()

# Pin names
global R, P, T, Y, ARM

# Ball tracking mode
global trackBall
trackBall = False
    
if __name__ == "__main__":
    # Create pins
    Y = Pin(13,pwmRange,f,1200)
    R = Pin(3, pwmRange,f,1200)
    P = Pin(18,pwmRange,f,1200)
    T = Pin(19,pwmRange,f,800)
    ARM = ArmPin(17,pwmRange,f,HI)

    # Setup key listener
    keyThread = threading.Thread(target=keyListener,args=[])
    keyThread.daemon = True
    keyThread.start()

    # Wait for user to be ready
    while not trackBall:
        try:
            printState()
            time.sleep(0.5)
        except:
            break
     
    # If the quad is not armed, do not enter the loop
##    if not ARM.isArmed():
##        trackBall = False

    
    # Begin ball tracking    
    noBallCount = 0

    # Get stable values for each input, approximately hover
    YMED = Y.getDutyCycle()
    RMED = R.getDutyCycle()
    PMED = P.getDutyCycle()
    TMED = T.getDutyCycle()

    ballLocation, ballRadius = ball()
    xErrOld = ballLocation[0] - DES_LOCATION[0]
    yErrOld = ballLocation[1] - DES_LOCATION[1]
    widthErrOld = ballRadius - DES_RADIUS
    while trackBall:
        try:
            ballLocation,ballRadius = ball()
            print("")
            if ballRadius != 0:
                xErr = ballLocation[0] - DES_LOCATION[0]
                yErr = ballLocation[1] - DES_LOCATION[1]
                print("x error: "+str(xErr))
                print("y error: "+str(yErr))
                widthErr = ballRadius - DES_RADIUS
                R.setDutyCycle(RMED + quadControl(xErr,xErrOld)*1)
                P.setDutyCycle(PMED + quadControl(widthErr,widthErrOld)*1)
                T.setDutyCycle(TMED + quadControl(yErr,yErrOld)*1)
                noBallCount = 0
                print("Found ball.")
                xErrOld = xErr
                yErrOld = yErr
                widthErrOld = widthErr
            else:
                R.setDutyCycle(RMED)
                P.setDutyCycle(PMED)
                T.setDutyCycle(TMED)
                Y.setDutyCycle(YMED)
                print("No ball detected.")
                noBallCount += 1
            if noBallCount > 200:
                trackBall = False
        except:
            # Exit the loop and land
            trackBall = False
        # Print out RPYT values    
        printState()

    # Land and disarm on loop exit
    land()
    arm(False)
    
    
   
    

    

    
    
