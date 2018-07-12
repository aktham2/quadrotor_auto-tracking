import dlib # dlib for accurate face detection
import cv2 # opencv
import imutils # helper functions from pyimagesearch.com
import time
import pigpio
from pynput import keyboard
import threading

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

class ArmPin(Pin):
    def isArmed(self):
        return self.pwmDutyCycle <= 900

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
# def quadControl(err):
#     # threshold for linear -> quadratic gradient descent
#     b = 100
#     # descent coefficient
#     k = 0.0005
#     # positive error means we need to decrease, and vice versa
#     negFlag = -1
#     if err<0:
#         err = err*-1
#         negFlag = 1
#     if err<=b:
#         f = 0.5*k*err**2
#     else:
#         f = k*b*err - 0.5*k*b**2
#     return (negFlag*f)

# Using PD control
def quadControl(err, oldErr):
    # gains
    kP = 0.09
    kD = 100
    # PD control
    output = kP*abs(err) - kD*(abs(oldErr-err))
    return output*np.sign(-err)


def land():
    T.setDutyCycle(LOW_THR)
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
    global MED_THR, LOW_THR, HI_THR, trackFace

    # Check if quad is armed before changing values
    if not ARM.isArmed() and key != 'm':
        print("Please arm first.")
        print("Arm: M   Track: Y   Pitch: WS   Roll: AD   Yaw: QE   Throttle: PL")
        return

    if key == 'p':
        MED_THR = MED_THR + 5
        T.setDutyCycle(MED_THR)
        LOW_THR = MED_THR - 25
        HI_THR = MED_THR + 25
    elif key == 'l':
        MED_THR = MED_THR - 4
        T.setDutyCycle(MED_THR)
        LOW_THR = MED_THR - 25
        HI_THR = MED_THR + 25
    elif key == 'w':
        P.setDutyCycle(HI)
        time.sleep(0.25)
        P.setDutyCycle(MED)
    elif key == 's':
        P.setDutyCycle(LOW)
        time.sleep(0.25)
        P.setDutyCycle(MED)
    elif key == 'a':
        R.setDutyCycle(LOW)
        time.sleep(0.25)
        R.setDutyCycle(MED)
    elif key == 'd':
        R.setDutyCycle(HI)
        time.sleep(0.25)
        R.setDutyCycle(MED)
    elif key == 'q':
        Y.setDutyCycle(LOW)
        time.sleep(0.25)
        Y.setDutyCycle(MED)
    elif key == 'e':
        Y.setDutyCycle(HI)
        time.sleep(0.25)
        Y.setDutyCycle(MED)
    elif key == 'm':
        toggleArm()
    elif key == 'y':
        trackFace = not trackFace
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
detector = dlib.get_frontal_face_detector()
# Screen width for processing
SCREENWIDTH = 600
# Desired face location and width
DES_LOCATION = (SCREENWIDTH/2,SCREENWIDTH/2)
DES_WIDTH = SCREENWIDTH/6

# Default values, disarm quad
global pwmRange, f, armed
pwmRange = 10000
f = 73.53
armed = False

# duty cycle values, out of 10000
global LOW, MED, HI
LOW = 1150
MED = 1200
HI = 1250

# Throttle values
global LOW_THR, MED_THR, HI_THR
LOW_THR = 1250
MED_THR = 1275
HI_THR = 1300

# Initialize GPIO Pins for PWM output: roll,pitch,yaw,throttle
global pi
pi = pigpio.pi()

# Pin names
global R, P, T, Y, ARM

# Face tracking mode
global trackFace
trackFace = False
    
if __name__ == "__main__":
    # Create pins
    Y = Pin(13,pwmRange,f,LOW)
    R = Pin(3, pwmRange,f,LOW)
    P = Pin(18,pwmRange,f,LOW)
    T = Pin(19,pwmRange,f,LOW_THR-500)
    ARM = ArmPin(17,pwmRange,f,HI)

    # Setup key listener
    keyThread = threading.Thread(target=keyListener,args=[])
    keyThread.daemon = True
    keyThread.start()

    # Wait for user to be ready
    while not trackFace:
        try:
            printState()
            time.sleep(0.5)
        except:
            break
     
    # If the quad is not armed, do not enter the loop
#     if not ARM.isArmed():
#         trackFace = False

    
    # Begin face tracking    
    noFaceCount = 0
    while trackFace:
        try:
            faceLocation,faceWidth = face()
            print("")
            if faceWidth != 0:
                xErr = faceLocation[0] - DES_LOCATION[0]
                yErr = -faceLocation[1] + DES_LOCATION[1]
                widthErr = faceWidth - DES_WIDTH
                R.setDutyCycle(MED + quadControl(xErr,xErrOld))
                P.setDutyCycle(MED + quadControl(widthErr,widthErrOld))
                T.setDutyCycle(MED_THR + quadControl(yErr,yErrOld))
                noFaceCount = 0
                print("Found face.")
                xErrOld = xErr
                yErrOld = yErr
                widthErrOld = widthErr
            else:
                R.setDutyCycle(MED)
                P.setDutyCycle(MED)
                T.setDutyCycle(MED_THR)
                print("No face detected.")
                noFaceCount += 1
            if noFaceCount > 20:
                trackFace = False
        except Exception as e:
            # Exit the loop and land
            print(e)
            trackFace = False
        # Print out RPYT values    
        printState()

    # Land and disarm on loop exit
    land()
    arm(False)
    
    
   
    

    

    
    
