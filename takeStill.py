import picamera 

def takeStill(filename):
	camera = picamera.PiCamera()
	camera.vflip = True
	fileName = 'pictures/' + str(filename) + '.jpg'
	camera.capture(fileName)
	return 0
