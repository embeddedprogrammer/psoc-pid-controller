import struct
import time
import serial
import matplotlib.pyplot as plt
import threading
import sys

#ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=None) #linux
ser = serial.Serial('COM11', 115200, timeout=None) #windows
stopFlag = False
printingReceivedCharacters = True

def writeFloat(f):
	ser.write(struct.pack('>i', int(f*1000)))
def readFloat():
	return float(struct.unpack('>i', ser.read(4))[0])/1000
def setPower(p1, p2, p3):
	ser.write('p')
	writeFloat(p1)
	writeFloat(p2)
	writeFloat(p3)
def setSpeed(s1, s2, s3):
	ser.write('s')
	writeFloat(s1)
	writeFloat(s2)
	writeFloat(s3)
def setPID(motor, p, i, qpps): #use motor = 0 to set all motors
	ser.write('k')
	ser.write(str(motor))
	writeFloat(p)
	writeFloat(i)
	writeFloat(qpps)
def setT(period_ms, tau_ms):
	ser.write('t')
	writeFloat(period_ms)
	writeFloat(tau_ms)
def getSpeed():
	global printingReceivedCharacters
	printingReceivedCharacters = False
	ser.write('v')
	retVal = readFloat(), readFloat(), readFloat()
	printingReceivedCharacters = True
	return retVal 
def getEncoderCount():
	global printingReceivedCharacters
	printingReceivedCharacters = False
	ser.write('e')
	retval = readFloat(), readFloat(), readFloat()
	printingReceivedCharacters = True
	return retval
def disengage():
	ser.write('d')
def serialRead():
	while not stopFlag:
		if printingReceivedCharacters and ser.inWaiting() > 0:
			sys.stdout.write(ser.read(ser.inWaiting())) #We use this function so that we don't write new lines
		time.sleep(1.0/1000) #Wait 1ms (prevent high cpu consumption)
	
#create a separate thread for reading from serial port
readLoop = threading.Thread(target=serialRead)
readLoop.start()

totalTime = 3   #seconds
sampleRate = 50 #samples per second
pulsePerRotation = 116.16 #New motors
#pulsePerRotation = 4955 #Old motors
speedM1 = 2
speedM2 = 2
speedM3 = 2

# Set the PIDQ values for all motors
setPID(0, 1, 1, 800)
time.sleep(.1)

# Set tick period (triggers PID control) and velocity filter corner frequency
setT(20, 50)
time.sleep(.1)

#initialize arrays
times = []
speedsM1 = []
speedsM2 = []
speedsM3 = []

#setPower(255, 255, 255)
#setSpeed(0, 0, 0)
setSpeed(speedM1*pulsePerRotation, speedM2*pulsePerRotation, speedM3*pulsePerRotation)
time.sleep(.1)

for i in range(0,totalTime * sampleRate):
	time.sleep(1.0/sampleRate)
	times.append(i*1.0/sampleRate)
	speed = getSpeed()
	speedsM1.append(speed[0]/pulsePerRotation)
	speedsM2.append(speed[1]/pulsePerRotation)
	speedsM3.append(speed[2]/pulsePerRotation)
disengage()

# stop debug printing and close serial port.
time.sleep(.1)
stopFlag = True
readLoop.join()
ser.close()

plt.plot(times, speedsM1) #blue
plt.plot(times, speedsM2) #green
plt.plot(times, speedsM3) #red
plt.legend(['motor1', 'motor2', 'motor3'], loc='lower right')
plt.plot([0, totalTime], [speedM1, speedM1])


plt.ylabel('Rotations per second for ' + str(len(speedsM1)) + ' samples')
plt.xlabel('Sample Time (s)')
plt.show()

