import struct
import time
import serial
import matplotlib.pyplot as plt

#ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=None) #linux
ser = serial.Serial('COM11', 115200, timeout=None) #windows

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
	ser.write('v')
	return readFloat(), readFloat(), readFloat()
def getEncoderCount():
	ser.write('e')
	return readFloat(), readFloat(), readFloat()
def disengage():
	ser.write('d')
	
totalTime = 3   #seconds
sampleRate = 50 #samples per second
pulsePerRotation = 116.16 #New motors
#pulsePerRotation = 4955 #Old motors
speedM1 = 2
speedM2 = 2
speedM3 = 2

# Set the PIDQ values for all motors
setPID(0, 1, 1, 800)
#setPID(0, 1, 0, 800)

# Set tick period (triggers PID control) and velocity filter corner frequency
setT(20, 50)

#initialize arrays
times = []
speedsM1 = []
speedsM2 = []
speedsM3 = []

#setPower(80, 80, 80)
#setSpeed(0, 0, 0)
setSpeed(speedM1*pulsePerRotation, speedM2*pulsePerRotation, speedM3*pulsePerRotation)

for i in range(0,totalTime * sampleRate):
	time.sleep(1.0/sampleRate)
	times.append(i*1.0/sampleRate)
	speed = getSpeed()
	speedsM1.append(speed[0]/pulsePerRotation)
	speedsM2.append(speed[1]/pulsePerRotation)
	speedsM3.append(speed[2]/pulsePerRotation)
disengage()

plt.plot(times, speedsM1) #blue
plt.plot(times, speedsM2) #green
plt.plot(times, speedsM3) #red
plt.legend(['motor1', 'motor2', 'motor3'], loc='lower right')
plt.plot([0, totalTime], [speedM1, speedM1])


plt.ylabel('Rotations per second for ' + str(len(speedsM1)) + ' samples')
plt.xlabel('Sample Time (s)')
plt.show()