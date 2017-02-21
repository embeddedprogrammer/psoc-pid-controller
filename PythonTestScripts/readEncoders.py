import struct
import time
import serial
import matplotlib.pyplot as plt

ser = serial.Serial('COM11', 115200, timeout=None) #windows
#ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=None) #linux
#ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=None) #linux, (read note on webpage about ttyAMA0 first)

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

# Set tick period (triggers PID control) and velocity filter corner frequency
setT(20, 50)

raw_input("Press Enter to continue...")
print getEncoderCount()
raw_input("Press Enter to continue...")
print getEncoderCount()
raw_input("Press Enter to continue...")
print getEncoderCount()