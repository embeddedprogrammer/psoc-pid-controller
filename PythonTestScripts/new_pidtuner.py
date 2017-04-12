import time
import struct
import serial
import matplotlib.pyplot as plt

#ser = serial.Serial('COM11', 115200, timeout=None) #windows
#ser = serial.Serial('/dev/ttyS0', 115200, timeout=None) #linux
ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=None) #linux, (read note on webpage about ttyAMA0 first)

# Note: If you would like to port the readFloat and writeFloat functions to C++, simply use the functions provided
# in the PSoC implementation: https://github.com/embeddedprogrammer/psoc-pid-controller/blob/master/serial.c
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
def setAdvanced(motor, pwm_offset, dither_pwm, dither_period): #use motor = 0 to set all motors
        ser.write('a')
        ser.write(str(motor))
        writeFloat(pwm_offset)
        writeFloat(dither_pwm)
        writeFloat(dither_period)
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

totalTime = 30 #5   #seconds
sampleRate = 50 #samples per second
pulsePerRotation = 4955 #Old motors
#pulsePerRotation = 116.2 #New motors


# Set the PIDQ values for all motors
setPID(0, 1.0, 0.3, 50000)
#setPID(1, -1.3, -0.7, 49000)
#setPID(2, -1.3, -0.7, 48000)
#setPID(3, -1.3, -0.7, 52000)

# setAdvanced(motor, pwm_offset, dither_pwm, dither_period)
offset = 30
dither_pwm = 0
dither_period = 0.04
setAdvanced(0, offset, dither_pwm, dither_period)


# Set tick period (triggers PID control) and velocity filter corner frequency
setT(20, 100)

#initialize arrays
times = []
speedsM1 = []
speedsM2 = []
speedsM3 = []
commands = []

#setPower(128,128,128)

speedM1 = 3.0 # rot/s
speedM2 = 3.0 # rot/s
speedM3 = 3.0 # rot/s

setSpeed(speedM1*pulsePerRotation, speedM2*pulsePerRotation, speedM3*pulsePerRotation)
total_samples = totalTime * sampleRate

for i in range(0,totalTime * sampleRate):
        time.sleep(1.0/sampleRate)
        times.append(i*1.0/sampleRate)
        speed = getSpeed()
        print speed
        speedsM1.append(speed[0]/pulsePerRotation)
        speedsM2.append(speed[1]/pulsePerRotation)
        speedsM3.append(speed[2]/pulsePerRotation)


speeds = getSpeed()
print(speeds)

disengage()


plt.plot(times, speedsM1) #blue
plt.plot(times, speedsM2) #green
plt.plot(times, speedsM3) #red
plt.legend(['motor1', 'motor2', 'motor3'], loc='lower right')
#plt.plot([0, totalTime], [speedM1, speedM1])
plt.plot(times, commands)

plt.ylabel('Rotations per second for ' + str(len(speedsM1)) + ' samples')
plt.xlabel('Sample Time (s)')
plt.show()
