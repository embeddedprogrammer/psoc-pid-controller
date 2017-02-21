#!/usr/bin/python

import threading, time, serial

stopFlag = False #Flag to determine when to end program
readAsBytes = False # Flag to read serial data as bytes instead of text

ser = serial.Serial('COM11', 115200, timeout=None) #windows
#ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=None) #linux
#ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=None) #linux, (read note on webpage about ttyAMA0 first)

def serialWrite():
  global stopFlag
  global readAsBytes
  while True:
    dataToSend = raw_input("Byte Command>>> ")
    if dataToSend == "":
      continue
    if dataToSend == "bytes":
      readAsBytes = True
      print "Incoming serial data will now be printed as individual bytes"
      continue
    if dataToSend == "text":
      readAsBytes = False
      print "Incoming serial data will now be treated as text"
      continue
    if dataToSend == "exit":
      stopFlag = True
      print "Exiting serialWrite"
      return
    bytesToSend = dataToSend.split()
    cmdToSend = ""
    try:
      for x in range(0, len(bytesToSend)):
        cmdToSend = cmdToSend + chr(int(bytesToSend[x]));
      ser.write(cmdToSend) 
    except ValueError:
      print "Error: Only byte-level commands can be sent. The command must only contain integers from 0-255 separated by whitespace"

def serialRead():
  while stopFlag==False:
    if ser.inWaiting() > 0:
      if readAsBytes:
        dataToRead = ser.read(ser.inWaiting())
        dataToPrint = ""
        for x in range(0, len(dataToRead)):
          dataToPrint = dataToPrint + str(ord(dataToRead[x])) + " "
        print dataToPrint
      else:
        dataToRead = ser.readline()
        if dataToRead != "":
          print dataToRead
      ser.flush()
  print "Exiting serialRead"

try:
  print "Type 'exit' to close program. Type byte commands delimited by whitespace to send"
  writeLoop = threading.Thread(target=serialWrite) #create a separate thread for writing to serial port
  readLoop = threading.Thread(target=serialRead) #create a separate thread for reading from serial port
  writeLoop.start()
  readLoop.start()
  writeLoop.join() #wait for this thread to finish
  readLoop.join() #wait for this thread to finish
  print "Exiting main thread"
  ser.close()
    
except KeyboardInterrupt:
    ser.close()