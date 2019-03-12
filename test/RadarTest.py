import Radar
import serial
import time # Time module
#Serial port to which lidar connected, Get it from device manager windows
#In linux type in terminal -- ls /dev/tty*
port = input("Enter port name which radar is connected:") #windows
#port = "/dev/ttyUSB0" #linux

Obj = Radar.OPS242(port)
if(Obj.Connect()):
    Obj.SetUp()
    print(Obj.Read())
    Obj.Disconnect()
else:
    print("Error connecting to device")