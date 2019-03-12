import serial
import time
import math
name = "Radar"
class OPS242:

    def __init__(self, port):
        """Initialize the connection and set port and baudrate."""
        self.__port = port
        self.__baudrate = 9600
        self.__is_scanning = False
        self.__is_connected = False

    def Connect(self):
        """Begin serial connection with Lidar by opening serial port.\nReturn success status True/False.\n"""
        try:
            if (not self.__is_connected):
                self.__s = serial.Serial(self.__port, self.__baudrate)
                self.__is_connected = True
                time.sleep(0.5)
                if self.__s.is_open:
                    return True
                else:
                    return False
            else:
                return False
        except Exception as e:
            return False

    def SetUp(self):
        self.__s.write(b'OT') #time report on
        self.__s.write(b'Oj') #json mode off
        return

    def Read(self): #Read 100 lines
        line = self.__s.read(100)
        return line

    def LED_test(self):
        self.__s.write(b'Ol')
        print('LEDs off')
        time.sleep(5)
        self.__s.write(b'OL')
        print('LEDs 0n')
        time.sleep(3)
        return

    def Disconnect(self):
        """Stop scanning and close serial communication with Lidar."""
        try:
            if (self.__is_connected):
                self.__s.close()
                self.__is_connected = False
                return True
            else:
                return False
        except Exception as e:
            return False