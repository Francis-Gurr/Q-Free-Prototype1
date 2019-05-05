"""
Authors: Francis Gurr, Vicky Miles
Date: 03/05/2019
Link:

Description: Establishes a serial connection with the OmniPreSense OPS242 using the pyserial package
             and reads the data to a CSV file.
"""

import csv
import math
import os
import serial
import time


def connect(port):
    """
    Description: Establishes a serial connection with the radar sensor

    :param port: Port name
    :return ser: Serial object
    :return t: Time the connection was established
    """
    ser = serial.Serial(port, 9600)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    setup(ser)
    return ser, t


def setup(ser):
    """
    Description: Resets the sensors clock and sets up the sensor to return time data and filter out values below 0.1 m/s

    :param ser: The serial object
    :return:
    """
    # Resets the sensors clock
    ser.write('C=0\r\n'.encode('utf-8'))
    time.sleep(0.5)
    print(ser.read_all())
    # Sets the time report to on
    ser.write(b'Ot\r\n')
    time.sleep(0.5)
    print(ser.read_all())
    # Sets the reported speed filter to filter out values below 0.1 m/s
    b = 'R>0.1\r\n'.encode('utf-8')
    ser.write(b)
    time.sleep(0.5)
    print(ser.read_all())
    ser.reset_input_buffer()
    ser.reset_output_buffer()




def getData(ser, t, t_max):
    """
    Description: Reads the data and applies the cosine error adjustment to the speed measurements
                 (See https://omnipresense.com/wp-content/uploads/2018/10/AN-011_Cosine-Error.pdf for more details)

    :param ser: The serial object
    :param t: Time the connection was established
    :param t_max: Maximum run time
    :return data: The radar sensor readings
    """
    data = []
    theta = math.radians(45)  # Angle of the radar to the road
    t_now = time.time()
    while t_now - t < t_max:
        data_val = ser.readline().decode('utf-8')[:-2]
        t_now = time.time()
        data.append([t_now, float(data_val) / math.cos(theta)])
    return data


def writeToFile(data):
    """
    Description: Writes the collected data to a CSV file

    :param data: Recorded sensor data
    :return:
    """
    data_dir = os.path.join(os.getcwd()[0:-5], 'Data')
    path = os.path.join(data_dir, 'radar_data.csv')
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerows(data)


ser, t = connect('COM6')
setup(ser)
data = getData(ser, t, 20)
writeToFile(data)
