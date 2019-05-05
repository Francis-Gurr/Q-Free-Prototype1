"""
Authors: Francis Gurr, Vicky Miles
Date: 03/05/2019
Link:

Description: Establishes a serial connection and obtains raw data with the YdLidarG4
            using the PyLidar3 package from Lakshman Mallidi.
            The raw data can be used for calibration of the sensor
            or processed and grouped by vehicle detected and written to a CSV file.
"""

import csv
import PyLidar3
import math
import matplotlib.pyplot as plt
import numpy as np
import numpy.ma as ma
import os
import time


def setConstants(indoor):
    """
    Description: Sets the calibrated constants

    :param indoor: Boolean used to select between indoor test rig calibration constants or outdoor constants
    :return:
    """
    # Port name
    global port
    port = 'COM3'
    # Correction angle to zero the horizontal plane of the LiDAR
    global zero
    # Minimum distance filter
    global x_min
    # Maximum distance filter
    global x_max
    # Minimum angle filter
    global theta_min
    # Maximum angle filter
    global theta_max

    # Path to the data directory
    global data_dir
    data_dir = os.path.join(os.getcwd()[0:-5], 'Data')

    if indoor:
        zero = 119
        x_min = 0.45
        x_max = 0.55
        theta_min = 100
        theta_max = 125
    else:
        zero = 119
        x_min = 2
        x_max = 3.5
        theta_min = 50
        theta_max = 150


def runLidar(t_max):
    """
    Description: Establishes a serial connection with the LiDAR, starts scanning and returns the data

    :param t_max: Maximum run time
    :return raw_data: The raw data
    """
    lidar = PyLidar3.YdLidarG4(port)
    if lidar.Connect():
        print(lidar.GetDeviceInfo())
        gen = lidar.StartScanning()
        t1 = time.time()  # start time
        t2 = t1
        raw_data = []
        while t2 - t1 < t_max:
            d = next(gen)
            t2 = time.time()
            raw_data.append([t2, d])
        lidar.StopScanning()
        lidar.Disconnect()
        return raw_data
    else:
        print("Error connecting to device")
        return 0


def processData(raw):
    """
    Description: Processes the raw data from a list of dictionaries to a table
    with the column headings 0 to 360 degrees
    This is then written to a file

    :param raw: Raw data
    :return proc_data: Processed data
    """
    # Process data
    proc_data = []
    degs = list(raw[0][1].keys())
    proc_data.append([raw[0][0]] + degs[theta_min:theta_max])
    for line in raw:
        data = list(line[1].values())
        data = list(map(lambda x: x * 0.00025, data))  # Converts the distance values to metres
        proc_data.append([line[0]] + data[theta_min:theta_max])
    # Write to file
    path = os.path.join(data_dir, 'lidar_data.csv')
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerows(proc_data)
    return proc_data


def calibrationPlot(data):
    """
    Description: Plots an average of the raw data collected to be used for sensor calibration

    :param data: LiDAR data
    :return:
    """
    angles = data[0][1:]
    dist = np.array(list(map(lambda x: x[1:], data[1:])))
    x = []
    y = []

    # Counts the number of zero and non zero measurements per degree and uses this to obtain a simple confidence value
    # The more data for a set of degree measurements the higher the confidence value
    total = len(dist)
    non_zero_count = np.count_nonzero(dist, axis=0)
    conf = np.divide(non_zero_count, total)

    # Calculates the mean measurement for each degree excluding the zero measurements
    mean = np.nanmean(ma.masked_where(dist == 0, dist), axis=0).data

    # Converts the polar measurements to cartesian coordinates
    for i in range(len(angles)):
        x.append(mean[i] * math.cos(math.radians(angles[i] - zero)))
        y.append(mean[i] * -math.sin(math.radians(angles[i] - zero)))

    # Reduces the limits of the x and y axes according to the current calibration constants
    # This visually shows what data will be kept and which will be discarded
    fig = plt.figure(figsize=(6.0, 4.0))
    xlim_mean = (x_max + x_min) / 2
    ylim_max = xlim_mean * -math.sin(math.radians(theta_min - zero))
    ylim_min = xlim_mean * -math.sin(math.radians(theta_max - zero))
    plt.ylim(ylim_min, ylim_max)
    plt.xlim(x_min, x_max)
    plt.xlabel('Horizontal Distance (m)')
    plt.ylabel('Height (m)')
    plt.scatter(x, y, c=conf, s=8, cmap='winter')
    plt.colorbar()
    # plt.show()
    fn = 'calibrationPlot' + '.png'
    path = os.path.join(data_dir, 'Plots', fn)
    fig.savefig(path, format='png', dpi=300)


def getVehicles(data):
    """
    Description: Groups and organises the data per vehicle detected and writes it to a file

    :param data: The processed data
    :return vehicles: The data grouped by vehicle
    """
    degs = data[0][1:]  # List of the degrees
    col_head = degs  # Column headings: 0, 1, ... , n
    gap = 11  # Number of consecutive data rows with no data measurements within calibrated range
    vehicles = []  # List of individual vehicle data sets
    vehicle_num = 0  # Number of vehicles
    row_num = 10  # Number of data rows per vehicle
    t_start = 0  # Absolute time of the start of a vehicle
    for row in data[1:]:
        t_row = row[0]  # Absolute time value of the data in current row
        d_val = row[1:]  # Distance values of current row
        x_horizontal = row[degs.index(123)] * math.cos(math.radians(123 - zero))  # Horizontal distance value

        # If there is no data measurement within the calibrated range increase the gap
        if x_horizontal < x_min or x_horizontal > x_max:
            gap = gap + 1
        # Else there is a vehicle passing
        else:
            # If the gap is larger than 10 start a new vehicle
            if gap > 10 and row_num > 0:
                row_num = 0
                vehicle_num = vehicle_num + 1
                t_start = t_row
                vehicles.append([[vehicle_num, t_start]])
                vehicles[-1][-1].extend(col_head)
                row_num = row_num + 1
                vehicles[-1].append([vehicle_num, 0])
            # Else still previous vehicle
            else:
                row_num = row_num + gap + 1
                t_rel = t_row - t_start
                vehicles[-1].append([vehicle_num, t_rel])

            gap = 0
            # Add data row to the vehicle
            for i in range(len(d_val)):
                angle = degs[i]
                x = (d_val[i] * math.cos(math.radians(angle - zero)))
                # If the x distance is out of the calibrated range set it to zero
                if x < x_min or x > x_max:
                    d_val[i] = 0
                # Else set it to the y value only as x can be ignored
                else:
                    if not angle - zero == 0:
                        y = (d_val[i] * -math.sin(math.radians(angle - zero)))
                        d_val[i] = y
                    else:
                        d_val[i] = 0.000001
            vehicles[-1][-1].extend(d_val)
    # Write to file
    path = os.path.join(data_dir, 'lidar_vehicle.csv')
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        for vehicle in vehicles:
            writer.writerows(vehicle)
    return vehicles


def loadData(fn):
    """
    Description: Loads previously collected data from a file

    :param fn: Filename
    :return data: Data from file
    """
    path = os.path.join(data_dir, fn)
    data = []
    with open(path, newline='') as f:
        reader = csv.reader(f)
        for row in reader:
            row_int = list(map(float, row))
            data.append(row_int)
    return data


setConstants(True)
# raw = runLidar(60)
# proc_data = processData(raw)
proc_data = loadData('lidar_data.csv')
# calibrationPlot(proc_data)
vehicles = getVehicles(proc_data)
