"""
Authors: Francis Gurr, Vicky Miles
Date: 03/05/2019
Link:

Description: Combines the LiDAR and radar data and produces a two dimensional plot of the side profile of a vehicle
"""

import csv
import math
import matplotlib.pyplot as plt
import numpy as np
import os


def setConstants():
    """
    Description: Calibration constants of the radar

    :return:
    """
    # Distance of the radar to the road
    global d
    d = 0.5
    # Angle of the radar to the road
    global theta
    theta = 45

    # Path to the data directory
    global data_dir
    data_dir = os.path.join(os.getcwd()[0:-5], 'Data')


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
            row_float = list(map(float, row))
            data.append(row_float)
    return data


def getVehicles(lidar_data):
    """
    Description: Groups the list of LiDAR data read from file into a list of vehicles

    :param lidar_data: List of LiDAR data
    :return vehicles: List of vehicles
    """
    vehicles = []
    vehicle = []
    vehicle_num = 1
    for line in lidar_data:
        if line[0] == vehicle_num:
            vehicle.append(line[1:])
        else:
            vehicles.append(vehicle)
            vehicle = []
            vehicle_num = vehicle_num + 1
            vehicle.append(line[1:])
    vehicles.append(vehicle)
    return vehicles


def time_offset(speed):
    """
    Description: Calculates the time offset between the vehicle passing the radar and LiDAR

    :param speed: speed of the vehicle
    :return time_offset: The time offset
    """
    dist_offset = d * math.tan(math.radians(theta))
    time_offset = dist_offset / speed
    return time_offset


def getSpeeds(vehicles, radar_data):
    """
    Description: Gets the speed of each vehicle from the radar_data

    :param vehicles: List of LiDAR vehicle data
    :param radar_data: Radar data
    :return v_speeds: List of vehicle speeds
    """
    v_speeds = []
    for vehicle in vehicles:
        t_min = vehicle[0][0]  # Start time of the vehicle passing the LiDAR
        t_max = t_min + vehicle[-1][0]  # End time of the vehicle passing the LiDAR
        speeds = []
        for row in radar_data:
            # Applies the time offset correction to the radar data
            t = row[0] + time_offset(row[1])
            # If the time is within the same range as that of the LiDAR data
            # add to the list of speed measurements for this vehicle
            if t_min < t < t_max:
                speeds.append(row[1])
        # Average the speed measurements of this vehicle and add to list of vehicle speeds
        speed_av = sum(speeds) / len(speeds)
        v_speeds.append(speed_av)
    return v_speeds


def getDist(vehicles, v_speeds):
    """
    Description: Replace the time measurements in the LiDAR data with the
                 calculated distance between each row using the vehicle speed

    :param vehicles: List of LiDAR vehicle data
    :param v_speeds: List of vehicle speeds
    :return:
    """
    for i in range(len(v_speeds)):
        freq = len(vehicles[i]) / vehicles[i][-1][0]
        dist_1s = v_speeds[i] / freq
        for line in vehicles[i]:
            line[0] = line[0] * dist_1s


def plot(vehicles):
    """
    Description: Plots a two dimensional side profile of each vehicle

    :param vehicles: List of vehicle data
    :return:
    """
    v_num = 0
    for vehicle in vehicles:
        all_y = np.array(vehicle[1:])
        x = []
        y = []
        for row in all_y:
            for i in range(1, row.shape[0]):
                if not row[i] == 0:
                    x.append(row[0])
                    y.append(row[i])

        fig = plt.figure(figsize=(6.0, 4.0))
        # plt.ylim(-1.75, 1.25)
        # plt.xlim(0, 0.7)
        plt.xlabel('Time (s)')
        plt.ylabel('Height (m)')
        plt.scatter(x, y, c='r', s=8)
        # plt.show()
        fn = 'plot' + str(v_num) + '.png'
        path = os.path.join(data_dir, 'Plots', fn)
        fig.savefig(path, format='png', dpi=300)
        v_num = v_num + 1


setConstants()
# radar_data = loadData('radar_data.csv')
lidar_data = loadData('lidar_vehicle.csv')
vehicles = getVehicles(lidar_data)
# v_speeds = getSpeeds(vehicles, radar_data)
# getDistances(vehicles, v_speeds)
plot(vehicles)
