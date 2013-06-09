#-------------------------------------------------------------------------------
# Name:        plot_sensors_event.py
# Purpose:     Plots logged sensors_event_t data from logger.c CSV files
#
# Author:      K. Townsend
#
# Created:     09/06/2013
# Copyright:   (c) K. Townsend 2013
# Licence:     BSD
#-------------------------------------------------------------------------------

import numpy as np
import matplotlib.pyplot as plt

filename = "C:\Users\Kevin\Desktop\sensors.txt"

def main():
    # Load the CSV file in 'data'
    data = np.genfromtxt(filename,
        delimiter=',',
        dtype="i32,i32,i32,f32,f32,f32,f32",
        names=['id','type','timestamp','x','y','z','a'])

    # Display the results
    plt.title("sensors_event_t Data")
    plt.xlabel('Timestamp (ms)')
    plt.ylabel('Value')
    plt.xlim(data['timestamp'].min(), data['timestamp'].max()*1.1)
    plt.grid(True)
    plt.plot(data['timestamp'], data['x'], color='r', alpha = 0.9, label='x')
    plt.plot(data['timestamp'], data['y'], color='g', alpha = 0.9, label='y')
    plt.plot(data['timestamp'], data['z'], color='b', alpha = 0.9, label='z')
    # plt.plot(data['timestamp'], data['a'], color='y', alpha = 0.9, label='a')
    plt.legend()
    plt.show()

    pass

if __name__ == '__main__':
    main()
