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
import Tkinter, tkFileDialog

# This program will plot X/Y/Z data logged via drivers/storage/logger.c, and
# assumes we are getting vector data in CSV format generated using the
# 'sensorsLogSensorsEvent' helper function in drivers/sensors/sensors.c
#
# Data should look similar to the this:
#
# 0,1,5714,6.001670,-6.629296,-4.785645,0.000000
# 0,1,5729,6.001670,-6.629296,-4.785645,0.000000
# 0,1,5734,5.883990,-6.590069,-4.746419,0.000000

def main():
    # Request the data file to process
    root = Tkinter.Tk()
    root.withdraw()
    filename = tkFileDialog.askopenfilename()

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
