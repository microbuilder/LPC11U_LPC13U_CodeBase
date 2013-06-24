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

import math
import numpy as np
import matplotlib.pyplot as plt
import Tkinter, tkFileDialog
from collections import deque

# This program will plot X/Y/Z data logged via drivers/storage/logger.c, and
# assumes we are getting vector data in CSV format generated using the
# 'sensorsLogSensorsEvent' helper function in drivers/sensors/sensors.c
#
# Data should look similar to the this:
#
# 0,1,5714,6.001670,-6.629296,-4.785645,0.000000
# 0,1,5729,6.001670,-6.629296,-4.785645,0.000000
# 0,1,5734,5.883990,-6.590069,-4.746419,0.000000
#
# In addition to the raw X/Y/Z data, vector magnitude is also calculated in
# a fourth data column

class RingBuffer(deque):
    def __init__(self, size_max):
        deque.__init__(self)
        self.size_max = size_max
    def append(self, datum):
        deque.append(self, datum)
        if len(self) > self.size_max:
            self.popleft( )
    def tolist(self):
        return list(self)

def main():
    # Variables for our moving average filter
    current = 0
    avg = 0
    total = 0
    mavals = []

    # Get window size (how many 'samples' are averaged together)
    windowsize = int(input("Windows size (0..65535): "))
    if (windowsize > 65535):
        print ('Setting window size to 65535')
        windowsize = 65535
    if (windowsize < 1):
        print ('Setting window size to 1')
        windowsize = 1

    # Request the data file to process
    root = Tkinter.Tk()
    root.withdraw()
    filename = tkFileDialog.askopenfilename()

    # Load the CSV file in 'data'
    data = np.genfromtxt(filename,
        delimiter=',',
        dtype="i32,i32,i32,f32,f32,f32,f32",
        names=['id','type','timestamp','x','y','z','a'])

    # Create a circular buffer for our moving average filter
    window = RingBuffer(size_max=windowsize)

    # Calculate magnitude in column a
    for x in np.nditer(data, op_flags=['readwrite']):
        x['a'] = math.sqrt(
            math.pow(x['x'], 2) +
            math.pow(x['y'], 2) +
            math.pow(x['z'], 2))
        # Perform the moving average filter operations
        current+=1
        # Add magnitude into the ringbuffer
        window.append(x['a'])
        # Make sure we've reached 'windowlength' samples in the buffer
        if (current <= windowsize):
            mavals.append(0)
        else:
            # Get the current average based on the window content
            li = window.tolist()
            total = 0
            for i in li:
                total += i
            avg = (float)(total/windowsize)
            # Append ma output for plotting below
            mavals.append(avg);

    # Display the results
    plt.title("SMA Filtered sensors_event_t Data (X/Y/Z + Magnitude)\nSMA Window Size = %d Samples"
        % (windowsize))
    plt.xlabel('Timestamp (ms)')
    plt.ylabel('Value')
    plt.xlim(data['timestamp'].min(), data['timestamp'].max()*1.1)
    plt.grid(True)
    plt.plot(data['timestamp'], data['x'], color='r', alpha = 0.25, label='x')
    plt.plot(data['timestamp'], data['y'], color='g', alpha = 0.25, label='y')
    plt.plot(data['timestamp'], data['z'], color='b', alpha = 0.25, label='z')
    plt.plot(data['timestamp'], data['a'], color='m', alpha = 0.25, label='mag')
    plt.plot(data['timestamp'], mavals, color="black", label="mag filtered")
    plt.legend()
    plt.show()

    pass

if __name__ == '__main__':
    main()
