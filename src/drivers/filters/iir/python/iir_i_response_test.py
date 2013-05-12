#-------------------------------------------------------------------------------
# Name:        iir_i_response_test
#
# Purpose:     Shows the response of an IIR filter over time. Used to determine
#              how many samples are required to reach n percent of the new
#              input value.
#
# Author:      K. Townsend (microBuilder.eu)
#
# Created:     05/05/2013
# Copyright:   (c) K. Townsend 2013
# Licence:     BSD
#
# This module requires the following libs
# matplotlib - http://matplotlib.org/
# numpy      - http://www.numpy.org/
#-------------------------------------------------------------------------------
from array import array
import matplotlib.pyplot as plt

def main():
    avg = 0
    samples = 0
    values = []

    # Get alpha
    alpha = int(input("IIR alpha (0..255): "))

    # Check alpha bounds
    if (alpha > 255):
        print ('Setting alpha to 255')
        alpha = 255
    if (alpha < 0):
        print ('Setting alpha to 0')
        alpha = 0

    # Start with a known value (0)
    values.append(0)

    # Run the filter until we arrive at 99% of newval or 500 samples
    # if we are using a small alpha since the integer filter will never
    # stabilize at 99.9% of the new value with heaver alpha values
    if (alpha < 24):
        while samples < 500:
            samples+=1
            avg = iirAddValue(avg, alpha, 1000)
            # Plot value in percent
            values.append(avg/10);
            print ("%d: %d" % (samples, avg))
    else:
        while avg < 990:
            samples+=1
            avg = iirAddValue(avg, alpha, 1000)
            # Plot value in percent
            values.append(avg/10);
            print ("%d: %d" % (samples, avg))

    # Display the results
    plt.title("IIR Response Over Time (Alpha = %d)" % (alpha))
    plt.ylim(0, 110)
    plt.xlabel('Samples')
    plt.ylabel('IIR Output (%)')
    plt.plot(values)
    plt.grid(True)
    plt.show()

    pass

def iirAddValue(avg, alpha, val):
    "Adds a new value to the IIR filter"
    return (int)((val*alpha+avg*(256-alpha))/256)

if __name__ == '__main__':
    main()
