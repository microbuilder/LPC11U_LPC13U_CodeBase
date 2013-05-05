#-------------------------------------------------------------------------------
# Name:        iir_f_response_test
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
    avg = 0.0
    samples = 0
    values = []

    # Get alpha
    alpha = float(input("IIR alpha (0..1.0): "))

    # Check alpha bounds
    if (alpha > 1.0):
        print ('Setting alpha to 1.0')
        alpha = 1.0
    if (alpha < 0):
        print ('Setting alpha to 0.0')
        alpha = 0.0

    # Run the filter until we arrive at 99.9% of newval
    values.append(0.0)
    while avg < 1.0 * 0.999:
        samples+=1
        avg = iirAddValue(avg, alpha, 1.0)
        # Plot value in percent
        values.append(avg/0.01);
        print ("%d: %g" % (samples, avg))

    # Display the results
    plt.title("IIR Response Over Time (Alpha = %g)" % (alpha))
    plt.ylim(0, 110)
    plt.xlabel('Samples')
    plt.ylabel('IIR Output (%)')
    plt.plot(values)
    plt.grid(True)
    plt.show()

    pass

def iirAddValue(avg, alpha, val):
    "Adds a new value to the IIR filter"
    return alpha * val + (1.0 - alpha) * avg

if __name__ == '__main__':
    main()
