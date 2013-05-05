#-------------------------------------------------------------------------------
# Name:        iir_f_tester
#
# Purpose:     Runs a simple IIR filter to determine the number of new samples
#              required to reach 90% of a new value
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
    alpha = float(input("Alpha (0..1.0): "))

    # Check alpha bounds
    if (alpha > 1.0):
        print ('Setting alpha to 1.0')
        alpha = 1.0
    if (alpha < 0):
        print ('Setting alpha to 0.0')
        alpha = 0.0

    # Run the filter until we arrive at 99.9% of newval
    while avg < 1.0 * 0.999:
        samples+=1
        avg = iirAddValue(avg, alpha, 1.0)
        # Plot value in percent
        values.append(avg/0.01);
        print (samples, ': ', avg)

    # Display the results
    plt.title("IIR Results (Alpha = %f)" % (alpha))
    plt.xlabel('Samples')
    plt.ylabel('Value (%)')
    plt.plot(values)
    plt.grid(True)
    plt.show()

    pass

def iirAddValue(avg, alpha, val):
    "Adds a new value to the IIR filter"
    return alpha * val + (1.0 - alpha) * avg

if __name__ == '__main__':
    main()
