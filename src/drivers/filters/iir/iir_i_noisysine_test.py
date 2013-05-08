#-------------------------------------------------------------------------------
# Name:        iir_i_tester
#
# Purpose:     Displays IIR output of a sine wave with optional random noise
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
import numpy as np
import matplotlib.pyplot as plt

def main():
    avg = 0.0
    current = 0
    iirvals = []

    # Get alpha (determines how 'quickly' the filter responds to changes)
    alpha = int(input("IIR alpha (0..255): "))

    # Set the noise level for the input sine wave
    noiselevel = float(input("Input noise level [0..1.0]: "))

    # Set the number of samples to use
    samples = float(input("Number of samples: "))

    # Check bounds
    if (alpha > 255):
        print ('Setting alpha to 255')
        alpha = 255
    if (alpha < 0):
        print ('Setting alpha to 0')
        alpha = 0
    if (noiselevel > 1.0):
        print ('Setting noise level to 1.0')
        noiselevel = 1.0
    if (noiselevel < 0):
        print ('Setting noise level to 0.0')
        noiselevel = 0.0
    if (samples < 0):
        print ('Setting samples to 100')
        samples = 100

    # Generate a sine wave with some noise on it
    x = np.linspace(0, 4*np.pi, samples)
    sine = np.sin(x)
    noise = np.random.uniform(-1, 1, size=len(x)) * noiselevel
    noisysine = sine*1000 + noise*1000

    # Run the IIR filter over the entire input dataset
    while current < len(x):
        current+=1
        # Add one sample to the IIR filter
        avg = iirAddValue(avg, alpha, noisysine[current-1])
        # Plot IIR filtered value
        iirvals.append(avg);
        print ("%d: %g" % (current, avg))

    # Display the results
    plt.title("Sine Wave Input vs. Int IIR Output \n (Alpha: %g, Noise Level: %g)"
        % (alpha, noiselevel))
    plt.xlabel('Samples')
    plt.ylabel('Values')
    plt.ylim(noisysine.min()*1.1, noisysine.max()*1.1)
    plt.grid(True)
    plt.plot(noisysine,
        color="blue",
        alpha = 0.4,
        linestyle="-",
        label="Raw Input")
    plt.plot(iirvals,
        color="red",
        linewidth='1.5',
        linestyle="-",
        label="IIR Output")
    plt.legend()
    plt.show()

    pass

def iirAddValue(avg, alpha, val):
    "Adds a new value to the IIR filter"
    return (int)((val*alpha+avg*(256-alpha))/256)

if __name__ == '__main__':
    main()
