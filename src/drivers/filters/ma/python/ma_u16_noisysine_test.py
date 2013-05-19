#-------------------------------------------------------------------------------
# Name:        ma_u16_tester
#
# Purpose:     Displays simple moving average output of a sine wave with
#              optional random noise
#
# Author:      K. Townsend (microBuilder.eu)
#
# Created:     19/05/2013
# Copyright:   (c) K. Townsend 2013
# Licence:     BSD
#
# This module requires the following libs
# matplotlib - http://matplotlib.org/
# numpy      - http://www.numpy.org/
#-------------------------------------------------------------------------------
import numpy as np
import matplotlib.pyplot as plt
from collections import deque


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
    current = 0
    avg = 0
    total = 0
    mavals = []

    # Set the noise level for the input sine wave
    noiselevel = float(input("Input noise level [0..1.0]: "))

    # Get window size (how many 'samples' are averaged together)
    windowsize = int(input("Windows size (0..65535): "))

    # Get amplitude (the numeric range of the data)
    amplitude = int(input("Peak amplitude (0..32767): "))

    # Set the number of samples to use
    samples = int(input("Number of samples: "))

    # Check bounds
    if (noiselevel > 1.0):
        print ('Setting noise level to 1.0')
        noiselevel = 1.0
    if (noiselevel < 0):
        print ('Setting noise level to 0.0')
        noiselevel = 0.0
    if (windowsize > 65535):
        print ('Setting window size to 65535')
        windowsize = 65535
    if (windowsize < 1):
        print ('Setting window size to 1')
        windowsize = 1
    if (amplitude > 32767):
        print ('Setting amplitude to 32767')
        amplitude = 32767
    if (amplitude < 1):
        print ('Setting amplitude to 1')
        amplitude = 1
    if (samples < windowsize):
        printf('Setting samples to match window size')
        samples = windowsize

    # Create a circular buffer for our window view
    window = RingBuffer(size_max=windowsize)

    # Generate a sine wave with some noise on it
    x = np.linspace(0, 4*np.pi, samples)
    sine = np.sin(x)
    noise = np.random.uniform(-1, 1, size=len(x)) * noiselevel
    noisysine = sine*amplitude + noise*amplitude

    # Run the ma filter over the entire input dataset
    while current < len(x):
        current+=1
        # Make sure we've reached 'windowlength' samples in the buffer
        if (current <= windowsize):
            window.append(noisysine[current-1])
            mavals.append(0)
        else:
            # Add the current sample to the 'window' ring buffer
            window.append(noisysine[current-1])
            # Get the current average based on the window content
            li = window.tolist()
            total = 0
            for i in li:
                total += i
            avg = (int)(total/windowsize)
            # Append ma output for plotting below
            mavals.append(avg);
            print ("%d: %d" % (current, avg))

    # Display the results
    plt.title("Sine Wave Input vs. U16 MA Output \n(Window Size: %d, Noise Level: %g)"
        % (windowsize, noiselevel))
    plt.xlabel('Samples')
    plt.ylabel('Values')
    plt.ylim(noisysine.min()*1.1, noisysine.max()*1.1)
    plt.grid(True)
    plt.plot(noisysine,
        color="blue",
        alpha = 0.4,
        linestyle="-",
        label="Raw Input")
    plt.plot(mavals,
        color="red",
        linewidth='1.5',
        linestyle="-",
        label="MA Output")
    plt.legend()
    plt.show()

    pass

if __name__ == '__main__':
    main()
