# DSP Filters and Statistical Helper Functions #

The helper functions in 'drivers/statistics/' can be used to provide basic statistical modelling and DSP filtering functions for sensor data, including:

* Running average, including standard deviation and standard variance
* IIR filter (A basic low pass filter that can be used to smooth out noisy sensor data)

# Running Average (avg\_i.c, avg\_f.c, avg\_d.c) #

These 'average' helper functions are optimised to use the smallest amount of memory possible by keeping a 'running average'.  Rather than storing an array of individual samples and then analysing the entire memory buffer, every time a sample is added it gets calculated into a single running value.  This makes the system more appropriate for small, memory constrained embedded devices, and signficantly increases the number of samples we can 'average' with the MCU.

Three functionally identical versions of the running average helpers are present, depending on the numeric type you wish to use:

* avg\_i.c - **32-bit integers** (int32\_t)
* avg\_f.c - **single-precision floats** (float) 
* avg\_d.c - **double-precision floats** (double)

## How Does it Work? ##

Depending on the data type being used (int32_t, float or double), you simply need to declare the appropriate avg\_X\_t placeholder.

For example, for float values we would use the avg\_f\_t typedef and functions, so after including "avg_f.h", we could declare the following placeholder for our running average, and then initiase the memory to set it to a known state:
```
  avg_f_t fs;
  avg_f_init(&fs);
```
Every time you want to add a new 'sample' to the running average, you simply need to call the appropriate 'record' function, as follows:
```
  avg_f_record(&fs, 10.5F);
  avg_f_record(&fs, 20.0F);
  avg_f_record(&fs, 31.27F);
  avg_f_record(&fs, 35.3F);
```
Once you have added the desired number of records, you can get the basic statistical information by inspecting the avg\_f\_t object we created as follows:
```
  printf("SAMPLES        : %d\n", fs.k);
  printf("MEAN (Average) : %f\n", fs.Mk);
  printf("STDEV          : %f\n", avg_f_stdev(&fs));
  printf("STVARIANCE     : %f\n", avg_f_stdvar(&fs));
  printf("\n");
```

## Results ##

Reading data from a gyroscope, accelerometer, magnetometer, etc., we can get a sense of the sensor's 'noise level' by keeping a running average over a number of samples, and then try to filter this out.  

Alternatively, we can use these helper functions to sample 10 or 100 readings each time, and only return the running average.  This is a good approach if we want to determine the average rotational velocity with a gyroscope, for example.

There is a tradeoff, of course: the more samples we have, the more the standard variance will drop, and the more reliable our data becomes ... but the increased accuracy comes at the expense of frequency or our 'update rate'.

Taking the X axis of an immobile gyroscope, for example, these are the results we get over 100 samples, which takes about 100ms to collect in this case for a 10Hz update rate (compared to the 1000Hz we could get just reading raw but noisy data):
```
  SAMPLES        : 100
  MEAN (Average) : -0.006787 rad/s
  STDEV          : 0.005671 rad/s
  STVARIANCE     : 0.000032 rad/s
```
And after 300 samples you can see the Standard Variances drops further, but we're limited to about 3Hz for a refresh rate:
```
  SAMPLES        : 300
  MEAN (Average) : -0.006304 rad/s
  STDEV          : 0.003722 rad/s
  STVARIANCE     : 0.000014 rad/s
```
Finding the right balance of refresh rate and samples to average takes a bit of trial and error, and will vary from one situation to the next.

## Source ##

This code is based on **"Calculating standard deviation in one pass"** by Peter Kankowski, available at: http://www.strchr.com/standard_deviation_in_one_pass

# IIR (Infinite Impulse Response) Filter - (iir\_f.c) #

A basic lowpass filter that can be used to 'smooth out' noisy sensor data, similar to the way an RC filter works with HW.

The IIR filter is essentially a 'moving average', where the center point of the average moves over time.  The number of samples around the 'center' is based on the alpha value supplied in iir_f_init: a small alpha will result in a slower response from the filter (more samples are required to change the current average), whereas a larger alpha will cause the average to respond more quickly to changes in the signal.

## How Does it Work? ##

After declaring a placeholder iir\_f\_t object, we need to call the init function and supply an alpha value between 0 and 1.0F.  0.01F is a good starting point, but this should be tweeked higher or lower depending on how quickly/slowly you want the average value to change):
```
  iir_f_t iir;
  iir_f_init(&iir, 0.01);
```
After initialising the filter above, you continually add in your samples via the iir_f_add function, and read the current 'iir.avg' value whenever you need it:
```
  iir_f_add(&iir, 10.5F);
  iir_f_add(&iir, 10.76F);
  iir_f_add(&iir, 10.69F);

  printf("SAMPLES  : %d \n", iir.k);
  printf("AVG      : %f \n", iir.avg);
  printf("\n");
```
