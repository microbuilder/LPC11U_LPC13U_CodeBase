# Statistics Module #

The helper functions in statistics\_* can be used to provide basic statistical modelling, including running calculations of mean/average, standard deviation, and standard variance.

Three functionaly identical versions are present, depending on the numeric type you wish to use for your running average:

* statistics\_i.c - **32-bit integers** (int32\_t)
* statistics\_f.c - **single-precision floats** (float) 
* statistics\_d.c - **double-precision floats** (double)

All of these function optimized to use the smallest amount of memory possible by keeping a 'running average'.  Rather than storing individual samples and later analyzing the entire memory buffer, every time a sample is added it is calculated into a single variable.  This makes system more appropriate for small, resource and memory constrained embedded devices, and signficantly increases the number of samples we can 'average' with the MCU.

## How Does it Work? ##

Depending on the data type being used (int32_t, float or double), you simply need to declare the appropriate statistics\_X\_t placeholder.

For example, for float values we would use the statistics\_f\_t typedef and functions, so after including "statistics_f.h", we could declare the following placeholder for our running average, and then initiase the memory to set it to a known state:
```
  statistics_f_t fs;

  statistics_f_init(&fs);
```

Every time you want to add a new 'sample' to the running average, you simply need to call the appropriate 'record' function, as follows:
```
  statistics_f_record(&fs, 10.5F);
  statistics_f_record(&fs, 20.0F);
  statistics_f_record(&fs, 31.27F);
  statistics_f_record(&fs, 35.3F);
```
Once you have added the desired number of records, you can get the basic statistical information by inspecting the statistics\_f\_t object we created as follows:
```
  printf("SAMPLES        : %d\n", fs.k);
  printf("MEAN (Average) : %f\n", fs.Mk);
  printf("STDEV          : %f\n", statistics_f_stdev(&fs));
  printf("STVARIANCE     : %f\n", statistics_f_stdvar(&fs));
  printf("\n");
```
## Results ##
Reading a data from an gyroscope, accelerometer, magnetometer, etc., we can get a sense of the sensor's 'noise level' by keeping a running average over a number of samples, and then try to filter this out.  

Alternatively, we can use these helper functions to sample 10 or 100 readings each time, and only return the running average if we are trying to determine average rotational velocity, for example.

There is a tradeoff, of course, in that the more samples we have the more the standard variance will drop, and the more reliable our data becomes ... but it's at the expense of frequency or our 'update rate'.

Taking the X axis of an immobile gyroscope for example, these are the results we get over 100 samples, which takes about 100ms to collect in this case for a 10Hz update rate (compared to the 1000Hz we could get just reading raw but noisy data):
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

## Source ##
This code is based on **"Calculating standard deviation in one pass"** by Peter Kankowski, available at: http://www.strchr.com/standard_deviation_in_one_pass