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
## Source ##
This code is based on **"Calculating standard deviation in one pass"** by Peter Kankowski, available at: http://www.strchr.com/standard_deviation_in_one_pass