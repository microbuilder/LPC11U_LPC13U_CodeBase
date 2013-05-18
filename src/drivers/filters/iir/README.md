# Infinite Impulse Response (IIR) Filter #

This simple IIR implementation is a basic single-pole low-pass filter. It's a 'low pass' filter since it attempts to filter out short term fluctuations, giving more weight to the longer-term average. You can use it to 'smooth out' fluctuating sensor data by slowing the response to new signals. It operates on the same principle as an RC filter in HW -- numerous samples are required before the output of the filter shifts to reach the 'new' value, gradual shifting up or down towards any newly integrated values.

You can configure the filter by adjusting the 'alpha' value, which controls the delay or amount of time required for new signals to be integrated into the current output value.

## What is the Effect of 'Alpha' on the IIR Output? ##

A small alpha will result in a slower response from the filter (more samples are required to change the current average), whereas a larger alpha will cause the average to respond more quickly to changes in the signal, at the expense of keeping more 'noise' in the signal.

You can see this effect in the charts below (generated with iir_i_response_test.py).  Setting the alpha value lower pushes the response curve further out, meaning more samples are required to reach any new values that are sent into the filter.

In the charts below, we start with a value of 0, and the same new value is added into the IIR filter until we reach 99.9% of the value with the IIR output.

(**Note**: These charts are a bit 'wobbly' because the output is based on the integer version of the filter ... the floating point version provides more precision and linearity at the expense of more work for the MCU.)

![Integer IIR Alpha 128](images/integer_responsecurve_alpha_128.png?raw=true)
![Integer IIR Alpha 64](images/integer_responsecurve_alpha_64.png?raw=true)
![Integer IIR Alpha 32](images/integer_responsecurve_alpha_32.png?raw=true)

As you can see above, a larger alpha means that new values are integrated more quickly so the filter 'responds' more quickly, but there is also less smoothing going on due to the fast response.  All the magic in any filter is finding the right values for variables like alpha that meet your needs!

## Is IIR the Right Filter For Me? ##

An IIR filter is very memory efficient, and is excellent for smoothing out noisy data, but it does have some limitations and drawbacks.

One big advantage of a simple IIR filter is that it requires very little memory. This filter keeps a single 'running average' value, which allows a much smaller memory footprint than some other filters. 

This also presents a potential problem, though, since any errors that are introduced in that single 'running average' -- floating point precision losses, etc. -- also get multiplied over time! After running 10,000 samples through the IIR filter, we have to take into account the effect of 10,000 accumulated errors, which may or may not be an issue for you.

Accumulated errors are generally only an issue over large, long-running sample sets, but it's important to keep this in mind when deciding which filter(s) to use with your data.

Two other tradeoffs of IIR filters is that they aren't 'phase stable', and some signal attentuation also inevitably occurs with stronger alpha values.

What this means is that using a high alpha (meaning a smaller number!) will very effectively smooth your data out, but it will also phase shift further and further to the right, and you will no longer have the same peak to peak range (or 'amplitude') of your source data.

You can see this in the following charts generated with the **iir\_i\_noisysine\_test.py** Python script included in this folder.  As the alpha value lowers, the effect of the filter increases and the data is much 'smoother', but it starts to shift to be out of phase with the source signal (in blue), and you lose some amplitude.

![Integer IIR Alpha 128](images/integer_sinewave_alpha_128.png?raw=true)
![Integer IIR Alpha 64](images/integer_sinewave_alpha_64.png?raw=true)
![Integer IIR Alpha 32](images/integer_sinewave_alpha_32.png?raw=true)
![Integer IIR Alpha 16](images/integer_sinewave_alpha_16.png?raw=true)
![Integer IIR Alpha 8](images/integer_sinewave_alpha_8.png?raw=true)

All of the magic is, of course, in finding the right numbers for your needs, and if phase shift is an issue in your situation, the IIR filter isn't a very good choice (which is why so many different DSP filters exist)!

## Which Filter Version Should I Use (int32_t or float)? ##

Two versions of this filter are included, one that uses single-precision floating point values (iir_f.c), and another that uses 32-bit signed integers (iir_i.c).

There are advantages and tradeoffs to both of these implementations that you should be aware of when using them!

### Integer Advantages/Limitations ###

The advantage of the integer version is that it will be much faster ... particularly if you use an ^2 alpha value -- 2, 4, 8, 16, 32, 64 or 128 -- since this allows the compiler to replace a division operation with a simple (single-cycle) shift.

The disadvantage is that at smaller alpha values (<24 or so) the filter will never reach 99.9% of the new value, regardless of how many samples are presented, and the output isn't nearly as smooth as the floating point version.  As such, the integer version is really only appropriate for higher alpha values and quicker responses.

### Floating Point Advantages/Limitations ###

The major advantage of the floating point version of this filter is that you have far more range with the alpha values, and you can always reach 99.9% (or more) of any new value if you run the filter over enough samples.  

The smallest equivalent alpha value in the integer version that reaches 99.9% over time is 24 (out of 255), which equals about 0.09375 in the floating point alpha notation of 0..1.0.

With the floating point version, you can easily use a very small alpha value, such as 0.001, and still get a nice response curve that eventually hits 99.9% of the new value.

You can confirm this by running the two python scripts, iir_i_noisysine_test.py (for integer math) and iir_f_noisysine_test.py (for floating point math).

The floating point version is run with an alpha of 0.015625, which corresponds to an alpha of 4 in the integer version (4/256).  As you can see in the two images below, the limitations of the integer math with a small alpha mean that it peaks at around 93% of the new value, whereas the floating point version has a smooth curve and hits 99.9% of the new value at around 439 samples:

![Integer IIR Response - Alpha 0.015625](images/integer_response_alpha_4.png?raw=true)
![Floaing Points IIR Response - Alpha 0.015625](images/float_response_alpha_0_015625.png?raw=true)

This advantage, of course, comes at a price, and the floating point math takes more clock cycles for the MCU to calculate, and floating point values are inherently 'lossy'.  

The choice between integer and floating point math will ultimately be based on your requirements, and how 'heavy' a filter you require, but you can use the python scripts in this folder to test different values on both integer and floating point implementations of the filter.

## How Do I Use This Code? ##

After declaring a placeholder **iir\_f\_t** (for floating point math) or **iir\_i\_t** (for integer math) object, we need to call the init function and supply a reference to our IIR placeholder as well as an appropriate alpha value.

For the floating point version, the alpha must be between 0 and 1.0F, and for the integer version it must be between 0 and 255.

We'll use the floating point version below as an example.
```
  iir_f_t iir;
  iir_f_init(&iir, 0.01);
```
After initialising the filter above, you continually add in your samples via the **iir\_f\_add** function, and read the current 'iir.avg' value whenever you need it:
```
  iir_f_add(&iir, 10.5F);
  iir_f_add(&iir, 10.76F);
  iir_f_add(&iir, 10.69F);

  printf("SAMPLES  : %d \n", iir.k);
  printf("AVG      : %f \n", iir.avg);
  printf("\n");
```

## Source ##

Thanks to [Robert Davidson](http://www.ambientsensors.com/about/) for suggesting this filter.
