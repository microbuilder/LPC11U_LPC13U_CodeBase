# Simple Moving Average Filter (sma_*) #

The **simple moving average** is a type of **FIR** filter, and is probably the most common filter type used due to it's speed and simplicity. Don't understimate or overlook it due to it's relative simplicity, though, since you can solve many problems admirably with this unassuming little filter!  

Effectively, the filter works by setting a 'window size', which defines the number of samples we want to average together.  We then move through our dataset, averaging the last 'n' samples, where 'n' is your window size. As new samples are added, the oldest sample is dropped to make room for the new one, and a new average is calculate based on the data in the current window range.

**For example**: We set a window size of eight, and we get ready to transmit 100 ADC samples in the moving average filter one by one.  The first seven samples will be meaningless since the filter is still setting up, filling the circular buffer. After the eighth sample is added, though, we will have an average of the previous eight samples, which is the output for our filter.  When we add sample number nine, the earliest sample from our ADC -- sample number one -- will be pushed out of the buffer, and the latest sample will take it's place.

## What are the advantage of the Moving Average Filter ##

One of the key advantages of the moving average filter (aside from it's speed an simplicity!) is that you don't have to deal with accumulated errors over time the way you do with something like an IIR filter.  

Because you are working with a continuously moving subset of your data (the 'window'), the rest of the historical data has no weight and the only accumulated error is over the small data subset in the current window.

The moving average filter is a good choice for removing random noise or spikes in the data, while still maintaining excellent responsivity to changes in the signal.  It's an excellent choice for filters in the time domain, since it is very responsive to changes over time, without introducing as much 'phase shift' or instability as an IIR filter might.

## Filter Results ##

You can see the results of the simple moving average filter in the images below, using a variety of window sizes on an integer sine wave with a little bit of noisy added on to it (since this is the most common problem you use an ma filter to solve).  A larger window increases the phase shift and requires more memory (for the larger window buffer), but results in smoother data.  Take note of the 'setup period' at the start of the images where the filter output is 0:

![Integer MA Window 4](images/ma_u16_win4_noise0_025_12-bitrange.png?raw=true)
![Integer MA Window 8](images/ma_u16_win8_noise0_025_12-bitrange.png?raw=true)
![Integer MA Window 16](images/ma_u16_win16_noise0_025_12-bitrange.png?raw=true)
![Integer MA Window 32](images/ma_u16_win32_noise0_025_12-bitrange.png?raw=true)

## Which Filter Version Should I Use (float, int32\_t, or uint16\_t)? ##

Three versions of this filter are included, one that uses single-precision floating point values (ma\_f.c), one that uses uses signed 32-bit integers (ma\_i.c), and another that uses unsigned 16-bit integers (ma\_u16.c).

There are advantages and tradeoffs to both of these implementations that you should be aware of when using them, but this will almost certainly be dictated by the data you wish 

## How Do I Use This Code? ##

### 1. Allocate Memory for the Window Buffer ###
The moving average filter requires a buffer to store the 'windowed' data, so before we can do anything with this filter, we need to set a chunk of SRAM aside for the filter to work with behind the scenes.  We do this by declaring an array with the right type and ^2 in length (meaning that the array must be 2, 4, 8, 16, 32, etc. values wide).

The following code will create an 8 sample buffer using floating point data (meaning 32 bytes of memory will be used, since a single float takes 4 bytes, multiplied by our window size of 8 samples):
```
  // Create a buffer named 'bufsma' 8 float values wide
  float bufsma[8];
```
To do the same for uint16\_t data, we would use the following code, which would require half of the memory since uint16_t uses 2 bytes instead of the four bytes in float:
```
  // Create a buffer named 'bufsma' 8 uint16_t values wide
  uint16_t bufsma[8];
```
### 2. Declare the Filter Object ###
The current filter code is based on a single 'struct' that contains all of the implementation details for our filter, including the buffer.  The benefit of this approach is that we can have many 'instances' of the filter, each encapsulated in a single 'variable', and we can cascade them by running the filter once, and then running those results through another filter a second time.

To declare a field that will hold the filter data, we use the following code, being careful to use the same window size (.size) and buffer name (.buffer) that we declared above!):

```
  // Now declare the filter with the window size and a buffer pointer
  sma_f_t sma = { .k = 0,
                  .size = 8,
                  .avg = 0,
                  .buffer = sma_buffer };
```
... or for uint16\_t data this would be:
```
  // Now declare the filter with the window size and a buffer pointer
  sma_u16_t sma = { .k = 0,
                    .size = 8,
                    .avg = 0,
                    .buffer = sma_buffer };
```
### 3. Initialise the Filter ###
After declaring a placeholder **sma\_f\_t** (for floating point math), **sma\_i\_t** (for 32-bit signed integer math) object, or **sma\_u16\_t** (for unsigned 16-bit integers), we need to call the init function and supply a reference to our ma placeholder.

This function essentially just does some basic error checking, and will return **false** if for some reason the filter couldn't be initialised, such as a problem with the window size (**which must be a ^2 value!**).

We'll use the floating point version below as an example.
```
  // Initialise the moving average filter (mostly error checks)
  if (sma_f_init(&sma))
  {
    printf("Something failed during filter init!\n");
  }
```
... or the uint16\_t version would look like this
```
  // Initialise the moving average filter (mostly error checks)
  if (sma_u16_init(&sma))
  {
    printf("Something failed during filter init!\n");
  }
```
### 4. Start Adding Values! ###
After initialising the filter, you continually add in your samples via the **sma\_*\_add** functions, and read the current 'sma.avg' value whenever you need it.

For example, for the floating point version we could use the following code:
```
  // Add some values
  sma_f_add(&sma, 10.0F);
  sma_f_add(&sma, 20.0F);
  sma_f_add(&sma, 30.15F);
  sma_f_add(&sma, 35.0F);
  sma_f_add(&sma, 12.0F);
  sma_f_add(&sma, -6.7);
  sma_f_add(&sma, 30.3F);
  sma_f_add(&sma, 20.0F);  // We should have an avg value starting here
  sma_f_add(&sma, 0.0F);
  sma_f_add(&sma, 10.0F);

  printf("WINDOW SIZE   : %d\n", sma.size);
  printf("TOTAL SAMPLES : %d\n", sma.k);
  printf("CURRENT AVG   : %f\n", sma.avg);
  printf("\n");
```
... or for the uint16_t version:
```
  // Add some values
  sma_u16_add(&sma, 10);
  sma_u16_add(&sma, 20);
  sma_u16_add(&sma, -30);
  sma_u16_add(&sma, 37);
  sma_u16_add(&sma, 11);  
  sma_u16_add(&sma, 31);
  sma_u16_add(&sma, 30);
  sma_u16_add(&sma, 20);  // We should have an avg value starting here
  sma_u16_add(&sma, 3);
  sma_u16_add(&sma, 10);

  printf("WINDOW SIZE   : %d\n", sma.size);
  printf("TOTAL SAMPLES : %d\n", sma.k);
  printf("CURRENT AVG   : %d\n", sma.avg);
  printf("\n");
```

## Implementation Warning ##

A major strength of the 'simple moving average' is that it is one of the fastest filters, particulary if you use a ^2 window size (2, 4, 8, 16, 32, etc.).  The ^2 window means the expensive division operator can be replaced with a single-cycle right-shift.

To avoid extra overhead, the current sma implementation will only allow you to init a filter with a ^2 window size.

## Further Reading ##

For more information on simple moving average filters, see [Moving Average Filters](http://www.dspguide.com/ch15.htm) in Steven Smith's excellent book **The Scientist and Engineer's Guide to Digital Signal Processing**.
