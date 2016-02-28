/**************************************************************************/
/*!
   @file     iir.c
   @brief    Helper functions for IIR filters generated with Octave or
             Matlab (supports 1, 2 or 3-pole filters)

   @note     Shamelessly taken from Holly Gates tutorial on using Octave
             to calculate IIR/FIR filters:
             http://tooling-up.blogspot.fr/2013/06/signal-acquisition-filtering-on-simple.html
*/
/**************************************************************************/
#include "iir.h"
#include <math.h>

#define IIR_PI 3.14159265358979323846

/**************************************************************************/
/*!
    @brief      Single pole IIR filter routine

    @param[in]  filt
                Pointer to the iir_filt_1p_instance instance containing
                the filter settings
    @param[in]  in
                The single-precision floating point value to feed into
                the filter
    @returns    The single-precision floating point filter output

    @code

    float32_t filtered = 0.0F;

    // Setup the single-pole filter values
    iir_filt_1p_instance iir =  { .a1=0.1F,
                                  .b0=0.9F, .b1=0.9F };

    // Pass some data into the filter
    filtered = iir_filt_1p(&iir, 12.345F);

    @endcode
*/
/**************************************************************************/
float32_t iir_filt_1p(iir_filt_1p_instance* filt, float32_t in)
{
  /* Calculate new output */
  filt->out = (filt->b0*in + filt->b1*filt->x1 - filt->a1*filt->y1);

  /* Shift input samples */
  filt->x1 = in;

  /* Shift output samples */
  filt->y1 = filt->out;

  return(filt->out);
}

/**************************************************************************/
/*!
    @brief     Two pole IIR filter routine

               For a two pole butterworth filter, use the iis_butter2
               function in this file to calculate the butterworth filter
               coefficients without having to make use of Octave.

    @param[in] filt
               Pointer to the iir_filt_2p_instance instance containing
               the filter settings
    @param[in] in
               The single-precision floating point value to feed into
               the filter
    @returns   The single-precision floating point filter output

    @code

    // Calculate a second order butterworth filter with the following
    // characteristics:
    //
    //   Fc = 10Hz     - Cutoff Frequency
    //   Fs = 1000Hz   - Sample Rate
    //
    // Enter the following in Octave:
    //
    //   [bb2 ba2] = butter(2, 10/500);
    //
    // Where: 2   = second order filter (3=third order, etc.)
    //        10  = Fc
    //        500 = Fnyq (1/2 Fs)
    //
    // Visualise the results (sanity check):
    //
    //   freqz(bb2,ba2)
    //
    // Get our values (enter 'bb2 and 'ba2'):
    //
    //    bb2 =
    //
    //      9.4469e-004  1.8894e-003  9.4469e-004
    //
    //    ba2 =
    //
    //       1.00000  -1.91120   0.91498

    // Setup the 2 pole butterworth filter with values calculated above
    iir_filt_2p_instance iir_butter =  { .a1=-1.91120F, .a2=0.91498F,
                                         .b0=9.4469e-004, .b1=1.8894e-003, .b2=9.4469e-004 };

    float32_t i, filtered;
    i = filtered = 0.0F;

    while (1)
    {
      // ToDo: Get ADC at the right frequency (Fs above)
      i++;

      // Apply the digital filter
      filtered = iir_filt_2p(&iir_butter, i);
      debug_printf("%f\n", filtered);
    }

    @endcode
*/
/**************************************************************************/
float32_t iir_filt_2p(iir_filt_2p_instance* filt, float32_t in)
{
  /* Calculate new output */
  filt->out = (filt->b0*in +
               filt->b1*filt->x1 + filt->b2*filt->x2 -
               filt->a1*filt->y1 - filt->a2*filt->y2);

  /* Shift input samples */
  filt->x2 = filt->x1;
  filt->x1 = in;

  /* Shift output samples */
  filt->y2 = filt->y1;
  filt->y1 = filt->out;

  return(filt->out);
}

/**************************************************************************/
/*!
    @brief     Two pole integer based IIR filter routine

    @param[in] filt
               Pointer to the iir_filt_i_2p_instance instance containing
               the filter settings
    @param[in] in
               The int32_t value to feed into the filter
    @returns   The int32_t filter output

    @code

    ToDo

    @endcode
*/
/**************************************************************************/
int32_t iir_filt_i_2p(iir_filt_i_2p_instance* filt, int32_t in)
{
  /* Calculate new output */
  filt->out = (filt->b0*in +
	  filt->b1*filt->x1 + filt->b2*filt->x2)>>(filt->bf);

  filt->out -= ((filt->a1*filt->y1 + filt->a2*filt->y2)>>(filt->af));

  /* Shift input samples */
  filt->x2 = filt->x1;
  filt->x1 = in;

  /* Shift output samples */
  filt->y2 = filt->y1;
  filt->y1 = filt->out;

  return(filt->out);
}

/**************************************************************************/
/*!
    @brief      Three pole IIR filter routine

    @param[in]  filt
                Pointer to the iir_filt_3p_instance instance containing
                the filter settings
    @param[in]  in
                The single-precision floating point value to feed into
                the filter
    @returns    The single-precision floating point filter output

    @code

    // Setup the 3 pole filter with values from Octave/Matlab
    iir_filt_3p_instance iir_filt = {
      .a1=-2.93717, .a2=2.87630, .a3=-0.93910,
      .b0=3.0044e-5, .b1=0.00000, .b2=0.00000, .b3=0.00000
    };

    float32_t in, filtered;
    in = filtered = 0.0F;

    while (1)
    {
      // ToDo: Read the ADC, etc., and populate 'in'
      in++;

      // Apply the digital filter
      filtered = iir_filt_3p(&iir_filt, i);
      printf("%f\n", filtered);
    }

    @endcode
*/
/**************************************************************************/
float32_t iir_filt_3p(iir_filt_3p_instance* filt, float32_t in)
{
  /* Calculate new output */
  filt->out = (filt->b0*in +
               filt->b1*filt->x1 + filt->b2*filt->x2 + filt->b3*filt->x3 -
               filt->a1*filt->y1 - filt->a2*filt->y2 - filt->a3*filt->y3);

  /* Shift input samples */
  filt->x3 = filt->x2;
  filt->x2 = filt->x1;
  filt->x1 = in;

  /* Shift output samples */
  filt->y3 = filt->y2;
  filt->y2 = filt->y1;
  filt->y1 = filt->out;

  return(filt->out);
}

/**************************************************************************/
/*!
    @brief      Four pole IIR filter routine

    @param[in]  filt
                Pointer to the iir_filt_4p_instance instance containing
                the filter settings
    @param[in]  in
                The single-precision floating point value to feed into
                the filter
    @returns    The single-precision floating point filter output
*/
/**************************************************************************/
float32_t iir_filt_4p(iir_filt_4p_instance* filt, float32_t in)
{
  /* Calculate new output */
  filt->out = (filt->b0*in +
               filt->b1*filt->x1 + filt->b2*filt->x2 + filt->b3*filt->x3 + filt->b4*filt->x4 -
               filt->a1*filt->y1 - filt->a2*filt->y2 - filt->a3*filt->y3 - filt->a4*filt->y4);

  /* Shift input samples */
  filt->x4 = filt->x3;
  filt->x3 = filt->x2;
  filt->x2 = filt->x1;
  filt->x1 = in;

  /* Shift output samples */
  filt->y4 = filt->y3;
  filt->y3 = filt->y2;
  filt->y2 = filt->y1;
  filt->y1 = filt->out;

  return(filt->out);
}

/**************************************************************************/
/*!
    @brief      Two pole butterworth low pass IIR filter coefficient
                generator routine

    @param[in]  filt
                Pointer to the iir_filt_2p_instance instance containing
                the filter data structure
    @param[in]  fs
                sampling frequency
    @param[in]  fc
                cut point frequency

    @code

    iir_filt_2p_instance test_filt; // The filter structure itself
    float fs = 100;                 // 100Hz sample frequency
    float fc = 5;                   // 5Hz cut frequency
    float input,output;

    iir_butter2(&test_filt,fs,fc);  // Generate the filter coeffs

    printf("filter coeffs:\r\n");
    printf("test_filt  a1:%f  a2:%f  b0:%f  b1:%f  b2:%f\r\n",
  	 test_filt.a1,test_filt.a2,test_filt.b0,test_filt.b1,test_filt.b2);

    input = 10.0;

    output = iir_filt_2p(&test_filt,input); // Run the filter once
    // Note: You need to run the filter at the frequency you specified for Fs!

    printf("input:%f  output:%f\r\n", input, output);

    @endcode

    You can verify the output of this filter against Octave as follows:

    OUTPUT at terminal when run with fs=100 fc=5
    -------------------------------
    filter coeffs:
    test_filt  a1:-1.561018  a2:0.641352  b0:0.020083  b1:0.040167  b2:0.020083
    input:10.000000  output:0.200834

    OCTAVE design: 2 pole, fc=5, fs=50 (Fnyq = 1/2 Fs)
    -------------------------------
    octave:4> pkg load signal
    octave:5> [b a] = butter(2,5/50)
    b =
       0.020083   0.040167   0.020083

    a =
       1.00000  -1.56102   0.64135
*/
/**************************************************************************/
void iir_butter2(iir_filt_2p_instance* filt, float32_t fs, float32_t fc)
{
  float ax  = tan(IIR_PI*fc/fs);
  float ax2 = ax*ax;
  float r   = sin(IIR_PI*3.0/4.0);
  float sx  = ax2 + 2.0*ax*r + 1.0;
  float A   = ax2/sx;
  float d1  = 2.0*(1-ax2)/sx;
  float d2  = -(ax2 - 2.0*ax*r + 1.0)/sx;

  filt->b0 = A;
  filt->b1 = 2*A;
  filt->b2 = A;
  filt->a1 = -1*d1;
  filt->a2 = -1*d2;
}
