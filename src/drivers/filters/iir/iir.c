/**************************************************************************/
/*!
   @file     iir.c
   @brief    Helper functions for IIR filters generated with Octave or
             Matlab (supports 1, 2 or 3-pole filters)

   @note     Shamelessly take from Holly Gates tutorial on using Octave
             to calculate IIR/FIR filters:
             http://tooling-up.blogspot.fr/2013/06/signal-acquisition-filtering-on-simple.html
*/
/**************************************************************************/
#include "iir.h"

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
