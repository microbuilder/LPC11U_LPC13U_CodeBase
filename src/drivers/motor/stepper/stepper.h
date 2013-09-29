/**************************************************************************/
/*!
    @file     stepper.h
    @author   K. Townsend (microBuilder.eu)

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2013, K. Townsend
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/

#ifndef _STEPPER_H_
#define _STEPPER_H_

#include "projectconfig.h"

typedef struct
{
  uint8_t   in1_port;           /**< The port number for the IN1 pin */
  uint8_t   in1_pin;            /**< The pin number for the IN1 pin */
  uint8_t   in2_port;           /**< The port number for the IN2 pin */
  uint8_t   in2_pin;            /**< The pin number for the IN2 pin */
  uint8_t   in3_port;           /**< The port number for the IN3 pin */
  uint8_t   in3_pin;            /**< The pin number for the IN3 pin */
  uint8_t   in4_port;           /**< The port number for the IN4 pin */
  uint8_t   in4_pin;            /**< The pin number for the IN4 pin */
  uint32_t  stepsPerRotation;   /**< Number of steps in a full 360° rotation */
  uint16_t  rpm;                /**< Motor speed in revolutions per minute */
  uint16_t  id;                 /**< Optional ID to identify the stepper motor */
  /* The following fields are assigned by stepper.c and are not user defined */
  uint32_t  delay;              /**< Delay in CPU ticks between steps */
  int64_t   position;           /**< The current position of the spindle (in steps) relative to the 'home' position */
  uint16_t  stepNumber;         /**< The current position of the spindle (in steps) relative to 0° */
} stepper_motor_t;

void     stepperInit      ( stepper_motor_t *motor );
void     stepperSetHome   ( stepper_motor_t *motor );
void     stepperMoveHome  ( stepper_motor_t *motor );
void     stepperSetZero   ( stepper_motor_t *motor );
void     stepperMoveZero  ( stepper_motor_t *motor );
void     stepperUpdateRPM ( stepper_motor_t *motor, uint32_t rpm );
void     stepperStep      ( stepper_motor_t *motor, int32_t steps );

#endif