/**************************************************************************/
/*!
    @file     stepper.c
    @author   Based on original code by Tom Igoe
              Modified by K. Townsend (microBuilder.eu)


    @brief    Simple bi-polar stepper motor controller, based on the
              Arduino stepper library by Tom Igoe.  Includes simple
              position handling methods to keep track of the motor's
              relative position and the spindle's current rotation.
              Requires an H-Bridge such as the L293D or SN754410N.

    @section Example

    @code

    stepper_motor_t stepper =
    {
      // H-Bridge GPIO Pin Locations
      .in1_port         = CFG_STEPPER_IN1_PORT,
      .in1_pin          = CFG_STEPPER_IN1_PIN,
      .in2_port         = CFG_STEPPER_IN2_PORT,
      .in2_pin          = CFG_STEPPER_IN2_PIN,
      .in3_port         = CFG_STEPPER_IN3_PORT,
      .in3_pin          = CFG_STEPPER_IN3_PIN,
      .in4_port         = CFG_STEPPER_IN4_PORT,
      .in4_pin          = CFG_STEPPER_IN4_PIN,

      // Motor Settings
      .stepsPerRotation = 200,  // 200 step motor
      .rpm              = 120,  // 2 rotations per second
      .id               = 'X',  // Optional motor ID

      // Do Not Populate ... these fields are set by stepper.c
      .delay            = 0,
      .position         = 0,
      .stepNumber       = 0
    };

    stepperInit(&stepper, 200);       // Initialise driver for 200-step motor
    stepperUpdateRPM(&stepper, 120);  // Set speed to 120 rpm (2 revolutions per second)

    while (1)
    {
      stepperStep(&stepper, 400);     // Move forward 400 steps
      stepperStep(&stepper, -200);    // Move backward 200 steps
      delay(1000);                    // Wait one second

      // Move 'home' after 10 loops (current position = 2000)
      if (stepper.position == 2000)
      {
        stepperMoveHome(&stepper);    // Move back to the starting position
        delay(1000);                  // Wait one second
      }
    }

    @endcode

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2013, K. Townsend (microBuilder.eu)
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
#include "projectconfig.h"

#ifdef CFG_STEPPER

#include <stdlib.h>

#include "stepper.h"
#include "core/gpio/gpio.h"
#include "core/timer32/timer32.h"

/**************************************************************************/
/*!
    Private - Cause the motor to step forward or backward one step
*/
/**************************************************************************/
void stepMotor(stepper_motor_t *motor, uint32_t thisStep)
{
  switch (thisStep)
  {
    case 0: // 1010
      LPC_GPIO->SET[motor->in1_port] =  (1 << motor->in1_pin);
      LPC_GPIO->CLR[motor->in2_port] =  (1 << motor->in2_pin);
      LPC_GPIO->SET[motor->in3_port] =  (1 << motor->in3_pin);
      LPC_GPIO->CLR[motor->in4_port] =  (1 << motor->in4_pin);
      break;
    case 1: // 0110
      LPC_GPIO->CLR[motor->in1_port] =  (1 << motor->in1_pin);
      LPC_GPIO->SET[motor->in2_port] =  (1 << motor->in2_pin);
      LPC_GPIO->SET[motor->in3_port] =  (1 << motor->in3_pin);
      LPC_GPIO->CLR[motor->in4_port] =  (1 << motor->in4_pin);
      break;
    case 2: // 0101
      LPC_GPIO->CLR[motor->in1_port] =  (1 << motor->in1_pin);
      LPC_GPIO->SET[motor->in2_port] =  (1 << motor->in2_pin);
      LPC_GPIO->CLR[motor->in3_port] =  (1 << motor->in3_pin);
      LPC_GPIO->SET[motor->in4_port] =  (1 << motor->in4_pin);
      break;
    case 3: // 1001
      LPC_GPIO->SET[motor->in1_port] =  (1 << motor->in1_pin);
      LPC_GPIO->CLR[motor->in2_port] =  (1 << motor->in2_pin);
      LPC_GPIO->CLR[motor->in3_port] =  (1 << motor->in3_pin);
      LPC_GPIO->SET[motor->in4_port] =  (1 << motor->in4_pin);
      break;
  }
}

/**************************************************************************/
/*!
    @brief      Initialises the GPIO pins and delay timer and sets any
                default values.

    @param[in]  motor
                Pointer to the stepper_motor_t ref containing the motor
                details
*/
/**************************************************************************/
void stepperInit(stepper_motor_t *motor)
{
  // Setup motor control pins
  LPC_GPIO->DIR[motor->in1_port] |=  (1 << motor->in1_pin);
  LPC_GPIO->DIR[motor->in2_port] |=  (1 << motor->in2_pin);
  LPC_GPIO->DIR[motor->in3_port] |=  (1 << motor->in3_pin);
  LPC_GPIO->DIR[motor->in4_port] |=  (1 << motor->in4_pin);

  LPC_GPIO->CLR[motor->in1_port] =  (1 << motor->in1_pin);
  LPC_GPIO->CLR[motor->in2_port] =  (1 << motor->in2_pin);
  LPC_GPIO->CLR[motor->in3_port] =  (1 << motor->in3_pin);
  LPC_GPIO->CLR[motor->in4_port] =  (1 << motor->in4_pin);

  // Initialise 32-bit timer (used for delays)
  timer32Init(CFG_STEPPER_TIMER32);
  timer32Enable(CFG_STEPPER_TIMER32);

  // Set the motor speed and delay values
  if (motor->rpm = 0)
  {
    // Default to 2 revolutions per second
    stepperUpdateRPM(motor, 120);
  }
  else
  {
    stepperUpdateRPM(motor, motor->rpm);
  }
}

/**************************************************************************/
/*!
    @brief    Sets the motor's current position to 'Home', meaning that
              any future movement will be relative to the current
              position.

    @param[in]  motor
                Pointer to the stepper_motor_t ref containing the motor
                details
*/
/**************************************************************************/
void stepperSetHome(stepper_motor_t *motor)
{
  motor->position = 0;
}

/**************************************************************************/
/*!
    @brief    Moves the motor back to the original 'Home' position.

    @param[in]  motor
                Pointer to the stepper_motor_t ref containing the motor
                details
*/
/**************************************************************************/
void stepperMoveHome(stepper_motor_t *motor)
{
  stepperStep(motor, motor->position * -1);
}

/**************************************************************************/
/*!
    @brief    Saves the spindle's current angle/position as 0°.  Each
              step the spindle takes will now be relative to the spindle's
              current position.

    @param[in]  motor
                Pointer to the stepper_motor_t ref containing the motor
                details
*/
/**************************************************************************/
void stepperSetZero(stepper_motor_t *motor)
{
  motor->stepNumber = 0;
}

/**************************************************************************/
/*!
    @brief    Moves the motor to its original rotation value. For example,
              if a 200-step motor is currently rotated to step 137, it
              will move the motor forward 63 steps to end at step 0 or 0°.

    @param[in]  motor
                Pointer to the stepper_motor_t ref containing the motor
                details
*/
/**************************************************************************/
void stepperMoveZero(stepper_motor_t *motor)
{
  if (!motor->stepNumber)
  {
    stepperStep(motor, motor->stepsPerRotation - motor->stepNumber);
  }
}

/**************************************************************************/
/*!
    @brief    Sets the motor speed in rpm, meaning the number of times the
              motor will fully rotate in a one minute period.

    @param[in]  motor
                Pointer to the stepper_motor_t ref containing the motor
                details
    @param[in]  rpm
                Motor speed in revolutions per minute (RPM)

    @warning  Not all motors will function at all speeds, and some trial
              and error may be required to find an appropriate speed for
              the motor.
*/
/**************************************************************************/
void stepperUpdateRPM(stepper_motor_t *motor, uint32_t rpm)
{
  uint32_t ticksOneRPM = (SystemCoreClock / motor->stepsPerRotation) * 60;

  // Set stepper RPM
  motor->delay = ticksOneRPM / rpm;
}

/**************************************************************************/
/*!
    @brief      Moves the motor forward or backward the specified number
                of steps.  A positive number moves the motor forward,
                while a negative number moves the motor backwards.

    @param[in]  motor
                Pointer to the stepper_motor_t ref containing the motor
                details
    @param[in]  steps
                The number of steps to move foreward (positive) or
                backward (negative)
*/
/**************************************************************************/
void stepperStep(stepper_motor_t *motor, int32_t steps)
{
  uint32_t stepsLeft = abs(steps);          // Force number to be positive

  while (stepsLeft > 0)
  {
    // Wait x ticks between individual steps
    timer32DelayTicks(CFG_STEPPER_TIMER32, motor->delay);

    // Increment or decrement step counters (depending on direction)
    if (steps > 0)
    {
      motor->position++;          // Increment global position counter
      motor->stepNumber++;        // Increment single rotation counter
      if (motor->stepNumber == motor->stepsPerRotation)
      {
        motor->stepNumber = 0;
      }
    }
    else
    {
      motor->position--;          // Decrement global position counter
      if (motor->stepNumber == 0)
      {
        motor->stepNumber = motor->stepsPerRotation;
      }
      motor->stepNumber--;        // Decrement single rotation counter
    }

    // Decrement number of remaining steps
    stepsLeft--;

    // Step the motor one step
    stepMotor(motor, motor->stepNumber % 4);
  }
}

#endif
