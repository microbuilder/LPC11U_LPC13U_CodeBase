/**************************************************************************/
/*!
    @file     accelerometers.c
    @author   Nguyen Quang Huy, Nguyen Thien Tin
    @ingroup  Sensors

    @brief    Helper functions for accelerometers

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
#include "projectconfig.h"
#include "accelerometers.h"
#include <string.h>
#include <math.h>

/**************************************************************************/
/*!
    @brief  Populates the .pitch/.roll fields in the sensors_vec_t
    struct with the right angular data
*/
/**************************************************************************/
void accelGetOrientation(sensors_event_t *event, sensors_vec_t *orientation)
{
	float t_pitch, t_roll;
	/* pitch = atan(x / sqrt(y*y + z*z)) */
	t_pitch = event->acceleration.y * event->acceleration.y + event->acceleration.z * event->acceleration.z;
	orientation->pitch = atan2(event->acceleration.x, sqrt(t_pitch));

	/* roll = atan(y / sqrt(x*x + z*z)) */
	t_roll = event->acceleration.x * event->acceleration.x + event->acceleration.z * event->acceleration.z;
	orientation->roll = atan2(event->acceleration.y, sqrt(t_roll));
}
