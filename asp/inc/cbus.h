/**
 * Copyright (c) 2011-2016, Jack Mo (mobangjack@foxmail.com).
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __CBUS_H__
#define __CBUS_H__

/**************************************************/
/*             Kylinbot Control Bus               */
/**************************************************/

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>

#define CBUS_CHASSIS_VELOCITY_RECIP 0.001f
#pragma pack(1)
typedef struct
{
	int16_t vx; // Bot linear velocity in x-axis, unit: mm/s
	int16_t vy; // Bot linear velocity in y-axis, unit: mm/s
	int16_t vz; // Bot angular velocity in z-axis, unit: rad/s
	int16_t pe; // Bot elevator position, unit: 0.001*rad
	int16_t pc; // Bot claw position, unit: 0.001*rad
	uint32_t fs; // Functional state control bits, 0: off, 1: on
}CBUS_t;
#pragma pack()

void CBUS_Init(CBUS_t* cbus);

#ifdef __cplusplus
}
#endif

#endif
