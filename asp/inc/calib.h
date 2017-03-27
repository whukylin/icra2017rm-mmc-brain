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
 
#ifndef __CALIB_H__
#define __CALIB_H__

/*****************************************/
/*              Calibration              */
/*****************************************/

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "pid.h"

#pragma pack(1)
	
#define PID_CALIB_TYPE_CHASSIS_VELOCITY 0x01
#define PID_CALIB_TYPE_GRABBER_VELOCITY 0x02
#define PID_CALIB_TYPE_GRABBER_POSITION 0x03
#define PID_CALIB_VALUE_SCALE 0.1f
typedef struct
{
	uint8_t type;
	uint16_t kp;
	uint16_t ki;
	uint16_t kd;
	uint16_t it;
	uint16_t Emax;
	uint16_t Pmax;
	uint16_t Imax;
	uint16_t Dmax;
	uint16_t Omax;
}PIDCalib_t; // PID Calibration

typedef struct
{
	float kp;
	float ki;
	float kd;
	float it;
	float Emax;
	float Pmax;
	float Imax;
	float Dmax;
	float Omax;
}PIDParam_t; // PID Parameters

#define IMU_CALIB_VALUE_SCALE 1.0f
typedef struct
{
	int16_t ax_offset;
	int16_t ay_offset;
	int16_t az_offset;
	int16_t gx_offset;
	int16_t gy_offset;
	int16_t gz_offset;
}IMUCalib_t; // IMU offset Calibration

typedef struct
{
	float ax_offset;
	float ay_offset;
	float az_offset;
	float gx_offset;
	float gy_offset;
	float gz_offset;
}IMUParam_t; // IMU Parameters

#define MAG_CALIB_VALUE_SCALE 1.0f
typedef struct
{
	int16_t mx_offset;
	int16_t my_offset;
	int16_t mz_offset;
}MagCalib_t; // Mag offset Calibration

typedef struct
{
	float mx_offset;
	float my_offset;
	float mz_offset;
}MagParam_t; // Mag offset Parameters

#define VEL_CALIB_VALUE_SCALE 1e-3f
typedef struct
{
	uint16_t x; // mm/s
	uint16_t y; // mm/s
	uint16_t z; // 1e-3rad/s
	uint16_t e; // mm/s
	uint16_t c; // 1e-3rad/s
}VelCalib_t; // Velocity Calibration

typedef struct
{
	float x; // m/s
	float y; // m/s
	float z; // rad/s
	float e; // m/s
	float c; // rad/s
}VelParam_t; // Velocity Parameters

#define MEC_CALIB_VALUE_SCALE 1e-3f
typedef struct
{
	uint16_t lx; // mm
	uint16_t ly; // mm
	uint16_t r1; // mm
	uint16_t r2; // mm
}MecCalib_t; // Mecanum Wheel Calibration

typedef struct
{
	float lx; // m
	float ly; // m
	float r1; // m
	float r2; // m
}MecParam_t; // Mecanum Wheel Parameters

#define POS_CALIB_VALUE_SCALE 1e-3f
typedef struct
{
	int16_t el; // unit: mm
	int16_t eh; // unit: mm
	int16_t cl; // unit: 1e-3*rad
	int16_t ch; // unit: 1e-3*rad
}PosCalib_t; // Position Calibration

typedef struct
{
	float el; // unit: m
	float eh; // unit: m
	float cl; // unit: rad
	float ch; // unit: rad
}PosParam_t; // Position Parameters

typedef struct
{
	PIDCalib_t cvl; // Chasis velocity loop calibration
	PIDCalib_t gvl; // Gimbal velocity loop calibration
	PIDCalib_t gpl; // Chasis position loop calibration
	IMUCalib_t imu; // IMU calibration
	MagCalib_t mag; // Mag calibration
	MecCalib_t mec; // Mecanum wheel calibration
	PosCalib_t pos; // Position calibration
}Calib_t; // Calibration

typedef struct
{
	PIDParam_t cvl; // Chasis velocity loop parameters
	PIDParam_t gvl; // Gimbal velocity loop parameters
	PIDParam_t gpl; // Chasis position loop parameters
	IMUParam_t imu; // IMU calibration
	MagParam_t mag; // Mag calibration
	MecParam_t mec; // Mecanum wheel calibration
	PosParam_t pos; // Position calibration
}Param_t; // Parameters

#pragma pack()

void Calib_PID(PID_t* pid, const PIDCalib_t* cal);

#ifdef __cplusplus
}
#endif

#endif




