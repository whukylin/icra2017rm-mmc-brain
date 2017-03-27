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
 
#include "calib.h"

/*****************************************/
/*              Calibration              */
/*****************************************/

void Calib_PID(PID_t* pid, const PIDCalib_t* cal)
{
	pid->kp = PID_CALIB_VALUE_SCALE * cal->kp;
	pid->ki = PID_CALIB_VALUE_SCALE * cal->ki;
	pid->kd = PID_CALIB_VALUE_SCALE * cal->kd;
	pid->it = PID_CALIB_VALUE_SCALE * cal->it;
	pid->Emax = PID_CALIB_VALUE_SCALE * cal->Emax;
	pid->Pmax = PID_CALIB_VALUE_SCALE * cal->Pmax;
	pid->Imax = PID_CALIB_VALUE_SCALE * cal->Imax;
	pid->Dmax = PID_CALIB_VALUE_SCALE * cal->Dmax;
	pid->Omax = PID_CALIB_VALUE_SCALE * cal->Omax;
}


