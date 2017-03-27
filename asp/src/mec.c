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
 
#include "mec.h"

/*******************************************/
/* Mecanum Wheel Power Transmission System */
/*******************************************/
/*              2        1                 */
/*                  |y                     */
/*                 b|___x                  */
/*               z    a                    */
/*              3        4                 */
/*                                         */
/*******************************************/

void Mec_Config(Mec_t* mec, float lx, float ly, float r1, float r2)
{
	mec->l = lx + ly;
	mec->r = r1 + r2;

	mec->cx = r1 / 4.0f;
	mec->cy = r2 / 4.0f;
	mec->cz = mec->r / 4.0f / mec->l;
}

void Mec_Synthe(const Mec_t* mec, const float* w, float* v)
{
	v[0] = ( w[0] + w[1] - w[2] - w[3]) * mec->cx;
	v[1] = (-w[0] + w[1] + w[2] - w[3]) * mec->cy;
	v[2] = ( w[0] + w[1] + w[2] + w[3]) * mec->cz;
}

void Mec_Decomp(const Mec_t* mec, const float* v, float* w)
{
	w[0] = ( v[0] - v[1] + v[2] * mec->l) / mec->r;
	w[1] = ( v[0] + v[1] + v[2] * mec->l) / mec->r;
	w[2] = (-v[0] + v[1] + v[2] * mec->l) / mec->r;
	w[3] = (-v[0] - v[1] + v[2] * mec->l) / mec->r;
}

