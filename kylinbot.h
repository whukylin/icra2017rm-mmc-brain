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

#ifndef __KYLINBOT_H__
#define __KYLINBOT_H__

#include "asp.h"
#include "serial.h"
#include "mythread.hpp"
#include <unistd.h>
#include <stdio.h>
#include <string.h>

#define KYLINBOT_MSG_BUF_LEN 256
#define KYLINBOT_SERIAL_PORT "/dev/ttyTHS2"
#define KYLINBOT_SERIAL_IO_TIMEOUT 30
#define KYLINBOT_SERIAL_BAUD_RATE 115200

int Kylinbot_Connect(const char* port = KYLINBOT_SERIAL_PORT);
int Kylinbot_PullMsg(KylinMsg_t* kylinMsg);
int Kylinbot_PushMsg(const KylinMsg_t* kylinMsg);
int Kylinbot_Disconnect();

#endif
