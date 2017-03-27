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

#include "kylinbot.h"

static uint8_t exit_flag = 0;

static FIFO_t rx_fifo;
static uint8_t rx_buf[2][KYLINBOT_MSG_BUF_LEN];
static KylinMsg_t rxKylinMsg;

static FIFO_t tx_fifo;
static uint8_t tx_buf[2][KYLINBOT_MSG_BUF_LEN];
static KylinMsg_t txKylinMsg;

static void init()
{
	FIFO_Init(&rx_fifo, rx_buf[0], KYLINBOT_MSG_BUF_LEN);
	FIFO_Init(&tx_fifo, tx_buf[0], KYLINBOT_MSG_BUF_LEN);
}

static void Dnl_ProcKylinMsg(const KylinMsg_t* kylinMsg)
{
        //printf("*************************************KYLIN********************************************\n");
        printf("fs=%x,px=%d,py=%d,pz=%d,pe=%d,pc=%d,vx=%d,vy=%d,vz=%d,ve=%d,vc=%d\n", kylinMsg->fs,
                kylinMsg->cp.x, kylinMsg->cp.y, kylinMsg->cp.z, kylinMsg->gp.e, kylinMsg->gp.c,
                kylinMsg->cv.x, kylinMsg->cv.y, kylinMsg->cv.z, kylinMsg->gv.e, kylinMsg->gv.c);
}

static void PullMsg()
{
	// Get fifo free space
	int len = FIFO_GetFree(&rx_fifo);
	// If fifo free space insufficient, pop one element out
	if (!len) {
		uint8_t b;
		len = FIFO_Pop(&rx_fifo, &b, 1);
	}
	// Read input stream according to the fifo free space left
	len = read_serial(rx_buf[1], len, KYLINBOT_SERIAL_IO_TIMEOUT);
	//printf("PULL LEN= %d\n", len);
	// Push stream into fifo
	FIFO_Push(&rx_fifo, rx_buf[1], len);
	// Check if any message received
	if (Msg_Pop(&rx_fifo, rx_buf[1], &msg_head_kylin, &rxKylinMsg)) {
	    Dnl_ProcKylinMsg(&rxKylinMsg);
	}
}

static void *KylinBotMsgPullerThreadFunc(void* param)
{
    while (exit_flag == 0) {
	      PullMsg();
	      usleep(4000);
	}
	return NULL;
}

static void PushMsg()
{
	txKylinMsg.fs = 1;
	txKylinMsg.cv.x = 2000;
	uint32_t len = Msg_Push(&tx_fifo, tx_buf[1], & msg_head_kylin, &txKylinMsg);
	FIFO_Pop(&tx_fifo, tx_buf[1], len);
	write_serial(tx_buf[1], len, KYLINBOT_SERIAL_IO_TIMEOUT);
}

static void *KylinBotMsgPusherThreadFunc(void* param)
{
    while (exit_flag == 0) {
	  PushMsg();
	  usleep(4000);
    }
}

int Kylinbot_Connect(const char* port)
{
    if (connect_serial(port,KYLINBOT_SERIAL_BAUD_RATE) == -1)
    {
	  printf("serial open error!\n");
	  return -1;
    }
    
    MyThread kylibotMsgPullerThread;
    MyThread kylibotMsgPusherThread;

    kylibotMsgPullerThread.create(KylinBotMsgPullerThreadFunc, NULL);
    kylibotMsgPusherThread.create(KylinBotMsgPusherThreadFunc, NULL);
    
    while (1) {
	  //PullMsg();
    }
	
    return 0;
}

int Kylinbot_PullMsg(KylinMsg_t* kylinMsg)
{
  memcpy(&kylinMsg, &rxKylinMsg, sizeof(KylinMsg_t));
  return 0;
}

int Kylinbot_PushMsg(const KylinMsg_t* kylinMsg)
{
  memcpy(&txKylinMsg, kylinMsg, sizeof(KylinMsg_t));
  return 0;
}

int Kylinbot_Disconnect()
{
  disconnect_serial();
  exit_flag = 1;
  return 0;
}
