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
 
#ifndef __MSG_H__
#define __MSG_H__

/********************************************************/
/*                 KylinBot Msg Type                    */
/*     Basic frame structure:                           */
/*       ________________________________________       */
/*      |id:8|length:8|token:16|data~|checksum:16|      */
/*      |______________________|_____|___________|      */
/*      |         head         |body~|    crc    |      */
/*      |________________________________________|      */
/*                        Msg                           */
/********************************************************/

#ifdef __cplusplus
extern "C" {
#endif

#include "cbus.h"
#include "dbus.h"
#include "fifo.h"
#include "calib.h"
#include "crc16.h"
	
/* Message length minimum/maximum limit */
#define MSG_LEN_MIN 6u
#define MSG_LEN_MAX 256u
#define MSG_LEN_EXT 6u

#pragma pack(1)

/* Message head union typedef */
typedef union MsgHead_t
{
	uint32_t value; // Message head value in 32bit
	struct
	{
		uint8_t id : 8; // Message ID
		uint8_t length : 8; // Message length (unit: byte)
		uint16_t token : 16; // Message CRC token
	}attr; // Message head attributes
}MsgHead_t; // Message head union typedef

typedef Rcp_t VirtualRC_t;
typedef Hcp_t VirtualHC_t;
typedef DBUS_t VirtualDBUS_t;
typedef CBUS_t VirtualCBUS_t;

#define ZGYRO_ANGLE_RECIP 1e-2f // To scale zgyro angle to deg
#define ZGYRO_RATE_RECIP 1e-5f // To scale zgyro rate to deg/s
typedef struct
{
	int32_t angle; // = (deg*100)
	int16_t rate; // = delta(angle)/1ms
}ZGyroMsg_t;

#define MOTOR1_ID 0
#define MOTOR2_ID 1
#define MOTOR3_ID 2
#define MOTOR4_ID 3
#define MOTOR5_ID 4
#define MOTOR6_ID 5
#define MOTOR_ECD_ANGLE_MAX 8191
#define MOTOR_ECD_ANGLE_MAX 8191
#define MOTOR_ESC_CURRENT_MAX 13000
typedef struct
{
	uint8_t id; // 0~5
	uint16_t ecd_angle; // Encoder angle, range from 0~8191
	int32_t angle; // Continuous angle, infinite
	int16_t rate; // Rate in ecd_diff/1ms
	int32_t current; // Motor current, max to 13000
}MotorMsg_t;

#define IMU9X_MSG_VALUE_SCALE 1.0f
typedef struct
{
	int16_t ax;
	int16_t ay;
	int16_t az;
	int16_t gx;
	int16_t gy;
	int16_t gz;
	int16_t mx;
	int16_t my;
	int16_t mz;
}IMU9XMsg_t;

#define ODOME_MSG_VALUE_SCALE 1e3f
typedef struct
{
	int32_t px; // Bot position (linear) in x-axis, unit: mm
	int32_t py; // Bot position (linear) in y-axis, unit: mm
	int32_t pz; // Bot position (angular) in z-axis, unit: mm
	int16_t vx; // Bot velocity (linear) in x-axis, unit: mm
	int16_t vy; // Bot velocity (linear) in y-axis, unit: mm
	int16_t vz; // Bot velocity (angular) in z-axis, unit: mm
}OdomeMsg_t;

#define GRASP_MSG_VALUE_SCALE 1e3f
typedef struct
{
	int16_t pe; // Elevator position
	int16_t pc; // Claw position
}GraspMsg_t;

#define CALIB_FLAG_BIT_IMU (1u<<0)
#define CALIB_FLAG_BIT_MAG (1u<<1)
#define CALIB_FLAG_BIT_POS (1u<<2)
typedef struct
{
	uint32_t auto_cali_flag; // Auto calibration control bits
}CalibMsg_t;

#define WDG_ERR_BIT_RCV         (1u<<0)
#define WDG_ERR_BIT_TTY         (1u<<1)
#define WDG_ERR_BIT_BTM         (1u<<2)
#define WDG_ERR_BIT_DBI         (1u<<3)
#define WDG_ERR_BIT_IMU         (1u<<4)
#define WDG_ERR_BIT_VRC         (1u<<5)
#define WDG_ERR_BIT_VHC         (1u<<6)
#define WDG_ERR_BIT_VDBUS       (1u<<7)
#define WDG_ERR_BIT_VCBUS       (1u<<8)
#define WDG_ERR_BIT_CALIB       (1u<<9)
#define WDG_ERR_BIT_ZGYRO       (1u<<10)
#define WDG_ERR_BIT_MOTOR1      (1u<<11)
#define WDG_ERR_BIT_MOTOR2      (1u<<12)
#define WDG_ERR_BIT_MOTOR3      (1u<<13)
#define WDG_ERR_BIT_MOTOR4      (1u<<14)
#define WDG_ERR_BIT_MOTOR5      (1u<<15)
#define WDG_ERR_BIT_MOTOR6      (1u<<16)

#define INI_FLAG_BIT_ZGYRO      (1u<<0)
#define INI_FLAG_BIT_MOTOR1     (1u<<1)
#define INI_FLAG_BIT_MOTOR2     (1u<<2)
#define INI_FLAG_BIT_MOTOR3     (1u<<3)
#define INI_FLAG_BIT_MOTOR4     (1u<<4)
#define INI_FLAG_BIT_MOTOR5     (1u<<5)
#define INI_FLAG_BIT_MOTOR6     (1u<<6)

typedef struct
{
	uint32_t wdg; // Watchdog
	uint32_t ini; // Initialization status
}StatuMsg_t;

typedef struct
{
	uint32_t msg_type;
}SubscMsg_t;

#define AXIS3_MSG_VALUE_SCALE 1e3f
typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
}Axis3Msg_t;

#define AXIS2_MSG_VALUE_SCALE 1e3f
typedef struct
{
	int16_t e;
	int16_t c;
}Axis2Msg_t;

#define KYLIN_MSG_VALUE_SCALE AXIS3_MSG_VALUE_SCALE
typedef struct
{
	uint32_t fs; // Functional state flag bits
	Axis3Msg_t cv; // Chassis velocity, unit: linear: mm/s, angular: 1e-3rad/s
	Axis3Msg_t cp; // Chassis position, unit: linear: mm, angular: 1e-3rad
	Axis2Msg_t gv; // Grabber velocity, unit: linear: mm/s, angular: 1e-3rad/s
	Axis2Msg_t gp; // Grabber position, unit: linear: mm, angular: rad
}KylinMsg_t; // Length unit: mm, time unit: s

#define WRAP_U8(V) ((uint8_t)V)
#define WRAP_U16(V) ((uint16_t)V)
#define WRAP_U32(V) ((uint32_t)V)

#define MSG_ID_VRC WRAP_U8(0x01)
#define MSG_ID_VHC WRAP_U8(0x02)
#define MSG_ID_VDBUS WRAP_U8(0x03)
#define MSG_ID_VCBUS WRAP_U8(0x04)
#define MSG_ID_ZGYRO WRAP_U8(0x05)
#define MSG_ID_IMU9X WRAP_U8(0x06)
#define MSG_ID_MOTOR WRAP_U8(0x07)
#define MSG_ID_ODOME WRAP_U8(0x08)
#define MSG_ID_GRASP WRAP_U8(0x09)
#define MSG_ID_STATU WRAP_U8(0x0a)
#define MSG_ID_SUBSC WRAP_U8(0x0b)
#define MSG_ID_CALIB WRAP_U8(0x0c)
#define MSG_ID_KYLIN WRAP_U8(0x0d)
#define MSG_ID_PID_CALIB WRAP_U8(0x0e)
#define MSG_ID_IMU_CALIB WRAP_U8(0x0f)
#define MSG_ID_MAG_CALIB WRAP_U8(0x10)
#define MSG_ID_VEL_CALIB WRAP_U8(0x11)
#define MSG_ID_MEC_CALIB WRAP_U8(0x12)
#define MSG_ID_POS_CALIB WRAP_U8(0x13)

#define MSG_LEN_VRC sizeof(VirtualRC_t)
#define MSG_LEN_VHC sizeof(VirtualHC_t)
#define MSG_LEN_VDBUS sizeof(VirtualDBUS_t)
#define MSG_LEN_VCBUS sizeof(VirtualCBUS_t)
#define MSG_LEN_ZGYRO sizeof(ZGyroMsg_t)
#define MSG_LEN_IMU9X sizeof(IMU9XMsg_t)
#define MSG_LEN_MOTOR sizeof(MotorMsg_t)
#define MSG_LEN_ODOME sizeof(OdomeMsg_t)
#define MSG_LEN_GRASP sizeof(GraspMsg_t)
#define MSG_LEN_STATU sizeof(StatuMsg_t)
#define MSG_LEN_SUBSC sizeof(SubscMsg_t)
#define MSG_LEN_CALIB sizeof(CalibMsg_t)
#define MSG_LEN_KYLIN sizeof(KylinMsg_t)
#define MSG_LEN_PID_CALIB sizeof(PIDCalib_t)
#define MSG_LEN_IMU_CALIB sizeof(IMUCalib_t)
#define MSG_LEN_MAG_CALIB sizeof(MagCalib_t)
#define MSG_LEN_VEL_CALIB sizeof(VelCalib_t)
#define MSG_LEN_MEC_CALIB sizeof(MecCalib_t)
#define MSG_LEN_POS_CALIB sizeof(PosCalib_t)
	
#define MSG_TOKEN_VRC WRAP_U16(0x1234)
#define MSG_TOKEN_VHC WRAP_U16(0x2345)
#define MSG_TOKEN_VDBUS WRAP_U16(0x3456)
#define MSG_TOKEN_VCBUS WRAP_U16(0x4567)
#define MSG_TOKEN_ZGYRO WRAP_U16(0x5678)
#define MSG_TOKEN_IMU9X WRAP_U16(0x6789)
#define MSG_TOKEN_MOTOR WRAP_U16(0x789a)
#define MSG_TOKEN_ODOME WRAP_U16(0x89ab)
#define MSG_TOKEN_GRASP WRAP_U16(0x9abc)
#define MSG_TOKEN_STATU WRAP_U16(0xabcd)
#define MSG_TOKEN_SUBSC WRAP_U16(0xbcde)
#define MSG_TOKEN_CALIB WRAP_U16(0xcdef)
#define MSG_TOKEN_KYLIN WRAP_U16(0xfedc)
#define MSG_TOKEN_PID_CALIB WRAP_U16(0xedcb)
#define MSG_TOKEN_IMU_CALIB WRAP_U16(0xdcba)
#define MSG_TOKEN_MAG_CALIB WRAP_U16(0xcba9)
#define MSG_TOKEN_VEL_CALIB WRAP_U16(0xba98)
#define MSG_TOKEN_MEC_CALIB WRAP_U16(0xa987)
#define MSG_TOKEN_POS_CALIB WRAP_U16(0x9876)

#define MSG_HEAD_VALUE(ID,LEN,TOKEN) ((WRAP_U32(TOKEN)<<16) | (WRAP_U32(LEN)<<8) | WRAP_U32(ID))
#define MSG_HEAD_VALUE_OF(NAME) MSG_HEAD_VALUE(MSG_ID_##NAME,MSG_LEN_##NAME,MSG_TOKEN_##NAME)

#define MSG_HEAD_VALUE_VRC MSG_HEAD_VALUE_OF(VRC)
#define MSG_HEAD_VALUE_VHC MSG_HEAD_VALUE_OF(VHC)
#define MSG_HEAD_VALUE_VDBUS MSG_HEAD_VALUE_OF(VDBUS)
#define MSG_HEAD_VALUE_VCBUS MSG_HEAD_VALUE_OF(VCBUS)
#define MSG_HEAD_VALUE_ZGYRO MSG_HEAD_VALUE_OF(ZGYRO)
#define MSG_HEAD_VALUE_IMU9X MSG_HEAD_VALUE_OF(IMU9X)
#define MSG_HEAD_VALUE_MOTOR MSG_HEAD_VALUE_OF(MOTOR)
#define MSG_HEAD_VALUE_ODOME MSG_HEAD_VALUE_OF(ODOME)
#define MSG_HEAD_VALUE_GRASP MSG_HEAD_VALUE_OF(GRASP)
#define MSG_HEAD_VALUE_STATU MSG_HEAD_VALUE_OF(STATU)
#define MSG_HEAD_VALUE_SUBSC MSG_HEAD_VALUE_OF(SUBSC)
#define MSG_HEAD_VALUE_CALIB MSG_HEAD_VALUE_OF(CALIB)
#define MSG_HEAD_VALUE_KYLIN MSG_HEAD_VALUE_OF(KYLIN)
#define MSG_HEAD_VALUE_PID_CALIB MSG_HEAD_VALUE_OF(PID_CALIB)
#define MSG_HEAD_VALUE_IMU_CALIB MSG_HEAD_VALUE_OF(IMU_CALIB)
#define MSG_HEAD_VALUE_MAG_CALIB MSG_HEAD_VALUE_OF(MAG_CALIB)
#define MSG_HEAD_VALUE_VEL_CALIB MSG_HEAD_VALUE_OF(VEL_CALIB)
#define MSG_HEAD_VALUE_MEC_CALIB MSG_HEAD_VALUE_OF(MEC_CALIB)
#define MSG_HEAD_VALUE_POS_CALIB MSG_HEAD_VALUE_OF(POS_CALIB)

#define MSG_HEAD_VRC { MSG_HEAD_VALUE_VRC }
#define MSG_HEAD_VHC { MSG_HEAD_VALUE_VHC }
#define MSG_HEAD_VDBUS { MSG_HEAD_VALUE_VDBUS }
#define MSG_HEAD_VCBUS { MSG_HEAD_VALUE_VCBUS }
#define MSG_HEAD_ZGYRO { MSG_HEAD_VALUE_ZGYRO }
#define MSG_HEAD_IMU9X { MSG_HEAD_VALUE_IMU9X }
#define MSG_HEAD_MOTOR { MSG_HEAD_VALUE_MOTOR }
#define MSG_HEAD_ODOME { MSG_HEAD_VALUE_ODOME }
#define MSG_HEAD_GRASP { MSG_HEAD_VALUE_GRASP }
#define MSG_HEAD_STATU { MSG_HEAD_VALUE_STATU }
#define MSG_HEAD_SUBSC { MSG_HEAD_VALUE_SUBSC }
#define MSG_HEAD_CALIB { MSG_HEAD_VALUE_CALIB }
#define MSG_HEAD_KYLIN { MSG_HEAD_VALUE_KYLIN }
#define MSG_HEAD_PID_CALIB { MSG_HEAD_VALUE_PID_CALIB }
#define MSG_HEAD_IMU_CALIB { MSG_HEAD_VALUE_IMU_CALIB }
#define MSG_HEAD_MAG_CALIB { MSG_HEAD_VALUE_MAG_CALIB }
#define MSG_HEAD_VEL_CALIB { MSG_HEAD_VALUE_VEL_CALIB }
#define MSG_HEAD_MEC_CALIB { MSG_HEAD_VALUE_MEC_CALIB }
#define MSG_HEAD_POS_CALIB { MSG_HEAD_VALUE_POS_CALIB }

#define MSG_TYPE_IDX_VRC 0u
#define MSG_TYPE_IDX_VHC 1u
#define MSG_TYPE_IDX_VDBUS 2u
#define MSG_TYPE_IDX_VCBUS 3u
#define MSG_TYPE_IDX_ZGYRO 4u
#define MSG_TYPE_IDX_IMU9X 5u
#define MSG_TYPE_IDX_MOTOR 6u
#define MSG_TYPE_IDX_ODOME 7u
#define MSG_TYPE_IDX_GRASP 8u
#define MSG_TYPE_IDX_STATU 9u
#define MSG_TYPE_IDX_SUBSC 10u
#define MSG_TYPE_IDX_CALIB 11u
#define MSG_TYPE_IDX_KYLIN 12u
#define MSG_TYPE_IDX_PID_CALIB 13u
#define MSG_TYPE_IDX_IMU_CALIB 14u
#define MSG_TYPE_IDX_MAG_CALIB 15u
#define MSG_TYPE_IDX_VEL_CALIB 16u
#define MSG_TYPE_IDX_MEC_CALIB 17u
#define MSG_TYPE_IDX_POS_CALIB 18u

typedef enum
{
	MSG_TYPE_VRC = 1u << MSG_TYPE_IDX_VRC,
	MSG_TYPE_VHC = 1u << MSG_TYPE_IDX_VHC,
	MSG_TYPE_VDBUS = 1u << MSG_TYPE_IDX_VDBUS,
	MSG_TYPE_VCBUS = 1u << MSG_TYPE_IDX_VCBUS,
	MSG_TYPE_ZGYRO = 1u << MSG_TYPE_IDX_ZGYRO,
	MSG_TYPE_IMU9X = 1u << MSG_TYPE_IDX_IMU9X,
	MSG_TYPE_MOTOR = 1u << MSG_TYPE_IDX_MOTOR,
	MSG_TYPE_ODOME = 1u << MSG_TYPE_IDX_ODOME,
	MSG_TYPE_GRASP = 1u << MSG_TYPE_IDX_GRASP,
	MSG_TYPE_STATU = 1u << MSG_TYPE_IDX_STATU,
	MSG_TYPE_SUBSC = 1u << MSG_TYPE_IDX_SUBSC,
	MSG_TYPE_CALIB = 1u << MSG_TYPE_IDX_CALIB,
	MSG_TYPE_KYLIN = 1u << MSG_TYPE_IDX_KYLIN,
	MSG_TYPE_PID_CALIB = 1u << MSG_TYPE_IDX_PID_CALIB,
	MSG_TYPE_IMU_CALIB = 1u << MSG_TYPE_IDX_IMU_CALIB,
	MSG_TYPE_MAG_CALIB = 1u << MSG_TYPE_IDX_MAG_CALIB,
	MSG_TYPE_VEL_CALIB = 1u << MSG_TYPE_IDX_VEL_CALIB,
	MSG_TYPE_MEC_CALIB = 1u << MSG_TYPE_IDX_MEC_CALIB,
	MSG_TYPE_POS_CALIB = 1u << MSG_TYPE_IDX_POS_CALIB,
}MsgType_t;

#pragma pack()

/**
 * @brief: Push a single message to message fifo. 
 * @param fifo Message fifo
 * @param buf Message buffer
 * @param head Message head
 * @param body Message body
 * @return Message length (num of bytes)
 */
uint32_t Msg_Push(FIFO_t* fifo, void* buf, const void* head, const void* body);

/**
 * @brief Pop a single message from message fifo. 
 * @param fifo Message fifo
 * @param buf Message buffer
 * @param head Message head
 * @param body Message body
 * @return Message length (num of bytes)
 */
uint32_t Msg_Pop(FIFO_t* fifo, void* buf, const void* head, void* body);

/**
 * @brief Get the full length of a specific message head. 
 * @param head Message head
 * @return Full (head + body + crc) message length (num of bytes)
 */
uint32_t Msg_GetFullLen(const void* head);

extern const MsgHead_t msg_head_vrc;
extern const MsgHead_t msg_head_vhc;
extern const MsgHead_t msg_head_vdbus;
extern const MsgHead_t msg_head_vcbus;
extern const MsgHead_t msg_head_zgyro;
extern const MsgHead_t msg_head_imu9x;
extern const MsgHead_t msg_head_motor;
extern const MsgHead_t msg_head_odome;
extern const MsgHead_t msg_head_grasp;
extern const MsgHead_t msg_head_statu;
extern const MsgHead_t msg_head_subsc;
extern const MsgHead_t msg_head_calib;
extern const MsgHead_t msg_head_kylin;
extern const MsgHead_t msg_head_pid_calib;
extern const MsgHead_t msg_head_imu_calib;
extern const MsgHead_t msg_head_mag_calib;
extern const MsgHead_t msg_head_vel_calib;
extern const MsgHead_t msg_head_mec_calib;
extern const MsgHead_t msg_head_pos_calib;

#ifdef __cplusplus
}
#endif

#endif


