/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *

 */
 
#ifndef _OBD_CAN_H_
#define _OBD_CAN_H_
#include "can.h"

/* OBD-II Modes */
#define OBD_MODE_SHOW_CURRENT_DATA        0x01
#define OBD_MODE_SHOW_FREEZE_FRAME        0x02
#define OBD_MODE_READ_DTC                 0x03
#define OBD_MODE_CLEAR_DTC                0x04
#define OBD_MODE_TEST_RESULTS_NON_CAN     0x05
#define OBD_MODE_TEST_RESULTS_CAN         0x06
#define OBD_MODE_READ_PENDING_DTC         0x07
#define OBD_MODE_CONTROL_OPERATIONS       0x08
#define OBD_MODE_VEHICLE_INFORMATION      0x09
#define OBD_MODE_READ_PERM_DTC            0x0A

typedef enum
{
	CeOBD_Service_MODE_Inactive,                /* 00 */
   CeOBD_Service_MODE_3E_KeepAlive,           /* 01 */      
   CeOBD_Service_MODE_10_ExtSession,            /* 02 */
   CeOBD_Service_MODE_29_ReqSeed,             /* 03 */
   CeOBD_Service_MODE_0E_SendKey_MF,           /* 04 mutil frame*/      
	   CeOBD_Service_MODE_0E_SendKey_MF1,        /* 05 mutil frame1*/
	   CeOBD_Service_MODE_0E_SendKey_MF2, /* 03 mutil frame2*/
	 CeOBD_Service_MODE_No_update         /* No_Update*/
} TeOBD_Service_MODE;

typedef enum
{
	CeOBD_Receive_list_0,                /* 00 */
   CeOBD_Receive_list_1,           /* 01 */      
   CeOBD_Receive_list_2,            /* 02 */
   CeOBD_Receive_list_3,             /* 03 */
   CeOBD_Receive_list_4,           /* 04 mutil frame*/      
   
} TeOBD_Receive_list;


typedef enum
{
	CeCAN_inactive,                /* 00 */
   CeCAN_500K,           /* 01 */      
   CeCAN_125K,            /* 02 */
   CeCAN_33K,             /* 03 */
   CeCAN_invalid,           /* 04 mutil frame*/      
   
} TeCAN_Config;

typedef enum
{
	CeCANFD_inactive,                /* 00 */
   CeCANFD_5M,           /* 01 */      
   CeCANFD_2M,            /* 02 */
   CeCANFD_invalid,             /* 03 */
    
   
} TeCANFD_Config;

typedef struct CAN_init
{
	TeCAN_Config Classis_CAN; 
	TeCANFD_Config FD_CAN;
	bool TRes_En;
	uint8_t channel;
	int16_t ID1;
	int16_t ID2;	
  int16_t ID3;
} TeCAN_init;


typedef enum
{
	CeOBD_Receive_NoCmd,                /* 00 */
   CeOBD_Receive_Config_init,           /* 01 */      
   CeOBD_Receive_Sending_Cmd,            /* 02 */
   CeOBD_Receive_Sending_Cmd_Multiframe,             /* 03 */
   CeOBD_Receive_Sending_Cmd_Multiframe_other,             /* 04 */
   CeOBD_Receive_Complete,           /* 05 mutil frame*/      
   
} TeOBD_Receive_Cmd;

typedef enum
{
	 CeUSART_Receive_inactive,                /* 00 */
   CeUSART_Receive_Start,           /* 01 */      
   CeUSART_Receive_OnGoing,            /* 02 */
	CeUSART_Receive_Complete,             /* 03 */               
   
} TeUSART_Receive_State;



extern TeOBD_Service_MODE OBD_Service_Mode_CMD;	




bool obd_can_TxMSG_Standard(CAN_Type *base, uint8_t mbIdx, can_frame_t *txFrame);

bool obd_Service(TeOBD_Service_MODE state);

void obd_Service_MsgTrasmit(CAN_Type *base, can_frame_t *txFrame);

void obd_can_TxMSG_Extend(CAN_Type *base, uint8_t mbIdx, can_frame_t *txFrame);

bool obd_Service_KeepAlive(CAN_Type *base);

can_frame_t obd_can_TxMSG_Pack(uint8_t x[]);
uint32_t obd_can_RxMSG_UnPack(uint8_t x[]);
uint16_t obd_can_TxMSG_Multiframe(TeOBD_Receive_Cmd cmd,uint8_t x[10][14],uint8_t Sending_Index);



#endif /* _OBD_CAN_H_ */






