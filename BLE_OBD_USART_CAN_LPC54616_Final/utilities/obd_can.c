/*
 * Create 2018/12/08
 Function: for CAN obd service;
 Receive from the BLE module via the USART
 Transmint the data to the P-CAN with UDS service;
 Receive the specifical signals from CAN ;
  
 */


#include "obd_can.h"
#include "can.h"
#include <stdbool.h>
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "can.h"

/*The Service list*/
uint8_t Service_KeepAlive[8] =  {0x02,0x3E,0x80,0x00,0x00,0x00,0x00,0x00};
uint8_t Service_ExtSession[8]=  {0x02,0x10,0x03,0x00,0x00,0x00,0x00,0x00};
uint8_t Service_ReqSeed[8]   =  {0x02,0x27,0x09,0x00,0x00,0x00,0x00,0x00};

uint8_t Service_SendKey[3][8]={ {0x10,0x0E,0x27,0x0A,0x20,0x20,0x20,0x20},
								{0x21,0x20,0x20,0x20,0x20,0x20,0x20,0x20},
								{0x22,0x20,0x00,0x00,0x00,0x00,0x00,0x00}};

																

bool message_transmitted = false;
bool message_updateToTransmit = false;
bool Service_FireWall_Pass_Cmd = false,Service_FireWall_Pass_Confirm=false;

uint8_t message_length = 8;
uint8_t message_length_cnt,Multiframe_Control_index,Multiframe_Send_index;


can_frame_t FrameToTransmit;	
can_frame_t Keep_alive_frame_3E;



															

																
TeOBD_Service_MODE OBD_Service_Mode_CMD,OBD_Service_Mode_CMD_Auto;	
TeOBD_Service_MODE OBD_Service_Mode_NextState;	
																
																
bool obd_Service(TeOBD_Service_MODE state)
{

		TeOBD_Service_MODE  LeService_CMD_Temp;

		FrameToTransmit.id=0x14dae1f1;
		FrameToTransmit.format =kCAN_FrameFormatExtend;
		FrameToTransmit.type = kCAN_FrameTypeData;
		FrameToTransmit.proto = kCAN_ProtoTypeFD;;
		FrameToTransmit.bitratemode = kCAN_BitrateModeTypeSwitch;
		FrameToTransmit.length = 8;
	
	OBD_Service_Mode_CMD=state;
	
	if((OBD_Service_Mode_CMD==CeOBD_Service_MODE_3E_KeepAlive||Service_FireWall_Pass_Cmd)&&Service_FireWall_Pass_Confirm == false)
	{
	Service_FireWall_Pass_Cmd =true;

	}
	else
	{
	Service_FireWall_Pass_Cmd=false;
	OBD_Service_Mode_CMD_Auto=(TeOBD_Service_MODE)0;
	}

	if(Service_FireWall_Pass_Cmd ==true)
	{
		OBD_Service_Mode_CMD_Auto++;
		LeService_CMD_Temp = OBD_Service_Mode_CMD_Auto;
		
		if(OBD_Service_Mode_CMD_Auto ==CeOBD_Service_MODE_No_update )
			{
			Service_FireWall_Pass_Confirm=true;
			}
	}
	else
	{
	Service_FireWall_Pass_Confirm = false;
		LeService_CMD_Temp = state;
	}
	
	
	switch( LeService_CMD_Temp)
	{
			case CeOBD_Service_MODE_3E_KeepAlive:
				
				for(message_length_cnt=0;message_length_cnt<message_length;message_length_cnt++)
				{
					FrameToTransmit.dataByte[message_length_cnt]=Service_KeepAlive[message_length_cnt];
				}
				message_updateToTransmit=true;
			//	OBD_Service_Mode_NextState=	CeOBD_Service_MODE_10_ExtSession;		
			break;
			
			case CeOBD_Service_MODE_10_ExtSession:
				for(message_length_cnt=0;message_length_cnt<message_length;message_length_cnt++)
				{
					FrameToTransmit.dataByte[message_length_cnt]=Service_ExtSession[message_length_cnt];
				}
				message_updateToTransmit=true;
				//OBD_Service_Mode_NextState=	CeOBD_Service_MODE_29_ReqSeed;						
			break;
				
			case CeOBD_Service_MODE_29_ReqSeed:
				for(message_length_cnt=0;message_length_cnt<message_length;message_length_cnt++)
				{
					FrameToTransmit.dataByte[message_length_cnt]=Service_ReqSeed[message_length_cnt];
				}
				message_updateToTransmit=true;	

			//	OBD_Service_Mode_NextState=	CeOBD_Service_MODE_0E_SendKey_MF;			
				
			break;
					
			case CeOBD_Service_MODE_0E_SendKey_MF:
				for(message_length_cnt=0;message_length_cnt<message_length;message_length_cnt++)
				{
					FrameToTransmit.dataByte[message_length_cnt]=Service_SendKey[0][message_length_cnt];
				}
				message_updateToTransmit=true;		
				
				//				OBD_Service_Mode_NextState=	CeOBD_Service_MODE_0E_SendKey_MF1;		
			break;
				
				case CeOBD_Service_MODE_0E_SendKey_MF1:
				for(message_length_cnt=0;message_length_cnt<message_length;message_length_cnt++)
				{
					FrameToTransmit.dataByte[message_length_cnt]=Service_SendKey[1][message_length_cnt];
				}
				message_updateToTransmit=true;		
				
				//				OBD_Service_Mode_NextState=	CeOBD_Service_MODE_0E_SendKey_MF2;		
			break;
				
			case CeOBD_Service_MODE_0E_SendKey_MF2:
				for(message_length_cnt=0;message_length_cnt<message_length;message_length_cnt++)
				{
					FrameToTransmit.dataByte[message_length_cnt]=Service_SendKey[2][message_length_cnt];
				}
				message_updateToTransmit=true;		
				
				//OBD_Service_Mode_NextState=	CeOBD_Service_MODE_No_update;		
			break;
						
			case CeOBD_Service_MODE_No_update:

			message_updateToTransmit=false;
			break;
							
			default:
				message_updateToTransmit=false;
		
			break;

	}
	
	if(message_updateToTransmit)
	{
		//obd_Service_MsgTrasmit(&FrameToTransmit);
		CAN_TransferSendBlocking(CAN0, 0, &FrameToTransmit);
		//CAN_TransferSendBlocking(CAN1, 0, &FrameToTransmit);
	}
	
return 0;
}	


bool obd_Service_KeepAlive(CAN_Type *base)
{
	/*Keep_alive_frame_3E data */	
	Keep_alive_frame_3E.id=0x14DA1Ef1;
	Keep_alive_frame_3E.format =kCAN_FrameFormatExtend;
	Keep_alive_frame_3E.type = kCAN_FrameTypeData;
	//Keep_alive_frame_3E.proto = kCAN_ProtoTypeClassic;
	Keep_alive_frame_3E.bitratemode = kCAN_BitrateModeTypeSwitch;
	//Keep_alive_frame_3E.proto = kCAN_ProtoTypeClassic;
	Keep_alive_frame_3E.proto = kCAN_ProtoTypeFD;
	Keep_alive_frame_3E.length = 8;
	Keep_alive_frame_3E.dataWord[0]=0x00803E02;


	obd_Service_MsgTrasmit(base,&Keep_alive_frame_3E);

return 0;
}


void obd_Service_MsgTrasmit(CAN_Type *base, can_frame_t *txFrame)
{
	
	obd_can_TxMSG_Standard(base, 0, txFrame);
	//obd_can_TxMSG_Standard(CAN1, 0, txFrame);
	
}

		
bool obd_can_TxMSG_Standard(CAN_Type *base, uint8_t mbIdx, can_frame_t *txFrame)
{
	
	        /* time to send messages from CAN0 */
           /* send 0x100 -> 0x102 on tx message buffer 0 */
	
            //for (b = 0; b < txFrame.length; b++) txFrame.dataByte[b] = b;
            /* use message buffer 0 */
            if (CAN_TransferSendBlocking(base, mbIdx, txFrame) != kStatus_Success)
            {
							
             // PRINTF("transmit");
							
            }
            else
            {
                /* toggle LED1 */
                
              return true;
                //message_transmitted = true;
             }
			
		return true;
}

void obd_can_TxMSG_Extend(CAN_Type *base, uint8_t mbIdx, can_frame_t *txFrame)
{
}

can_frame_t obd_can_TxMSG_Pack(uint8_t x[])
{
	can_frame_t CAN_frame;
	
	uint32_t Proto_Temp,Bitratemode_Temp;
	if(Get_Board_CANFD_Enable_State()==true)
	{
		Proto_Temp=kCAN_ProtoTypeFD;
		Bitratemode_Temp=kCAN_BitrateModeTypeSwitch;
	}
	else
	{
		Proto_Temp=kCAN_ProtoTypeClassic;
		Bitratemode_Temp=kCAN_BitrateModeTypeNoSwitch;
	}
	
	
	CAN_frame.format =kCAN_FrameFormatExtend;
	CAN_frame.type = kCAN_FrameTypeData;
	CAN_frame.bitratemode = Bitratemode_Temp;
	CAN_frame.proto = Proto_Temp;
	CAN_frame.length = 8;
	
	uint8_t ByteIndex;
	
	uint8_t ID_StartBype=1;
	CAN_frame.id= (x[ID_StartBype]<<24) + (x[ID_StartBype+1]<<16)+(x[ID_StartBype+2]<<8)+x[ID_StartBype+3];
	for(ByteIndex=0;ByteIndex<8;ByteIndex++)
	{
	CAN_frame.dataByte[ByteIndex]= x[ByteIndex+5];
	}

	return CAN_frame;
}

uint32_t obd_can_RxMSG_UnPack(uint8_t x[])
{
	uint32_t Rx_CAN_frameID;

	uint8_t ID_StartBype=1;
	Rx_CAN_frameID= (x[ID_StartBype]<<24) + (x[ID_StartBype+1]<<16)+(x[ID_StartBype+2]<<8)+x[ID_StartBype+3];
	

	return Rx_CAN_frameID;
}

uint16_t obd_can_TxMSG_Multiframe(TeOBD_Receive_Cmd cmd,uint8_t x[10][14],uint8_t Sending_Index)
{
uint8_t KeepAlive_Count;
	bool Send_complete=false;
	can_frame_t tx_frame1;
		
	if(cmd == CeOBD_Receive_Sending_Cmd_Multiframe||cmd ==CeOBD_Receive_Sending_Cmd_Multiframe_other)
		{
	
		tx_frame1=obd_can_TxMSG_Pack(x[Multiframe_Send_index]);	
												
		if(tx_frame1.id!=0)
		{
			obd_Service_MsgTrasmit(Get_CAN_Channel(), &tx_frame1);
			KeepAlive_Count=0;
		}

		if(Multiframe_Send_index>=Sending_Index)
		{
			//cmd=CeOBD_Receive_NoCmd;
			Send_complete=true;
			Multiframe_Send_index=0;
		}
		else
		{
			Multiframe_Send_index++;
		}
	}

return Send_complete;

}



