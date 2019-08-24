/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
 /* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

/*  Standard C Included Files */
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "can.h"
#include "fsl_usart.h"
#include "obd_can.h"
#include "obd_usart.h"
#include "obd_control.h"
#include "pin_mux.h"
//#include "can_5461x.h"


#include <stdbool.h>
/*******************************************************************************
 * Definitions for RTOS
 ******************************************************************************/
 
#define TOUCHTASK_STACKSIZE 100
#define TOUCHTASK_PRIORITY  (tskIDLE_PRIORITY + 1UL)
#define LCDTASK_PRIORITY  (tskIDLE_PRIORITY + 0UL)
#define LCDTASK_STACKSIZE 100

/**
 * Touch status check delay
 */
#define TOUCH_DELAY   (50)
#define LCD_DELAY   (20)


static void vTouchTask(void *pvParameters);
static void vLcdTask(void *pvParameters);

void vTask_UsartReceive_Detection(void);
void vTask_UsartReceive_UnPack(void);


//void USART_ReceiveData(void);

TaskHandle_t xTouchTaskHandle = NULL;
TaskHandle_t xLcdTaskHandle = NULL;
 
volatile uint32_t g_systickCounter;



void SysTick_DelayTicks(uint32_t n)
{
    g_systickCounter = n;
    while(g_systickCounter != 0U)
    {
    }
}
/*******************************************************************************
 * Definitions For GPIO
 ******************************************************************************/

#define APP_BOARD_TEST_GPIO_PORT1 BOARD_LED3_GPIO_PORT
#define APP_BOARD_TEST_GPIO_PORT2 BOARD_LED1_GPIO_PORT
#define APP_BOARD_TEST_GPIO_PORT3 BOARD_LED2_GPIO_PORT
#define APP_BOARD_TEST_LED1_PIN BOARD_LED3_GPIO_PIN
#define APP_BOARD_TEST_LED2_PIN BOARD_LED1_GPIO_PIN
#define APP_BOARD_TEST_LED3_PIN BOARD_LED2_GPIO_PIN

/*******************************************************************************
 * Definitions for USART
 ******************************************************************************/


/*! @brief Ring buffer size (Unit: Byte). */
#define DEMO_RING_BUFFER_SIZE 16

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

#define TICKRATE_HZ (1000)	          /* 1000 ticks per second */
#define TRANSMIT_PERIOD (500)         /* milliseconds between transmission */
#define KeepAlive_PERIOD (2000)         /* milliseconds between transmission */

static volatile uint32_t gTimCnt = 0,gTimCnt_old; /* incremented every millisecond */

static volatile bool KeepAlive_PERIOD_flag_interrupt;

volatile uint16_t txIndex; /* Index of the data to send out. */

uint16_t rxIndex_old,rxIndex_count,rxIndex_loop,delay_count,debug_count;
bool rxIndex_updated,tx_CAN_Enable,message_received,Keep_Service_Active,KeepAlive_PERIOD_flag;
bool Keep_Service_Active_Send;
uint16_t Rx_Msg_Cnt,Rx_Msg_Loop_Cnt;

#define KeepAlive_Peroid_Cnt_2s (2000/TOUCH_DELAY)
#define KeepAlive_Peroid_Cnt_100ms (100/TOUCH_DELAY)

uint16_t KeepAlive_Peroid_2s_Count,total_index;


uint8_t VfCANH_RxMSG_Data;
uint16_t VfCANH_RxMSG_ID,Array_Cycle,USART_rxIndex,KeepSendTimeCnt,KeepSendOneTime;

uint8_t VfUSART_Data[DEMO_RING_BUFFER_SIZE];
uint8_t USART_Data[DEMO_RING_BUFFER_SIZE],i;
uint8_t demoRingBuffer[DEMO_RING_BUFFER_SIZE];
uint8_t demoRingBuffer_Total[280];
uint8_t demoRingBuffer_init[DEMO_RING_BUFFER_SIZE]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

uint8_t Usart_Send_Test[14]={0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee};

uint8_t Usart_Received_Feedback_1[14]={0xA1,0x81,0x00,0x00,0x04,0xC9,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t Usart_Received_Feedback_2[14]={0xA1,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t Usart_Received_Feedback_3[14]={0xA1,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

uint8_t Multiframe_FireWall_Cmd[10][14];

uint8_t Usart_Config_Init[14];

uint8_t Multiframe_FireWall_index,FrameIndex,FrameIndex_rest;
bool Multiframe_FireWall_Send;
/**
Multiframe_FireWall_index=1 -->Multiframe_FireWall_Cmd
*/

uint8_t ReceiveDataFromCAN_to_USART[12],USART_Sending_Block_Cnt;

usart_handle_t usart0_Define;
bool usart_first_Datareceived,usart_Receive_Complete;

uint8_t g_tipString[] =
    "Usart functional API interrupt example\r\nBoard receives characters then sends them out\r\nNow please input:\r\n";
		
uint8_t Multi_Frame_Key[2][8]={{0x21,0x20,0x20,0x20,0x20,0x20,0x20,0x20},
																{0x22,0x20,0x00,0x00,0x00,0x00,0x00,0x00}};

																
//按键消息队列的数量
#define KEYMSG_Q_NUM    1  		//按键消息队列的数量  
#define MESSAGE_Q_NUM   4   	//发送数据的消息队列的数量 
QueueHandle_t Key_Queue;   		//按键值消息队列句柄
QueueHandle_t Message_Queue;	//信息队列句柄
#define USART_REC_LEN  			20  	//定义最大接收字节数 50
																
/* Usart queue*/
extern QueueHandle_t Message_Queue;	//信息队列句柄


bool G_Ble_Connect_Status,G_BLE_CAN_Init;



TeOBD_Receive_list BLE_Command_Receive_list;
TeOBD_Receive_Cmd BLE_Receive_Command;



TeCAN_init BLE_Config_CAN;



TeUSART_Receive_State  VeUSART_Receive_State;
																
can_frame_t Rxmsg_TransOilTem = { 0 };

void vBLE_Command_Mode_Action(TeOBD_Control_MODE cmdMode);

/*******************************************************************************
 * Code
 ******************************************************************************/
 void DEMO_USART_IRQHandler(void)
{
    uint8_t data;
		rxIndex_updated=false;
		delay_count=0;

	
    /* If new data arrived. */
    if ((kUSART_RxFifoNotEmptyFlag | kUSART_RxError) & USART_GetStatusFlags(USART0))
    {
			
      			data = USART_ReadByte(USART0);
      			demoRingBuffer[USART_rxIndex] = data;
				demoRingBuffer_Total[total_index]=data;
				total_index++;
				VeUSART_Receive_State= CeUSART_Receive_Start;

				
			if(total_index>=200)
			{
			total_index=0;
			}
				/*All cmd start as 0xE...*/
			
				if((demoRingBuffer[0]>>4)==0x0E)
				{
					usart_first_Datareceived=true;//Start Valid frame
				}
				
			if(usart_first_Datareceived==false)
			{
        		USART_rxIndex=0;
			}
			else
			{
				USART_rxIndex++;
			}
			
			if(USART_rxIndex>=15)
			{
			USART_rxIndex=0;
			}
						
			if(USART_rxIndex == (demoRingBuffer[0]>>4)  )
			{	
								
				usart_Receive_Complete=true;
				USART_rxIndex=0;
			}
			
			

				  
    }
		
	
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}


/*!
 * @brief Keeps track of time
 */
////void SysTick_Handler(void)
//{
//	// count milliseconds
//	gTimCnt++;

//}

/*!
 * @brief Main function
 */

bool CAN_enable_flag;

int main(void)
{
  
 
    /* Board pin, clock, debug console init */
			
    CLOCK_EnableClock(kCLOCK_Gpio0);
    CLOCK_EnableClock(kCLOCK_Gpio1);
    CLOCK_EnableClock(kCLOCK_Gpio2);
    CLOCK_EnableClock(kCLOCK_Gpio3);

	/* attach 12 MHz clock to FLEXCOMM0 (debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);
		
		BOARD_Init();
    /* print a note to terminal */
  //  PRINTF("\r\n CAN-FD \n");
   
    /* Enable SysTick Timer */
    SysTick_Config(SystemCoreClock / TICKRATE_HZ);	
 		
  	xTaskCreate(vTouchTask,"Touch Task",TOUCHTASK_STACKSIZE,NULL,TOUCHTASK_PRIORITY,&xTouchTaskHandle);
	  xTaskCreate(vLcdTask,"LCD Task",LCDTASK_STACKSIZE,NULL,LCDTASK_PRIORITY,&xLcdTaskHandle);
	  vTaskStartScheduler();
		
	while(1)
	{
		;
	}
	

}

static void vTouchTask(void *pvParameters)
{
	
	
CAN_Type *base_Channel;
	
	//创建消息队列
    //Key_Queue=xQueueCreate(KEYMSG_Q_NUM,sizeof(uint8_t));        //创建消息Key_Queue
    //Message_Queue=xQueueCreate(MESSAGE_Q_NUM,USART_REC_LEN); //创建消息Message_Queue,队列项长度是串口接收缓冲区长度
	for(;;)
	{

		/*confirm the logic running*/			
		vControl_Loop_Run_indicator(2);// LED3 used as loop run indication
		
		//KeepSendTimeCnt++;
		vTask_UsartReceive_Detection();
		if(G_BLE_CAN_Init==true)
		{
		base_Channel=Get_CAN_Channel();
			//obd_Service_KeepAlive(base_Channel);
		}
		
	
	if(0)
		{
			KeepAlive_Peroid_2s_Count++;
			if(KeepAlive_Peroid_2s_Count>=KeepAlive_Peroid_Cnt_2s)
			{
			
				KeepAlive_Peroid_2s_Count=0;
				if(CAN_enable_flag==false)
				{
					CAN_enable_flag=true;
				//	CAN_SetTimingConfig(CAN0, &timingConfig_500K_5M);
			//	CAN_Enable(CAN1, CAN_enable_flag);
				}
				else
				{
				
				//CAN_enable_flag=false;
				}
				obd_Service_KeepAlive(CAN0);
			LED1_TOGGLE();
			}
	
		}
		
			CAN0_ResH_TOGGLE();  
		
		vTaskDelay(TOUCH_DELAY);
	}
}

static void vLcdTask(void *pvParameters)
{

	can_frame_t Rx_frame_Mask_temp;
	bool send_comp;
	
	//can_frame_t Rx_frame_Mask[3]={0,0,0};
	
	for(;;)
	{

//			CeOBD_Receive_NoCmd,                /* 00 */
//   CeOBD_Receive_Config_init,           /* 01 */      
//   CeOBD_Receive_Sending_Cmd,            /* 02 */
//   CeOBD_Receive_Sending_Cmd_Multiframe,             /* 03 */
//   CeOBD_Receive_Sending_Cmd_Multiframe_other,             /* 04 */
//   CeOBD_Receive_Complete,           /* 05 mutil frame*/ 
		
		switch(BLE_Receive_Command)
		{
			case CeOBD_Receive_Config_init:
				
			break;
			
			case CeOBD_Receive_Sending_Cmd_Multiframe:
			case CeOBD_Receive_Sending_Cmd_Multiframe_other:
				send_comp=obd_can_TxMSG_Multiframe(BLE_Receive_Command,Multiframe_FireWall_Cmd,FrameIndex);
			if(send_comp==true)
			{
				BLE_Receive_Command=CeOBD_Receive_NoCmd;
				
			}
			break;
			
			default:
			break;
		}

	

    
				
			//uint8_t Fifo_Rxed_index;
//			for(Fifo_Rxed_index=0;Fifo_Rxed_index<8;Fifo_Rxed_index++)
//			{
//			if (CAN_ReadRxFifo(CAN0,Fifo_Rxed_index, &Rx_frame_Mask_temp) == kStatus_Success)
//				{

//						PRINTF("Rx FIFO_%d: Received message 0x%3.3X\r\n",Fifo_Rxed_index,Rx_frame_Mask_temp.id);
//			
//				}
//			}
			if (CAN_ReadRxFifo(CAN0,0, &Rx_frame_Mask_temp) == kStatus_Success)
				{
					uint8_t Fifo_RX_Index;
					Fifo_RX_Index=(CAN0->RXF0S&CAN_RXF0S_F0PI_MASK)>>CAN_RXF0S_F0PI_SHIFT;
					//PRINTF("Rx FIFO_0: Received message 0x%3.3X\r\n", Rx_frame_Mask_temp.id);
					//PRINTF("Rx FIFO_index:  0x%3.3X\r\n", Fifo_RX_Index);
					
			
				}
				
				if (CAN_ReadRxFifo(CAN0,1, &Rx_frame_Mask_temp) == kStatus_Success)
				{

						//PRINTF("Rx FIFO_1: Received message 0x%3.3X\r\n", Rx_frame_Mask_temp.id);
			
				}
				
				if (CAN_ReadRxFifo(CAN0,2, &Rx_frame_Mask_temp) == kStatus_Success)
				{

						//PRINTF("Rx FIFO_2: Received message 0x%3.3X\r\n", Rx_frame_Mask_temp.id);
			
				}
	/*mask with 0x4C9*/
		//for(ReceiveIndex=0;ReceiveIndex<4;ReceiveIndex++)
		//{
			//Rxmsg_TransOilTem.id=0x4C9;
			
			//Rxmsg_TransOilTem.id = BLE_Receive_Service_ID_List[ReceiveIndex+1];
			
			if (CAN_ReadRxMb(CAN0,0, &Rx_frame_Mask_temp) == kStatus_Success)
				{
					for(i=0;i<8;i++)
					{
						Usart_Received_Feedback_1[6+i]=Rx_frame_Mask_temp.dataByte[i];
					}
					//GPIO_TogglePinsOutput(GPIO, BOARD_LED1_GPIO_PORT, 1u << BOARD_LED1_GPIO_PIN);
					
					Usart_Received_Feedback_1[2]=ReceiveID_Setting[0];
					Usart_Received_Feedback_1[3]=ReceiveID_Setting[1];
					Usart_Received_Feedback_1[4]=Rx_frame_Mask_temp.id>>8;
					Usart_Received_Feedback_1[5]=Rx_frame_Mask_temp.id&0xFF;
					
					if(USART_Sending_Block_Cnt>=5)
					{
					USART_WriteBlocking(DEMO_USART,Usart_Received_Feedback_1,14);
						USART_Sending_Block_Cnt=0;
					}
//PRINTF("Rx buf 0: Received message 0x%3.3X\r", Rx_frame_Mask_temp.id);
//					if(Rx_frame_Mask_temp.id==0x4C9)
//					{GPIO_TogglePinsOutput(GPIO, BOARD_LED2_GPIO_PORT, 1u << BOARD_LED2_GPIO_PIN);}
//					
//						if(Rx_frame_Mask_temp.id==0x4C8)
//					{GPIO_TogglePinsOutput(GPIO, BOARD_LED1_GPIO_PORT, 1u << BOARD_LED1_GPIO_PIN);}
//					
//				if(Rx_frame_Mask_temp.id==0x4C7)
//					{GPIO_TogglePinsOutput(GPIO, BOARD_LED3_GPIO_PORT, 1u << BOARD_LED3_GPIO_PIN);}
//				
//				if(Rx_frame_Mask_temp.id==0x4C6)
//					{GPIO_TogglePinsOutput(GPIO, BOARD_LED3_GPIO_PORT, 1u << BOARD_LED3_GPIO_PIN);}
				}
				
				
				if (CAN_ReadRxMb(CAN0,1, &Rx_frame_Mask_temp) == kStatus_Success)
				{
					for(i=0;i<8;i++)
					{
						Usart_Received_Feedback_2[6+i]=Rx_frame_Mask_temp.dataByte[i];
					}
					
					GPIO_TogglePinsOutput(GPIO, BOARD_LED1_GPIO_PORT, 1u << BOARD_LED1_GPIO_PIN);
					Usart_Received_Feedback_2[2]=ReceiveID_Setting[0];
					Usart_Received_Feedback_2[3]=ReceiveID_Setting[1];
					Usart_Received_Feedback_2[4]=Rx_frame_Mask_temp.id>>8;
					Usart_Received_Feedback_2[5]=Rx_frame_Mask_temp.id&0xFF;
						//PRINTF("Rx buf 1: Received message 0x%3.3X\r\n", Rx_frame_Mask_temp.id);
					if(USART_Sending_Block_Cnt>=5)
					{
					USART_WriteBlocking(DEMO_USART,Usart_Received_Feedback_2,14);
						USART_Sending_Block_Cnt=0;
					}
				}
				
				if (CAN_ReadRxMb(CAN0,2, &Rx_frame_Mask_temp) == kStatus_Success)
				{
					for(i=0;i<8;i++)
					{
						Usart_Received_Feedback_3[6+i]=Rx_frame_Mask_temp.dataByte[i];
					}
					GPIO_TogglePinsOutput(GPIO, BOARD_LED3_GPIO_PORT, 1u << BOARD_LED3_GPIO_PIN);
					Usart_Received_Feedback_3[2]=ReceiveID_Setting[0];
					Usart_Received_Feedback_3[3]=ReceiveID_Setting[1];
					Usart_Received_Feedback_3[4]=Rx_frame_Mask_temp.id>>8;
					Usart_Received_Feedback_3[5]=Rx_frame_Mask_temp.id&0xFF;
						//PRINTF("Rx buf 2: Received message 0x%3.3X\r\n", Rx_frame_Mask_temp.id);
					if(USART_Sending_Block_Cnt>=5)
					{
					USART_WriteBlocking(DEMO_USART,Usart_Received_Feedback_3,14);
						USART_Sending_Block_Cnt=0;
					}
				}


				if(USART_Sending_Block_Cnt>=10)
				{
				USART_Sending_Block_Cnt=10;
				}
				else
				{
					USART_Sending_Block_Cnt++;
				}
	
	//vBLE_Command_Mode_Action(BLE_Command_Mode);

	//LED2_TOGGLE();
	vTaskDelay(LCD_DELAY);
	
	}
	
}


void vTask_UsartReceive_Detection()
{


	if(usart_first_Datareceived==true&&Rx_Msg_Loop_Cnt<=10)
	{
		Rx_Msg_Loop_Cnt++;
	}
	else
	{
		/*Receive complete, record the data to array*/
		if(usart_Receive_Complete==true)
		{
			vTask_UsartReceive_UnPack();
			LED1_TOGGLE();
			for(i=0;i<total_index;i++)
			{
				demoRingBuffer_Total[i]=0;
			}
			usart_first_Datareceived=false;
			usart_Receive_Complete=false;
		}
		Rx_Msg_Loop_Cnt=0;
		USART_rxIndex=0;
		total_index=0;
	}

	
	if(VeUSART_Receive_State== CeUSART_Receive_Start)
	{

	}



}


void vTask_UsartReceive_UnPack()
{
	
	uint16_t Receive_ID1=0,Receive_ID2=0,Receive_ID3=0;
	TeCAN_Config BLE_Config_CAN_init;
	TeCANFD_Config BLE_Config_CANFD_init;
	
	uint8_t CAN_Config_Channel;
	
	/*0xEA config the init data*/
	if(demoRingBuffer_Total[0]==0xEA)
	{
		for(i=0;i<14;i++)
		{
			Usart_Config_Init[i]=demoRingBuffer_Total[i];
		}
		
		BLE_Config_CAN.TRes_En=Usart_Config_Init[1]&0x01;
		
		BLE_Config_CAN_init=(TeCAN_Config)(Usart_Config_Init[1]>>6);
		BLE_Config_CANFD_init= (TeCANFD_Config)((Usart_Config_Init[1]>>4)&0x03);
		CAN_Config_Channel = Usart_Config_Init[1]>>1&0x07;\

		Receive_ID1 = (Usart_Config_Init[4]<<8 ) + Usart_Config_Init[5];
		Receive_ID2 = (Usart_Config_Init[8]<<8 ) + Usart_Config_Init[9];
		Receive_ID3 = (Usart_Config_Init[12]<<8 ) + Usart_Config_Init[13];
		
		BLE_Config_CAN.Classis_CAN=BLE_Config_CAN_init;
		BLE_Config_CAN.FD_CAN = BLE_Config_CANFD_init;
		BLE_Config_CAN.channel = CAN_Config_Channel;
		BLE_Config_CAN.ID1=Receive_ID1;
		BLE_Config_CAN.ID2=Receive_ID2;
		BLE_Config_CAN.ID3=Receive_ID3;

		CAN_TerminalResitor_Enable(CAN_Config_Channel, BLE_Config_CAN.TRes_En);
		VeUSART_Receive_State=CeUSART_Receive_Complete;
		BLE_Receive_Command = CeOBD_Receive_Config_init;
		
	//CAN_Init(CAN0, &reconfig, SystemCoreClock);
		
		BOARD_ReInitCAN(BLE_Config_CAN);
		G_BLE_CAN_Init=true;
		
	}

	/*0xE6 config the multiframe data*/
	if(demoRingBuffer_Total[0]==0xE6)
	{
		for(i=0;i<total_index;i++)
		{
			FrameIndex=i/14; 
			FrameIndex_rest = i%14;
			Multiframe_FireWall_Cmd[FrameIndex][FrameIndex_rest]=demoRingBuffer_Total[i];	
		}
		VeUSART_Receive_State=CeUSART_Receive_Complete;
		BLE_Receive_Command = CeOBD_Receive_Sending_Cmd_Multiframe;
	}

	/*0xE1 config other multiframe data*/
	if(demoRingBuffer_Total[0]==0xE1)
	{
		for(i=0;i<total_index;i++)
		{
			FrameIndex=i/14; 
			FrameIndex_rest = i%14;
			Multiframe_FireWall_Cmd[FrameIndex][FrameIndex_rest]=demoRingBuffer_Total[i];	
		}
		VeUSART_Receive_State=CeUSART_Receive_Complete;
		
		BLE_Receive_Command = CeOBD_Receive_Sending_Cmd_Multiframe_other;
	}

}


