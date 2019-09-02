/******************** COPYRIGHT*************************************************
* File Name          : can.h
* Author             : 
* Version            : 
* Date               : 04/5/2009
* Description        : 
********************************************************************************
* 


*******************************************************************************/
#ifndef CAN_H_
#define CAN_H_

#define CANDEBUG   1

#define CANUSELOOP 0

#define CANSENDTIMEOUT (200) 

#define SUCCESS         (0)
#define FAIL           (!0)

#define CAN_MAX_CHAR_IN_MESSAGE (8)

typedef struct {
	// identifier CAN_xxxID
	unsigned char  extended_identifier; 
	// either extended (the 18 LSB) or standard (the 11 LSB)
	unsigned int identifier; 
	// data length:	 29bit;
	unsigned char  dlc;
	unsigned char  dta[CAN_MAX_CHAR_IN_MESSAGE];
	
	// used for receive only:
	// Received Remote Transfer Bit 
	//  (0=no... 1=remote transfer request received)
	unsigned char  rtr;  
	// Acceptence Filter that enabled the reception
	unsigned char  filhit;
} CAN_MESSAGE;


extern unsigned char can_init(void);
extern unsigned char can_check_receive(void);
extern void init_message_struct(CAN_MESSAGE* msg);
extern unsigned char can_send_message(const CAN_MESSAGE* msg);
extern unsigned char can_read_message(CAN_MESSAGE *msg);
extern unsigned char can_check_receive(void);
extern unsigned char can_check_error(void);
extern unsigned char can_test_transmit(const unsigned char ext);
#endif
