/* 
 * File:   ecan.c   
 * Author: dracula
 * Comments:
 * Revision history: EDITION 0.1 
 */

/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

/* Device header file */
#if defined(__XC16__)
    #include <xc.h>
#elif defined(__C30__)
    #if defined(__dsPIC33E__)
    	#include <p33Exxxx.h>
    #elif defined(__dsPIC33F__)
    	#include <p33Fxxxx.h>
    #endif
#endif

#include "ecan.h"

ECAN1MSGBUF ecan1MsgBuf __attribute__((space(dma)));
mID canTxMessage[5];
mID canRxMessage[6];

void ecanRtrRespond(mID *message)
{
	unsigned long word0=0;
	unsigned long word1=0;
	unsigned long word2=0;
	
	/*
	Message Format: 
	Word0 : 0bUUUx xxxx xxxx xxxx
			     |____________|||
 					SID10:0   SRR IDE(bit 0)     
	Word1 : 0bUUUU xxxx xxxx xxxx
			   	   |____________|
						EID17:6
	Word2 : 0bxxxx xxx0 UUU0 xxxx
			  |_____||	     |__|
			  EID5:0 RTR   	  DLC
	
	Remote Transmission Request Bit for standard frames 
	SRR->	"0"	 Normal Message 
			"1"  Message will request remote transmission
	Substitute Remote Request Bit for extended frames 
	SRR->	should always be set to "1" as per CAN specification
	
	Extended  Identifier Bit			
	IDE-> 	"0"  Message will transmit standard identifier
	   		"1"  Message will transmit extended identifier
	
	Remote Transmission Request Bit for extended frames 
	RTR-> 	"0"  Message transmitted is a normal message
			"1"  Message transmitted is a remote message
	Don't care for standard frames 
	*/
		
	/* check to see if the message has an extended ID */
	if(message->frame_type==CAN_FRAME_EXT)
	{
		/* get the extended message id EID28..18*/		
		word0=(message->id & 0x1FFC0000) >> 16;			
		/* set the SRR and IDE bit */
		word0=word0+0x0003;
		/* the the value of EID17..6 */
		word1=(message->id & 0x0003FFC0) >> 6;
		/* get the value of EID5..0 for word 2 */
		word2=(message->id & 0x0000003F) << 10;			
	}	
	else
	{
		/* get the SID */
		word0=((message->id & 0x000007FF) << 2);	
	}
	/* check to see if the message is an RTR message */
	if(message->message_type==CAN_MSG_RTR)
	{		
		if(message->frame_type==CAN_FRAME_EXT)
			word2=word2 | 0x0200;
		else
			word0=word0 | 0x0002;	
								
		ecan1MsgBuf[message->buffer][0]=word0;
		ecan1MsgBuf[message->buffer][1]=word1;
		ecan1MsgBuf[message->buffer][2]=word2;
	}
	else
	{
		word2=word2+(message->data_length & 0x0F);
		ecan1MsgBuf[message->buffer][0]=word0;
		ecan1MsgBuf[message->buffer][1]=word1;
		ecan1MsgBuf[message->buffer][2]=word2;
		/* fill the data */
		ecan1MsgBuf[message->buffer][3]=((message->data[1] << 8) + message->data[0]);
		ecan1MsgBuf[message->buffer][4]=((message->data[3] << 8) + message->data[2]);
		ecan1MsgBuf[message->buffer][5]=((message->data[5] << 8) + message->data[4]);
		ecan1MsgBuf[message->buffer][6]=((message->data[7] << 8) + message->data[6]);
	}
}

void sendECAN(mID *message)
{
	unsigned long word0=0;
	unsigned long word1=0;
	unsigned long word2=0;
	
	/*
	Message Format: 
	Word0 : 0bUUUx xxxx xxxx xxxx
			     |____________|||
 					SID10:0   SRR IDE(bit 0)     
	Word1 : 0bUUUU xxxx xxxx xxxx
			   	   |____________|
						EID17:6
	Word2 : 0bxxxx xxx0 UUU0 xxxx
			  |_____||	     |__|
			  EID5:0 RTR   	  DLC
	
	Remote Transmission Request Bit for standard frames 
	SRR->	"0"	 Normal Message 
			"1"  Message will request remote transmission
	Substitute Remote Request Bit for extended frames 
	SRR->	should always be set to "1" as per CAN specification
	
	Extended  Identifier Bit			
	IDE-> 	"0"  Message will transmit standard identifier
	   		"1"  Message will transmit extended identifier
	
	Remote Transmission Request Bit for extended frames 
	RTR-> 	"0"  Message transmitted is a normal message
			"1"  Message transmitted is a remote message
	Don't care for standard frames 
	*/
		
	/* check to see if the message has an extended ID */
	if(message->frame_type==CAN_FRAME_EXT)
	{
		/* get the extended message id EID28..18*/		
		word0=(message->id & 0x1FFC0000) >> 16;			
		/* set the SRR and IDE bit */
		word0=word0+0x0003;
		/* the the value of EID17..6 */
		word1=(message->id & 0x0003FFC0) >> 6;
		/* get the value of EID5..0 for word 2 */
		word2=(message->id & 0x0000003F) << 10;			
	}	
	else
	{
		/* get the SID */
		word0=((message->id & 0x000007FF) << 2);	
	}
	/* check to see if the message is an RTR message */
	if(message->message_type==CAN_MSG_RTR)
	{		
		if(message->frame_type==CAN_FRAME_EXT)
			word2=word2 | 0x0200;
		else
			word0=word0 | 0x0002;	
								
		ecan1MsgBuf[message->buffer][0]=word0;
		ecan1MsgBuf[message->buffer][1]=word1;
		ecan1MsgBuf[message->buffer][2]=word2;
	}
	else
	{
		word2=word2+(message->data_length & 0x0F);
		ecan1MsgBuf[message->buffer][0]=word0;
		ecan1MsgBuf[message->buffer][1]=word1;
		ecan1MsgBuf[message->buffer][2]=word2;
		/* fill the data */
		ecan1MsgBuf[message->buffer][3]=((message->data[1] << 8) + message->data[0]);
		ecan1MsgBuf[message->buffer][4]=((message->data[3] << 8) + message->data[2]);
		ecan1MsgBuf[message->buffer][5]=((message->data[5] << 8) + message->data[4]);
		ecan1MsgBuf[message->buffer][6]=((message->data[7] << 8) + message->data[6]);
	}

    /* check to see if buffer 0 is selected */
    if(message->buffer==0)
        /* set the message for transmission */
        C1TR01CONbits.TXREQ0=1;	
	/* check to see if buffer 1 is selected */
	else if(message->buffer==1)
		/* set the message for transmission */
		C1TR01CONbits.TXREQ1=1;				
	/* check to see if buffer 2 is selected */
	else if(message->buffer==2)
		/* set the message for transmission */
		C1TR23CONbits.TXREQ2=1;
    /* check to see if buffer 6 is selected */
	else if(message->buffer==6)
        /* set the message for transmission */
        C1TR67CONbits.TXREQ6=1;	
	/* check to see if buffer 7 is selected */
	else if(message->buffer==7)
		/* set the message for transmission */
		C1TR67CONbits.TXREQ7=1;
    else;
}

/******************************************************************************
*                                                                             
*    Function:			rxECAN
*    Description:       moves the message from the DMA memory to RAM
*                                                                             
*    Arguments:			*message: a pointer to the message structure in RAM 
*						that will store the message. 
*	 Author:            Jatinder Gharoo                                                      
*	                                                                 
*                                                                              
******************************************************************************/
void rxECAN(mID *message)
{
	unsigned int ide=0;
	unsigned int rtr=0;
	unsigned long id=0;
			
	/*
	Standard Message Format: 
	Word0 : 0bUUUx xxxx xxxx xxxx
			     |____________|||
 					SID10:0   SRR IDE(bit 0)     
	Word1 : 0bUUUU xxxx xxxx xxxx
			   	   |____________|
						EID17:6
	Word2 : 0bxxxx xxx0 UUU0 xxxx
			  |_____||	     |__|
			  EID5:0 RTR   	  DLC
	word3-word6: data bytes
	word7: filter hit code bits
	
	Remote Transmission Request Bit for standard frames 
	SRR->	"0"	 Normal Message 
			"1"  Message will request remote transmission
	Substitute Remote Request Bit for extended frames 
	SRR->	should always be set to "1" as per CAN specification
	
	Extended  Identifier Bit			
	IDE-> 	"0"  Message will transmit standard identifier
	   		"1"  Message will transmit extended identifier
	
	Remote Transmission Request Bit for extended frames 
	RTR-> 	"0"  Message transmitted is a normal message
			"1"  Message transmitted is a remote message
	Don't care for standard frames 
	*/
		
	/* read word 0 to see the message type */
	ide=ecan1MsgBuf[message->buffer][0] & 0x0001;			
	
	/* check to see what type of message it is */
	/* message is standard identifier */
	if(ide==0)
	{
		message->id=(ecan1MsgBuf[message->buffer][0] & 0x1FFC) >> 2;		
		message->frame_type=CAN_FRAME_STD;
		rtr=ecan1MsgBuf[message->buffer][0] & 0x0002;
	}
	/* mesage is extended identifier */
	else
	{
		id=ecan1MsgBuf[message->buffer][0] & 0x1FFC;		
		message->id=id << 16;
		id=ecan1MsgBuf[message->buffer][1] & 0x0FFF;
		message->id=message->id+(id << 6);
		id=(ecan1MsgBuf[message->buffer][2] & 0xFC00) >> 10;
		message->id=message->id+id;		
		message->frame_type=CAN_FRAME_EXT;
		rtr=ecan1MsgBuf[message->buffer][2] & 0x0200;
	}
	/* check to see what type of message it is */
	/* RTR message */
	if(rtr==1)
	{
		message->message_type=CAN_MSG_RTR;	
	}
	/* normal message */
	else
	{
		message->message_type=CAN_MSG_DATA;
		message->data[0]=(unsigned char)ecan1MsgBuf[message->buffer][3];
		message->data[1]=(unsigned char)((ecan1MsgBuf[message->buffer][3] & 0xFF00) >> 8);
		message->data[2]=(unsigned char)ecan1MsgBuf[message->buffer][4];
		message->data[3]=(unsigned char)((ecan1MsgBuf[message->buffer][4] & 0xFF00) >> 8);
		message->data[4]=(unsigned char)ecan1MsgBuf[message->buffer][5];
		message->data[5]=(unsigned char)((ecan1MsgBuf[message->buffer][5] & 0xFF00) >> 8);
		message->data[6]=(unsigned char)ecan1MsgBuf[message->buffer][6];
		message->data[7]=(unsigned char)((ecan1MsgBuf[message->buffer][6] & 0xFF00) >> 8);
		message->data_length=(unsigned char)(ecan1MsgBuf[message->buffer][2] & 0x000F);
	}
	clearRxFlags(message->buffer);	
}

/******************************************************************************
*                                                                             
*    Function:			clearRxFlags
*    Description:       clears the rxfull flag after the message is read
*                                                                             
*    Arguments:			buffer number to clear 
*	 Author:            Jatinder Gharoo                                                      
*	                                                                 
*                                                                              
******************************************************************************/
void clearRxFlags(unsigned char buffer_number)
{
    /* check to see if buffer 3 is full */
	if((C1RXFUL1bits.RXFUL3) && (buffer_number==3))
		/* clear flag */
    {C1RXFUL1bits.RXFUL3=0;}
	/* check to see if buffer 4 is full */
	else if((C1RXFUL1bits.RXFUL4) && (buffer_number==4))
		/* clear flag */
    {C1RXFUL1bits.RXFUL4=0;}				
	/* check to see if buffer 5 is full */
	else if((C1RXFUL1bits.RXFUL5) && (buffer_number==5))
		/* clear flag */
    {C1RXFUL1bits.RXFUL5=0;}
    /* check to see if buffer 8 is full */
	else if((C1RXFUL1bits.RXFUL8) && (buffer_number==8))
		/* clear flag */
    {C1RXFUL1bits.RXFUL8=0;}		
	/* check to see if buffer 9 is full */
	else if((C1RXFUL1bits.RXFUL9) && (buffer_number==9))
		/* clear flag */
    {C1RXFUL1bits.RXFUL9=0;}				
	/* check to see if buffer 10 is full */
	else if((C1RXFUL1bits.RXFUL10) && (buffer_number==10))
		/* clear flag */
	{C1RXFUL1bits.RXFUL10=0;}
    else;

}

/******************************************************************************
*                                                                             
*    Function:			ECANInit
*    Description:       Initialises the ECAN module                                                        
*                                                                             
*    Arguments:			none 
*	 Author:            Jatinder Gharoo                                                      
*	                                                                 
*                                                                              
******************************************************************************/
void ECANInit (void)
{

	/* put the module in configuration mode */
	C1CTRL1bits.REQOP=4;
	while(C1CTRL1bits.OPMODE != 4);
	/*
	Bit Time = (Sync Segment + Propagation Delay + Phase Segment 1 + Phase Segment 2)=20*TQ
	Phase Segment 1 = 8TQ
	Phase Segment 2 = 6TQ
	Propagation Delay = 5TQ
	Sync Segment = 1TQ
	CiCFG1<BRP> = ((FCAN/(2*NTQ*BITRATE))-1)
	BIT RATE OF 1Mbps
	*/	
	/* Phase Segment 1 time is 8 TQ */
	C1CFG2bits.SEG1PH=0x7;
	/* Phase Segment 2 time is set to be programmable */
	C1CFG2bits.SEG2PHTS = 0x1;
	/* Phase Segment 2 time is 6 TQ */
	C1CFG2bits.SEG2PH = 0x5;
	/* Propagation Segment time is 5 TQ */
	C1CFG2bits.PRSEG = 0x4;
	/* Bus line is sampled three times at the sample point */
	C1CFG2bits.SAM = 0x1;
    /* Synchronization Jump Width set to 4 TQ */
	C1CFG1bits.SJW = 0x3;
    /* Baud Rate Prescaler bits set to 1:1, i.e., TQ = (2*1*1)/ FCAN */
	C1CFG1bits.BRP = BRP_VAL;
//	/* 8 CAN Messages to be buffered in DMA RAM */	
//	C1FCTRLbits.DMABS=0b10;
    /* 32 CAN Messages to be buffered in DMA RAM */	
	C1FCTRLbits.DMABS=0b110;
	
	/* Filter configuration */
	/* Enable window to access the filter configuration registers */
	C1CTRL1bits.WIN=0b1;
    
	/* select acceptance mask 0 filter 0 buffer 3 */
	C1FMSKSEL1bits.F0MSK=0;
	/* configure accpetence mask 0 - match the id in filter 0 
	setup the mask to check every bit of the extended message, 
	the macro when called as CAN_FILTERMASK2REG_EID0(0xFFFF) 
	will write the register C1RXM1EID to include extended 
	message id bits EID0 to EID15 in filter comparison. 
	the macro when called as CAN_FILTERMASK2REG_EID1(0x1FFF) 
	will write the register C1RXM1SID to include extended 
	message id bits EID16 to EID28 in filter comparison. 	
	*/ 	
    C1RXM0EID=CAN_FILTERMASK2REG_EID0(0xFFFF);
	C1RXM0SID=CAN_FILTERMASK2REG_EID1(0x1FFF);
	/* configure accpetence filter 0 
	configure accpetence filter 0 - accept only XTD ID 0x12345677 
	setup the filter to accept only extended message 0x12345677, 
	the macro when called as CAN_FILTERMASK2REG_EID0(0x5677) 
	will write the register C1RXF1EID to include extended 
	message id bits EID0 to EID15 when doing filter comparison. 
	the macro when called as CAN_FILTERMASK2REG_EID1(0x1234) 
	will write the register C1RXF1SID to include extended 
	message id bits EID16 to EID28 when doing filter comparison. 	
	*/ 	
    C1RXF0EID=CAN_FILTERMASK2REG_EID0(0x5677);
	C1RXF0SID=CAN_FILTERMASK2REG_EID1(0x1234);
	/* set filter to check for extended ID and accept extended id only */
	C1RXM0SID=CAN_SETMIDE(C1RXM0SID);
	C1RXF0SID=CAN_FILTERXTD(C1RXF0SID);	
	/* acceptance filter to use buffer 3 for incoming messages */
	C1BUFPNT1bits.F0BP=0b0011;
	/* enable filter 0 */
	C1FEN1bits.FLTEN0=1;
	
	/* select acceptance mask 1 filter 1 and buffer 4 */
	C1FMSKSEL1bits.F1MSK=0b01;
	/* configure accpetence mask 1 - match id in filter 1 	
	setup the mask to check every bit of the extended message, 
	the macro when called as CAN_FILTERMASK2REG_EID0(0xFFFF) 
	will write the register C1RXM1EID to include extended 
	message id bits EID0 to EID15 in filter comparison. 
	the macro when called as CAN_FILTERMASK2REG_EID1(0x1FFF) 
	will write the register C1RXM1SID to include extended 
	message id bits EID16 to EID28 in filter comparison. 	
	*/ 			
	C1RXM1EID=CAN_FILTERMASK2REG_EID0(0xFFFF);
	C1RXM1SID=CAN_FILTERMASK2REG_EID1(0x1FFF);
	/* configure acceptance filter 1 
	configure accpetence filter 1 - accept only XTD ID 0x12345678 
	setup the filter to accept only extended message 0x12345678, 
	the macro when called as CAN_FILTERMASK2REG_EID0(0x5678) 
	will write the register C1RXF1EID to include extended 
	message id bits EID0 to EID15 when doing filter comparison. 
	the macro when called as CAN_FILTERMASK2REG_EID1(0x1234) 
	will write the register C1RXF1SID to include extended 
	message id bits EID16 to EID28 when doing filter comparison. 	
	*/ 
	C1RXF1EID=CAN_FILTERMASK2REG_EID0(0x5678);
	C1RXF1SID=CAN_FILTERMASK2REG_EID1(0x1234);		
	/* filter to check for extended ID only */
	C1RXM1SID=CAN_SETMIDE(C1RXM1SID);
	C1RXF1SID=CAN_FILTERXTD(C1RXF1SID);
	/* acceptance filter to use buffer 4 for incoming messages */
	C1BUFPNT1bits.F1BP=0b0100;
	/* enable filter 1 */
	C1FEN1bits.FLTEN1=1;
	
	/* select acceptance mask 2 filter 2 and buffer 5 */
	C1FMSKSEL1bits.F2MSK=0b10;	
    /* configure accpetence mask 2 - match id in filter 2 	
	setup the mask to check every bit of the extended message, 
	the macro when called as CAN_FILTERMASK2REG_EID0(0xFFFF) 
	will write the register C1RXM1EID to include extended 
	message id bits EID0 to EID15 in filter comparison. 
	the macro when called as CAN_FILTERMASK2REG_EID1(0x1FFF) 
	will write the register C1RXM1SID to include extended 
	message id bits EID16 to EID28 in filter comparison. 	
	*/ 			
	C1RXM2EID=CAN_FILTERMASK2REG_EID0(0xFFFF);
	C1RXM2SID=CAN_FILTERMASK2REG_EID1(0x1FFF);
	/* configure acceptance filter 2 
	configure accpetence filter 2 - accept only XTD ID 0x12345679 
	setup the filter to accept only extended message 0x12345679, 
	the macro when called as CAN_FILTERMASK2REG_EID0(0x5679) 
	will write the register C1RXF1EID to include extended 
	message id bits EID0 to EID15 when doing filter comparison. 
	the macro when called as CAN_FILTERMASK2REG_EID1(0x1234) 
	will write the register C1RXF1SID to include extended 
	message id bits EID16 to EID28 when doing filter comparison. 	
	*/ 
	C1RXF2EID=CAN_FILTERMASK2REG_EID0(0x5679);
	C1RXF2SID=CAN_FILTERMASK2REG_EID1(0x1234);		
	/* filter to check for extended ID only */
    C1RXM2SID=CAN_SETMIDE(C1RXM2SID);	
	C1RXF2SID=CAN_FILTERXTD(C1RXF2SID);
	/* acceptance filter to use buffer 5 for incoming messages */
	C1BUFPNT1bits.F2BP=0b0101;
	/* enable filter 2 */
	C1FEN1bits.FLTEN2=1;
	
    /* select acceptance mask 0 filter 3 buffer 8 */
	C1FMSKSEL1bits.F3MSK=0;
	/* configure accpetence mask 0 - match the id in filter 3 */ 	
    C1RXM0EID=CAN_FILTERMASK2REG_EID0(0xFFFF);
	C1RXM0SID=CAN_FILTERMASK2REG_EID1(0x1FFF);
	/* configure accpetence filter 3 - accept only XTD ID 0x12345657 */ 	
    C1RXF3EID=CAN_FILTERMASK2REG_EID0(0x5657);
	C1RXF3SID=CAN_FILTERMASK2REG_EID1(0x1234);
	/* set filter to check for extended ID and accept extended id only */
	C1RXM0SID=CAN_SETMIDE(C1RXM0SID);
	C1RXF3SID=CAN_FILTERXTD(C1RXF3SID);	
	/* acceptance filter to use buffer 8 for incoming messages */
	C1BUFPNT1bits.F3BP=0b1000;
	/* enable filter 3 */
	C1FEN1bits.FLTEN3=1;
    
    /* select acceptance mask 1 filter 4 buffer 9 */
	C1FMSKSEL1bits.F4MSK=1;
	/* configure accpetence mask 1 - match the id in filter 4 */ 	
    C1RXM1EID=CAN_FILTERMASK2REG_EID0(0xFFFF);
	C1RXM1SID=CAN_FILTERMASK2REG_EID1(0x1FFF);
	/* configure accpetence filter 4 - accept only XTD ID 0x12345658 */ 	
    C1RXF4EID=CAN_FILTERMASK2REG_EID0(0x5658);
	C1RXF4SID=CAN_FILTERMASK2REG_EID1(0x1234);
	/* set filter to check for extended ID and accept extended id only */
	C1RXM1SID=CAN_SETMIDE(C1RXM1SID);
	C1RXF4SID=CAN_FILTERXTD(C1RXF4SID);	
	/* acceptance filter to use buffer 9 for incoming messages */
	C1BUFPNT2bits.F4BP=0b1001;
	/* enable filter 4 */
	C1FEN1bits.FLTEN4=1;

    /* select acceptance mask 2 filter 5 buffer 10 */
	C1FMSKSEL1bits.F5MSK=2;
	/* configure accpetence mask 2 - match the id in filter 5 */ 	
    C1RXM2EID=CAN_FILTERMASK2REG_EID0(0xFFFF);
	C1RXM2SID=CAN_FILTERMASK2REG_EID1(0x1FFF);
	/* configure accpetence filter 5 - accept only XTD ID 0x12345659 */ 	
    C1RXF5EID=CAN_FILTERMASK2REG_EID0(0x5659);
	C1RXF5SID=CAN_FILTERMASK2REG_EID1(0x1234);
	/* set filter to check for extended ID and accept extended id only */
	C1RXM2SID=CAN_SETMIDE(C1RXM2SID);
	C1RXF5SID=CAN_FILTERXTD(C1RXF5SID);	
	/* acceptance filter to use buffer 10 for incoming messages */
	C1BUFPNT2bits.F5BP=0b1010;
	/* enable filter 5 */
	C1FEN1bits.FLTEN5=1;
    
	/* clear window bit to access ECAN control registers */
	C1CTRL1bits.WIN=0;
		
	/* put the module in normal mode */
	C1CTRL1bits.REQOP=0;
	while(C1CTRL1bits.OPMODE != 0);	
	
	/* clear the buffer and overflow flags */
	C1RXFUL1=C1RXFUL2=C1RXOVF1=C1RXOVF2=0x0000;
	/* ECAN1, Buffer 0 is a Transmit Buffer */
	C1TR01CONbits.TXEN0=1;			
	/* ECAN1, Buffer 1 is a Transmit Buffer */
	C1TR01CONbits.TXEN1=1;	
	/* ECAN1, Buffer 2 is a Transmit Buffer */
	C1TR23CONbits.TXEN2=1;	
    
	/* ECAN1, Buffer 3 is a Receive Buffer */
	C1TR23CONbits.TXEN3=0;
    /* ECAN1, Buffer 4 is a Receive Buffer */
	C1TR45CONbits.TXEN4=0;
    /* ECAN1, Buffer 5 is a Receive Buffer */
	C1TR45CONbits.TXEN5=0;	
    
    /* ECAN1, Buffer 6 is a Transmit Buffer */
	C1TR67CONbits.TXEN6=1;			
	/* ECAN1, Buffer 7 is a Transmit Buffer */
	C1TR67CONbits.TXEN7=1;	
    
	/* Message Buffer 0 Priority Level */
	C1TR01CONbits.TX0PRI=0b11; 	
    /* Message Buffer 1 Priority Level */
	C1TR01CONbits.TX1PRI=0b11;
    /* Message Buffer 2 Priority Level */
	C1TR23CONbits.TX2PRI=0b11;	
    
    /* Message Buffer 0 Priority Level */
	C1TR67CONbits.TX6PRI=0b11; 	
    /* Message Buffer 1 Priority Level */
	C1TR67CONbits.TX7PRI=0b11;
		
	/* configure the device to interrupt on the receive buffer full flag */
	/* clear the buffer full flags */
	C1INTFbits.RBIF=0;
    /*ECAN1 Interrupt Priority*/
    IPC8bits.C1IP = 0b101;
    /*ECAN1 Receive Interrupt Priority*/
    IPC8bits.C1RXIP = 0b101;
    /*ECAN1 Transmit Interrupt Priority*/
    IPC17bits.C1TXIP = 0b101;
    /* Enable ECAN1 Interrupt */     	
	IEC2bits.C1IE=1;	
	/* enable Transmit interrupt */
	C1INTEbits.TBIE=1;
	/* Enable Receive interrupt */
	C1INTEbits.RBIE=1;
}

/******************************************************************************
*                                                                             
*    Function:			DMAInit
*    Description:       Initialises the DMA to be used with ECAN module                                                        
*                       Channel 0 of the DMA is configured to Tx ECAN messages
* 						of ECAN module 1. 
*						Channel 2 is uconfigured to Rx ECAN messages of module 1.                                                      
*    Arguments:			
*	 Author:            Jatinder Gharoo                                                      
*	                                                                 
*                                                                              
******************************************************************************/
void DMAInit(void)
{
//	/* initialise the DMA channel 0 for ECAN Tx */
//	/* clear the collission flags */
//	DMACS0=0;	
//    /* setup channel 0 for peripheral indirect addressing mode 
//    normal operation, word operation and select as Tx to peripheral */
//    DMA0CON=0x2020; 
//    /* setup the address of the peripheral ECAN1 (C1TXD) */ 
//	DMA0PAD=0x0442;
    
    /* Data Transfer Size: Word Transfer Mode */
    DMA0CONbits.SIZE = 0x0;
    /* Data Transfer Direction: DMA RAM to Peripheral */
    DMA0CONbits.DIR = 0x1;
    /* DMA Addressing Mode: Peripheral Indirect Addressing mode */
    DMA0CONbits.AMODE = 0x2;
    /* Operating Mode: Continuous, Ping-Pong modes disabled */
    DMA0CONbits.MODE = 0x0;
    /* automatic DMA Tx initiation by DMA request */
	DMA0REQ=0x0046;	
	/* Set the data block transfer size of 8 */
 	DMA0CNT=7;
    /* Peripheral Address: ECAN1 Transmit Register */
    DMA0PAD = &C1TXD;
	/* DPSRAM atart adddress offset value */ 
//	DMA0STA=__builtin_dmaoffset(ecan1msgBuf);
    DMA0STA=__builtin_dmaoffset(&ecan1MsgBuf);	
	/* enable the channel */
	DMA0CONbits.CHEN=1;
	
//	/* initialise the DMA channel 2 for ECAN Rx */
//	/* clear the collission flags */
//	DMACS0=0;
//    /* setup channel 2 for peripheral indirect addressing mode 
//    normal operation, word operation and select as Rx to peripheral */
//    DMA2CON=0x0020;
//    /* setup the address of the peripheral ECAN1 (C1RXD) */ 
//	DMA2PAD=0x0440;	
    /* Data Transfer Size: Word Transfer Mode */
    DMA2CONbits.SIZE = 0x0;
    /* Data Transfer Direction: Peripheral to DMA RAM */
    DMA2CONbits.DIR = 0x0;
    /* DMA Addressing Mode: Peripheral Indirect Addressing mode */
    DMA2CONbits.AMODE = 0x2;
    /* Operating Mode: Continuous, Ping-Pong modes disabled */
    DMA2CONbits.MODE = 0x0;
    /* Assign ECAN1 Receive event for DMA Channel 2 */
    /* automatic DMA Rx initiation by DMA request */
	DMA2REQ=0x0022;
 	/* Set the data block transfer size of 8 */
 	DMA2CNT=7;
    /* Peripheral Address: ECAN1 Receive Register */
    DMA2PAD = &C1RXD;	
	/* DPSRAM atart adddress offset value */ 
	DMA2STA=__builtin_dmaoffset(&ecan1MsgBuf);	
	/* enable the channel */
	DMA2CONbits.CHEN=1;
}	 
