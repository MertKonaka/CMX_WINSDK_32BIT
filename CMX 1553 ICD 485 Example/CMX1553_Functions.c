#include "Windows.h"
#include <rs232.h>
#include <ansi_c.h>
#include "Support.h"
#include "GlobalVariables.h"

#define COMMAND_LEN				80

int SerialError;
unsigned int crc_result;
unsigned char CrcArray[4];


unsigned int Cmx1553OpenBoard(unsigned int PortNumber)
{
	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned char RX_MESSAGE[COMMAND_LEN] = {0};
	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_OPEN_BOARD;
	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);
	SerialError = OpenComConfig (PortNumber, NULL, 115200, 0, 8, 1, 4096, 512);	
	if (SerialError >=0)
	{
		SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
		{
			printf("Command Send Error\n");
			CloseCom(PortNumber);
		}
		else
		{
			SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
			if (SerialError < 0)
				printf("Command Read Error\n");
			else
			{
				for (uint8_t i = 0; i < COMMAND_LEN; i++)
					printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);
			}
		}
	}
	else
		printf("Com port Open Error\n");
	return PortNumber;
}

unsigned int Cmx1553InitBoard (unsigned int PortNumber){
	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned char RX_MESSAGE[COMMAND_LEN] = {0};
	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_INIT_BOARD;
	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);
	SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
	if (SerialError < 0)
		printf("Command Send Error\n");
	else
	{
		SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
			printf("Command Read Error\n");
		else
		{
			for (uint8_t i = 0; i < COMMAND_LEN; i++)
				printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);
		}
	}

	return 1;
}
unsigned int Cmx1553ResetBoard (unsigned int PortNumber){
	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned char RX_MESSAGE[COMMAND_LEN] = {0};
	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_RESET_BOARD;
	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);
	SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
			printf("Command Send Error\n");
		else
		{
			SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
			if (SerialError < 0)
				printf("Command Read Error\n");
			else
			{
				for (uint8_t i = 0; i < COMMAND_LEN; i++)
					printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);
			}
		}
	
	return 1;
}
unsigned int Cmx1553BoardSetMode (unsigned int PortNumber,unsigned int bc_mode,unsigned int rt1_mode,unsigned int rt2_mode,unsigned int mt_mode ){
	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned char RX_MESSAGE[COMMAND_LEN] = {0};
	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_BOARD_SET_1553_MODE;
	TX_MESSAGE[2] =(unsigned char)bc_mode;
	TX_MESSAGE[3] =(unsigned char)rt1_mode;
	TX_MESSAGE[4] =(unsigned char)rt2_mode;
	TX_MESSAGE[5] =(unsigned char)mt_mode;
	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);
	SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
			printf("Command Send Error\n");
		else
		{
			SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
			if (SerialError < 0)
				printf("Command Read Error\n");
			else
			{
				for (uint8_t i = 0; i < COMMAND_LEN; i++)
					printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);
			}
		}
		
	return 1;
}

unsigned int Cmx1553BoardGetMode (unsigned int PortNumber, unsigned int *Mode){
	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned char RX_MESSAGE[COMMAND_LEN] = {0};
	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_BOARD_GET_1553_MODE;
	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);
	SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
			printf("Command Send Error\n");
		else
		{
			SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
			if (SerialError < 0)
				printf("Command Read Error\n");
			else
			{
				for (uint8_t i = 0; i < COMMAND_LEN; i++)
					printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);
				*Mode = (unsigned int) RX_MESSAGE[2];
				if (*Mode & BC_MODE_MASK)
					printf("BC MODE ENABLE\n");
				if (*Mode & RT1_MODE_MASK)
					printf("RT1 MODE ENABLE\n");
				if (*Mode & RT2_MODE_MASK)
					printf("RT2 MODE ENABLE\n");
				if (*Mode & MT_MODE_MASK)
					printf("MT MODE ENABLE\n");
				
			}
		}	
	return 1;
}
unsigned int Cmx1553Reset (unsigned int PortNumber){
	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned char RX_MESSAGE[COMMAND_LEN] = {0};
	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_1553_RESET;
	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);
	SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
			printf("Command Send Error\n");
		else
		{
			SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
			if (SerialError < 0)
				printf("Command Read Error\n");
			else
			{
				for (uint8_t i = 0; i < COMMAND_LEN; i++)
					printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);
			}
		}	
	return 1;
}

unsigned int Cmx1553SharedInit (unsigned int PortNumber)
{
	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned char RX_MESSAGE[COMMAND_LEN] = {0};
	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_SHARED_INIT;
	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);
	SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
			printf("Command Send Error\n");
		else
		{
			SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
			if (SerialError < 0)
				printf("Command Read Error\n");
			else
			{
				for (uint8_t i = 0; i < COMMAND_LEN; i++)
					printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);
			}
		}
	return 1;
}

unsigned int Cmx1553BcInitMsg (unsigned int PortNumber,unsigned int message_type,unsigned int Index,unsigned int data_count,unsigned short *data,unsigned short control_word,unsigned int RT_Addr,unsigned int RT_subaddr,unsigned int Mcode ){
	

	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned char RX_MESSAGE[COMMAND_LEN] = {0};
	
	unsigned char packet_temp[2]={0,0};
	unsigned char packet_temp2[2]={0,0};
	serialize_uint16B(packet_temp2,&control_word);
	
	if(message_type==0||message_type==2||message_type==4){
		for(int i =0;i<32;i+=1){
			serialize_uint16B(packet_temp,&data[i]);
			memcpy(TX_MESSAGE+5+(i*2),packet_temp,2);
		}
	}
	
	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_BC_INIT_MSG;
	TX_MESSAGE[2] =(unsigned char)message_type;
	TX_MESSAGE[3] =(unsigned char)Index;
	TX_MESSAGE[4] =(unsigned char)data_count;
	TX_MESSAGE[69] =(unsigned char)packet_temp2[0];
	TX_MESSAGE[70] =(unsigned char)packet_temp2[1];
	TX_MESSAGE[71] =(unsigned char)RT_Addr;
	TX_MESSAGE[72] =(unsigned char)RT_subaddr;
	TX_MESSAGE[73] =(unsigned char)Mcode;
	
	
	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);
	SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
			printf("Command Send Error\n");
		else
		{
			SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
			if (SerialError < 0)
				printf("Command Read Error\n");
			else
			{
				for (uint8_t i = 0; i < COMMAND_LEN; i++)
					printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);
			}
		}
		
	return 1;

}

unsigned int Cmx1553BcUpdateMsg (unsigned int PortNumber,unsigned int message_type,unsigned int Index,unsigned int data_count,unsigned short *data,unsigned short control_word,unsigned int RT_Addr,unsigned int RT_subaddr,unsigned int Mcode ){

	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned char RX_MESSAGE[COMMAND_LEN] = {0};
	
	unsigned char packet_temp[2]={0,0};
	unsigned char packet_temp2[2]={0,0};
	serialize_uint16B(packet_temp2,&control_word);
	
	if(message_type==0||message_type==2||message_type==4){
		for(int i =0;i<32;i+=1){
			serialize_uint16B(packet_temp,&data[i]);
			memcpy(TX_MESSAGE+5+(i*2),packet_temp,2);
		}
	}
	
	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_BC_UPDATE_MSG;
	TX_MESSAGE[2] =(unsigned char)message_type;
	TX_MESSAGE[3] =(unsigned char)Index;
	TX_MESSAGE[4] =(unsigned char)data_count;
	TX_MESSAGE[69] =(unsigned char)packet_temp2[0];
	TX_MESSAGE[70] =(unsigned char)packet_temp2[1];
	TX_MESSAGE[71] =(unsigned char)RT_Addr;
	TX_MESSAGE[72] =(unsigned char)RT_subaddr;
	TX_MESSAGE[73] =(unsigned char)Mcode;
	
	
	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);

		SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
			printf("Command Send Error\n");
		else
		{
			SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
			if (SerialError < 0)
				printf("Command Read Error\n");
			else
			{
				for (uint8_t i = 0; i < COMMAND_LEN; i++)
					printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);
			}
		}
	return 1;

}
unsigned int Cmx1553BcInitMsgList (unsigned int PortNumber, unsigned int mode,unsigned int loop, unsigned int msg_gap_time ,unsigned int period_us ){
	
	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned char RX_MESSAGE[COMMAND_LEN] = {0};
	
	unsigned char packet_temp[4]={0,0,0,0};
	unsigned char packet_temp2[4]={0,0,0,0};
	deserialize_uint32B(packet_temp,&msg_gap_time);
	deserialize_uint32B(packet_temp2,&period_us);
	

	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_BC_INIT_MSG_INST_LIST;
	TX_MESSAGE[2] =(unsigned char)mode;
	TX_MESSAGE[3] =(unsigned char)loop;
	TX_MESSAGE[4] =(unsigned char)packet_temp[0];
	TX_MESSAGE[5] =(unsigned char)packet_temp[1];
	TX_MESSAGE[6] =(unsigned char)packet_temp[2];
	TX_MESSAGE[7] =(unsigned char)packet_temp[3];
	TX_MESSAGE[8] =(unsigned char)packet_temp2[0];
	TX_MESSAGE[9] =(unsigned char)packet_temp2[1];
	TX_MESSAGE[10] =(unsigned char)packet_temp2[2];
	TX_MESSAGE[11] =(unsigned char)packet_temp2[3];
	
	
	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);
		
	SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
			printf("Command Send Error\n");
		else
		{
			SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
			if (SerialError < 0)
				printf("Command Read Error\n");
			else
			{
				for (uint8_t i = 0; i < COMMAND_LEN; i++)
					printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);
			}
		}
	return 1;
}
unsigned int Cmx1553BcSetTTAG (unsigned int PortNumber,unsigned int HiLo, unsigned int BcTTAG)
{
	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned char RX_MESSAGE[COMMAND_LEN] = {0};
	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_BC_SET_TTAG;
	TX_MESSAGE[2] =(unsigned char)HiLo;
	TX_MESSAGE[3] =(unsigned char)BcTTAG;
	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);
		
	SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
			printf("Command Send Error\n");
		else
		{
			SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
			if (SerialError < 0)
				printf("Command Read Error\n");
			else
			{
				for (uint8_t i = 0; i < COMMAND_LEN; i++)
					printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);
			}
		}
	return 1;
}
unsigned int Cmx1553BcGetMsgInProgress (unsigned int PortNumber, unsigned int *msgprogress)
{
	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned char RX_MESSAGE[COMMAND_LEN] = {0};
	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_BC_GET_PROGRESS;

	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);
		
	SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
			printf("Command Send Error\n");
			
		else
		{
			SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
			if (SerialError < 0)
				printf("Command Read Error\n");
			else
			{
				for (uint8_t i = 0; i < COMMAND_LEN; i++)
					printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);
				
				*msgprogress = (unsigned int) RX_MESSAGE[2];
				if(*msgprogress)
					printf("BC MESSAGE IS BEING PROCESSED");
				else
					printf("BC MESSAGE IS NOT BEING PROCESSED");
			}
		}
		
	return 1;
}
unsigned int Cmx1553BcGetMsgBlock(unsigned int PortNumber, unsigned short*msgblock)
{
	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned char RX_MESSAGE[COMMAND_LEN] = {0};
	
	unsigned char packet_temp[2]={0,0};

	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_BC_GET_MSG_BLOCK;

	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);
		
	SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
			printf("Command Send Error\n");
			
		else
		{
			SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
			if (SerialError < 0)
				printf("Command Read Error\n");
			else
			{
				
				for(int i=2;i<21;i+=2){
	    		packet_temp[0]=RX_MESSAGE[i];
	    		packet_temp[1]=RX_MESSAGE[i+1];
	    		msgblock[(i/2)-1]=deserialize_uint16(packet_temp);
	   			 }
				for (int i = 0; i < 10; i++) {
					printf("msgblock[%d]=%x\n", i, msgblock[i]);
				}
			}
		}
		
	return 1;
}
//unsigned int Cmx1553BcGetMsgData (unsigned int PortNumber, unsigned short*msgdata,unsigned int num_packet)
//{
//	crcInit();
//	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};

//	unsigned int num_messages_to_send = (num_packet > 20) ? 20 : num_packet;
//	unsigned int RX_MESSAGE_SIZE = 2 + (num_messages_to_send * 64) + 6;
//	unsigned char* RX_MESSAGE = (unsigned char*)malloc(RX_MESSAGE_SIZE);

//	 if (RX_MESSAGE == NULL)
//		 return;
//			
//	
//	unsigned char packet_temp[2]={0,0};
//	
//	TX_MESSAGE[0] = 0x01; 	// BoardID
//	TX_MESSAGE[1] =CM_BC_GET_MSG_DATA;
//	TX_MESSAGE[2] =(unsigned char)num_packet;
//	TX_MESSAGE[74] = 0x01;	// No Error
//	crc_result = CRCCALC(TX_MESSAGE, 76);
//	deserialize_uint32B(CrcArray,&crc_result);
//	memcpy(TX_MESSAGE+76, CrcArray, 4);
//		
//	SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
//		if (SerialError < 0)
//			printf("Command Send Error\n");
//			
//		else
//		{
//			SerialError = ComRd(PortNumber, RX_MESSAGE,RX_MESSAGE_SIZE);
//			if (SerialError < 0)
//				printf("Command Read Error\n");
//			else
//			{
//				/*for (uint8_t i = 0; i < RX_MESSAGE_SIZE; i++)
//					printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);*/
//				
//				for(int i=2;i< (num_packet * 64) + 2;i+=2){
//	    		packet_temp[0]=RX_MESSAGE[i];
//	    		packet_temp[1]=RX_MESSAGE[i+1];
//	    		msgdata[(i/2)-1]=deserialize_uint16(packet_temp);
//	    		}
//				for (int i = 0; i < (num_packet * 32) ; i++) {
//					printf("msgdata[%d]=%x\n", i, msgdata[i]);
//				}
//			}
//		}

//	free(RX_MESSAGE);
//	return 1;
//}
unsigned int Cmx1553BcGetMsgData (unsigned int PortNumber, unsigned short*msgdata,unsigned int num_packet)
{
	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned char RX_MESSAGE[COMMAND_LEN] = {0};
	
	unsigned char packet_temp[2]={0,0};

	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_BC_GET_MSG_DATA;

	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);
		
	SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
			printf("Command Send Error\n");
			
		else
		{
			SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
			if (SerialError < 0)
				printf("Command Read Error\n");
			else
			{
				
				for (int i = 2; i < 66; i += 2) 
				{
						packet_temp[0] = RX_MESSAGE[i];
						packet_temp[1] = RX_MESSAGE[i + 1];
						msgdata[(i / 2) - 1] = deserialize_uint16(packet_temp);
				}
				for (int i = 0; i < 32; i++) 
				{
						printf("msgdata[%d]=%x\n", i, msgdata[i]);
				}

			}
		}
		
	return 1;
}
unsigned int Cmx1553BcEnable (unsigned int PortNumber)
{
	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned char RX_MESSAGE[COMMAND_LEN] = {0};
	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_BC_ENABLE;
	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);
		
	SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
			printf("Command Send Error\n");
		else
		{
			SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
			if (SerialError < 0)
				printf("Command Read Error\n");
			else
			{
				for (uint8_t i = 0; i < COMMAND_LEN; i++)
					printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);
			}
		}
	return 1;
}
unsigned int Cmx1553BcStop (unsigned int PortNumber)
{
	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned char RX_MESSAGE[COMMAND_LEN] = {0};
	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_BC_STOP;
	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);
		
	SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
			printf("Command Send Error\n");
		else
		{
			SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
			if (SerialError < 0)
				printf("Command Read Error\n");
			else
			{
				for (uint8_t i = 0; i < COMMAND_LEN; i++)
					printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);
			}
		}	
	return 1;
}
unsigned int Cmx1553BcTrigger (unsigned int PortNumber)
{
	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned char RX_MESSAGE[COMMAND_LEN] = {0};
	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_BC_TRIGGER;
	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);
		
	SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
			printf("Command Send Error\n");
		else
		{
			SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
			if (SerialError < 0)
				printf("Command Read Error\n");
			else
			{
				for (uint8_t i = 0; i < COMMAND_LEN; i++)
					printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);
			}
		}
		
	return 1;
}
unsigned int Cmx1553BcStart (unsigned int PortNumber)
{
	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned char RX_MESSAGE[COMMAND_LEN] = {0};
	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_BC_START;
	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);
		
	SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
			printf("Command Send Error\n");
		else
		{
			SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
			if (SerialError < 0)
				printf("Command Read Error\n");
			else
			{
				for (uint8_t i = 0; i < COMMAND_LEN; i++)
					printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);
			}
		}
		
	return 1;
}

unsigned int Cmx1553MtSetTTAG (unsigned int PortNumber,unsigned int HiLo,unsigned int MTTag)
{
	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned char RX_MESSAGE[COMMAND_LEN] = {0};
	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_MT_SET_TTAG;
	TX_MESSAGE[2] =(unsigned char)HiLo;
	TX_MESSAGE[3] =(unsigned char)MTTag;

	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);
		
	SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
			printf("Command Send Error\n");
		else
		{
			SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
			if (SerialError < 0)
				printf("Command Read Error\n");
			else
			{
				for (uint8_t i = 0; i < COMMAND_LEN; i++)
					printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);
			}
		}
	return 1;
}
unsigned int Cmx1553MtSetFilter (unsigned int PortNumber)
{
	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned char RX_MESSAGE[COMMAND_LEN] = {0};
	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_MT_SET_FILTER;

	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);
		
	SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
			printf("Command Send Error\n");
		else
		{
			SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
			if (SerialError < 0)
				printf("Command Read Error\n");
			else
			{
				for (uint8_t i = 0; i < COMMAND_LEN; i++)
					printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);
			}
		}
	
	return 1;
}
unsigned int Cmx1553MtGetLastCommand (unsigned int PortNumber,unsigned short* lastcommand )
{
	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned char RX_MESSAGE[COMMAND_LEN] = {0};
	
	unsigned char packet_temp[2]={0,0};
	
	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_MT_GET_LAST_COMMAND;

	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);
		
	SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
			printf("Command Send Error\n");
			
		else
		{
			SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
			if (SerialError < 0)
				printf("Command Read Error\n");
			else
			{
				for (uint8_t i = 0; i < COMMAND_LEN; i++)
					printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);
				
				packet_temp[0]=RX_MESSAGE[2];
				packet_temp[1]=RX_MESSAGE[3];
				*lastcommand=deserialize_uint16(packet_temp);
				printf("command=%x\n",lastcommand);
			}
		}

	return 1;
}
unsigned int Cmx1553MtGetLastMessage (unsigned int PortNumber,unsigned short* lastmsg,unsigned int num_packet)
{
	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned int num_messages_to_send = (num_packet > 20) ? 20 : num_packet;
	unsigned int RX_MESSAGE_SIZE = 2 + (num_messages_to_send * 64) + 6;
	unsigned char* RX_MESSAGE = (unsigned char*)malloc(RX_MESSAGE_SIZE);

	 if (RX_MESSAGE == NULL)
		 return;
	 
	 unsigned char packet_temp[2]={0,0};
		
	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_MT_GET_LAST_MSG;
	TX_MESSAGE[2] =(unsigned char)num_packet;
	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);
		
	SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
			printf("Command Send Error\n");
		else
		{
			SerialError = ComRd(PortNumber, RX_MESSAGE, RX_MESSAGE_SIZE);
			if (SerialError < 0)
				printf("Command Read Error\n");
			else
			{
				for (uint8_t i = 0; i < RX_MESSAGE_SIZE; i++)
					printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);
				
				for(int i=2;i< (num_packet * 64) + 2;i+=2){
	    		packet_temp[0]=RX_MESSAGE[i];
	    		packet_temp[1]=RX_MESSAGE[i+1];
	    		lastmsg[(i/2)-1]=deserialize_uint16(packet_temp);
	    		}
				for (int i = 0; i < (num_packet * 32) ; i++) {
					printf("msgdata[%d]=%x\n", i, lastmsg[i]);
				}
				
			}
		}

	free(RX_MESSAGE);
	return 1;
}
unsigned int Cmx1553MtEnable (unsigned int PortNumber)
{
	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned char RX_MESSAGE[COMMAND_LEN] = {0};
	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_MT_ENABLE;
	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);
		
	SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
			printf("Command Send Error\n");
			
		else
		{
			SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
			if (SerialError < 0)
				printf("Command Read Error\n");
			else
			{
				for (uint8_t i = 0; i < COMMAND_LEN; i++)
					printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);
			}
		}
	
	return 1;
}
unsigned int Cmx1553MtStop (unsigned int PortNumber)
{
	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned char RX_MESSAGE[COMMAND_LEN] = {0};
	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_MT_STOP;
	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);
		
	SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
			printf("Command Send Error\n");
		else
		{
			SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
			if (SerialError < 0)
				printf("Command Read Error\n");
			else
			{
				for (uint8_t i = 0; i < COMMAND_LEN; i++)
					printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);
			}
		}
	return 1;
}

unsigned int Cmx1553Rt1Init (unsigned int PortNumber, unsigned int support_broadcast,unsigned int undef_mcodes_valid,unsigned int use_smcp)
{
	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned char RX_MESSAGE[COMMAND_LEN] = {0};
	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_RT1_INIT;
	TX_MESSAGE[2] =(unsigned char)support_broadcast;
	TX_MESSAGE[3] =(unsigned char)undef_mcodes_valid;
	TX_MESSAGE[4] =(unsigned char)use_smcp;
	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);
		
	SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
			printf("Command Send Error\n");
		else
		{
			SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
			if (SerialError < 0)
				printf("Command Read Error\n");
			else
			{
				for (uint8_t i = 0; i < COMMAND_LEN; i++)
					printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);
			}
		}
	return 1;
}
unsigned int Cmx1553Rt1Init1 (unsigned int PortNumber,unsigned int use_smcp)
{
	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned char RX_MESSAGE[COMMAND_LEN] = {0};
	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_RT1_INIT1;
	TX_MESSAGE[2] =(unsigned char)use_smcp;
	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);
		
	SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
			printf("Command Send Error\n");
		else
		{
			SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
			if (SerialError < 0)
				printf("Command Read Error\n");
			else
			{
				for (uint8_t i = 0; i < COMMAND_LEN; i++)
					printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);
			}
		}
	return 1;
}
unsigned int Cmx1553Rt1Init2 (unsigned int PortNumber, unsigned int illegalcmd_detect)
{
	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned char RX_MESSAGE[COMMAND_LEN] = {0};
	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_RT1_INIT2;
	TX_MESSAGE[2] =(unsigned char)illegalcmd_detect;
	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);
		
	SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
			printf("Command Send Error\n");
		else
		{
			SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
			if (SerialError < 0)
				printf("Command Read Error\n");
			else
			{
				for (uint8_t i = 0; i < COMMAND_LEN; i++)
					printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);
			}
		}
	
		
	return 1;
}

unsigned int Cmx1553Rt1SetAddr (unsigned int PortNumber,unsigned int addr)
{
	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned char RX_MESSAGE[COMMAND_LEN] = {0};
	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_RT1_SET_ADDR;
	TX_MESSAGE[2] =(unsigned char)addr;
	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);
		
	SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
			printf("Command Send Error\n");
		else
		{
			SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
			if (SerialError < 0)
				printf("Command Read Error\n");
			else
			{
				for (uint8_t i = 0; i < COMMAND_LEN; i++)
					printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);
			}
		}
	return 1;
}
unsigned int Cmx1553Rt1SetTTAG (unsigned int PortNumber,unsigned int RT1ttag )
{
	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned char RX_MESSAGE[COMMAND_LEN] = {0};
	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_RT1_SET_TTAG;
	TX_MESSAGE[2] =(unsigned char)RT1ttag;

	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);
		
	SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
			printf("Command Send Error\n");
		else
		{
			SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
			if (SerialError < 0)
				printf("Command Read Error\n");
			else
			{
				for (uint8_t i = 0; i < COMMAND_LEN; i++)
					printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);
			}
		}
	return 1;
}
unsigned int Cmx1553Rt1SetTXData (unsigned int PortNumber,unsigned short *rt1txdata,unsigned int data_count)
{
	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned char RX_MESSAGE[COMMAND_LEN] = {0};
	
	unsigned char packet_temp[2]={0,0};
	for(int i =0;i<32;i+=1){
		serialize_uint16B(packet_temp,&rt1txdata);
		memcpy(TX_MESSAGE+2+(i*2),packet_temp,2);
	}
	
	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_RT1_SET_TX_DATA;
	TX_MESSAGE[2] =(unsigned char)data_count;

	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);
		
	SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
			printf("Command Send Error\n");
		else
		{
			SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
			if (SerialError < 0)
				printf("Command Read Error\n");
			else
			{
				for (uint8_t i = 0; i < COMMAND_LEN; i++)
					printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);
			}
		}
	return 1;
}
unsigned int Cmx1553Rt1Enable (unsigned int PortNumber)
{
	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned char RX_MESSAGE[COMMAND_LEN] = {0};
	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_RT1_ENABLE;

	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);
		
	SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
			printf("Command Send Error\n");
		else
		{
			SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
			if (SerialError < 0)
				printf("Command Read Error\n");
			else
			{
				for (uint8_t i = 0; i < COMMAND_LEN; i++)
					printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);
			}
		}

		
	return 1;
}
unsigned int Cmx1553Rt1Stop (unsigned int PortNumber)
{
	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned char RX_MESSAGE[COMMAND_LEN] = {0};
	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_RT1_STOP;

	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);
		
	SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
			printf("Command Send Error\n");
		else
		{
			SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
			if (SerialError < 0)
				printf("Command Read Error\n");
			else
			{
				for (uint8_t i = 0; i < COMMAND_LEN; i++)
					printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);
			}
		}

	return 1;
}
unsigned int Cmx1553Rt1Start (unsigned int PortNumber)
{
	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned char RX_MESSAGE[COMMAND_LEN] = {0};
	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_RT1_START;

	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);

	SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
			printf("Command Send Error\n");
		else
		{
			SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
			if (SerialError < 0)
				printf("Command Read Error\n");
			else
			{
				for (uint8_t i = 0; i < COMMAND_LEN; i++)
					printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);
			}
		}	
	return 1;
}

unsigned int Cmx1553Rt2Init (unsigned int PortNumber, unsigned int support_broadcast,unsigned int undef_mcodes_valid,unsigned int use_smcp)
{
	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned char RX_MESSAGE[COMMAND_LEN] = {0};
	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_RT2_INIT;
	TX_MESSAGE[2] =(unsigned char)support_broadcast;
	TX_MESSAGE[3] =(unsigned char)undef_mcodes_valid;
	TX_MESSAGE[4] =(unsigned char)use_smcp;
	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);

	SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
			printf("Command Send Error\n");
		else
		{
			SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
			if (SerialError < 0)
				printf("Command Read Error\n");
			else
			{
				for (uint8_t i = 0; i < COMMAND_LEN; i++)
					printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);
			}
		}	
	return 1;
}
unsigned int Cmx1553Rt2Init1 (unsigned int PortNumber,unsigned int use_smcp)
{
	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned char RX_MESSAGE[COMMAND_LEN] = {0};
	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_RT2_INIT1;
	TX_MESSAGE[2] =(unsigned char)use_smcp;

	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);

	SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
			printf("Command Send Error\n");
		else
		{
			SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
			if (SerialError < 0)
				printf("Command Read Error\n");
			else
			{
				for (uint8_t i = 0; i < COMMAND_LEN; i++)
					printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);
			}
		}

	return 1;
}
unsigned int Cmx1553Rt2Init2 (unsigned int PortNumber, unsigned int illegalcmd_detect)
{
	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned char RX_MESSAGE[COMMAND_LEN] = {0};
	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_RT2_INIT2;
	TX_MESSAGE[2] =(unsigned char)illegalcmd_detect;

	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);
		
	SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
			printf("Command Send Error\n");
		else
		{
			SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
			if (SerialError < 0)
				printf("Command Read Error\n");
			else
			{
				for (uint8_t i = 0; i < COMMAND_LEN; i++)
					printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);
			}
		}

	return 1;
}
unsigned int Cmx1553Rt2SetAddr (unsigned int PortNumber,unsigned int addr)
{
	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned char RX_MESSAGE[COMMAND_LEN] = {0};
	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_RT2_SET_ADDR;
	TX_MESSAGE[2] =(unsigned char)addr;

	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);
		

	SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
			printf("Command Send Error\n");
		else
		{
			SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
			if (SerialError < 0)
				printf("Command Read Error\n");
			else
			{
				for (uint8_t i = 0; i < COMMAND_LEN; i++)
					printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);
			}
		}
	return 1;
}
unsigned int Cmx1553Rt2SetTTAG (unsigned int PortNumber,unsigned int RT2ttag)
{
	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned char RX_MESSAGE[COMMAND_LEN] = {0};
	
	
	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_RT2_SET_TTAG;
	TX_MESSAGE[2] =(unsigned char)RT2ttag;

	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);
		
	SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
			printf("Command Send Error\n");
		else
		{
			SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
			if (SerialError < 0)
				printf("Command Read Error\n");
			else
			{
				for (uint8_t i = 0; i < COMMAND_LEN; i++)
					printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);
			}
		}
	
	return 1;
}
unsigned int Cmx1553Rt2SetTXData (unsigned int PortNumber,unsigned short *rt2txdata,unsigned int data_count)
{
	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned char RX_MESSAGE[COMMAND_LEN] = {0};
	
	unsigned char packet_temp[2]={0,0};
	for(int i =0;i<32;i+=1){
		serialize_uint16B(packet_temp,&rt2txdata);
		memcpy(TX_MESSAGE+2+(i*2),packet_temp,2);
	}
	
	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_RT2_SET_TX_DATA;
	TX_MESSAGE[2] =(unsigned char)data_count;

	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);

	SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
			printf("Command Send Error\n");
		else
		{
			SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
			if (SerialError < 0)
				printf("Command Read Error\n");
			else
			{
				for (uint8_t i = 0; i < COMMAND_LEN; i++)
					printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);
			}
		}
		
	return 1;
}
unsigned int Cmx1553Rt2Enable (unsigned int PortNumber)
{
	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned char RX_MESSAGE[COMMAND_LEN] = {0};
	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_RT2_ENABLE;


	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);
		
	SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
			printf("Command Send Error\n");
		else
		{
			SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
			if (SerialError < 0)
				printf("Command Read Error\n");
			else
			{
				for (uint8_t i = 0; i < COMMAND_LEN; i++)
					printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);
			}
		}

	return 1;
}

unsigned int Cmx1553Rt2Stop (unsigned int PortNumber)
{
	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned char RX_MESSAGE[COMMAND_LEN] = {0};
	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_RT2_STOP;


	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);
		
	SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
			printf("Command Send Error\n");
		else
		{
			SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
			if (SerialError < 0)
				printf("Command Read Error\n");
			else
			{
				for (uint8_t i = 0; i < COMMAND_LEN; i++)
					printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);
			}
		}	
	return 1;
}
unsigned int Cmx1553Rt2Start (unsigned int PortNumber)
{
	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned char RX_MESSAGE[COMMAND_LEN] = {0};
	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_RT2_START;


	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);
		
	SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
			printf("Command Send Error\n");
		else
		{
			SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
			if (SerialError < 0)
				printf("Command Read Error\n");
			else
			{
				for (uint8_t i = 0; i < COMMAND_LEN; i++)
					printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);
			}
		}	
	return 1;
}
unsigned int Cmx1553RtGetMessage (unsigned int PortNumber,unsigned short* RTmessage,unsigned int rt_num,unsigned int num_packet)
{
	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned int num_messages_to_send = (num_packet > 20) ? 20 : num_packet;
	unsigned int RX_MESSAGE_SIZE = 2 + (num_messages_to_send * 64) + 6;
	unsigned char* RX_MESSAGE = (unsigned char*)malloc(RX_MESSAGE_SIZE);

	 if (RX_MESSAGE == NULL)
		 return;
	 
	unsigned char packet_temp[2]={0,0};
	
	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_RT_GET_MSG;
	TX_MESSAGE[2] =(unsigned char)rt_num;
	TX_MESSAGE[3] =(unsigned char)num_packet;
	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);
		
	SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
			printf("Command Send Error\n");
		else
		{
			SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
			if (SerialError < 0)
				printf("Command Read Error\n");
			else
			{
				for (uint8_t i = 0; i < COMMAND_LEN; i++)
					printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);
				
				
				packet_temp[0]=RX_MESSAGE[2];
    			packet_temp[1]=RX_MESSAGE[3];
    			RTmessage[0]=deserialize_uint16(packet_temp);
    			printf("RT%d Message Command word=%x\n",rt_num,RTmessage[0]);
    			packet_temp[0]=RX_MESSAGE[4];
    			packet_temp[1]=RX_MESSAGE[5];
    			RTmessage[1]=deserialize_uint16(packet_temp);
    			printf("RT%d Message Data Length=%x\n",rt_num,RTmessage[1]);
    			packet_temp[0]=RX_MESSAGE[6];
    			packet_temp[1]=RX_MESSAGE[7];
    			RTmessage[2]=deserialize_uint16(packet_temp);
    			printf("RT%d Mode Code=%x\n",rt_num,RTmessage[2]);


	   			 for (int i = 8; i < (num_packet * 64)+2; i += 2) {
					packet_temp[0] = RX_MESSAGE[i];
					packet_temp[1] = RX_MESSAGE[i + 1];
					RTmessage[(i / 2) - 1] = deserialize_uint16(packet_temp);
	   			 }
	  			 for(int i =0;i<num_packet*32;i++){
					printf("RT%d message[%d]=%x\n",rt_num,i,RTmessage[i+3]);
				 }	
			}
		}
	free(RX_MESSAGE);
	return 1;
}

unsigned int Cmx1553RtCheckAddr (unsigned int PortNumber,unsigned int rt_num)
{
	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned char RX_MESSAGE[COMMAND_LEN] = {0};
	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_RT_CHECK_ADDR;
	TX_MESSAGE[2] =(unsigned char)rt_num;

	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);
		
	SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
			printf("Command Send Error\n");
		else
		{
			SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
			if (SerialError < 0)
				printf("Command Read Error\n");
			else
			{
				for (uint8_t i = 0; i < COMMAND_LEN; i++)
					printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);
			}
		}
	return 1;
}

unsigned int Cmx1553SetIp (unsigned int PortNumber,unsigned int ipaddress[4])
{
	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned char RX_MESSAGE[COMMAND_LEN] = {0};
	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_SET_IP;
	TX_MESSAGE[2] =(unsigned char)ipaddress[0];
	TX_MESSAGE[3] =(unsigned char)ipaddress[1];
	TX_MESSAGE[4] =(unsigned char)ipaddress[2];
	TX_MESSAGE[5] =(unsigned char)ipaddress[3];
	
	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);
		
	SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
			printf("Command Send Error\n");
		else
		{
			SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
			if (SerialError < 0)
				printf("Command Read Error\n");
			else
			{
				for (uint8_t i = 0; i < COMMAND_LEN; i++)
					printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);
			}
		}	
	return 1;
}
unsigned int Cmx1553GetIp (unsigned int PortNumber,unsigned int *ipreturn)
{
	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned char RX_MESSAGE[COMMAND_LEN] = {0};
	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_GET_IP;

	
	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);
		
	SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
			printf("Command Send Error\n");
		else
		{
			SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
			if (SerialError < 0)
				printf("Command Read Error\n");
			else
			{
				for (uint8_t i = 0; i < COMMAND_LEN; i++)
					printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);
				
				unsigned char ip_array[4]={0,0,0,0};

				ipreturn[0]=(unsigned char)ip_array[0];
				ipreturn[1]=(unsigned char)ip_array[1];
				ipreturn[2]=(unsigned char)ip_array[2];
				ipreturn[3]=(unsigned char)ip_array[3];

			for(int i =2;i<6;i++){
				printf("IP:%d\n",RX_MESSAGE[i]);
			}
			}
		}	
	return 1;
}
unsigned int Cmx1553ResetIP (unsigned int PortNumber)
{
	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned char RX_MESSAGE[COMMAND_LEN] = {0};
	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_RESET_IP;

	
	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);
		
	SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
			printf("Command Send Error\n");
		else
		{
			SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
			if (SerialError < 0)
				printf("Command Read Error\n");
			else
			{
				for (uint8_t i = 0; i < COMMAND_LEN; i++)
					printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);
			}
		}
	return 1;
}
unsigned int Cmx1553SetMac (unsigned int PortNumber,unsigned int new_mac[6])
{
	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned char RX_MESSAGE[COMMAND_LEN] = {0};
	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_SET_MAC;
	TX_MESSAGE[2] =(unsigned char)new_mac[0];
	TX_MESSAGE[3] =(unsigned char)new_mac[1];
	TX_MESSAGE[4] =(unsigned char)new_mac[2];
	TX_MESSAGE[5] =(unsigned char)new_mac[3];
	TX_MESSAGE[6] =(unsigned char)new_mac[4];
	TX_MESSAGE[7] =(unsigned char)new_mac[5];
	
	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);
		
	SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
			printf("Command Send Error\n");
		else
		{
			SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
			if (SerialError < 0)
				printf("Command Read Error\n");
			else
			{
				for (uint8_t i = 0; i < COMMAND_LEN; i++)
					printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);
			}
		}	
	return 1;
}
unsigned int Cmx1553GetMac (unsigned int PortNumber,unsigned int *return_mac)
{
	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned char RX_MESSAGE[COMMAND_LEN] = {0};
	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_GET_MAC;

	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);
		
	SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
			printf("Command Send Error\n");
		else
		{
			SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
			if (SerialError < 0)
				printf("Command Read Error\n");
			else
			{
				for (uint8_t i = 0; i < COMMAND_LEN; i++)
					printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);
				
				unsigned char mac_array[6]={0,0,0,0,0,0};

				return_mac[0]=(unsigned int)mac_array[0];
				return_mac[1]=(unsigned int)mac_array[1];
				return_mac[2]=(unsigned int)mac_array[2];
				return_mac[3]=(unsigned int)mac_array[3];
				return_mac[0]=(unsigned int)mac_array[4];
				return_mac[0]=(unsigned int)mac_array[5];

				for(int i =2;i<8;i++){
				printf("MAC:%d\n",RX_MESSAGE[i]);
				}
			}
		}
		
	return 1;
}

unsigned int Cmx1553GetSerial (unsigned int PortNumber,unsigned int *serialno)
{
	crcInit();
	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
	unsigned char RX_MESSAGE[COMMAND_LEN] = {0};
	
	unsigned char serial_array[4]={0x00,0x00,0x00,0x00};
	TX_MESSAGE[0] = 0x01; 	// BoardID
	TX_MESSAGE[1] =CM_GET_SERIAL_NUMBER;

	
	TX_MESSAGE[74] = 0x01;	// No Error
	crc_result = CRCCALC(TX_MESSAGE, 76);
	deserialize_uint32B(CrcArray,&crc_result);
	memcpy(TX_MESSAGE+76, CrcArray, 4);

	SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
		if (SerialError < 0)
			printf("Command Send Error\n");
		else
		{
			SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
			if (SerialError < 0)
				printf("Command Read Error\n");
			else
			{
				for (uint8_t i = 0; i < COMMAND_LEN; i++)
					printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);
				
				unsigned int serial_=deserialize_uint32(serial_array);
				*serialno=serial_;
				printf("SERIAL NO :%x\n",*serialno);
			}
		}	
	return 1;
}

//unsigned int Cmx1553GetFirmware (unsigned int PortNumber,double  *fw_version)
//{
//	crcInit();
//	unsigned char TX_MESSAGE[COMMAND_LEN] ={0};
//	unsigned char RX_MESSAGE[COMMAND_LEN] = {0};
//	TX_MESSAGE[0] = 0x01; 	// BoardID
//	TX_MESSAGE[1] =CM_GET_FIRMWARE_VERSION;

//	
//	TX_MESSAGE[74] = 0x01;	// No Error
//	crc_result = CRCCALC(TX_MESSAGE, 76);
//	deserialize_uint32B(CrcArray,&crc_result);
//	memcpy(TX_MESSAGE+76, CrcArray, 4);
//		
//	
//
//		SerialError = ComWrt (PortNumber, TX_MESSAGE, COMMAND_LEN);
//		if (SerialError < 0)
//			printf("Command Send Error\n");
//		else
//		{
//			SerialError = ComRd(PortNumber, RX_MESSAGE, COMMAND_LEN);
//			if (SerialError < 0)
//				printf("Command Read Error\n");
//			else
//			{
//				for (uint8_t i = 0; i < COMMAND_LEN; i++)
//					printf("RX Message[%d] = %02X\n",i, RX_MESSAGE[i]);
//				
//				unsigned char fw_array[4]={0x00,0x00,0x00,0x00};
//				unsigned int fw_=deserialize_uint32(fw_array);
//				*fw_version=unpack754_32(fw_);
//				printf("Firmware Version:%.2lf\n",*fw_version);
//			}
//		}
//		
//	return 1;
//}

unsigned int Cmx1553DiscardBoard(unsigned int PortNumber)
{
	SerialError = CloseCom(PortNumber);
	if (SerialError < 0)
		printf("Com Port Close Error\n");
	return 1;
}