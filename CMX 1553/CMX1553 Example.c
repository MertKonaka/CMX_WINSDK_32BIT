
#include <ansi_c.h>
#include "CMXAPI32.h"
#include "windows.h"
#include "stdio.h"
double Sample;
#define ETHERNET	1
#define RS_485_INTERFACE  2
unsigned int IP[4] = {192, 168, 10,30};
unsigned int error = 0;
unsigned int Handle ;


unsigned int CMX1553_boardID=1;

unsigned short controlWord;
unsigned int msgdata[38];
unsigned int msgblock[10];
unsigned int mtmessage[36];
unsigned int rtmessage[32];
unsigned int command=0;
unsigned int active_mode=0;
		//CONTROL WORD OPTIONS
				
			// -TXTTMC17		// only applies for mode code 17: transmit BC time tag count
			//- MEMASK			// if bit 0 = 0, Status Set occurs for RT Status Word Msg Error bit
			//- SRQMASK			// if bit 0 = 0, Status Set occurs for RT Status Word Svc Request bit
			//- BSYMASK			// if bit 0 = 0, Status Set occurs for RT Status Word Busy bit
			//- SSFMASK			// if bit 0 = 0, Status Set occurs for RT Status Word Subsystem Fail bit
			//- TFMASK			// if bit 0 = 0, Status Set occurs for RT Status Word Terminal Flag bit
			//- RSVMASK			// if bit 0 = 0, Status Set occurs for any RT Status Word Reserved bit 7-5
			//- RTRYENA			// if retry enabled in BC Config reg, retry occurs for Status Set
			//- USEBUSA			// if bit = 1 then use Bus A,
			//- USEBUSB			//if bit = 0 then use Bus B
			//- SFTEST			// if bit = 1 then use offline self-test
			//- MSKBCR			// if BCRME = 1 in BC Config, this bit INVERTED reflects expected BCR status,
			//				        //mismatch when BCR = 1 causes status set
			//				        // if BCRME = 0 in BC Config, this bit reflects expected BCR status,
			//				        // mismatch = status set
			//- EOMINT		// if BCEOM interrupt is enabled, this bit causes message EOM interrupt
			//- MCODE			// select mode code message format
			//- BCST			// select broadcast message format
			//- RT_RT			// select RT-to-RT message format


unsigned int data1[5]={0x1111, 0x2222, 0x3333, 0x4444, 0xDDDD};

unsigned int data2[32]={0x2222, 0x1111, 0x2222, 0xDEF0, 0x1357, 0x2468, 0xABCD,
			0xDCBA, 0xAAAA, 0xBBBB, 0xCCCC, 0xDDDD, 0xEEEE, 0xFFFF,
			0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666,
			0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666,
			};
unsigned int data3[32]={0x3333, 0x4444, 0x4444, 0x4444, 0x1357, 0x2468, 0xABCD,
			0xDCBA, 0xAAAA, 0xBBBB, 0xCCCC, 0xDDDD, 0xEEEE, 0xEEEE,
			0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666,
			0x4444, 0x4444, 0x4444, 0x4444, 0x4444, 0x4444, 0x4444, 0x4444};
unsigned int data4[32]={0x4444, 0x4444, 0x4444, 0xDEF0, 0x1357, 0x2468, 0xABCD,
			0xDCBA, 0xAAAA, 0xBBBB, 0xCCCC, 0xDDDD, 0xEEEE, 0xFFFF,
			0x0001, 0x0002, 0x0003, 0x0004, 0x0005, 0x0006, 0x0007, 0x0008,
			0x0009, 0x000A, 0x000B, 0x000C, 0x000D, 0x000E, 0x000F, 0x0010,
			0x4444, 0x4444};
unsigned int data5[32]={0x5555, 0x5555, 0x5555, 0x3131, 0x3131, 0x3131, 0x3131,
			0xDCBA, 0xAAAA, 0xBBBB, 0xCCCC, 0xDDDD, 0xEEEE, 0xFFFF,
			0x0001, 0x0002, 0x0003, 0x0004, 0x0005, 0x0006, 0x0007, 0x0008,
			0x0009, 0x000A, 0x000B, 0x000C, 0x000D, 0x5555, 0x5555, 0x5555,0x5555, 0x5555};
unsigned int data6[32]={0x6666, 0x6666, 0x6666, 0x3232, 0x3232, 0x3232, 0x3232,
			0xDCBA, 0xAAAA, 0xBBBB, 0xCCCC, 0xDDDD, 0xEEEE, 0xFFFF,
			0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666,
			0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666};
unsigned int data7[32]={0x7777, 0x7777, 0x7777, 0x3232, 0x3232, 0x3232, 0x3232,
			0xDCBA, 0xAAAA, 0xBBBB, 0xCCCC, 0xDDDD, 0xEEEE, 0xFFFF,
			0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666,
			0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666,
			0x7777, 0x7777};

int main (int argc, char *argv[])
{
	//////1553
	Handle = CM1553OpenBoard(ETHERNET, 4, IP,CMX1553_boardID, &error);//PORT NO 4
	printf("1553 open board error is :%d \n ",error);
	Sleep(1000);
	CM1553BoardSetMode(ETHERNET,IP,Handle,CMX1553_boardID,1,0,0,1,&error);
	printf("1553 Board set mode error is :%d \n ",error);
	Sleep(1000);
	CM1553ResetIC(ETHERNET,IP,Handle,CMX1553_boardID,&error);
	printf("1553 reset error is :%d \n ",error);
	Sleep(1000);
	CM1553SharedInit(ETHERNET,IP,Handle,CMX1553_boardID,&error);
	printf("1553 SharedInit error is :%d \n ",error);
	Sleep(1000);
	CM1553BcEnable(ETHERNET,IP,Handle,CMX1553_boardID,&error);
	printf("1553 Bc enable error is :%d \n ",error);
	Sleep(1000);

	CM1553BoardGetMode(ETHERNET,IP,Handle,CMX1553_boardID,&active_mode,&error);
	printf("1553 Board Get mode error is :%d \n ",error);
	Sleep(1000);
	controlWord=MEMASK|USEBUSA;//4080
		CM1553BcInitMsg(ETHERNET,IP,Handle,CMX1553_boardID,TX_MSG_CMD,1,26,data2,controlWord,RT_ADDR1,RT_SUBBADDR2,NO_MC,&error);
	printf("1553 Initmsg1 error is :%d \n ",error);
	Sleep(1000);
		CM1553BcInitMsg(ETHERNET,IP,Handle,CMX1553_boardID,RX_MSG_CMD,1,30,data3,controlWord,RT_ADDR1,RT_SUBBADDR2,NO_MC,&error);
	printf("1553 Initmsg1 error is :%d \n ",error);
	Sleep(1000);

	CM1553BcInitMsgList(ETHERNET,IP,Handle,CMX1553_boardID, TIMESTAMP_MODE,LOOP_ENABLE, 100000,100000 ,&error);
	printf("1553 Initmsglist error is :%d \n ",error);
	Sleep(500);
			
	CM1553BcSetTTAG(ETHERNET,IP,Handle,CMX1553_boardID, HIGH_RES, TTAG_64U,&error);
	printf("1553 Bcsetttag error is :%d \n ",error);
	Sleep(500);
	
	CM1553BcStart(ETHERNET,IP,Handle,CMX1553_boardID,&error);
	printf("1553 Bcstart error is :%d \n ",error);
	Sleep(500);
			
	for (int i=0; i<10; i++)
		{
		CM1553BcGetMsgData(ETHERNET,IP,Handle,CMX1553_boardID, msgdata,&error);
		printf("1553 Getdata error is :%d \n ",error);
		Sleep(500);
		}
		CM1553BcUpdateMsg(ETHERNET,IP,Handle,CMX1553_boardID,RX_MSG_CMD,1,30,data6,controlWord,RT_ADDR1,RT_SUBBADDR10,NO_MC,&error);
		printf("1553 Initmsg1 error is :%d \n ",error);
		Sleep(1000);
		
		for (int i=0; i<1000; i++)
		{
		CM1553BcGetMsgData(ETHERNET,IP,Handle,CMX1553_boardID, msgdata,&error);
		printf("1553 Getdata error is :%d \n ",error);
		Sleep(500);
		}	
		CM1553BcStop(ETHERNET,IP,Handle,CMX1553_boardID,&error);
		printf("1553 BC Stop error is :%d \n ",error);
		Sleep(500);
		CM1553DiscardBoard(ETHERNET,IP,Handle,CMX1553_boardID,&error);
		printf("1553 Discard error is :%d \n ",error);
		Sleep(500);
	return 0;
}

