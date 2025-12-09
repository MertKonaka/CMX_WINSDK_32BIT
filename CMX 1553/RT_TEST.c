#include "CMXAPI32.h"
#include <stdio.h>
#include <windows.h>
#include <ansi_c.h>

double Sample;
#define ETHERNET	1
#define RS_485_INTERFACE  2
unsigned int IP[4] = {192, 168, 10, 30};
unsigned int Error = 0;
unsigned int Handle ;

unsigned int CMX1553_boardID=1;

unsigned short controlWord;
unsigned int msgdata[32];
unsigned int msgblock[10];
unsigned int mtmessage[36];
unsigned int rtmessage[36];
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


unsigned int data1[5]={0x09, 0x58, 0xBC, 0x01, 0x17} ;
unsigned int data2[32]={0x2222, 0x1111, 0x2222, 0xDEF0, 0x1357, 0x2468, 0xABCD,
			0xDCBA, 0xAAAA, 0xBBBB, 0xCCCC, 0xDDDD, 0xEEEE, 0xFFFF,
			0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666,
			0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666,
			0x0666, 0x2222};
unsigned int data3[32]={0x3333, 0x4444, 0x4444, 0x4444, 0x1357, 0x2468, 0xABCD,
			0xDCBA, 0xAAAA, 0xBBBB, 0xCCCC, 0xDDDD, 0xEEEE, 0xFFFF,
			0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666,
			0x4444, 0x4444, 0x4444, 0x4444, 0x4444, 0x4444, 0x4444, 0x4444,
			0x4444, 0x3333};
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
			0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666,
			0x6666, 0x7777};
unsigned int data7[32]={0x1111, 0x2222, 0x3333, 0x4444, 0x5555, 0x6666, 0x7777,
			0x8888, 0x9999, 0x1010, 0xCCCC, 0xDDDD, 0xEEEE, 0xFFFF,
			0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666,
			0x0666, 0x0666, 0x0666, 0x0666};
unsigned int data8[32] = { 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
			0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
			0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
			0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
			0x0000, 0x0000 };


int main (int argc, char *argv[])
{	

	Handle = CM1553OpenBoard(ETHERNET, 4, IP, CMX1553_boardID, &Error);//PORT NO 4
	printf("1553 Openboard error is : %d\n ", Error);
	Sleep(500);

	CM1553BoardSetMode(ETHERNET, IP, Handle, CMX1553_boardID, 0, 1, 1, 1, &Error);
	printf("1553 Boardsetmode error is : %d\n ", Error);
	Sleep(500);

	CM1553Rt1SetAddr(ETHERNET, IP, Handle, CMX1553_boardID, RT_ADDR15, &Error);
	printf("1553 RT1 set addr error is : %d\n ", Error);
	Sleep(500);

	CM1553ResetIC(ETHERNET, IP, Handle, CMX1553_boardID, &Error);
	printf("1553 Reset error is : %d\n ", Error);
	Sleep(500);
	
	CM1553Rt1Enable(ETHERNET, IP, Handle, CMX1553_boardID, &Error);
	printf("1553 RT1 enable error is : %d\n ", Error);
	Sleep(500);
	
	CM1553SharedInit(ETHERNET, IP, Handle, CMX1553_boardID, &Error);
	printf("1553 SharedInit error is : %d\n ", Error);
	Sleep(500);
	
	CM1553RtCheckAddr(ETHERNET, IP, Handle, CMX1553_boardID, RT1, &Error);
	printf("1553 RT Check Addr error is : %d\n ", Error);
	Sleep(500);

	CM1553Rt1Init(ETHERNET, IP, Handle, CMX1553_boardID, SUPPORT_BRDCST, UNDEF_MCODES_VALID, USE_SMCP, &Error);
	printf("1553 RT1 init error is : %d\n ", Error);
	Sleep(500);
	CM1553Rt1Init1(ETHERNET, IP, Handle, CMX1553_boardID, USE_SMCP, &Error);
	printf("1553 RT1 init1 error is : %d\n ", Error);
	Sleep(500);

	CM1553Rt1Init2(ETHERNET, IP, Handle, CMX1553_boardID, ILLEGAL_CMD_DETECT, &Error);
	printf("1553 RT1 init2 error is : %d\n ", Error);
	Sleep(500);
	
	CM1553Rt1SetTXData(ETHERNET, IP, Handle, CMX1553_boardID, data7, 26, &Error);
	printf("1553 RT1 Set Tx data error is : %d\n ", Error);
	Sleep(500);

	CM1553Rt1SetTTAG(ETHERNET, IP, Handle, CMX1553_boardID, TTAG_64U, &Error);
	printf("1553 RT1 Set TTAG error is : %d\n ", Error);
	Sleep(500);

	CM1553Rt1Start(ETHERNET, IP, Handle, CMX1553_boardID, &Error);
	printf("1553 RT1 Start error is : %d\n ", Error);
	Sleep(500);
	
	/*CM1553MtSetTTAG(ETHERNET,IP,Handle,CMX1553_boardID,HIGH_RES,MTTAG_64U, &Error);
	printf("1553 SharedInit error is : %d\n ", Error);
		
	CM1553MtEnable(ETHERNET,IP,Handle,CMX1553_boardID,&Error);
	printf("1553 SharedInit error is : %d\n ", Error);
	
	CM1553BoardGetMode(ETHERNET, IP, Handle, CMX1553_boardID, &active_mode, &Error);
	printf("1553 Get mode error is : %d\n ", Error);
	Sleep(2);
		
	for(int i=0;i<20;i++)
	{
				CM1553MtGetLastMessage(ETHERNET,IP,Handle,CMX1553_boardID,mtmessage,&Error);
				printf("1553 MTmessage error is : %d\n ", Error);
				Sleep(500);
	}*/


		for(int a=0;a<3000;a++)
		{
					CM1553RtGetMessage(ETHERNET,IP,Handle,CMX1553_boardID,rtmessage,RT1,&Error);
					printf("1553 SharedInit error is : %d\n ", Error);
					Sleep(500);
		}


		CM1553Rt1Stop(ETHERNET, IP, Handle, CMX1553_boardID, &Error);
		printf("1553 RT1 Stop error is : %d\n ", Error);
		Sleep(500);
		CM1553DiscardBoard(ETHERNET,IP,Handle,CMX1553_boardID,&Error);
		printf("1553 Discard error is :%d \n ",Error);
		Sleep(500);






	return 0;
}