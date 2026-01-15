#include "Windows.h"
#include <ansi_c.h>
#include "GlobalVariables.h"
#include "CMX1553_Functions.h"




int main (int argc, char *argv[])
{
	
			unsigned short data1[32]={0x1111, 0x1111, 0x1111, 0XBBBB, 0x1357, 0x2468, 0xABCD,
						0xDCBA, 0xAAAA, 0xBBBB, 0xCCCC, 0xDDDD, 0xEEEE, 0xFFFF,
						0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666,
						0x4444, 0x4444, 0x4444, 0x4444, 0x4444, 0x4444, 0x4444, 0x4444,
						0x1111, 0x1111};
			unsigned short data2[32]={0x2222, 0x2222, 0x2222, 0x3131, 0x3131, 0x3131, 0x3131,
						0xDCBA, 0xAAAA, 0xBBBB, 0xCCCC, 0xDDDD, 0xEEEE, 0xFFFF,
						0x0001, 0x0002, 0x0003, 0x0004, 0x0005, 0x0006, 0x0007, 0x0008,
						0x0009, 0x000A, 0x000B, 0x000C, 0x000D, 0x000E, 0x000F, 0x0010,
						0x2222, 0x2222};
			unsigned short data3[32]={0x3333, 0x3333, 0x3333, 0x3232, 0x3232, 0x3232, 0x3232,
						0xDCBA, 0xAAAA, 0xBBBB, 0xCCCC, 0xDDDD, 0xEEEE, 0xFFFF,
						0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666,
						0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666,
						0x3333, 0x3333};
			unsigned short data4[32]={0x4444, 0x4444, 0x4444, 0xDEF0, 0x1357, 0x2468, 0xABCD,
						0xDCBA, 0xAAAA, 0xBBBB, 0xCCCC, 0xDDDD, 0xEEEE, 0xFFFF,
						0x0001, 0x0002, 0x0003, 0x0004, 0x0005, 0x0006, 0x0007, 0x0008,
						0x0009, 0x000A, 0x000B, 0x000C, 0x000D, 0x000E, 0x000F, 0x0010,
						0x4444, 0x4444};
			unsigned short data5[32]={0x5555, 0x5555, 0x5555, 0x3131, 0x3131, 0x3131, 0x3131,
						0xDCBA, 0xAAAA, 0xBBBB, 0xCCCC, 0xDDDD, 0xEEEE, 0xFFFF,
						0x0001, 0x0002, 0x0003, 0x0004, 0x0005, 0x0006, 0x0007, 0x0008,
						0x0009, 0x000A, 0x000B, 0x000C, 0x000D, 0x5555, 0x5555, 0x5555,0x5555, 0x5555};
			unsigned short data6[32]={0x6666, 0x6666, 0x6666, 0x3232, 0x3232, 0x3232, 0x3232,
						0xDCBA, 0xAAAA, 0xBBBB, 0xCCCC, 0xDDDD, 0xEEEE, 0xFFFF,
						0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666,
						0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666,
						0x6666, 0x6666};
			unsigned short data7[32]={0x7777, 0x7777, 0x7777, 0x3232, 0x3232, 0x3232, 0x3232,
						0xDCBA, 0xAAAA, 0xBBBB, 0xCCCC, 0xDDDD, 0xEEEE, 0xFFFF,
						0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666,
						0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666, 0x0666,
						0x7777, 0x7777};
		

			
			
			int PortNumber = 4;
			int Handle = 0;
			unsigned short controlWord;
			unsigned short msgdata[640];
			unsigned short msgblock[10];
			unsigned short mtmessage[640];
			unsigned short rtmessage[640];
			unsigned short command=0;
			unsigned int active_mode=0;
		
			
			unsigned int ip[4]={192,168,10,90};
			unsigned int ip2[4]={0};
			unsigned int mac[6]={12,23,10,55,66,77};
			unsigned int mac2[6]={0};
			double fw;
			unsigned int serial_no;

			unsigned int ip4[4]={192,168,90,55};
			unsigned int ip3[4]={0};
			unsigned int mac4[6]={11,55,87,45,65,9};
			unsigned int mac3[6]={0};
			double fw2;
			unsigned int serial_no2;
			//*******************************BC TEST**********************************************//

	
			Handle = Cmx1553OpenBoard(PortNumber);
			printf("1553 open board error is : \n ");
			Sleep(500);
			
			
//			Cmx1553SetIp(Handle, ip);
//			printf("1553 set ýp error is : \n ");
//			Sleep(500);
//			
//			Cmx1553SetMac(Handle, mac);
//			printf("1553 set mac error is : \n ");
//			Sleep(500);
//			
//			Cmx1553GetIp(Handle, ip2);
//			printf("1553 get ýp error is : \n ");
//			Sleep(500);
//			
//			Cmx1553GetSerial(Handle, &serial_no);
//			printf("1553 get serial error is : \n ");
//			Sleep(500);
//			
///*			Cmx1553GetFirmware(Handle, &fw);
//			printf("1553 get firmware error is : \n ");
//			Sleep(500)*/;
//			Cmx1553GetMac(Handle, mac2);
//			printf("1553 get mac error is : \n ");
//			Sleep(500);
//			




			Cmx1553BoardSetMode(Handle, 1, 0, 0, 0);//bc mode
			printf("1553 Board set mode error is : \n ");
			Sleep(500);
			
			Cmx1553Reset(Handle);
			printf("1553 reset error is : \n ");
			Sleep(500);
			 
			Cmx1553BcEnable(Handle);
			printf("1553 Bc enable error is : \n ");
			Sleep(500);

			Cmx1553BoardGetMode(Handle, &active_mode);
			printf("1553 Board Get mode error is : \n ");
			Sleep(500);
			
			Cmx1553SharedInit(Handle);
			printf("1553 SharedInit error is : \n ");
			Sleep(500);

			controlWord=MEMASK|USEBUSA;//4080
			Cmx1553BcInitMsg(Handle, RX_MSG_CMD,1,32,data1, controlWord, RT_ADDR4, RT_SUBBADDR1,NO_MC);//rxmessage1
			printf("1553 Initmsg1 error is : \n ");
			Sleep(500);

			//controlWord=MEMASK|USEBUSA;
			//Cmx1553BcInitMsg(Handle, TX_MSG_CMD,1,32,data6, controlWord, RT_ADDR4, RT_SUBBADDR2,NO_MC);//txmessage1 //data parameters insignificant
			//printf("1553 Initmsg1 error is : \n ");
			//Sleep(500);
			
			controlWord=MEMASK|USEBUSA;//4000
			Cmx1553BcInitMsg(Handle, RX_MSG_CMD,2,32,data2, controlWord, RT_ADDR4, RT_SUBBADDR2,NO_MC);//rxmessage2
			printf("1553 Initmsg1 error is : \n ");
			Sleep(500);

			
			//controlWord=MEMASK|USEBUSA|MSKBCR|BCST;
			//Cmx1553BcInitMsg(Handle, BRDCST_MSG_CMD,1,32,data3, controlWord, RT_ADDR4, RT_SUBBADDR9,NO_MC);//broadcast message1
			//printf("1553 Initmsg1 error is : \n ");
			//Sleep(500);
			
			//controlWord=MEMASK|USEBUSA;
			//Cmx1553BcInitMsg(Handle, RX_MSG_CMD,3,32,data3, controlWord, RT_ADDR4, RT_SUBBADDR3,NO_MC);//rxmessage3
			//printf("1553 Initmsg1 error is : \n ");
			//Sleep(500);

		/*	controlWord=MEMASK|USEBUSA;
			Cmx1553BcInitMsg(Handle, RX_MSG_CMD,4,32,data4, controlWord, RT_ADDR4, RT_SUBBADDR21,NO_MC);//rxmessage4
			printf("1553 Initmsg1 error is : \n ");
			Sleep(500);*/
			
	

			Cmx1553BcInitMsgList(Handle, TIMESTAMP_MODE,LOOP_ENABLE, 10000,100000 );
			printf("1553 Initmsglist error is : \n ");
			Sleep(500);
					
			Cmx1553BcSetTTAG(Handle, HIGH_RES, TTAG_64U);
			printf("1553 Bcsetttag error is : \n ");
			Sleep(500);
			
			Cmx1553BcStart(Handle);
			printf("1553 Bcstart error is : \n ");
			Sleep(500);
			
		for (int i=0; i<10; i++)
				{
			   //Cmx1553BcTrigger(Handle);// only use trigger mode 
			   //printf("1553 Bctrigger error is : \n ");
			   //Sleep(0.5);
				Cmx1553BcGetMsgData(Handle, msgdata,1);
				printf("1553 Getdata error is : \n ");
				Sleep(100);
				}
			
			controlWord=MEMASK|USEBUSA;
			Cmx1553BcUpdateMsg(Handle, RX_MSG_CMD,1,32,data4, controlWord, RT_ADDR4, RT_SUBBADDR1,NO_MC);//rxmessage
			printf("1553 Update error is : \n ");
			Sleep(10);
			controlWord=MEMASK|USEBUSA;
			Cmx1553BcUpdateMsg(Handle, RX_MSG_CMD,1,32,data5, controlWord, RT_ADDR4, RT_SUBBADDR1,NO_MC);//rxmessage
			printf("1553 Update error is : \n ");
			Sleep(10);
			controlWord=MEMASK|USEBUSA;
			Cmx1553BcUpdateMsg(Handle, RX_MSG_CMD,1,32,data6, controlWord, RT_ADDR4, RT_SUBBADDR1,NO_MC);//rxmessage
			printf("1553 Update error is : \n ");
			Sleep(10);
			controlWord=MEMASK|USEBUSA;
			Cmx1553BcUpdateMsg(Handle, RX_MSG_CMD,1,32,data7, controlWord, RT_ADDR4, RT_SUBBADDR1,NO_MC);//rxmessage
			printf("1553 Update error is : \n ");
			Sleep(10);
			
			controlWord=MEMASK|USEBUSA;
			Cmx1553BcUpdateMsg(Handle, RX_MSG_CMD,2,32,data1, controlWord, RT_ADDR4, RT_SUBBADDR2,NO_MC);//rxmessage
			printf("1553 Update error is : \n ");
			Sleep(10);
			controlWord=MEMASK|USEBUSA;
			Cmx1553BcUpdateMsg(Handle, RX_MSG_CMD,2,32,data2, controlWord, RT_ADDR4, RT_SUBBADDR2,NO_MC);//rxmessage
			printf("1553 Update error is : \n ");
			Sleep(10);
			controlWord=MEMASK|USEBUSA;
			Cmx1553BcUpdateMsg(Handle, RX_MSG_CMD,2,32,data3, controlWord, RT_ADDR4, RT_SUBBADDR2,NO_MC);//rxmessage
			printf("1553 Update error is : \n ");
			Sleep(10);
			controlWord=MEMASK|USEBUSA;
			Cmx1553BcUpdateMsg(Handle, RX_MSG_CMD,2,32,data4, controlWord, RT_ADDR4, RT_SUBBADDR2,NO_MC);//rxmessage
			printf("1553 Update error is : \n ");
			Sleep(10);
			
			
				for (int i=0; i<500; i++)
				{
			   //Cmx1553BcTrigger(Handle);// only use trigger mode 
			   //printf("1553 Bctrigger error is : \n ");
			   //Sleep(0.5);
				Cmx1553BcGetMsgData(Handle, msgdata,1);
				printf("1553 Getdata error is : \n ");
				Sleep(10);
				}
			

		



			
			
			Cmx1553BcStop(Handle);
			printf("1553 BcStop error is : \n ");
			Sleep(500);





			/*********************MT TEST*******************/
/*          Cmx1553OpenBoard(PortNumber);
			printf("1553 openboard error is : \n ");
			Sleep(500);
			Cmx1553BoardSetMode(Handle, 0, 0, 0, 1);
			printf("1553 Boardsetmode error is : \n ");
			Sleep(500);
			Cmx1553Reset(Handle);
			printf("1553 reset error is : \n ");
			Sleep(500);
			Cmx1553SharedInit(Handle);
			printf("1553 shared init error is : \n ");
			Sleep(2);
			Cmx1553MtSetTTAG(Handle, HIGH_RES, MTTAG_64U);
			printf("1553 MT set TTAG error is : \n ");
			Sleep(2);
			Cmx1553MtEnable(Handle);
			printf("1553 MT enable error is : \n ");
			Sleep(2);
			for(int i=0;i<100;i++)
			{
				Cmx1553MtGetLastCommand(Handle, &command);
				printf("1553  MT Command error is : \n ");
				Sleep(500);

				Cmx1553MtGetLastMessage(Handle, mtmessage,5);
				printf("1553  MT Data error is : \n ");
				Sleep(500);

			}

*/

		//*******************RT1 TEST************************/

/*			Cmx1553OpenBoard(PortNumber);
			printf("1553 Openboard error is : \n ");
			Sleep(2);
			Cmx1553BoardSetMode(Handle, 0, 1 ,1 ,1);
			printf("1553 Boardsetmode error is : \n ");
			Sleep(2);
			Cmx1553Rt1SetAddr(Handle, RT_ADDR4);
			printf("1553 RT1 set addr error is : \n ");
			Sleep(2);
			Cmx1553Reset(Handle);
			printf("1553 Reset error is : \n ");
			Sleep(2);
			Cmx1553Rt1Enable(Handle);
			printf("1553 RT1 enable error is : \n ");
			Sleep(2);

			Cmx1553SharedInit(Handle);
			printf("1553 SharedInit error is : \n ");
			Sleep(500);
			Cmx1553RtCheckAddr(Handle, RT1);
			printf("1553 RT Check Addr error is : \n ");
			Sleep(500);
			Cmx1553Rt1Init(Handle, SUPPORT_BRDCST, UNDEF_MCODES_VALID, USE_SMCP);
			printf("1553 RT1 init error is : \n ");
			Sleep(500);
			Cmx1553BoardGetMode(Handle, &active_mode);
			printf("1553 Get mode error is : \n ");
			Sleep(2);
			Cmx1553Rt1Init1(Handle,USE_SMCP);
			printf("1553 RT1 init1 error is : \n ");
			Sleep(500);
			Cmx1553Rt1Init2(Handle, ILLEGAL_CMD_DETECT);
			printf("1553 RT1 init2 error is : \n ");
			Sleep(500);
			Cmx1553Rt1SetTXData(Handle, data2, 0x20);
			printf("1553 RT1 Set Tx data error is : \n ");
			Sleep(500);
			Cmx1553Rt1SetTTAG(Handle, TTAG_64U);
			printf("1553 RT1 Set TTAG error is : \n ");
			Sleep(500);
			Cmx1553Rt1Start(Handle);
			printf("1553 RT1 Start error is : \n ");
			Sleep(500);
			
			Cmx1553RtGetMessage(Handle, rtmessage,RT1,5);
			printf("1553 RT1 Get Message error is : \n ");
			Sleep(500);
			

*/
		//************************RT2 TEST****************************//

/*			Cmx1553OpenBoard(PortNumber);
			printf("1553 Openboard error is : \n ");
			Sleep(2);
			Cmx1553BoardSetMode(Handle, 0, 0, 1, 0);
			printf("1553 Boardsetmode error is : \n ");
			Sleep(3);
			Cmx1553Rt2SetAddr(Handle, RT_ADDR3);
			printf("1553 RT2 set addr error is : \n ");
			Sleep(500);
			Cmx1553Reset(Handle);
			printf("1553 Reset error is : \n ");
			Sleep(500);
			Cmx1553Rt2Enable(Handle);
			printf("1553 RT2 enable error is : \n ");
			Sleep(500);
			Cmx1553SharedInit(Handle);
			printf("1553 SharedInit error is : \n ");
			Sleep(500);
			Cmx1553RtCheckAddr(Handle, RT2);
			printf("1553 RT Check Addr error is : \n ");
			Sleep(500);
			Cmx1553Rt2Init(Handle, SUPPORT_BRDCST, UNDEF_MCODES_VALID, USE_SMCP);
			printf("1553 RT2 init error is : \n ");
			Sleep(500);
			Cmx1553Rt2Init1(Handle, USE_SMCP);
			printf("1553 RT2 init1 error is : \n ");
			Sleep(500);
			Cmx1553Rt2Init2(Handle, ILLEGAL_CMD_DETECT);
			printf("1553 RT2 init2 error is : \n ");
			Sleep(500);
			Cmx1553Rt2SetTXData(Handle, data2, 0x20);
			printf("1553 RT2 Set Tx data error is : \n ");
			Sleep(500);
			Cmx1553Rt2SetTTAG(Handle, TTAG_64U);
			printf("1553 RT2 Set TTAG error is : \n ");
			Sleep(500);
			Cmx1553Rt2Start(Handle);
			printf("1553 RT2 Start error is : \n ");
			Sleep(500);
			
			Cmx1553RtGetMessage(Handle, mtmessage, RT2,5);
			printf("1553 RT2 Get Message error is : \n ");
			Sleep(500);
			

			 Cmx1553MtSetTTAG(Handle, HIGH_RES, MTTAG_64U);
					printf("1553 MT set TTAG error is : \n ");
					Sleep(2);
					 Cmx1553MtEnable(Handle);
					printf("1553 MT enable error is : \n ");
					Sleep(2);
					for(int i=0;i<100;i++)
					{
						 Cmx1553MtGetLastCommand(Handle, &command);
						printf("1553  MT Command error is : \n ");
						Sleep(500);

						 Cmx1553MtGetLastMessage(Handle, mtmessage);
						printf("1553  MT Data error is : \n ");
						Sleep(500);

					}
*/
		//END 1553/
	Cmx1553DiscardBoard(Handle);
	
	return 0;
}

