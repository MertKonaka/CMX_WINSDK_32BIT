//==============================================================================
//
// Title:		CMXWinSDK
// Purpose:		This library was developed to interface with Konaka Defence CMX hardware platform.
//
// Released on:	15/04/2023  by Konaka Defence
// Copyright:	. All Rights Reserved.
//
//==============================================================================

#ifndef __CMXWinSDK_H__
#define __CMXWinSDK_H__

#ifdef __cplusplus
    extern "C" {
#endif

// 5015 MACROS	//

typedef enum{
	TIMER_CHANNEL_0=0,
	TIMER_CHANNEL_1,
	TIMER_CHANNEL_2,
	TIMER_CHANNEL_3,
	TIMER_CHANNEL_4,
	TIMER_CHANNEL_5,
	TIMER_CHANNEL_ALL
}TimerChannel;

#define	CH0_MASK 0X01
#define	CH1_MASK 0X02	
#define	CH2_MASK 0X04	
#define	CH3_MASK 0X08	
#define	CH4_MASK 0X10	
#define	CH5_MASK 0X20	

/**********************/


//*BC MACROS//
#define ENABLE 	1
#define DISABLE 0

//1553 GET MSG DATA MACROS
#define BC_MODE_MASK	0X01
#define RT1_MODE_MASK	0X02
#define RT2_MODE_MASK	0X04
#define MT_MODE_MASK	0X08

//BC INIT MSG MACROS
#define TXTTMC17		1<<15	// only applies for mode code 17: transmit BC time tag count
#define MEMASK			1<<14	// if bit 0 = 0, Status Set occurs for RT Status Word Msg Error bit
#define SRQMASK			1<<13	// if bit 0 = 0, Status Set occurs for RT Status Word Svc Request bit
#define BSYMASK			1<<12	// if bit 0 = 0, Status Set occurs for RT Status Word Busy bit
#define SSFMASK			1<<11	// if bit 0 = 0, Status Set occurs for RT Status Word Subsystem Fail bit
#define TFMASK			1<<10	// if bit 0 = 0, Status Set occurs for RT Status Word Terminal Flag bit
#define RSVMASK			1<<9	// if bit 0 = 0, Status Set occurs for any RT Status Word Reserved bit 7-5
#define RTRYENA			1<<8	// if retry enabled in BC Config reg, retry occurs for Status Set
#define USEBUSA			1<<7	// if bit = 1 then use Bus A, if bit = 0 then use Bus B
#define USEBUSB			0<<7
#define SFTEST			1<<6	// if bit = 1 then use offline self-test
#define MSKBCR			1<<5	// if BCRME = 1 in BC Config, this bit INVERTED reflects expected BCR status,
							//	mismatch when BCR = 1 causes status set
							// if BCRME = 0 in BC Config, this bit reflects expected BCR status,
							//	mismatch = status set
#define EOMINT			1<<4	// if BCEOM interrupt is enabled, this bit causes message EOM interrupt
							// bit 3 reserved
#define MCODE			1<<2	// select mode code message format
#define BCST			1<<1	// select broadcast message format
#define RT_RT			1<<0	// select RT-to-RT message format

#define TIMESTAMP_MODE		0	//USE TIMESTAMP BETWEEN MESSAGES
#define TRIGGER_MODE			1	//USE BC TRIG PÝN FOR SEND EVERY MESSAGE

//BC INIT MSG MACROS
#define BUSA				1<<7
#define BUSB				0<<7

//BC INIT MSG DATA MACROS
#define SUBBADDR_RX_DATA	0
#define SUBBADDR_TX_DATA	1
#define BRDCST_RX_DATA		2
#define MC_RX_DATA				3

//BC INIT MSG MACROS
#define RX_MSG_CMD				0
#define TX_MSG_CMD				1
#define BRDCST_MSG_CMD  	2
#define TX_MC_MSG_CMD   	3
#define RX_MC_MSG_CMD   	4

//RT ADDRESS MACROS
#define RT_ADDR0 				0
#define RT_ADDR1 				1
#define RT_ADDR2 				2
#define RT_ADDR3 				3
#define RT_ADDR4 				4
#define RT_ADDR5 				5
#define RT_ADDR6 				6
#define RT_ADDR7 				7
#define RT_ADDR8 				8
#define RT_ADDR9 				9
#define RT_ADDR10 			10
#define RT_ADDR11 			11
#define RT_ADDR12 			12
#define RT_ADDR13 			13
#define RT_ADDR14 			14
#define RT_ADDR15 			15
#define RT_ADDR16 			16
#define RT_ADDR17 			17
#define RT_ADDR18 			18
#define RT_ADDR19 			19
#define RT_ADDR20 			20
#define RT_ADDR21 			21
#define RT_ADDR22 			22
#define RT_ADDR23 			23
#define RT_ADDR24 			24
#define RT_ADDR25 			25
#define RT_ADDR26 			26
#define RT_ADDR27 			27
#define RT_ADDR28 			28
#define RT_ADDR29 			29
#define RT_ADDR30 			30
#define RT_ADDR31 			31

//RT SUBBADDRESS MACROS
#define RT_SUBBADDR1		1
#define RT_SUBBADDR2		2
#define RT_SUBBADDR3		3
#define RT_SUBBADDR4		4
#define RT_SUBBADDR5		5
#define RT_SUBBADDR6		6
#define RT_SUBBADDR7		7
#define RT_SUBBADDR8		8
#define RT_SUBBADDR9		9
#define RT_SUBBADDR10		10
#define RT_SUBBADDR11		11
#define RT_SUBBADDR12		12
#define RT_SUBBADDR13		13
#define RT_SUBBADDR14		14
#define RT_SUBBADDR15		15
#define RT_SUBBADDR16		16
#define RT_SUBBADDR17		17
#define RT_SUBBADDR18		18
#define RT_SUBBADDR19		19
#define RT_SUBBADDR20		20
#define RT_SUBBADDR21		21
#define RT_SUBBADDR22		22
#define RT_SUBBADDR23		23
#define RT_SUBBADDR24		24
#define RT_SUBBADDR25		25
#define RT_SUBBADDR26		26
#define RT_SUBBADDR27		27
#define RT_SUBBADDR28		28
#define RT_SUBBADDR29		29
#define RT_SUBBADDR30		30

#define NO_MC						0

//BC INIT MSG LIST
#define LOOP_ENABLE			1
#define LOOP_DISABLE		0

//MSG GAP TIME
#define MS_1					0x000A
#define MS_10					0x0064
#define MS_100				0x03E8
#define MS_1000				0x2710

//BCTTAG MACROS
#define HIGH_RES			1		//High Resolution (32 bit)
#define LOW_RES				0		//Low Resolution (16 bit)

#define TTAG_2U				2<<0	// BC & RT time tag counter uses internally generated 2us clock
#define TTAG_4U				3<<0	// BC & RT time tag counter uses internally generated 4us clock
#define TTAG_8U				4<<0	// BC & RT time tag counter uses internally generated 8us clock
#define TTAG_16U			5<<0	// BC & RT time tag counter uses internally generated 16us clock
#define TTAG_32U			6<<0	// BC & RT time tag counter uses internally generated 32us clock
#define TTAG_64U			7<<0	// BC & RT time tag counter uses internally generated 64us clock



//*MT MACROS//

//MT TTAG MACROS
#define MTTAG_2U				2<<4	// MT time tag counter uses internally generated 2us clock
#define MTTAG_4U				3<<4	// MT time tag counter uses internally generated 4us clock
#define MTTAG_8U				4<<4	// MT time tag counter uses internally generated 8us clock
#define MTTAG_16U				5<<4	// MT time tag counter uses internally generated 16us clock
#define MTTAG_32U				6<<4	// MT time tag counter uses internally generated 32us clock
#define MTTAG_64U				7<<4	// MT time tag counter uses internally generated 64us clock
#define MTTAG_100N			8<<4	// MT time tag counter uses internally generated 100ns clock

//*RT MACROS//
#define RT1						1
#define RT2						2


#define SUPPORT_BRDCST					1		//Commands to RT31 are valid
#define NOSUPPORT_BRDCST				0		//Commands to RT31 are unvalid
#define UNDEF_MCODES_VALID			1		//Undefined & reserved mode commands are valid
#define UNDEF_MCODES_NOVALID		0		//Undefined & reserved mode commands are unvalid
#define USE_SMCP								1		//Mode command results are stored in descriptor table itself,not in assigned RAM buffer
#define DONT_USE_SMCP						0		//Descriptor table is initialized to store mode command results in assigned RAM buffer
#define ILLEGAL_CMD_DETECT			1		//"in form" response for all valid or unvalid commands
#define ILLEGAL_CMD_NOT_DETECT	0		//"in form" response for all valid commands

/*END OF 1553*/

/*	CMX 6022 Thermocouple Input Module Specific Defines BEGIN	*/
// Channels
#define CHANNEL0						0
#define CHANNEL1						1
#define CHANNEL2						2
#define CHANNEL3						3
#define CHANNEL4						4
#define CHANNEL5						5
#define CHANNEL6						6
#define CHANNEL7						7
// Thermocouple Types
#define CHANNEL_OFF					0
#define TYPE_B							1
#define TYPE_E							2
#define TYPE_J							3
#define TYPE_K							4
#define TYPE_N							5
#define TYPE_R							6
#define TYPE_S							7
#define TYPE_T							8
#define TYPE_C							9
#define RAW_INPUT						10

// Cold junction Types
#define BUILT_IN_CJC				0
#define CONSTANT_CJC				1

// Units
#define UNIT_CELSIUS				0
#define UNIT_FAHRENHEIT			1
#define UNIT_KELVIN					2

void CmxGetApiVersion(unsigned char *Version); 

// CMX 5010 Functions Begin
unsigned int CmxDTOpenBoard (char* IPAddress, unsigned int BoardID, unsigned int BoardReset, unsigned int *Error);
unsigned int CmxDTSetIOMode(unsigned int Handle, unsigned int Channel, unsigned int Mode, unsigned int *Error);
unsigned int CmxDTGetIOMode(unsigned int Handle, unsigned int Channel, unsigned int *IOMode, unsigned int *Error);
unsigned int CmxDTSetThreshold (unsigned int Handle, unsigned int Channel, unsigned int Type, double Threshold, unsigned int *Error);
unsigned int CmxDTGetThreshold (unsigned int Handle, unsigned int Channel, unsigned int Type, double *Threshold, unsigned int *Error);
unsigned int CmxDTSetExcitation(unsigned int Handle, unsigned int Channel, unsigned int State, unsigned int *Error);
unsigned int CmxDTGetExcitation(unsigned int Handle, unsigned int Channel, unsigned int *State, unsigned int *Error);
unsigned int CmxDTSetState(unsigned int Handle, unsigned int Channel, unsigned int State, unsigned int *Error);
unsigned int CmxDTGetState(unsigned int Handle, unsigned int Channel, unsigned int *State, unsigned int *Error);
unsigned int CmxDTSetAllState(unsigned int Handle, unsigned int AllState, unsigned int *Error);
unsigned int CmxDTGetAllState(unsigned int Handle, unsigned int *AllState, unsigned int *Error);
unsigned int CmxDTGetTime(unsigned int Handle, unsigned int *Time, unsigned int *Error);
unsigned int CmxDTGetTransition(unsigned int Handle, unsigned int Channel, unsigned int Type, unsigned int *Transition, unsigned int *Time, unsigned int *Error);
unsigned int CmxDTGetAllTransition(unsigned int Handle, unsigned int Type, unsigned int *AllTransition, unsigned int *Error);
unsigned int CmxDTGetChannelFault(unsigned int Handle, unsigned int Channel, unsigned int *Fault, unsigned int *Time, unsigned int *Error);
unsigned int CmxDTGetVoltage(unsigned int Handle, unsigned int Channel, double *Voltage, unsigned int *Error);
unsigned int CmxDTResetTime(unsigned int Handle, unsigned int *Error);
unsigned int CmxDTResetFaults(unsigned int Handle, unsigned int *Error);
unsigned int CmxDTResetTransitions(unsigned int Handle, unsigned int *Error);
unsigned int CmxDTBoardReset(unsigned int Handle, unsigned int *Error);
unsigned int CmxDTSetIP(unsigned int Handle, unsigned int NewIpAddress[4], unsigned int *Error);
unsigned int CmxDTGetIP(unsigned int Handle, unsigned int IpAddressReturn[], unsigned int *Error);
unsigned int CmxDTResetIP(unsigned int Handle, unsigned int *Error);
unsigned int CmxDTFirmwareVersion(unsigned int Handle, double *FWVersion, unsigned int *Error);
unsigned int CmxDTBoardSerialNumber(unsigned int Handle, unsigned int *SerialNo, unsigned int *Error);
unsigned int CmxDTSetBoardID(unsigned int Handle, unsigned int NewBoardID, unsigned int *Error);
unsigned int CmxDTGetBoardID(unsigned int Handle, unsigned int *ReturnBoardID, unsigned int *Error);
unsigned int CmxDTRunBIT(unsigned int Handle, unsigned int *Error);
unsigned int CmxDTGetBITResult(unsigned int Handle, unsigned int *BITResult, unsigned int *Error);
unsigned int CmxDTStopConversion(unsigned int Handle, unsigned int *Error);
unsigned int CmxDTStartConversion(unsigned int Handle, unsigned int *Error);
unsigned int CmxDTSetPWMFrequency(unsigned int Handle, unsigned int Channel, unsigned int PWMFrequency, unsigned int *Error);
unsigned int CmxDTGetPWMFrequency(unsigned int Handle, unsigned int Channel, unsigned int *PWMFrequency, unsigned int *Error);
unsigned int CmxDTSetPWMDuty(unsigned int Handle, unsigned int Channel, double PWMDuty, unsigned int *Error);
unsigned int CmxDTGetPWMDuty(unsigned int Handle, unsigned int Channel, double *PWMDuty, unsigned int *Error);
unsigned int CmxDTRunPWM(unsigned int Handle, unsigned int Channel, unsigned int *Error);
unsigned int CmxDTStopPWM(unsigned int Handle, unsigned int Channel, unsigned int *Error);
unsigned int CmxDTSetVccLevel(unsigned int Handle, double VccLevel, unsigned int *Error);
unsigned int CmxDTGetVccLevel(unsigned int Handle, double *VccLevel, unsigned int *Error);
unsigned int CmxDTSetMAC(unsigned int Handle, unsigned int NewMAC[6], unsigned int *Error);
unsigned int CmxDTGetMAC(unsigned int Handle, unsigned int ReturnMac[], unsigned int *Error);
unsigned int CmxDTSetDebounce(unsigned int Handle, unsigned int Channel, unsigned int DebounceTime, unsigned int *Error);
unsigned int CmxDTGetDebounce(unsigned int Handle, unsigned int Channel, unsigned int *DebounceTime, unsigned int *Error);
unsigned int CmxDTSetSerial(unsigned int Handle, unsigned int SerialNo, unsigned int *Error);
unsigned int CmxDTSetFirmware(unsigned int Handle, double Firmware, unsigned int *Error);
unsigned int CmxDTRunSelfCalibration(unsigned int Handle, unsigned int *Error);
unsigned int CmxDTSetScaleGain(unsigned int Handle, unsigned int Channel, double ScaleGain, unsigned int *Error);
unsigned int CmxDTGetScaleGain(unsigned int Handle, unsigned int Channel, double *ScaleGain, unsigned int *Error);
unsigned int CmxDTSetScaleOffset(unsigned int Handle, unsigned int Channel, double ScaleOffset, unsigned int *Error);
unsigned int CmxDTGetScaleOffset(unsigned int Handle, unsigned int Channel, double *ScaleOffset, unsigned int *Error);
unsigned int CmxDTGetADCGain(unsigned int Handle, unsigned int Channel, unsigned int *ADCGain, unsigned int *Error);
unsigned int CmxDTGetADCOffset(unsigned int Handle, unsigned int Channel, unsigned int *ADCOffset, unsigned int *Error);
unsigned int CmxDTDiscardBoard (unsigned int Handle, unsigned int *Error);
// CMX 5010 Functions End

// CMX 6012 and CMX 6012C Functions Begin
unsigned int CmxADOpenBoard (char* IPAddress, unsigned int BoardID, unsigned int BoardReset, unsigned int *Error);
unsigned int CmxADSetIOMode (unsigned int Handle, unsigned int Channel, unsigned int Mode, unsigned int InputConfiguration, unsigned int *Error);
unsigned int CmxADGetIOMode (unsigned int Handle, unsigned int Channel, unsigned int *IOMode, unsigned int *Error);
unsigned int CmxADSetSampleRate (unsigned int Handle, unsigned int SampleRate, unsigned int *Error);
unsigned int CmxADGetSampleRate (unsigned int Handle, unsigned int *SampleRate, unsigned int *Error);
unsigned int CmxADSetRange (unsigned int Handle, unsigned int Channel, unsigned int Range, unsigned int *Error);
unsigned int CmxADGetRange (unsigned int Handle, unsigned int Channel, unsigned int *Range, unsigned int *Error);
unsigned int CmxADSetTrigger (unsigned int Handle, unsigned int TriggerType, unsigned int *Error);
unsigned int CmxADGetTrigger (unsigned int Handle, unsigned int *TriggerType, unsigned int *Error);
unsigned int CmxADStartConversion (unsigned int Handle, unsigned int *Error);
unsigned int CmxADReadBuffer (unsigned int Handle, unsigned int TimeOut, unsigned int NumberOfSamples, double *Buffer, unsigned int *FetchedSampleCount, unsigned int *Error);
unsigned int CmxADSetGain (unsigned int Handle, unsigned int Channel, unsigned int Range, double Gain, unsigned int *Error);
unsigned int CmxADGetGain (unsigned int Handle, unsigned int Channel, unsigned int Range, double *Gain, unsigned int *Error);
unsigned int CmxADSetOffset (unsigned int Handle, unsigned int Channel, unsigned int Range, double Offset, unsigned int *Error);
unsigned int CmxADGetOffset (unsigned int Handle, unsigned int Channel, unsigned int Range, double *Offset, unsigned int *Error);
unsigned int CmxADSetGain2 (unsigned int Handle, unsigned int Channel, unsigned int Range, double Gain, unsigned int *Error);
unsigned int CmxADGetGain2 (unsigned int Handle, unsigned int Channel, unsigned int Range, double *Gain, unsigned int *Error);
unsigned int CmxADSetOffset2 (unsigned int Handle, unsigned int Channel, unsigned int Range, double Offset, unsigned int *Error);
unsigned int CmxADGetOffset2 (unsigned int Handle, unsigned int Channel, unsigned int Range, double *Offset, unsigned int *Error);
unsigned int CmxADStopConversion (unsigned int Handle, unsigned int *Error);
unsigned int CmxADDiscardBoard (unsigned int Handle, unsigned int *Error);
// CMX 6012 and CMX 6012C Functions End

// CMX 4010, CMX 4011 and CMX 4010C Functions Begin
unsigned int CmxDAOpenBoard (char* IPAddress, unsigned int BoardID, unsigned int BoardReset, unsigned int *Error);
unsigned int CmxDASetReadBack (unsigned int Handle, unsigned int ReadBackOnOff, unsigned int *Error);
unsigned int CmxDAGetReadBack (unsigned int Handle, unsigned int *ReadBack, unsigned int *Error);
unsigned int CmxDASetChannelMode (unsigned int Handle, unsigned int Channel, unsigned int Mode, unsigned int *Error);
unsigned int CmxDAGetChannelMode (unsigned int Handle, unsigned int Channel, unsigned int *ChannelMode, unsigned int *Error);
unsigned int CmxDASetRange (unsigned int Handle, unsigned int Channel, unsigned int Range, unsigned int *Error);
unsigned int CmxDAGetRange (unsigned int Handle, unsigned int Channel, unsigned int *Range, unsigned int *Error);
unsigned int CmxDASetUpdateMode (unsigned int Handle, unsigned int UpdateMode, unsigned int *Error);
unsigned int CmxDAGetUpdateMode (unsigned int Handle, unsigned int *UpdateMode, unsigned int *Error);
unsigned int CmxDAUpdateOutput (unsigned int Handle, unsigned int *Error);
unsigned int CmxDASetBroadcast (unsigned int Handle, unsigned int BroadcastOnOff, unsigned int *Error);
unsigned int CmxDAGetBroadcast (unsigned int Handle, unsigned int *Broadcast, unsigned int *Error);
unsigned int CmxDASetBroadcastData (unsigned int Handle, double BroadcastData, unsigned int *Error);
unsigned int CmxDAGetBroadcastData (unsigned int Handle, double *BroadcastData, unsigned int *Error);
unsigned int CmxDASetGain (unsigned int Handle, unsigned int Channel,unsigned int Range, double Gain, unsigned int *Error);
unsigned int CmxDAGetGain (unsigned int Handle, unsigned int Channel,unsigned int Range, double *Gain, unsigned int *Error);
unsigned int CmxDASetOffset (unsigned int Handle, unsigned int Channel,unsigned int Range, double Offset, unsigned int *Error);
unsigned int CmxDAGetOffset (unsigned int Handle, unsigned int Channel,unsigned int Range, double *Offset, unsigned int *Error);
unsigned int CmxDAStart (unsigned int Handle, unsigned int *Error);
unsigned int CmxDAStop (unsigned int Handle, unsigned int *Error);
unsigned int CmxDASetChannelData (unsigned int Handle, unsigned int Channel, double OutputVoltage, unsigned int *Error);
unsigned int CmxDAGetChannelData (unsigned int Handle, unsigned int Channel, double *OutputVoltage, unsigned int *Error);
unsigned int CmxDASetIP (unsigned int Handle, unsigned int NewIpAddress[4], unsigned int *Error);
unsigned int CmxDAGetIP (unsigned int Handle, unsigned int IpAddressReturn[], unsigned int *Error);
unsigned int CmxDAResetIP (unsigned int Handle, unsigned int *Error);
unsigned int CmxDABoardReset (unsigned int Handle, unsigned int *Error);
unsigned int CmxDASetFirmware (unsigned int Handle, double Firmware, unsigned int *Error);
unsigned int CmxDAGetFirmware (unsigned int Handle, double *FWVersion, unsigned int *Error);
unsigned int CmxDASetSerialNo (unsigned int Handle, unsigned int SerialNo, unsigned int *Error);
unsigned int CmxDAGetSerialNo (unsigned int Handle, unsigned int *SerialNo, unsigned int *Error);
unsigned int CmxDASetMAC (unsigned int Handle, unsigned int NewMAC[6], unsigned int *Error);
unsigned int CmxDAGetMAC (unsigned int Handle, unsigned int ReturnMac[], unsigned int *Error);
unsigned int CmxDASetGenMode (unsigned int Handle, unsigned int Channel, unsigned int GenerationMode, unsigned int *Error);
unsigned int CmxDASetBufferSize (unsigned int Handle, unsigned int Channel, unsigned int BufferSize, unsigned int *Error);
unsigned int CmxDAWriteArrayPoint (unsigned int Handle, unsigned int Channel, unsigned int ArrayIndex, double Data, unsigned int *Error);
unsigned int CmxDASetUpdateTime (unsigned int Handle, unsigned int TimeinMilliSecond, unsigned int *Error);
unsigned int CmxDADiscardBoard (unsigned int Handle, unsigned int *Error);
// CMX 4010, CMX 4011 and CMX 4010C Functions End

// Chassis Functions Begin
unsigned int CmxOpenChassis (char* IpAddress, unsigned int *Error);
unsigned int CmxSetChassisIP (unsigned int ChassisHandle, unsigned int NewIpAddress[4], unsigned int *Error);
unsigned int CmxGetChassisIP (unsigned int ChassisHandle, unsigned int IpAddressReturn[], unsigned int *Error);
unsigned int CmxSetChassisSubMask (unsigned int ChassisHandle, unsigned int NewSubMask[4], unsigned int *Error);
unsigned int CmxGetChassisSubMask (unsigned int ChassisHandle, unsigned int SubMaskReturn[], unsigned int *Error);
unsigned int CmxSetChassisGateway (unsigned int ChassisHandle, unsigned int NewGateway[4], unsigned int *Error);
unsigned int CmxGetChassisGateway (unsigned int ChassisHandle, unsigned int GateWayReturn[], unsigned int *Error);
unsigned int CmxSetModuleName (unsigned int ChassisHandle, unsigned int SlotNumber, char* ModuleName, unsigned int *Error);
unsigned int CmxGetModuleName (unsigned int ChassisHandle, unsigned int SlotNumber, unsigned int *ModuleType, unsigned int *Error);
unsigned int CmxDeploymentStatus (unsigned int ChassisHandle, unsigned int *DeploymentStatus, unsigned int *Error);
unsigned int CmxDiscardChassis(unsigned int ChassisHandle, unsigned int *Error);
unsigned int CmxRebootSystem (char* IpAddress, unsigned int *Error);
unsigned int CmxDebugChassis (unsigned int ChassisIP[], unsigned int *Error);
// Chassis Functions End

//CMX 1553 Functions Begin
unsigned int Cmx1553OpenBoard (char* IPAddress, unsigned int BoardID, unsigned int BoardReset, unsigned int *Error);
unsigned int Cmx1553InitBoard (unsigned int Handle, unsigned int *Error);
unsigned int Cmx1553ResetBoard (unsigned int Handle, unsigned int *Error);
unsigned int Cmx1553BoardSetMode (unsigned int Handle,unsigned int BcMode,unsigned int Rt1Mode, unsigned int Rt2Mode,unsigned int MtMode, unsigned int *Error);
unsigned int Cmx1553BoardGetMode (unsigned int Handle, unsigned int *Mode,unsigned int *Error);
unsigned int Cmx1553ResetIC (unsigned int Handle, unsigned int *Error);
unsigned int Cmx1553SharedInit (unsigned int Handle, unsigned int *Error);
unsigned int Cmx1553BcInitMsg (unsigned int Handle, unsigned int MessageType,unsigned int Index, unsigned int DataCount, unsigned int *Data, unsigned short ControlWord, unsigned int RtAddr,unsigned int RtSubaddr,unsigned int Mcode,  unsigned int *Error);
unsigned int Cmx1553BcUpdateMsg (unsigned int Handle, unsigned int MessageType,unsigned int Index,unsigned int DataCount,unsigned int *Data, unsigned short ControlWord,unsigned int RtAddr,unsigned int RtSubaddr,unsigned int Mcode,  unsigned int *Error);
unsigned int Cmx1553BcInitMsgList (unsigned int Handle,unsigned int Mode, unsigned int Loop, unsigned int MsgGapTime, unsigned int PeriodUs, unsigned int *Error);
unsigned int Cmx1553BcSetTTAG (unsigned int Handle,unsigned int HiLo, unsigned int BcTTAG, unsigned int *Error);
unsigned int Cmx1553BcGetMsgInProgress (unsigned int Handle, unsigned int *MsgProgress, unsigned int *Error);
unsigned int Cmx1553BcGetMsgBlock (unsigned int Handle, unsigned int *MsgBlock, unsigned int *Error);
unsigned int Cmx1553BcGetMsgData (unsigned int Handle, unsigned int *MsgData, unsigned int *Error);
unsigned int Cmx1553BcEnable (unsigned int Handle, unsigned int *Error);
unsigned int Cmx1553BcStop (unsigned int Handle, unsigned int *Error);
unsigned int Cmx1553BcTrigger (unsigned int Handle, unsigned int *Error);
unsigned int Cmx1553BcStart (unsigned int Handle, unsigned int *Error);
unsigned int Cmx1553MtSetTTAG (unsigned int Handle,unsigned int HiLo,unsigned int MTTag,unsigned int *Error);
unsigned int Cmx1553MtSetFilter (unsigned int Handle, unsigned int *Error);
unsigned int Cmx1553MtGetLastCommand (unsigned int Handle,unsigned int *LastCommand, unsigned int *Error);
unsigned int Cmx1553MtGetLastMessage (unsigned int Handle, unsigned int *LastMessage, unsigned int *Error);
unsigned int Cmx1553MtEnable (unsigned int Handle, unsigned int *Error);
unsigned int Cmx1553MtStop (unsigned int Handle, unsigned int *Error);
unsigned int Cmx1553Rt1Init (unsigned int Handle, unsigned int SupportBroadcast,unsigned int UndefMcodesValid,unsigned int UseSmcp, unsigned int *Error);
unsigned int Cmx1553Rt1Init1 (unsigned int Handle,unsigned int UseSmcp, unsigned int *Error);
unsigned int Cmx1553Rt1Init2 (unsigned int Handle, unsigned int IllegalCmdDetect,unsigned int *Error);
unsigned int Cmx1553Rt1SetAddr (unsigned int Handle,unsigned int Address, unsigned int *Error);
unsigned int Cmx1553Rt1SetTTAG (unsigned int Handle,unsigned int RT1ttag ,unsigned int *Error);
unsigned int Cmx1553Rt1SetTXData (unsigned int Handle,unsigned int *Rt1TxData,unsigned int DataCount, unsigned int *Error);
unsigned int Cmx1553Rt1Enable (unsigned int Handle, unsigned int *Error);
unsigned int Cmx1553Rt1Stop (unsigned int Handle, unsigned int *Error);
unsigned int Cmx1553Rt1Start (unsigned int Handle, unsigned int *Error);
unsigned int Cmx1553Rt2Init (unsigned int Handle, unsigned int SupportBroadcast,unsigned int UndefMcodesValid,unsigned int UseSmcp, unsigned int *Error);
unsigned int Cmx1553Rt2Init1 (unsigned int Handle,unsigned int UseSmcp, unsigned int *Error);
unsigned int Cmx1553Rt2Init2 (unsigned int Handle, unsigned int IllegalCmdDetect,unsigned int *Error);
unsigned int Cmx1553Rt2SetAddr (unsigned int Handle,unsigned int Address, unsigned int *Error);
unsigned int Cmx1553Rt2SetTTAG (unsigned int Handle,unsigned int RT1ttag ,unsigned int *Error);
unsigned int Cmx1553Rt2SetTXData (unsigned int Handle,unsigned int *Rt2Txdata,unsigned int DataCount, unsigned int *Error);
unsigned int Cmx1553Rt2Enable (unsigned int Handle, unsigned int *Error);
unsigned int Cmx1553Rt2Stop (unsigned int Handle, unsigned int *Error);
unsigned int Cmx1553Rt2Start (unsigned int Handle, unsigned int *Error);
unsigned int Cmx1553RtGetMessage (unsigned int Handle,unsigned int *RTmessage,unsigned int RtNum,unsigned int *Error);
unsigned int Cmx1553RtCheckAddr (unsigned int Handle,unsigned int RtNum, unsigned int *Error);
unsigned int Cmx1553SetIp (unsigned int Handle, unsigned int ipaddress[4], unsigned int *Error);
unsigned int Cmx1553GetIp (unsigned int Handle, unsigned int *IpReturn, unsigned int *Error);
unsigned int Cmx1553ResetIp (unsigned int Handle, unsigned int *Error);
unsigned int Cmx1553SetMac (unsigned int Handle, unsigned int NewMAC[6], unsigned int *Error);
unsigned int Cmx1553GetMac (unsigned int Handle, unsigned int *ReturnMAC, unsigned int *Error);
unsigned int Cmx1553GetSerial (unsigned int Handle, unsigned int *SerialNo, unsigned int *Error);
unsigned int Cmx1553GetFirmware (unsigned int Handle, double *FwVersion, unsigned int *Error);
unsigned int Cmx1553DiscardBoard (unsigned int Handle, unsigned int *Error);
//CMX 1553 Functions End

//CMX 5015 and CMX 5020 Functions Begin
unsigned int CmxMTOpenBoard(char* IPAddress, unsigned int BoardID, unsigned int BoardReset, unsigned int *Error);
unsigned int CmxMTSetChannel(unsigned int Handle, TimerChannel Channel,unsigned int PreScaler,unsigned int Period,unsigned int *Error);
unsigned int CmxMTGetChannel(unsigned int Handle,  unsigned int *Mode, unsigned int *Error);
unsigned int CmxMTStartChannel(unsigned int Handle, TimerChannel  Channel,unsigned int Pulse,unsigned int EndState,unsigned int *Error);
unsigned int CmxMTStopChannel(unsigned int Handle,  TimerChannel  Channel,  unsigned int *Error);
unsigned int CmxMTUpdateChannel(unsigned int Handle, TimerChannel Channel, unsigned int PreScaler,unsigned int Period,unsigned int *Error);
unsigned int CmxMTSetIp (unsigned int Handle, unsigned int IpAddress[4], unsigned int *Error);
unsigned int CmxMTGetIp (unsigned int Handle, unsigned int *IpReturn, unsigned int *Error);
unsigned int CmxMTResetIp (unsigned int Handle, unsigned int *Error);
unsigned int CmxMTSetMAC (unsigned int Handle, unsigned int MACAddress[6], unsigned int *Error);
unsigned int CmxMTGetMAC (unsigned int Handle, unsigned int *ReturnMAC, unsigned int *Error);
unsigned int CmxMTGetSerial (unsigned int Handle, unsigned int *SerialNo, unsigned int *Error);
unsigned int CmxMTGetFirmware (unsigned int Handle, double *FwVersion, unsigned int *Error);
unsigned int CmxMTDiscardBoard (unsigned int Handle, unsigned int *Error);
//CMX 5015 and CMX 5020 Functions End

// CMX 6022 Functions Begin
unsigned int CmxTCOpenBoard (char* IPAddress, unsigned int BoardID, unsigned int BoardReset, unsigned int *Error);
unsigned int CmxTCSetType (unsigned int Handle, unsigned int Channel, unsigned int TCType, unsigned int *Error);
unsigned int CmxTCGetType (unsigned int Handle, unsigned int Channel, unsigned int *TCType, unsigned int *Error);
unsigned int CmxTCSetCjCChannel (unsigned int Handle, unsigned int Channel, unsigned int CjcChannel, unsigned int *Error);
unsigned int CmxTCGetCjcChannel (unsigned int Handle, unsigned int Channel, unsigned int *CjcChannel, unsigned int *Error);
unsigned int CmxTCSetTemperatureUnit (unsigned int Handle, unsigned int TemperatureUnit, unsigned int *Error);
unsigned int CmxTCGetTemperatureUnit (unsigned int Handle, unsigned int *TemperatureUnit, unsigned int *Error);
unsigned int CmxTCSetTrigger (unsigned int Handle, unsigned int TriggerType, unsigned int *Error);
unsigned int CmxTCGetTrigger (unsigned int Handle, unsigned int *TriggerType, unsigned int *Error);
unsigned int CmxTCStartConversion (unsigned int Handle, unsigned int *Error);
unsigned int CmxTCStopConversion (unsigned int Handle, unsigned int *Error);
unsigned int CmxTCReadBuffer (unsigned int Handle, unsigned int TimeOut, unsigned int NumberOfSamples, double *Buffer, unsigned int *FetchedSampleCount, unsigned int *Error);
unsigned int CmxTCSetCjcValue (unsigned int Handle, unsigned int Channel, double CjcTemperatureValue, unsigned int *Error);
unsigned int CmxTCGetCjcValue (unsigned int Handle, double *CjcTemperatureValue, unsigned int *Error);
unsigned int CmxTCSetGain (unsigned int Handle, unsigned int Channel, double Gain, unsigned int *Error);
unsigned int CmxTCGetGain (unsigned int Handle, unsigned int Channel, double *Gain, unsigned int *Error);
unsigned int CmxTCSetOffset (unsigned int Handle, unsigned int Channel, double Offset, unsigned int *Error);
unsigned int CmxTCGetOffset (unsigned int Handle, unsigned int Channel, double *Offset, unsigned int *Error);
unsigned int CmxTCSetTemperatureGain (unsigned int Handle, unsigned int Channel, unsigned int Type, double TemperatureGain, unsigned int *Error);
unsigned int CmxTCSetTemperatureOffset (unsigned int Handle, unsigned int Channel, unsigned int Type, double TemperatureOffset, unsigned int *Error);
unsigned int CmxTCDiscardBoard (unsigned int Handle, unsigned int *Error);
// CMX 6022 Functions End

//CMX 485HS Functions Begin
unsigned int CmxSCOpenBoard (char* IPAddress, unsigned int BoardID, unsigned int BoardReset, unsigned int *Error);
unsigned int CmxSCSetBaud (unsigned int Handle, unsigned int Port,unsigned int BaudRate , unsigned int *Error);
unsigned int CmxSCSetPort (unsigned int Handle, unsigned int Port,unsigned int WordLength, unsigned int StopBit,  unsigned int Parity,unsigned int WireMode, unsigned int Termination,  unsigned int *Error);
unsigned int CmxSCGetPort (unsigned int Handle, unsigned int Port,unsigned int *BaudRate,unsigned int *WordLength, unsigned int *StopBit,  unsigned int *Parity,unsigned int* WireMode, unsigned int *Termination,  unsigned int *Error);
unsigned int CmxSCSetEcho (unsigned int Handle, unsigned int Port, unsigned int Echo, unsigned int *Error);
unsigned int CmxSCGetEcho (unsigned int Handle, unsigned int Port, unsigned int *Echo, unsigned int *Error);
unsigned int CmxSCSetLoopBack (unsigned int Handle, unsigned int Port, unsigned int LoopBack, unsigned int *Error);
unsigned int CmxSCGetLoopBack (unsigned int Handle, unsigned int Port, unsigned int *LoopBack, unsigned int *Error);
unsigned int CmxSCStart (unsigned int Handle, unsigned int *Error);
unsigned int CmxSCTransmit (unsigned int Handle, unsigned int Port,  unsigned int DataCount,unsigned char *TXPacket, unsigned int *Error);
unsigned int CmxSCReceive (unsigned int Handle, unsigned int Port, unsigned int DataCount, unsigned int TimeOut, unsigned char *RXPacket, unsigned int *ReturnByteCount, unsigned int *Error);
unsigned int CmxSCGetByteCount (unsigned int Handle,unsigned int Port,unsigned int *ByteCount, unsigned int *Error);
unsigned int CmxSCStop (unsigned int Handle,unsigned int Port, unsigned int *Error);
unsigned int CmxSCFlushBuffer (unsigned int Handle, unsigned int Port, unsigned int *Error);
unsigned int CmxSCSetIp (unsigned int Handle, unsigned int IpAddress[4], unsigned int *Error);
unsigned int CmxSCGetIp (unsigned int Handle, unsigned int *IpReturn, unsigned int *Error);
unsigned int CmxSCResetIp (unsigned int Handle, unsigned int *Error);
unsigned int CmxSCSetMAC (unsigned int Handle, unsigned int MACAddress[6], unsigned int *Error);
unsigned int CmxSCGetMAC (unsigned int Handle, unsigned int *ReturnMAC, unsigned int *Error);
unsigned int CmxSCGetSerial (unsigned int Handle, unsigned int *SerialNo, unsigned int *Error);
unsigned int CmxSCGetFirmware (unsigned int Handle, double *FwVersion, unsigned int *Error);
unsigned int CmxSCDiscardBoard (unsigned int Handle, unsigned int *Error);
//CMX 485HS Functions End
#ifdef __cplusplus
    }
#endif

#endif  /* ndef __CMXAPI_H__ */
