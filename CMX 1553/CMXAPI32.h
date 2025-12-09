//==============================================================================
//
// Title:		CMXAPI
// Purpose:		This library was developed to interface with Konaka Defence CMX hardware platform.
//
// Released on:	01/12/2021  by Konaka Defence
// Copyright:	. All Rights Reserved.
//
//==============================================================================

#ifndef __CMXAPI_H__
#define __CMXAPI_H__

#ifdef __cplusplus
    extern "C" {
#endif

typedef enum{
	API_TIMER_CHANNEL_0=0,
	API_TIMER_CHANNEL_1,
	API_TIMER_CHANNEL_2,
	API_TIMER_CHANNEL_3,
	API_TIMER_CHANNEL_4,
	API_TIMER_CHANNEL_5
}TimerChannelEnum;
	

unsigned int DTOpenBoard (unsigned int Interface, unsigned int Port, unsigned int IpAddress[4], unsigned int BoardID, unsigned int *Error);
unsigned int DTSetIOMode(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int Channel, unsigned int Mode, unsigned int *Error); 
unsigned int DTGetIOMode(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle,unsigned int BoardID, unsigned int Channel, unsigned int *IOMode, unsigned int *Error);
unsigned int DTSetThreshold(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int Channel, unsigned int Type, double Threshold, unsigned int *Error);
unsigned int DTGetThreshold(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int Channel, unsigned int Type, double *Threshold, unsigned int *Error);
unsigned int DTSetExcitation(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int Channel, unsigned int State, unsigned int *Error);
unsigned int DTGetExcitation(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int Channel, unsigned int *State, unsigned int *Error);
unsigned int DTSetState(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int Channel, unsigned int State, unsigned int *Error);
unsigned int DTGetState(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int Channel, unsigned int *State, unsigned int *Error);
unsigned int DTSetAllState(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int AllState, unsigned int *Error);
unsigned int DTGetAllState(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int *AllState, unsigned int *Error);   
unsigned int DTGetTime(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int *Time, unsigned int *Error);
unsigned int DTGetTransition(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int Channel, unsigned int Type, unsigned int *Transition, unsigned int *Time, unsigned int *Error);
unsigned int DTGetAllTransition(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int Type, unsigned int *AllTransition, unsigned int *Error);
unsigned int DTGetChannelFault(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int Channel, unsigned int *Fault,unsigned int *Time, unsigned int *Error);
unsigned int DTGetVoltage(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int Channel, double *Voltage, unsigned int *Error);
unsigned int DTResetTime(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int *Error);
unsigned int DTResetFaults(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int *Error);
unsigned int DTResetTransitions(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int *Error);
unsigned int DTBoardReset(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int *Error);
unsigned int DTSetIP(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int NewIpAddress[4], unsigned int *Error);
unsigned int DTGetIP(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int IpAddressReturn[], unsigned int *Error); 
unsigned int DTResetIP(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int *Error);
unsigned int DTFirmwareVersion(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, double *FWVersion, unsigned int *Error);
unsigned int DTBoardSerialNumber(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int *SerialNo, unsigned int *Error);
unsigned int DTSetBoardID(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int NewBoardID, unsigned int *Error);
unsigned int DTGetBoardID(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int *ReturnBoardID, unsigned int *Error);
unsigned int DTRunBIT(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int *Error);
unsigned int DTGetBITResult(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int *BITResult, unsigned int *Error);
unsigned int DTStopConversion(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int *Error);
unsigned int DTStartConversion(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int *Error);
unsigned int DTSetPWMFrequency(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int Channel, unsigned int PWMFrequency, unsigned int *Error);
unsigned int DTGetPWMFrequency(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int Channel, unsigned int *PWMFrequency, unsigned int *Error);
unsigned int DTSetPWMDuty(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int Channel, double PWMDuty, unsigned int *Error);
unsigned int DTGetPWMDuty(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int Channel, double *PWMDuty, unsigned int *Error);
unsigned int DTRunPWM(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int Channel, unsigned int *Error);
unsigned int DTStopPWM(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int Channel, unsigned int *Error);
unsigned int DTSetVccLevel(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, double VccLevel, unsigned int *Error);
unsigned int DTGetVccLevel(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, double *VccLevel, unsigned int *Error);
unsigned int DTSetMAC(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int NewMAC[6], unsigned int *Error);
unsigned int DTGetMAC(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int ReturnMAC[], unsigned int *Error);
unsigned int DTSetDebounce(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int Channel, unsigned int DebounceTime, unsigned int *Error);
unsigned int DTGetDebounce(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int Channel, unsigned int *DebounceTime, unsigned int *Error); 
unsigned int DTSetSerial(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int SerialNo, unsigned int *Error);
unsigned int DTSetFirmware(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, double Firmware, unsigned int *Error); 
unsigned int DTRunSelfCalibration(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int *Error);
unsigned int DTSetScaleGain(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int Channel, double ScaleGain, unsigned int *Error);
unsigned int DTGetScaleGain(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int Channel, double *ScaleGain, unsigned int *Error);
unsigned int DTSetScaleOffset(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int Channel, double ScaleOffset, unsigned int *Error);
unsigned int DTGetScaleOffset(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int Channel, double *ScaleOffset, unsigned int *Error);
unsigned int DTGetADCGain(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int Channel, unsigned int *ADCGain, unsigned int *Error);
unsigned int DTGetADCOffset(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int Channel, unsigned int *ADCOffset, unsigned int *Error);
unsigned int DTDiscardBoard(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int *Error);

unsigned int ADOpenBoard(unsigned int Interface, unsigned int Port, unsigned int IpAddress[4], unsigned int BoardID, unsigned int *Error);
unsigned int ADSetIOMode(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int Channel, unsigned int Mode, unsigned int InputConfiguration, unsigned int *Error);
unsigned int ADGetIOMode(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle,unsigned int BoardID, unsigned int Channel, unsigned int *IOMode, unsigned int *Error);
unsigned int ADSetSampleRate(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int SampleRate, unsigned int *Error);
unsigned int ADGetSampleRate(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int *SampleRate, unsigned int *Error);
unsigned int ADSetRange(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int Channel, unsigned int Range, unsigned int *Error);
unsigned int ADGetRange(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int Channel, unsigned int *Range, unsigned int *Error);
unsigned int ADSetCjCChannel(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int CjcChannel, unsigned int *Error);
unsigned int ADGetCjCChannel(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int *CjcChannel, unsigned int *Error);
unsigned int ADSetRTDExcCurrent(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int ExcCurrent, unsigned int *Error);
unsigned int ADGetRTDExcCurrent(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int *ExcCurrent, unsigned int *Error);
unsigned int ADSetTemperatureUnit(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int TemperatureUnit, unsigned int *Error);
unsigned int ADGetTemperatureUnit(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int *TemperatureUnit, unsigned int *Error);
unsigned int ADSetTCOC(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int OCDetect, unsigned int *Error);
unsigned int ADGetTCOC(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int *OCDetect, unsigned int *Error);
unsigned int ADGetChannelFault(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int Channel, unsigned int *ChannelFault, unsigned int *Error);
unsigned int ADGetCjCValue(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, double *CjC, unsigned int *Error);
unsigned int ADSetFilter(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int Filter, unsigned int *Error);
unsigned int ADGetFilter(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int *Filter, unsigned int *Error);
unsigned int ADSetTrigger(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int TriggerType, unsigned int *Error);
unsigned int ADGetTrigger(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle,unsigned int BoardID, unsigned int *TriggerType, unsigned int *Error);
unsigned int ADGetADCFault(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int *Fault, unsigned int *Error);
unsigned int ADSetFIFOEnable(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int Channel, unsigned int Num_of_Samples, unsigned int *Error);
unsigned int ADSetFIFODisable(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle,unsigned int BoardID, unsigned int Channel, unsigned int *Error);
unsigned int ADStartConversion(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int *Error);
unsigned int ADStopConversion(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int *Error);
unsigned int ADGetSample(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int Channel, double *Sample, unsigned int *Error);
unsigned int ADGetAllSample(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, double Sample[16], unsigned int *Error);
unsigned int ADGetFIFO(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int Channel, unsigned int Timeout, double FIFO[50], unsigned int *FIFOCount, unsigned int *Error);
unsigned int ADSetIP(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int NewIpAddress[4], unsigned int *Error);
unsigned int ADGetIP(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int IpAddressReturn[], unsigned int *Error);
unsigned int ADResetIP(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int *Error);
unsigned int ADBoardReset(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int *Error);
unsigned int ADFirmwareVersion(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, double *FWVersion, unsigned int *Error);
unsigned int ADBoardSerialNumber(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int *SerialNo, unsigned int *Error);
unsigned int ADSetBoardID(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int NewBoardID, unsigned int *Error);
unsigned int ADGetBoardID(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int *ReturnBoardID, unsigned int *Error);
unsigned int ADSetMAC(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int NewMAC[6], unsigned int *Error);
unsigned int ADGetMAC(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int ReturnMac[], unsigned int *Error);
unsigned int ADSetSerial(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int SerialNo, unsigned int *Error);
unsigned int ADSetFirmware(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, double Firmware, unsigned int *Error);
unsigned int ADRunSelfCalibration(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int *Error);
unsigned int ADSetGain(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int Channel, double Gain, unsigned int *Error);
unsigned int ADGetGain(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int Channel, double *Gain, unsigned int *Error);
unsigned int ADSetOffset(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int Channel, double Offset, unsigned int *Error);
unsigned int ADGetOffset(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int Channel, double *Offset, unsigned int *Error);
unsigned int ADDiscardBoard(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int *Error);

unsigned int DAOpenBoard(unsigned int Interface, unsigned int Port, unsigned int IpAddress[4], unsigned int BoardID, unsigned int *Error);
unsigned int DASetReadBack(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int ReadBackOnOff, unsigned int *Error);
unsigned int DAGetReadBack(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle,unsigned int BoardID, unsigned int *ReadBack, unsigned int *Error);
unsigned int DASetChannelMode(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int Channel, unsigned int Mode, unsigned int *Error);
unsigned int DAGetChannelMode(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle,unsigned int BoardID, unsigned int Channel, unsigned int *ChannelMode, unsigned int *Error);
unsigned int DASetRange(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int Channel, unsigned int Range, unsigned int *Error);
unsigned int DAGetRange(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle,unsigned int BoardID, unsigned int Channel, unsigned int *Range, unsigned int *Error);
unsigned int DASetUpdateMode(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int UpdateMode, unsigned int *Error);
unsigned int DAGetUpdateMode(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle,unsigned int BoardID, unsigned int *UpdateMode, unsigned int *Error);
unsigned int DAUpdateOutput(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int *Error);
unsigned int DASetBroadcast(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int BroadcastOnOff, unsigned int *Error);
unsigned int DAGetBroadcast(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle,unsigned int BoardID, unsigned int *Broadcast, unsigned int *Error);
unsigned int DASetBroadcastData(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, double BroadcastData, unsigned int *Error);
unsigned int DAGetBroadcastData(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, double *BroadcastData, unsigned int *Error);
unsigned int DASetGain(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int Channel, double Gain, unsigned int *Error);
unsigned int DAGetGain(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle,unsigned int BoardID, unsigned int Channel, double *Gain, unsigned int *Error);
unsigned int DASetOffset(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int Channel, double Offset, unsigned int *Error);
unsigned int DAGetOffset(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle,unsigned int BoardID, unsigned int Channel, double *Offset, unsigned int *Error);
unsigned int DAStart(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int *Error);
unsigned int DAStop(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int *Error);
unsigned int DASetChannelData(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int Channel, double OutputVoltage, unsigned int *Error);
unsigned int DAGetChannelData(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int Channel, double *OutputVoltage, unsigned int *Error);
unsigned int DASetIP(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int NewIpAddress[4], unsigned int *Error);
unsigned int DAGetIP(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int IpAddressReturn[], unsigned int *Error);
unsigned int DAResetIP(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int *Error);
unsigned int DABoardReset(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int *Error);
unsigned int DAFirmwareVersion(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, double *FWVersion, unsigned int *Error);
unsigned int DABoardSerialNumber(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int *SerialNo, unsigned int *Error);
unsigned int DASetBoardID(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int NewBoardID, unsigned int *Error);
unsigned int DAGetBoardID(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int *ReturnBoardID, unsigned int *Error);
unsigned int DASetMAC(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int NewMAC[6], unsigned int *Error);
unsigned int DAGetMAC(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int ReturnMac[], unsigned int *Error);
unsigned int DASetSerial(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int SerialNo, unsigned int *Error);
unsigned int DASetFirmware(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, double Firmware, unsigned int *Error);
unsigned int DADiscardBoard(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int *Error);

//CMX1553 Functions
unsigned int CM1553OpenBoard (unsigned int Interface, unsigned int Port,unsigned int IpAddress[4], unsigned int BoardID, unsigned int *Error);
unsigned int CM1553InitBoard (unsigned int Interface, unsigned int IpAddress[4],unsigned int Handle,unsigned int BoardID, unsigned int *Error);
unsigned int CM1553ResetBoard (unsigned int Interface, unsigned int IpAddress[4],unsigned int Handle,unsigned int BoardID, unsigned int *Error);
unsigned int CM1553BoardSetMode (unsigned int Interface, unsigned int IpAddress[4],unsigned int Handle,unsigned int BoardID,unsigned int bc_mode,unsigned int rt1_mode,unsigned int rt2_mode,unsigned int mt_mode, unsigned int *Error);
unsigned int CM1553BoardGetMode (unsigned int Interface, unsigned int IpAddress[4],unsigned int Handle,unsigned int BoardID, unsigned int *Mode,unsigned int *Error);
unsigned int CM1553ResetIC (unsigned int Interface, unsigned int IpAddress[4],unsigned int Handle,unsigned int BoardID, unsigned int *Error);
unsigned int CM1553SharedInit (unsigned int Interface, unsigned int IpAddress[4],unsigned int Handle,unsigned int BoardID, unsigned int *Error);

unsigned int CM1553BcInitMsg (unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle,unsigned int BoardID,unsigned int MessageType,unsigned int Index,unsigned int DataCount,unsigned int *Data, unsigned short Control_Word,unsigned int RT_Addr,unsigned int Rt_subaddr,unsigned int Mcode,  unsigned int *Error);
unsigned int CM1553BcUpdateMsg (unsigned int Interface, unsigned int IpAddress[4],unsigned int Handle,unsigned int BoardID,unsigned int MessageType,unsigned int Index,unsigned int DataCount,unsigned int *Data, unsigned short Control_Word,unsigned int RT_Addr,unsigned int Rt_subaddr,unsigned int Mcode,  unsigned int *Error);
unsigned int CM1553BcInitMsgList (unsigned int Interface,unsigned int IpAddress[4],unsigned int Handle,unsigned int BoardID,unsigned int mode,unsigned int loop, unsigned int MsgGapTime, unsigned int period_us, unsigned int *Error);

unsigned int CM1553BcSetTTAG (unsigned int Interface, unsigned int IpAddress[4],unsigned int Handle,unsigned int BoardID,unsigned int HiLo, unsigned int BcTTAG, unsigned int *Error);
unsigned int CM1553BcGetMsgInProgress (unsigned int Interface, unsigned int IpAddress[4],unsigned int Handle,unsigned int BoardID, unsigned int *msgprogress, unsigned int *Error);
unsigned int CM1553BcGetMsgBlock (unsigned int Interface, unsigned int IpAddress[4],unsigned int Handle,unsigned int BoardID, unsigned int *msgblock, unsigned int *Error);
unsigned int CM1553BcGetMsgData (unsigned int Interface, unsigned int IpAddress[4],unsigned int Handle,unsigned int BoardID, unsigned int *msgdata, unsigned int *Error);
unsigned int CM1553BcEnable (unsigned int Interface, unsigned int IpAddress[4],unsigned int Handle,unsigned int BoardID, unsigned int *Error);
unsigned int CM1553BcStop (unsigned int Interface, unsigned int IpAddress[4],unsigned int Handle,unsigned int BoardID, unsigned int *Error);
unsigned int CM1553BcTrigger (unsigned int Interface, unsigned int IpAddress[4],unsigned int Handle,unsigned int BoardID, unsigned int *Error);
unsigned int CM1553BcStart (unsigned int Interface, unsigned int IpAddress[4],unsigned int Handle,unsigned int BoardID, unsigned int *Error);
unsigned int CM1553MtSetTTAG (unsigned int Interface, unsigned int IpAddress[4],unsigned int Handle,unsigned int BoardID,unsigned int HiLo,unsigned int MTTag,unsigned int *Error);
unsigned int CM1553MtSetFilter (unsigned int Interface, unsigned int IpAddress[4],unsigned int Handle,unsigned int BoardID, unsigned int *Error);
unsigned int CM1553MtGetLastMessage (unsigned int Interface, unsigned int IpAddress[4],unsigned int Handle,unsigned int BoardID, unsigned int *LastMessage, unsigned int *Error);
unsigned int CM1553MtEnable (unsigned int Interface, unsigned int IpAddress[4],unsigned int Handle,unsigned int BoardID, unsigned int *Error);
unsigned int CM1553MtStop (unsigned int Interface, unsigned int IpAddress[4],unsigned int Handle,unsigned int BoardID, unsigned int *Error);
unsigned int CM1553Rt1Init (unsigned int Interface, unsigned int IpAddress[4],unsigned int Handle,unsigned int BoardID, unsigned int support_broadcast,unsigned int undef_mcodes_valid,unsigned int use_smcp, unsigned int *Error);
unsigned int CM1553Rt1Init1 (unsigned int Interface, unsigned int IpAddress[4],unsigned int Handle,unsigned int BoardID,unsigned int use_smcp, unsigned int *Error);
unsigned int CM1553Rt1Init2 (unsigned int Interface, unsigned int IpAddress[4],unsigned int Handle,unsigned int BoardID, unsigned int illegalcmd_detect,unsigned int *Error);
unsigned int CM1553Rt1SetAddr (unsigned int Interface, unsigned int IpAddress[4],unsigned int Handle,unsigned int BoardID,unsigned int addr, unsigned int *Error);
unsigned int CM1553Rt1SetTTAG (unsigned int Interface, unsigned int IpAddress[4],unsigned int Handle,unsigned int BoardID,unsigned int RT1ttag ,unsigned int *Error);
unsigned int CM1553Rt1SetTXData (unsigned int Interface, unsigned int IpAddress[4],unsigned int Handle,unsigned int BoardID,unsigned int *rt1txdata,unsigned int data_count, unsigned int *Error);
unsigned int CM1553Rt1Enable (unsigned int Interface, unsigned int IpAddress[4],unsigned int Handle,unsigned int BoardID, unsigned int *Error);
unsigned int CM1553Rt1Stop (unsigned int Interface, unsigned int IpAddress[4],unsigned int Handle,unsigned int BoardID, unsigned int *Error);
unsigned int CM1553Rt1Start (unsigned int Interface, unsigned int IpAddress[4],unsigned int Handle,unsigned int BoardID, unsigned int *Error);
unsigned int CM1553Rt2Init (unsigned int Interface, unsigned int IpAddress[4],unsigned int Handle,unsigned int BoardID, unsigned int support_broadcast,unsigned int undef_mcodes_valid,unsigned int use_smcp, unsigned int *Error);
unsigned int CM1553Rt2Init1 (unsigned int Interface, unsigned int IpAddress[4],unsigned int Handle,unsigned int BoardID,unsigned int use_smcp, unsigned int *Error);
unsigned int CM1553Rt2Init2 (unsigned int Interface, unsigned int IpAddress[4],unsigned int Handle, unsigned int BoardID,unsigned int illegalcmd_detect,unsigned int *Error);
unsigned int CM1553Rt2SetAddr (unsigned int Interface, unsigned int IpAddress[4],unsigned int Handle,unsigned int BoardID,unsigned int addr, unsigned int *Error);
unsigned int CM1553Rt2SetTTAG (unsigned int Interface, unsigned int IpAddress[4],unsigned int Handle,unsigned int BoardID,unsigned int RT2ttag ,unsigned int *Error);
unsigned int CM1553Rt2SetTXData (unsigned int Interface, unsigned int IpAddress[4],unsigned int Handle,unsigned int BoardID,unsigned int *rt2txdata,unsigned int data_count, unsigned int *Error);
unsigned int CM1553Rt2Enable (unsigned int Interface, unsigned int IpAddress[4],unsigned int Handle,unsigned int BoardID, unsigned int *Error);
unsigned int CM1553Rt2Stop (unsigned int Interface, unsigned int IpAddress[4],unsigned int Handle, unsigned int BoardID,unsigned int *Error);
unsigned int CM1553Rt2Start (unsigned int Interface, unsigned int IpAddress[4],unsigned int Handle,unsigned int BoardID, unsigned int *Error);
unsigned int CM1553RtGetMessage (unsigned int Interface, unsigned int IpAddress[4],unsigned int Handle,unsigned int BoardID,unsigned int *RTmessage,unsigned int rt_num,unsigned int *Error);
unsigned int CM1553RtCheckAddr (unsigned int Interface, unsigned int IpAddress[4],unsigned int Handle,unsigned int BoardID,unsigned int rt_num, unsigned int *Error);
unsigned int CM1553SetIp(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int NewIpAddress[4], unsigned int *Error);
unsigned int CM1553GetIp(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int IpAddressReturn[], unsigned int *Error);
unsigned int CM1553ResetIp(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int *Error);
unsigned int CM1553SetMac(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID,unsigned int NewMAC[6], unsigned int *Error);
unsigned int CM1553GetMac(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID,unsigned int ReturnMac[], unsigned int *Error);
unsigned int CM1553GetSerialNumber(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int *SerialNo, unsigned int *Error);
unsigned int CM1553GetFirmware(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, double *FWVersion, unsigned int *Error);
unsigned int CM1553DiscardBoard (unsigned int Interface,unsigned int IpAddress[4],unsigned int Handle,unsigned int BoardID, unsigned int *Error);


unsigned int MTOpenBoard(unsigned int Interface, unsigned int Port,unsigned int IpAddress[4], unsigned int BoardID, unsigned int *Error);
unsigned int MTSetChannel(unsigned int Interface,unsigned int IpAddress[4],unsigned int Handle,unsigned int BoardID, TimerChannelEnum Channel,unsigned int MODE, unsigned int PRESCALER,unsigned int PERIOD,unsigned int PULSE, unsigned int *Error);
unsigned int MTGetChannel(unsigned int Interface, unsigned int IpAddress[4],unsigned int Handle,unsigned int BoardID, unsigned int *Channels, unsigned int *Error);
unsigned int MTStartChannel(unsigned int Interface,unsigned int IpAddress[4],unsigned int Handle,unsigned int BoardID, TimerChannelEnum  Channel,unsigned int MODE,unsigned int sync,unsigned int *Error);
unsigned int MTStopChannel(unsigned int Interface, unsigned int IpAddress[4],unsigned int Handle,unsigned int BoardID, TimerChannelEnum  Channel, unsigned int MODE, unsigned int *Error);
unsigned int MTUpdateChannel(unsigned int Interface,unsigned int IpAddress[4],unsigned int Handle,unsigned int BoardID, TimerChannelEnum Channel,unsigned int MODE, unsigned int PRESCALER,unsigned int PERIOD,unsigned int PULSE,unsigned int sync, unsigned int *Error);
unsigned int MTDiscardBoard (unsigned int Interface, unsigned int IpAddress[4],unsigned int Handle, unsigned int BoardID,unsigned int *Error);
unsigned int MTSetIp(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int NewIpAddress[4], unsigned int *Error);
unsigned int MTGetIp(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int IpAddressReturn[], unsigned int *Error);
unsigned int MTResetIp(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int *Error);
unsigned int MTSetMac(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID,unsigned int NewMAC[6], unsigned int *Error);
unsigned int MTGetMac(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID,unsigned int ReturnMac[], unsigned int *Error);
unsigned int MTGetSerialNumber(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, unsigned int *SerialNo, unsigned int *Error);
unsigned int MTGetFirmware(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int BoardID, double *FWVersion, unsigned int *Error);



unsigned int OpenChassis (unsigned int Interface, unsigned int Port, unsigned int IpAddress[4], unsigned int *Error);
unsigned int SetChassisIP(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int NewIpAddress[4], unsigned int *Error);
unsigned int GetChassisIP(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int IpAddressReturn[], unsigned int *Error);
unsigned int DiscardChassis(unsigned int Interface, unsigned int IpAddress[4], unsigned int Handle, unsigned int *Error);

unsigned int SC485OpenBoard(unsigned int IpAddress[4], unsigned int Port, unsigned int *Error);
unsigned int SC485InitBoard(unsigned int IpAddress[4], unsigned int Handle,unsigned int Port,unsigned int BaudRate, unsigned int WordLength, unsigned int StopBit,unsigned int Parity, unsigned int *Error);
unsigned int SC485SendMsg(unsigned int IpAddress[4], unsigned int Handle, unsigned int Port, unsigned int DataCount, unsigned char *Data, unsigned int *Error);
unsigned int SC485ReceiveMsg(unsigned int IpAddress[4], unsigned int Handle, unsigned int Port, unsigned int DataCount, unsigned char *Data,unsigned int timeout, unsigned int *Error);
unsigned int SC485DiscardBoard(unsigned int IpAddress[4], unsigned int Handle, unsigned int *Error);
void GetApiVersion(unsigned char *Version); 
#ifdef __cplusplus
    }
#endif

#endif  /* ndef __CMXAPI_H__ */
