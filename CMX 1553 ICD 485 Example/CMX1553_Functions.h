

#ifndef __CMX1553_Functions_H__
#define __CMX1553_Functions_H__

#ifdef __cplusplus
    extern "C" {
#endif



#include "cvidef.h"
#include "Support.h"

		
		
unsigned int Cmx1553OpenBoard (unsigned int PortNumber);
unsigned int Cmx1553InitBoard (unsigned int PortNumber);
unsigned int Cmx1553ResetBoard (unsigned int PortNumber);
unsigned int Cmx1553BoardSetMode (unsigned int PortNumber,unsigned int bc_mode,unsigned int rt1_mode,unsigned int rt2_mode,unsigned int mt_mode);
unsigned int Cmx1553BoardGetMode (unsigned int PortNumber, unsigned int *Mode);
unsigned int Cmx1553Reset (unsigned int PortNumber);
unsigned int Cmx1553SharedInit (unsigned int PortNumber);

unsigned int Cmx1553BcInitMsg (unsigned int PortNumber,unsigned int message_type,unsigned int Index,unsigned int data_count,unsigned short *data,unsigned short control_word,unsigned int RT_Addr,unsigned int RT_subaddr,unsigned int Mcode );
unsigned int Cmx1553BcUpdateMsg (unsigned int PortNumber,unsigned int message_type,unsigned int Index,unsigned int data_count,unsigned short *data,unsigned short control_word,unsigned int RT_Addr,unsigned int RT_subaddr,unsigned int Mcode );
unsigned int Cmx1553BcInitMsgList (unsigned int PortNumber, unsigned int mode,unsigned int loop, unsigned int msg_gap_time ,unsigned int period_us );
unsigned int Cmx1553BcSetTTAG (unsigned int PortNumber,unsigned int HiLo, unsigned int BcTTAG);
unsigned int Cmx1553BcGetMsgInProgress (unsigned int PortNumber, unsigned int *msgprogress);
unsigned int Cmx1553BcGetMsgBlock(unsigned int PortNumber, unsigned short*msgblock);
unsigned int Cmx1553BcGetMsgData (unsigned int PortNumber, unsigned short*msgdata,unsigned int num_packet);
unsigned int Cmx1553BcEnable (unsigned int PortNumber);
unsigned int Cmx1553BcStop (unsigned int PortNumber);
unsigned int Cmx1553BcTrigger (unsigned int PortNumber);
unsigned int Cmx1553BcStart (unsigned int PortNumber);

unsigned int Cmx1553MtSetTTAG (unsigned int PortNumber,unsigned int HiLo,unsigned int MTTag);
unsigned int Cmx1553MtSetFilter (unsigned int PortNumber);
unsigned int Cmx1553MtGetLastCommand (unsigned int PortNumber,unsigned short* lastcommand );
unsigned int Cmx1553MtGetLastMessage (unsigned int PortNumber,unsigned short* lastmsg);
unsigned int Cmx1553MtEnable (unsigned int PortNumber);
unsigned int Cmx1553MtStop (unsigned int PortNumber);

unsigned int Cmx1553Rt1Init (unsigned int PortNumber, unsigned int support_broadcast,unsigned int undef_mcodes_valid,unsigned int use_smcp);
unsigned int Cmx1553Rt1Init1 (unsigned int PortNumber,unsigned int use_smcp);
unsigned int Cmx1553Rt1Init2 (unsigned int PortNumber, unsigned int illegalcmd_detect);

unsigned int Cmx1553Rt1SetAddr (unsigned int PortNumber,unsigned int addr);
unsigned int Cmx1553Rt1SetTTAG (unsigned int PortNumber,unsigned int RT1ttag );
unsigned int Cmx1553Rt1SetTXData (unsigned int PortNumber,unsigned short *rt1txdata,unsigned int data_count);
unsigned int Cmx1553Rt1Enable (unsigned int PortNumber);
unsigned int Cmx1553Rt1Stop (unsigned int PortNumber);
unsigned int Cmx1553Rt1Start (unsigned int PortNumber);

unsigned int Cmx1553Rt2Init (unsigned int PortNumber, unsigned int support_broadcast,unsigned int undef_mcodes_valid,unsigned int use_smcp);
unsigned int Cmx1553Rt2Init1 (unsigned int PortNumber,unsigned int use_smcp);
unsigned int Cmx1553Rt2Init2 (unsigned int PortNumber, unsigned int illegalcmd_detect);
unsigned int Cmx1553Rt2SetAddr (unsigned int PortNumber,unsigned int addr);
unsigned int Cmx1553Rt2SetTTAG (unsigned int PortNumber,unsigned int RT1ttag );
unsigned int Cmx1553Rt2SetTXData (unsigned int PortNumber,unsigned short *rt2txdata,unsigned int data_count);
unsigned int Cmx1553Rt2Enable (unsigned int PortNumber);
unsigned int Cmx1553Rt2Stop (unsigned int PortNumber);
unsigned int Cmx1553Rt2Start (unsigned int PortNumber);
unsigned int Cmx1553RtGetMessage (unsigned int PortNumber,unsigned short* RTmessage,unsigned int rt_num);
unsigned int Cmx1553RtCheckAddr (unsigned int PortNumber,unsigned int rt_num);

unsigned int Cmx1553SetIp (unsigned int PortNumber,unsigned int ipaddress[4]);
unsigned int Cmx1553GetIp (unsigned int PortNumber,unsigned int *ipreturn);
unsigned int Cmx1553ResetIP (unsigned int PortNumber);
unsigned int Cmx1553SetMac (unsigned int PortNumber,unsigned int new_mac[6]);
unsigned int Cmx1553GetMac (unsigned int PortNumber,unsigned int *return_mac);
unsigned int Cmx1553GetSerial (unsigned int PortNumber,unsigned int *serialno);
//unsigned int Cmx1553GetFirmware (unsigned int PortNumber,double  *fw_version);
unsigned int Cmx1553DiscardBoard(unsigned int PortNumber);
#ifdef __cplusplus
    }
#endif

#endif  /* ndef __CMX1553_Functions_H__ */
