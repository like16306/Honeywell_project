#ifndef SPARKLIGHT_H
#define SPARKLIGHT_H

#include "MICODefine.h"

#define FRAME_HEAD (0xAAAA)
#define SRC_SUBNET_ID (0x0C)
#define SRC_DEV_ID (0xFE)
#define SRC_DEV_TYPE (0xFE)
#define DEVTYPE_LIGHT_HBUS (8)
#define DEVTYPE_CURTAIN_HBUS (9)
#define CMD_8AND1_315_H (0xDB)
#define CMD_8AND1_315_L (0x00)
#define CMD_8AND1_314_H (0x16)
#define CMD_8AND1_314_L (0x45)
#define CMD_12AND1_H (0x16)
#define CMD_12AND1_L (0x46)
#define CMD_INONE_H (0x16)
#define CMD_INONE_L (0x04)
#define CMD_SINGEL_LOOP_H (0x00)
#define CMD_SINGEL_LOOP_L (0x31)

#define OPCODE_LEN_0x0031 (4)

typedef struct _sparkRS485Frame{
	unsigned short head;
	unsigned char len;
	unsigned char source_sub_net;
	unsigned char source_dev_addr;
	unsigned short source_dev_type;
	unsigned short op_code;
	unsigned char dist_sub_net;
	unsigned char dist_dev_addr;
	unsigned char* data;
	unsigned short crc16;
}st_SparkRS485Frame;

typedef enum _e_sparkaddress
{
    byleadHeader=0,
    byDataLen=2,
    bySrcSubNetId=3,
    bySrcDevId=4,
    bySrcDevType=5,
    byOpCode=7,
    byDesSubNetId=9,
    byDesDevId=10,
    sparkArray_maxLen=11
};

int LightCfgPackage(int srcSubNet,int srcDevId,int pdistSubnet,int pdistDevId,int pLoopId, int pDimm);

#endif // HDLCONTROL_H

