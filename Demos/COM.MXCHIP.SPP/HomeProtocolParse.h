#ifndef __SPPPROTOCOL_H
#define __SPPPROTOCOL_H

#include "Common.h"
#include "MICODefine.h"


OSStatus HomeWlanCommandProcess(unsigned char *inBuf, int *inBufLen, int inSocketFd, mico_Context_t * const inContext);

#endif
