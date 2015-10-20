/*
 * File:    enet.c
 * Purpose: Driver for the ENET controller
 *
 * Notes:
 */

#include "common.h"
#include "enet.h"
#include "eth.h"
#include "macnet.h"/*register definition*/
#include "macnet_hal.h"/*SoC implementation*/


/********************************************************************/
/* Initialize the MIB counters
 *
 * Parameters:
 *  ch      FEC channel
 */
void
enet_mib_init(int ch)
{
//To do
}
/********************************************************************/
/* Display the MIB counters
 *
 * Parameters:
 *  ch      FEC channel
 */
void
enet_mib_dump(int ch)
{
//To do
}
/********************************************************************/
/*
 * Set the duplex on the selected FEC controller
 *
 * Parameters:
 *  ch      FEC channel
 *  duplex  enet_MII_FULL_DUPLEX or enet_MII_HALF_DUPLEX
 */
void
enet_duplex (int ch, ENET_DUPLEX duplex)
{
    volatile macnet_t *enet;
    
    if(ch)
      enet = (macnet_t *)(MACNET_BASE_PTR+MAC_NET_OFFSET);
    else
      enet = (macnet_t *)MACNET_BASE_PTR;
  
    switch (duplex)
    {
        case MII_HDX:
            enet->rcr |= MACNET_RCR_DRT_MASK;
            enet->tcr &= (uint32_t)~MACNET_TCR_FDEN_MASK;
            break;
        case MII_FDX:
        default:
            enet->rcr &= ~MACNET_RCR_DRT_MASK;
            enet->tcr |= MACNET_TCR_FDEN_MASK;
            break;
    }
}


/***********AddMulticastGroup****************/
/*
* addr: The multicast group address
*/
void Enet_Add_Multicast_Group(const uint8_t* addr){
  
    uint32_t crcValue;
    
#if (MACNET_PORT==0)
  volatile macnet_t *enet = (macnet_t *)MACNET_BASE_PTR;
#else
  volatile macnet_t *enet = (macnet_t *)(MACNET_BASE_PTR+MAC_NET_OFFSET);
#endif
  
    crcValue = enet_hash_address(addr);
    if(crcValue >= 32)
        enet->gaur |= (uint32_t)(1 << (crcValue - 32));
    else
        enet->galr |= (uint32_t)(1 << crcValue);
}
  
  
/***********AddMulticastGroup****************/
/*
* addr: The multicast group address
*/    
void Enet_Leave_Multicast_Group(const uint8_t* addr){
  
    uint32_t crcValue;
    
#if (MACNET_PORT==0)
  volatile macnet_t *enet = (macnet_t *)MACNET_BASE_PTR;
#else
  volatile macnet_t *enet = (macnet_t *)(MACNET_BASE_PTR+MAC_NET_OFFSET);
#endif
  
    crcValue = enet_hash_address(addr);
    
    if (crcValue >= 32)
      { 
        enet->gaur &= ~(uint32_t)(1 << (crcValue - 32));
      }
        else
      {
        enet->galr &= ~(uint32_t)(1 << crcValue);
      }
}    

/********************************************************************/
/*
 * Generate the hash table settings for the given address
 *
 * Parameters:
 *  addr    48-bit (6 byte) Address to generate the hash for
 *
 * Return Value:
 *  The 6 most significant bits of the 32-bit CRC result
 */
uint8_t
enet_hash_address(const uint8_t* addr)
{
    uint32_t crc;
    uint8_t byte;
    int i, j;

    crc = 0xFFFFFFFF;
    for(i=0; i<6; ++i)
    {
        byte = addr[i];
        for(j=0; j<8; ++j)
        {
            if((byte & 0x01)^(crc & 0x01))
            {
                crc >>= 1;
                crc = crc ^ 0xEDB88320;
            }
            else
                crc >>= 1;
            byte >>= 1;
        }
    }
    return (uint8_t)(crc >> 26);
}
/********************************************************************/
/*
 * Set the Physical (Hardware) Address and the Individual Address
 * Hash in the selected FEC
 *
 * Parameters:
 *  ch  FEC channel
 *  pa  Physical (Hardware) Address for the selected FEC
 */
void
enet_set_address (int ch, uint8_t *pa)
{
    uint8_t crc;
    volatile macnet_t *enet;
    
    if(ch)
      enet = (macnet_t *)(MACNET_BASE_PTR+MAC_NET_OFFSET);
    else
      enet = (macnet_t *)MACNET_BASE_PTR;
    /*
     * Set the Physical Address
     */
    enet->palr = (uint32_t)(uint32_t)((pa[0]<<24) | (pa[1]<<16) | (pa[2]<<8) | pa[3]);
    enet->paur = (uint32_t)(uint32_t)((pa[4]<<24) | (pa[5]<<16));

    /*
     * Calculate and set the hash for given Physical Address
     * in the  Individual Address Hash registers
     */
    crc = enet_hash_address(pa);
    if(crc >= 32)
        enet->iaur |= (uint32_t)(1 << (crc - 32));
    else
        enet->ialr |= (uint32_t)(1 << crc);
}
/********************************************************************/
/*
 * Reset the selected FEC controller
 *
 * Parameters:
 *  ch      FEC channel
 */
void
enet_reset (int ch)
{
    int i;
    volatile macnet_t *enet;
    
    if(ch)
      enet = (macnet_t *)(MACNET_BASE_PTR+MAC_NET_OFFSET);
    else
      enet = (macnet_t *)MACNET_BASE_PTR;

    /* Set the Reset bit and clear the Enable bit */
    enet->ecr = MACNET_ECR_RESET_MASK;

    /* Wait at least 8 clock cycles */
    for (i=0; i<10; ++i)
        NOP_ASM;
}
/********************************************************************/
/*
 * Initialize the selected FEC
 *
 * Parameters:
 *  config: ENET parameters
 *
 *
 */
void
enet_init (ENET_CONFIG *config)
{
    volatile macnet_t *enet;
    
    if(config->ch)
      enet = (macnet_t *)(MACNET_BASE_PTR+MAC_NET_OFFSET);
    else
      enet = (macnet_t *)MACNET_BASE_PTR;

#ifndef MACNET_SWITCH_SUPPORT
    /* Clear the Individual and Group Address Hash registers */
    enet->ialr = 0;
    enet->iaur = 0;
    enet->galr = 0;
    enet->gaur = 0;

    /* Set the Physical Address for the selected FEC */
    enet_set_address(config->ch, config->mac);

    /* Mask all FEC interrupts */
    enet->eimr = 0;//FSL:MACNET_EIMR_MASK_ALL_MASK;

    /* Clear all FEC interrupt events */
    enet->eir = 0xFFFFFFFF;//FSL:MACNET_EIR_CLEAR_ALL_MASK;
#endif
    
    /* Initialize the Receive Control Register */
    enet->rcr = 0
        | MACNET_RCR_MAX_FL(ETH_MAX_FRM)
        | MACNET_RCR_MII_MODE_MASK /*always*/
        | MACNET_RCR_CRCFWD_MASK;  /*no CRC pad required*/

    if ( config->interface == mac_rmii )
    {
      enet->rcr |= MACNET_RCR_RMII_MODE_MASK;
      
      /*only set speed in RMII mode*/
      if( config->speed == MII_10BASET )
      {
         enet->rcr |= MACNET_RCR_RMII_10T_MASK;
      }
    }/*no need to configure MAC MII interface*/ 
    
    enet->tcr = 0;    
    
    /* Set the duplex */
    enet_duplex(config->ch, config->duplex);        

#warning "Need to understand if promiscuous is really needed with the switch"
#ifndef MACNET_SWITCH_SUPPORT
    if (config->prom)
    {
        enet->rcr |= MACNET_RCR_PROM_MASK; 
    }
#endif
    
#if (ENHANCED_BD==1)
    enet->ecr = MACNET_ECR_EN1588_MASK;
#else
    enet->ecr = 0;//clear register
#endif
    
#if ((BYTE_SWAPPING_EN==1)&&(MACNET_VERSION==MACNET_ENHANCED_V2))/*new!*/
    enet->ecr |= MACNET_ECR_DBSWP_MASK;
#endif

#warning "Seems like internal loopback is not supported while running switch"
#ifndef MACNET_SWITCH_SUPPORT
    if(config->loopback == INTERNAL_LOOPBACK)
    {
        /*seems like RMII internal loopback works, even if it's not supported*/
        enet->rcr |= MACNET_RCR_LOOP_MASK;
    }
#endif
}
/********************************************************************/
void
enet_start (int ch)
{
  volatile macnet_t *enet;
    
  if(ch)
      enet = (macnet_t *)(MACNET_BASE_PTR+MAC_NET_OFFSET);
  else
      enet = (macnet_t *)MACNET_BASE_PTR;
  
  // Enable FEC
  enet->ecr |= MACNET_ECR_ETHEREN_MASK;
}

/********************************************************************/
int 
enet_wait_for_frame_receive(int ch, int timeout)
{
    volatile macnet_t *enet;
    
    if(ch)
      enet = (macnet_t *)(MACNET_BASE_PTR+MAC_NET_OFFSET);
    else
      enet = (macnet_t *)MACNET_BASE_PTR;
  
    int i, return_val = 1;
    
    for (i=0; i < timeout; i++)
    {
        if (enet->eir & MACNET_EIR_RXF_MASK)
        {
           enet->eir = MACNET_EIR_RXF_MASK;
           break;		
        }
    }

    if(i == timeout)
    {
       return_val = 0;
    }
    return return_val;
}
/********************************************************************/



