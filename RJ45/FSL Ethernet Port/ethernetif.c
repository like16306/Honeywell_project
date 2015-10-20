/**
 * @file
 * Ethernet Interface Skeleton
 *
 */

/*
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED 
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 * 
 * Author: Adam Dunkels <adam@sics.se>
 *
 */

/*
 * This file is a skeleton for developing Ethernet network interface
 * drivers for lwIP. Add code to the low_level functions and do a
 * search-and-replace for the word "ethernetif" to replace it with
 * something that better describes your network interface.
 */

#include "FreeRTOSConfig.h"
#include "fsl_device_registers.h"
#include "fsl_interrupt_manager.h"

#define portCHAR                           char
#define portFLOAT                         float
#define portDOUBLE                      double
#define portBASE_TYPE                long
#define portLONG                 long
#define portSHORT               short
//#define uint32_t   uint32_t
//#define uint8_t    uint8_t
#define TRUE  0
#define FALSE 1
#define pdFALSE 0


#if 1 /* ready */

#include "common.h"
/* Standard library includes. */
#include <string.h>

/* Demo includes. */
#include "eth_phy.h"
#include "enet.h"
#include "mii.h"
#include "macnet.h"/*register definition*/

#include "MICORTOS.h"

/* Define those to better describe your network interface. */
#define IFNAME0 'p'
#define IFNAME1 '2'

/* PHY hardware specifics. */
#define PHY_STATUS								( 0x1F )
#define PHY_DUPLEX_STATUS							( 4<<2 )
#define PHY_SPEED_STATUS							( 1<<2 )

/* Delay to wait for a DMA buffer to become available if one is not already
available. */
#define netifBUFFER_WAIT_ATTEMPTS					        10
#define netifBUFFER_WAIT_DELAY						        (10 / portTICK_RATE_MS)

/* Delay between polling the PHY to see if a link has been established. */
#define netifLINK_DELAY								( 500 / portTICK_RATE_MS )

/* Delay between looking for incoming packets.  In ideal world this would be
infinite. */
#define netifBLOCK_TIME_WAITING_FOR_INPUT			                netifLINK_DELAY

/* Very short delay to use when waiting for the Tx to finish with a buffer if
we run out of Rx buffers. */
#define enetMINIMAL_DELAY							( 2 / portTICK_RATE_MS )

/*FSL: arrays to be used*/

static unsigned char xENETTxDescriptors_unaligned[ ( configNUM_ENET_TX_BUFFERS * sizeof( NBUF ) ) + 16 ];
static unsigned char xENETRxDescriptors_unaligned[ ( configNUM_ENET_RX_BUFFERS * sizeof( NBUF ) ) + 16 ];

#ifdef ENET_BUFFERS_OUTSIDE_OF_SRAM
#pragma location = ".ddr_data"
#endif
static unsigned char ucENETTxBuffers[ ( configNUM_ENET_TX_BUFFERS * configENET_TX_BUFFER_SIZE ) + 16 ];

#ifdef ENET_BUFFERS_OUTSIDE_OF_SRAM
#pragma location = ".ddr_data"
#endif
static unsigned char ucENETRxBuffers[ ( configNUM_ENET_RX_BUFFERS * configENET_RX_BUFFER_SIZE ) + 16 ];

/* The DMA descriptors.  This is a char array to allow us to align it correctly. */
static NBUF *xENETTxDescriptors;
static NBUF *xENETRxDescriptors;

/* The DMA buffers.  These are char arrays to allow them to be alligned correctly. */
static unsigned portBASE_TYPE uxNextRxBuffer = 0, uxNextTxBuffer = 0;

#if 1
/* Semaphores used by the ENET interrupt handler to wake the handler task. */
static mico_semaphore_t xRxENETSemaphore;
static mico_semaphore_t xTxENETSemaphore;
#endif

extern int periph_clk_khz;

//xTaskHandle xEthIntTask;

/**
 * Helper struct to hold private data used to operate your ethernet interface.
 * Keeping the ethernet address of the MAC in this struct is not necessary
 * as it is already kept in the struct netif.
 * But this is only an example, anyway...
 */
struct ethernetif {
  struct eth_addr *ethaddr;
  /* Add whatever per-interface state that is needed here. */
};

/* Standard lwIP netif handlers. */
static void prvInitialiseENETBuffers( void );
void mico_ethif_low_level_init(char *mac);
int mico_ethif_low_level_output(uint8_t *buf, int len);
int mico_ethif_low_level_input(uint8_t *buf, int maxlen);


/* If Line down after reset*/
extern int line_down_reset;

/**
 * In this function, the hardware should be initialized.
 * Called from ethernetif_init().
 *
 * @param netif the already initialized lwip network interface structure
 *        for this ethernetif
 */
void mico_ethif_low_level_init(char *mac)
{
  int usData, phy_reg_temp;

//FSL:struct ethernetif *ethernetif = netif->state;
#if (MACNET_PORT==0)
  volatile macnet_t *enet = (macnet_t *)MACNET_BASE_PTR;
#else
  volatile macnet_t *enet = (macnet_t *)(MACNET_BASE_PTR+MAC_NET_OFFSET);
#endif

  /* Do whatever else is needed to initialize interface. */  
  ENABLE_MAC_NET;        
        
  prvInitialiseENETBuffers();
  
#if 1
  /*FSL: create semaphores*/
  mico_rtos_init_semaphore(&xRxENETSemaphore, 1);
  mico_rtos_set_semaphore(&xRxENETSemaphore);
  mico_rtos_init_semaphore(&xTxENETSemaphore, 1);
  mico_rtos_set_semaphore(&xTxENETSemaphore);
#endif

  /* Set the Reset bit and clear the Enable bit */
  enet->ecr = MACNET_ECR_RESET_MASK;

  /* Wait at least 8 clock cycles */
  for( usData = 0; usData < 10; usData++ )
  {
    NOP_ASM;
  }
    
  /*FSL: start MII interface*/
  mii_init(MACNET_PORT, periph_clk_khz/1000/*MHz*/);       
        
  //enet_interrupt_routine
  INT_SYS_EnableIRQ(ENET_Transmit_IRQn);/*ENET xmit interrupt*/                  
  INT_SYS_EnableIRQ(ENET_Receive_IRQn);/*ENET rx interrupt*/                   
  INT_SYS_EnableIRQ(ENET_Error_IRQn);/*ENET error and misc interrupts*/ 

#if 0   //already done in hardware_init.c
  /*
   * Make sure the external interface signals are enabled
   */
  ENET_MII_PINS;   

#if configUSE_MII_MODE
  ENET_MII_MODE_PINS;
#else
  ENET_RMII_MODE_PINS;
#endif   
#endif
  
  /* Can we talk to the PHY? */
  do
  {
    //vTaskDelay( netifLINK_DELAY );	//cutworth, need to change to something from Mico OS
    msleep(500);
    usData = 0xffff;
    mii_read( MACNET_PORT, configPHY_ADDRESS, PHY_PHYIDR1, &usData );
        
  } while( usData == 0xffff );
 
  
  
  
  /**********************************/
  /*    Enable PHY Link Up-Down  Interrupt  *****************Like add     */
  mii_read( MACNET_PORT, configPHY_ADDRESS, PHY_BMSR, &phy_reg_temp);
  
  if(!(phy_reg_temp & 0x00000004)){
    line_down_reset = 1;
    mii_write( MACNET_PORT, configPHY_ADDRESS, PHY_INTCTL, ( PHY_INTCTL_LINK_DOWN_ENABLE | PHY_INTCTL_LINK_UP_ENABLE ) );
    mii_read( MACNET_PORT, configPHY_ADDRESS, PHY_INTCTL, &phy_reg_temp);
    PORTB_PCR9 |= PORT_PCR_ISF_MASK;
    msleep(500);
    INT_SYS_EnableIRQ(60);        //place PHY and GPIO in a certain status
    return;
  }
  /************************************/

  
  
  /* Start auto negotiate. */
  mii_write( MACNET_PORT, configPHY_ADDRESS, PHY_BMCR, ( PHY_BMCR_AN_RESTART | PHY_BMCR_AN_ENABLE ) );
  mii_read( MACNET_PORT, configPHY_ADDRESS, PHY_INTCTL, &phy_reg_temp); //add
  /* Wait for auto negotiate to complete. */
  do
  {
    //vTaskDelay( netifLINK_DELAY );	//cutworth, need to change to something from Mico OS
    msleep(500);
    mii_read( MACNET_PORT, configPHY_ADDRESS, PHY_BMSR, &usData );
  } while( !( usData & PHY_BMSR_AN_COMPLETE ) );

  /* When we get here we have a link - find out what has been negotiated. */
  usData = 0;
  mii_read( MACNET_PORT, configPHY_ADDRESS, PHY_STATUS, &usData );  

  /* Clear the Individual and Group Address Hash registers */
  enet->ialr = 0;
  enet->iaur = 0;
  enet->galr = 0;
  enet->gaur = 0;
  
  /* Set the Physical Address for the selected ENET */
  enet_set_address( MACNET_PORT, (uint8_t *)mac );
        
#if configUSE_MII_MODE        
  /* Various mode/status setup. */
  enet->rcr = MACNET_RCR_MAX_FL(configENET_MAX_PACKET_SIZE) | MACNET_RCR_MII_MODE_MASK
#if (MACNET_VERSION!=MACNET_LEGACY)
            | MACNET_RCR_CRCFWD_MASK
#endif
            ;
#else
  enet->rcr = MACNET_RCR_MAX_FL(configENET_MAX_PACKET_SIZE) | MACNET_RCR_MII_MODE_MASK 
#if (MACNET_VERSION!=MACNET_LEGACY)
            | MACNET_RCR_CRCFWD_MASK
#endif
            | MACNET_RCR_RMII_MODE_MASK;
#endif

  /*FSL: clear rx/tx control registers*/
  enet->tcr = 0;
        
  /* Setup half or full duplex. */
  if(usData & PHY_DUPLEX_STATUS)
  {
    /*Full duplex*/
    enet->rcr &= (unsigned portLONG)~MACNET_RCR_DRT_MASK;
    enet->tcr |= MACNET_TCR_FDEN_MASK;
  }
  else
  {
    /*half duplex*/
    enet->rcr |= MACNET_RCR_DRT_MASK;
    enet->tcr &= (unsigned portLONG)~MACNET_TCR_FDEN_MASK;
  }
#if (MACNET_VERSION!=MACNET_LEGACY)
  /* Setup speed */
  if( usData & PHY_SPEED_STATUS )
  {
    /*10Mbps*/
    enet->rcr |= MACNET_RCR_RMII_10T_MASK;
  }
#endif

  #if( configUSE_PROMISCUOUS_MODE == 1 )
  {
    enet->rcr |= MACNET_RCR_PROM_MASK;
  }
  #endif

  #if (ENHANCED_BD==1)
    enet->ecr = MACNET_ECR_EN1588_MASK;
  #else
    enet->ecr = 0;
  #endif

  #if ((BYTE_SWAPPING_EN==1)&&(MACNET_VERSION==MACNET_ENHANCED_V2))/*new!*/
    enet->ecr |= MACNET_ECR_DBSWP_MASK;
  #endif
    
  /*new!*/
#if (ENET_HARDWARE_CHECKSUM && (MACNET_VERSION!=MACNET_LEGACY) )
  // Enable discard on wrong protocol checksum and other nice features
  enet->racc = 0
                         | MACNET_RACC_IPDIS_MASK
                         | MACNET_RACC_PRODIS_MASK
                         | MACNET_RACC_LINEDIS_MASK
                         | MACNET_RACC_IPDIS_MASK
                         | MACNET_RACC_PADREM_MASK
                         ;
  
  // Enable Protocol Checksum Insertion
  enet->tacc = 0
                         | MACNET_TACC_IPCHK_MASK 
                         | MACNET_TACC_PROCHK_MASK
                         ;
#endif
  
  enet->tfwr = MACNET_TFWR_STRFWD_MASK;/*FSL: new! store and FW for checksum property*/
  
  /*new!*/
#if (ENET_HARDWARE_SHIFT && (MACNET_VERSION!=MACNET_LEGACY) )
  // Enable Ethernet header alignment for rx
  enet->racc |= 0
                         | MACNET_RACC_SHIFT16_MASK
                         ;
  
  // Enable Ethernet header alignment for tx
  enet->tacc |= 0
                         | MACNET_TACC_SHIFT16_MASK
                         ;
#endif
  
  /* Set Rx Buffer Size */
  enet->mrbr = (unsigned portSHORT) configENET_RX_BUFFER_SIZE;

  /* Point to the start of the circular Rx buffer descriptor queue */
  enet->rdsr = ( unsigned portLONG ) xENETRxDescriptors ;

  /* Point to the start of the circular Tx buffer descriptor queue */
  enet->tdsr = ( unsigned portLONG ) xENETTxDescriptors ;

  /* Clear all ENET interrupt events */
  enet->eir = ( unsigned portLONG ) -1;

  /* Enable interrupts. */
  enet->eimr = 0 
            /*rx irqs*/
            | MACNET_EIMR_RXF_MASK/*FSL: only for complete frame, not partial buffer descriptor | MACNET_EIMR_RXB_MASK*/
            /*xmit irqs*/
            | MACNET_EIMR_TXF_MASK/*FSL: only for complete frame, not partial buffer descriptor | MACNET_EIMR_TXB_MASK*/
            /*enet irqs*/
            | MACNET_EIMR_UN_MASK | MACNET_EIMR_RL_MASK | MACNET_EIMR_LC_MASK | MACNET_EIMR_BABT_MASK | MACNET_EIMR_BABR_MASK | MACNET_EIMR_EBERR_MASK
            ;

  /* Create the task that handles the MAC ENET. */
  //xTaskCreate( ethernetif_input, ( signed char * ) "ETH_INT", configETHERNET_INPUT_TASK_STACK_SIZE, (void *)netif, configETHERNET_INPUT_TASK_PRIORITY, &xEthIntTask );  
  
  /* Enable the MAC itself. */
  enet->ecr |= MACNET_ECR_ETHEREN_MASK;

  /* Indicate that there have been empty receive buffers produced */
  enet->rdar = MACNET_RDAR_RDAR_MASK;
  
 
  
  /*    Enable PHY Link Up-Down  Interrupt  *****************Like add     */
  mii_write( MACNET_PORT, configPHY_ADDRESS, PHY_INTCTL, ( PHY_INTCTL_LINK_DOWN_ENABLE | PHY_INTCTL_LINK_UP_ENABLE ) );
  mii_read( MACNET_PORT, configPHY_ADDRESS, PHY_INTCTL, &phy_reg_temp);
  PORTB_PCR9 |= PORT_PCR_ISF_MASK;
  msleep(500);
  INT_SYS_EnableIRQ(60);        //place PHY and GPIO in a certain status
  
  /*mii_read( MACNET_PORT, configPHY_ADDRESS, PHY_BMSR, &phy_reg_temp);
  mii_read( MACNET_PORT, configPHY_ADDRESS, PHY_BMSR, &phy_reg_temp);
  mii_read( MACNET_PORT, configPHY_ADDRESS, PHY_BMSR, &phy_reg_temp);
  
  mii_read( MACNET_PORT, configPHY_ADDRESS, PHY_INTCTL, &phy_reg_temp);
  mii_read( MACNET_PORT, configPHY_ADDRESS, PHY_INTCTL, &phy_reg_temp);
  mii_read( MACNET_PORT, configPHY_ADDRESS, PHY_INTCTL, &phy_reg_temp); 
  
  eth_phy_reg_dump(MACNET_PORT, configPHY_ADDRESS);*/           //test code
  
  
  mico_ethif_up();

}

/**
 * This function should do the actual transmission of the packet. The packet is
 * contained in the pbuf that is passed to the function. This pbuf
 * might be chained.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
 * @return ERR_OK if the packet could be sent
 *         an err_t value if the packet couldn't be sent
 *
 * @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
 *       strange results. You might consider waiting for space in the DMA queue
 *       to become availale since the stack doesn't retry to send a packet
 *       dropped because of memory failure (except for the TCP timers).
 */
int mico_ethif_low_level_output(uint8_t *buf, int len)
{
  unsigned char *pcTxData = NULL;
  portBASE_TYPE i;
#if (MACNET_PORT==0)
  volatile macnet_t *enet = (macnet_t *)MACNET_BASE_PTR;
#else
  volatile macnet_t *enet = (macnet_t *)(MACNET_BASE_PTR+MAC_NET_OFFSET);
#endif
  //initiate transfer();
#if (ENET_HARDWARE_SHIFT==0)
#if ETH_PAD_SIZE
  //pbuf_header(p, -ETH_PAD_SIZE); /* drop the padding word */	//cutworth, do we need to handle padding word?
#endif
#endif

  /* Get a DMA buffer into which we can write the data to send. */
  for( i = 0; i < netifBUFFER_WAIT_ATTEMPTS; i++ )
  {
    if( xENETTxDescriptors[ uxNextTxBuffer ].status & TX_BD_R )
    {
      /* Wait for the buffer to become available. */
      //vTaskDelay( netifBUFFER_WAIT_DELAY );	//cutworth, need something from Mico OS for the delay
      msleep(10);
    }
    else
    {
      pcTxData = (unsigned char *)REVERSE32((uint32_t)xENETTxDescriptors[ uxNextTxBuffer ].data);
      break;
    }
  }

  if( pcTxData == NULL ) 
  {
    /* For break point only. */
    NOP_ASM;
    return -1;
  }
  else
  { 
     memcpy(pcTxData, buf, len);
  }
  
  //signal that packet should be sent();
  
  /* Setup the buffer descriptor for transmission */
  xENETTxDescriptors[ uxNextTxBuffer ].length = REVERSE16(len);//nbuf->length + ETH_HDR_LEN;
  xENETTxDescriptors[ uxNextTxBuffer ].status |= (TX_BD_R | TX_BD_L);
  
  uxNextTxBuffer++;
  if( uxNextTxBuffer >= configNUM_ENET_TX_BUFFERS )
  {
    uxNextTxBuffer = 0;
  }

#if (ENET_HARDWARE_SHIFT==0)
#if ETH_PAD_SIZE
  //pbuf_header(p, ETH_PAD_SIZE); /* reclaim the padding word */
#endif
#endif

  /* only one task can be here. wait until pkt is sent, then go ahead */
  /* semaphore released inside isr */
  /*start expiring semaphore: no more than 3 ticks*/
  /*no blocking code*/
  mico_rtos_get_semaphore( &xTxENETSemaphore, 100);	//how to interact with ISR?
    
  /* Request xmit process to MAC-NET */
  enet->tdar = MACNET_TDAR_TDAR_MASK;

  return 0;
}


#if configENET_RX_SINGLE_BUFFER
/**
 * Should allocate a pbuf and transfer the bytes of the incoming
 * packet from the interface into the pbuf.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return a pbuf filled with the received packet (including MAC header)
 *         NULL on memory error
 */
int mico_ethif_low_level_input(uint8_t *buf, int maxlen)
{
  uint16_t len;
  
#if (MACNET_PORT==0)
  volatile macnet_t *enet = (macnet_t *)MACNET_BASE_PTR;
#else
  volatile macnet_t *enet = (macnet_t *)(MACNET_BASE_PTR+MAC_NET_OFFSET);
#endif
  
  uint8_t *data_temp;
  
  /* Obtain the size of the packet and put it into the "len"
     variable. */
  len = REVERSE16(xENETRxDescriptors[ uxNextRxBuffer ].length);

  if( ( len != 0 ) && ( ( xENETRxDescriptors[ uxNextRxBuffer ].status & RX_BD_E ) == 0 ) )
  {  
     #if (ENET_HARDWARE_SHIFT==0)
     #if ETH_PAD_SIZE
     len += ETH_PAD_SIZE; /* allow room for Ethernet padding */
     #endif
     #endif
     
     data_temp = (uint8_t *)REVERSE32((uint32_t)xENETRxDescriptors[ uxNextRxBuffer ].data);
     
     if ((len > 0) && (len <= maxlen)) {
        memcpy(buf, data_temp, len);
     } else {
        len = 0;
     }
    
     //acknowledge that packet has been read();
     /* Free the descriptor. */
     xENETRxDescriptors[ uxNextRxBuffer ].status |= RX_BD_E;
     enet->rdar = MACNET_RDAR_RDAR_MASK;
    
     uxNextRxBuffer++;
     if( uxNextRxBuffer >= configNUM_ENET_RX_BUFFERS )
     {
        uxNextRxBuffer = 0;
     }
  } 
  
  return len;  
}
#else
int mico_ethif_low_level_input(uint8_t *buf, int maxlen)
{
    uint16_t len;
#if (MACNET_PORT==0)
  volatile macnet_t *enet = (macnet_t *)MACNET_BASE_PTR;
#else
  volatile macnet_t *enet = (macnet_t *)(MACNET_BASE_PTR+MAC_NET_OFFSET);
#endif
    uint8_t *data_temp;
    uint8_t more_pkts;

  //// (void)netif;

    more_pkts = TRUE;

    /*initial pkt handling*/
	if (!(xENETRxDescriptors[uxNextRxBuffer].status & RX_BD_E))/*if pkt is filled*/
	{
		if (xENETRxDescriptors[uxNextRxBuffer].status & RX_BD_L)
		{
			more_pkts = FALSE;
			if (xENETRxDescriptors[uxNextRxBuffer].status & (RX_BD_LG | RX_BD_NO | RX_BD_CR | RX_BD_OV))
			{
				/*wrong packet*/
				goto exit_rx_pkt;
			}
			else
			{
				len = REVERSE16(xENETRxDescriptors[uxNextRxBuffer].length);
			}
		}
		else/*if not L bit, then buffer's length*/
			len = configENET_RX_BUFFER_SIZE;

		data_temp = (uint8_t *)REVERSE32((uint32_t)xENETRxDescriptors[uxNextRxBuffer].data);
                
                if((len > 0) && (len < maxlen))
		  memcpy(buf, data_temp), len);
                else
                  len = 0;
	exit_rx_pkt:
		xENETRxDescriptors[uxNextRxBuffer++].status |= RX_BD_E;/*consumed pkt*/
		enet->rdar = MACNET_RDAR_RDAR_MASK;
		if (uxNextRxBuffer >= configNUM_ENET_RX_BUFFERS)
			uxNextRxBuffer = 0;
	}
	else
		return -1; // (struct pbuf *)NULL;/*FSL: special NULL case*/

    /*more pkts handling*/	
    while(more_pkts)
    {
       //if(!(xENETRxDescriptors[ uxNextRxBuffer ].status & RX_BD_E) )
       ///*if pkt is filled*/
       //{
       if( xENETRxDescriptors[ uxNextRxBuffer ].status & RX_BD_L )
       {
          more_pkts = FALSE; 
          if( xENETRxDescriptors[ uxNextRxBuffer ].status & (RX_BD_LG|RX_BD_NO|RX_BD_CR|RX_BD_OV) )
          {
             /*wrong packet*/
             goto exit_rx_pkt2;
          }
          else
          {
             len = REVERSE16(xENETRxDescriptors[ uxNextRxBuffer ].length);
             /*buffer with L bit has total frame's length instead of remaining bytes from frame's lenght*/
             len %= configENET_RX_BUFFER_SIZE;
          }
       }
       else/*if not L bit, then buffer's length*/
          len = configENET_RX_BUFFER_SIZE;

      data_temp = (uint8_t *)REVERSE32((uint32_t)xENETRxDescriptors[uxNextRxBuffer].data);
           
      if((len > 0)&&(len < maxlen))
	memcpy((uint8_t*)buf, data_temp[0], len);
      else
        len = 0;
       exit_rx_pkt2:
           xENETRxDescriptors[ uxNextRxBuffer++ ].status |= RX_BD_E;/*consumed pkt*/
           enet->rdar = MACNET_RDAR_RDAR_MASK;
           if( uxNextRxBuffer >= configNUM_ENET_RX_BUFFERS )
              uxNextRxBuffer = 0;  
       //}
    }
    
	return 0;// first_pbuf;
}
#endif

/*-----------------------------------------------------------*/

static void prvInitialiseENETBuffers( void )
{
unsigned portBASE_TYPE ux;
unsigned char *pcBufPointer;

  pcBufPointer = &( xENETTxDescriptors_unaligned[ 0 ] );
  while( ( ( unsigned long ) pcBufPointer & 0x0fUL ) != 0 )
  {
    pcBufPointer++;
  }
  
  xENETTxDescriptors = ( NBUF * ) pcBufPointer;
  
  pcBufPointer = &( xENETRxDescriptors_unaligned[ 0 ] );
  while( ( ( unsigned long ) pcBufPointer & 0x0fUL ) != 0 )
  {
    pcBufPointer++;
  }
  
  xENETRxDescriptors = ( NBUF * ) pcBufPointer;

  /* Setup the buffers and descriptors. */
  pcBufPointer = &( ucENETTxBuffers[ 0 ] );
  while( ( ( unsigned long ) pcBufPointer & 0x0fUL ) != 0 )
  {
    pcBufPointer++;
  }

  for( ux = 0; ux < configNUM_ENET_TX_BUFFERS; ux++ )
  {
    xENETTxDescriptors[ ux ].status = TX_BD_TC;
    xENETTxDescriptors[ ux ].data = (uint8_t *)REVERSE32((uint32_t)pcBufPointer);
    pcBufPointer += configENET_TX_BUFFER_SIZE;
    xENETTxDescriptors[ ux ].length = 0;
    #if (ENHANCED_BD==1)
    xENETTxDescriptors[ ux ].ebd_status = TX_BD_INT/*new!*/
#if (ENET_HARDWARE_CHECKSUM && (MACNET_VERSION!=MACNET_LEGACY) )
                                        | TX_BD_IINS
                                        | TX_BD_PINS
    #endif
    ;
    #endif
  }

  pcBufPointer = &( ucENETRxBuffers[ 0 ] );
  while( ( ( unsigned long ) pcBufPointer & 0x0fUL ) != 0 )
  {
    pcBufPointer++;
  }
  
  for( ux = 0; ux < configNUM_ENET_RX_BUFFERS; ux++ )
  {
      xENETRxDescriptors[ ux ].status = RX_BD_E;
      xENETRxDescriptors[ ux ].length = 0;
      xENETRxDescriptors[ ux ].data = (uint8_t *)REVERSE32((uint32_t)pcBufPointer);
      pcBufPointer += configENET_RX_BUFFER_SIZE;
      #if (ENHANCED_BD==1)
      xENETRxDescriptors[ ux ].bdu = 0x00000000;
      xENETRxDescriptors[ ux ].ebd_status = RX_BD_INT;
      #endif    
  }

  /* Set the wrap bit in the last descriptors to form a ring. */
  xENETTxDescriptors[ configNUM_ENET_TX_BUFFERS - 1 ].status |= TX_BD_W;
  xENETRxDescriptors[ configNUM_ENET_RX_BUFFERS - 1 ].status |= RX_BD_W;

  uxNextRxBuffer = 0;
  uxNextTxBuffer = 0;
}
/*-----------------------------------------------------------*/


ISR_PREFIX void vENETISRHandler( void )
{
  unsigned long eir;
  unsigned long eimr;
  unsigned long ulEvent;
//portBASE_TYPE xHighPriorityTaskWoken = pdFALSE;
#if (MACNET_PORT==0)
  volatile macnet_t *enet = (macnet_t *)MACNET_BASE_PTR;
#else
  volatile macnet_t *enet = (macnet_t *)(MACNET_BASE_PTR+MAC_NET_OFFSET);
#endif

  eir = enet-> eir;
  eimr = enet->eimr;
  
  /* Determine the cause of the interrupt. */
  ulEvent = eir & eimr;
  enet->eir = ulEvent;

  /*Tx Process: only aware of a complete eth frame*/
  if( /*( ulEvent & MACNET_EIR_TXB_MASK ) ||*/ ( ulEvent & MACNET_EIR_TXF_MASK ) )
  {    
    /* xmit task completed, go for next one! */
    mico_rtos_set_semaphore( &xTxENETSemaphore);
  }
  /*Rx process*/
  if( /*( ulEvent & MACNET_EIR_RXB_MASK ) ||*/ ( ulEvent & MACNET_EIR_RXF_MASK ) )
  {    
    /* A packet has been received.  Wake the handler task. */
    mico_rtos_set_semaphore( &xRxENETSemaphore);
    mico_ethif_notify_irq();
  }
  /*Error handling*/
  if (ulEvent & ( MACNET_EIR_UN_MASK | MACNET_EIR_RL_MASK | MACNET_EIR_LC_MASK | MACNET_EIR_EBERR_MASK | MACNET_EIR_BABT_MASK | MACNET_EIR_BABR_MASK | MACNET_EIR_EBERR_MASK ) )
  {
    /*(void)ulEvent;*/
    /* Sledge hammer error handling. */
    prvInitialiseENETBuffers();
    enet->rdar = MACNET_RDAR_RDAR_MASK;
  }
}


void ENET_Transmit_IRQHandler(){
  vENETISRHandler();
}

void ENET_Receive_IRQHandler(){
  vENETISRHandler();
}

void ENET_Error_IRQHandler() {
  vENETISRHandler();
}

#endif /* 0 */
