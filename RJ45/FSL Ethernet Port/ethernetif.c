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

////#include "lwip/opt.h"
#include "FreeRTOSConfig.h"

#define portCHAR                           char
#define portFLOAT                         float
#define portDOUBLE                      double
#define portBASE_TYPE                long
#define portLONG                 long
#define portSHORT               short
#define uint32_t   u32_t
#define uint8_t    u8_t
#define TRUE  0
#define FALSE 1
#define pdFALSE 0


#if 1 /* ready */

////#include "lwip/def.h"
////#include "lwip/mem.h"
////#include "lwip/pbuf.h"
////#include "lwip/sys.h"
////#include <lwip/stats.h>
////#include <lwip/snmp.h>
//#include "netif/etharp.h"
//#include "netif/ppp_oe.h"

#if 0
/* Kernel includes. */
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#endif

#include "common.h"
/* Standard library includes. */
#include <string.h>

/* Demo includes. */
#include "eth_phy.h"
#include "enet.h"
#include "mii.h"
#include "macnet.h"/*register definition*/

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

#if 0
/* Semaphores used by the ENET interrupt handler to wake the handler task. */
static xSemaphoreHandle xRxENETSemaphore;
static xSemaphoreHandle xTxENETSemaphore;
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
static void low_level_init( struct netif *netif );
//static err_t low_level_output(struct netif *netif, struct pbuf *p);
static struct pbuf *low_level_input(struct netif *netif);

/* Forward declarations. */
static void  ethernetif_input(/*FSL:struct netif *netif*/void *pParams);

/**
 * In this function, the hardware should be initialized.
 * Called from ethernetif_init().
 *
 * @param netif the already initialized lwip network interface structure
 *        for this ethernetif
 */
 void mico_ethif_low_level_init(char *mac)
{
/*unsigned portLONG*/ int usData;
const unsigned portCHAR ucMACAddress[6] = 
{
  configMAC_ADDR0, configMAC_ADDR1,configMAC_ADDR2,configMAC_ADDR3,configMAC_ADDR4,configMAC_ADDR5
};

//FSL:struct ethernetif *ethernetif = netif->state;
#if (MACNET_PORT==0)
  volatile macnet_t *enet = (macnet_t *)MACNET_BASE_PTR;
#else
  volatile macnet_t *enet = (macnet_t *)(MACNET_BASE_PTR+MAC_NET_OFFSET);
#endif
  
#if 0
  /* set MAC hardware address length */
  netif->hwaddr_len = ETHARP_HWADDR_LEN;

  /* set MAC hardware address */
  netif->hwaddr[0] = configMAC_ADDR0;
  netif->hwaddr[1] = configMAC_ADDR1;
  netif->hwaddr[2] = configMAC_ADDR2;
  netif->hwaddr[3] = configMAC_ADDR3;
  netif->hwaddr[4] = configMAC_ADDR4;
  netif->hwaddr[5] = configMAC_ADDR5;

  /* maximum transfer unit */
  netif->mtu = configENET_MAX_PACKET_SIZE-20;
  
  /* device capabilities */
  /* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
  netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP;
#endif

  /* Do whatever else is needed to initialize interface. */  
  
  ENABLE_MAC_NET;        
        
  prvInitialiseENETBuffers();
  
#if 0
  /*FSL: create semaphores*/
  vSemaphoreCreateBinary( xRxENETSemaphore );
  vSemaphoreCreateBinary( xTxENETSemaphore );
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
  ENET_INTERRUPTS_ENABLE;
        
  /*
   * Make sure the external interface signals are enabled
   */
  ENET_MII_PINS;   

#if configUSE_MII_MODE
  ENET_MII_MODE_PINS;
#else
  ENET_RMII_MODE_PINS;
#endif   
    
  /* Can we talk to the PHY? */
  do
  {
    //vTaskDelay( netifLINK_DELAY );	//cutworth, need to change to something from Mico OS
    usData = 0xffff;
    mii_read( MACNET_PORT, configPHY_ADDRESS, PHY_PHYIDR1, &usData );
        
  } while( usData == 0xffff );

  /* Start auto negotiate. */
  mii_write( MACNET_PORT, configPHY_ADDRESS, PHY_BMCR, ( PHY_BMCR_AN_RESTART | PHY_BMCR_AN_ENABLE ) );

  /* Wait for auto negotiate to complete. */
  do
  {
    //vTaskDelay( netifLINK_DELAY );	//cutworth, need to change to something from Mico OS
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
  enet_set_address( MACNET_PORT, ucMACAddress );
        
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

 //// if (EthStatus == ETH_SUCCESS) {
	  mico_ethif_up();
 //// }
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
 int mico_ethif_low_level_output/*(struct netif *netif, struct pbuf *p)*/(uint8_t *buf, int len)
{
//FSL:  struct ethernetif *ethernetif = netif->state;
  uint16_t l = 0;
  struct pbuf *q;
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

   // return ERR_BUF;
    return -1;
  }
  else
  { 
	  memcpy(&pcTxData[l], buf, len);
#if 0
    for(q = p; q != NULL; q = q->next) 
    {
      /* Send the data from the pbuf to the interface, one pbuf at a
         time. The size of the data in each pbuf is kept in the ->len
         variable. */
      memcpy( &pcTxData[l], (u8_t*)q->payload, q->len );
      l += q->len;
    }    
#endif
  }
  
  //signal that packet should be sent();
  
  /* Setup the buffer descriptor for transmission */
  xENETTxDescriptors[ uxNextTxBuffer ].length = REVERSE16(l);//nbuf->length + ETH_HDR_LEN;
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
  
  ////LINK_STATS_INC(link.xmit);

  /* only one task can be here. wait until pkt is sent, then go ahead */
  /* semaphore released inside isr */
  /*start expiring semaphore: no more than 3 ticks*/
  /*no blocking code*/
  //xSemaphoreTake( xTxENETSemaphore, 3/*1/portTICK_RATE_MS*//*portMAX_DELAY*/);	//how to interact with ISR?
    
  /* Request xmit process to MAC-NET */
  enet->tdar = MACNET_TDAR_TDAR_MASK;
    
  //return ERR_OK;
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
static struct pbuf * low_level_input/*(struct netif *netif)*/(uint8_t *buf, int maxlen)
{
  u32_t l = 0;
  struct pbuf *p, *q;
  u16_t len;
  
#if (MACNET_PORT==0)
  volatile macnet_t *enet = (macnet_t *)MACNET_BASE_PTR;
#else
  volatile macnet_t *enet = (macnet_t *)(MACNET_BASE_PTR+MAC_NET_OFFSET);
#endif
  
  u8_t *data_temp;

  ( void ) netif;
  
  l = 0;
  p = NULL;
  
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

     /* We allocate a pbuf chain of pbufs from the pool. */
     p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);
  
     if (p != NULL) 
     {
        #if (ENET_HARDWARE_SHIFT==0)
        #if ETH_PAD_SIZE
        pbuf_header(p, -ETH_PAD_SIZE); /* drop the padding word */
        #endif
        #endif

        /* We iterate over the pbuf chain until we have read the entire
         * packet into the pbuf. */
        for(q = p; q != NULL; q = q->next) 
        {
           /* Read enough bytes to fill this pbuf in the chain. The
            * available data in the pbuf is given by the q->len
            * variable.
            * This does not necessarily have to be a memcpy, you can also preallocate
            * pbufs for a DMA-enabled MAC and after receiving truncate it to the
            * actually received size. In this case, ensure the tot_len member of the
            * pbuf is the sum of the chained pbuf len members.
            */
            data_temp = (u8_t *)REVERSE32((u32_t)xENETRxDescriptors[ uxNextRxBuffer ].data);
            memcpy((u8_t*)q->payload, &( data_temp[l] ), q->len);
            l = l + q->len;
        }

        #if (ENET_HARDWARE_SHIFT==0)
        #if ETH_PAD_SIZE
        pbuf_header(p, ETH_PAD_SIZE); /* reclaim the padding word */
        #endif
        #endif
        LINK_STATS_INC(link.recv);        
     }
     else
     {
        //drop packet();
        LINK_STATS_INC(link.memerr);
        LINK_STATS_INC(link.drop);     
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
  
  return p;  
}
#else
int mico_ethif_low_level_input/*(struct netif *netif)*/(uint8_t *buf, int maxlen)
{
    uint16_t l, temp_l;
    struct pbuf *first_pbuf,*next_pbuf,*q;
    uint16_t len;
#if (MACNET_PORT==0)
  volatile macnet_t *enet = (macnet_t *)MACNET_BASE_PTR;
#else
  volatile macnet_t *enet = (macnet_t *)(MACNET_BASE_PTR+MAC_NET_OFFSET);
#endif
    uint8_t *data_temp;
    uint8_t more_pkts, processing_error;

  //// (void)netif;

    more_pkts = TRUE;
    processing_error = FALSE;

    /*initial pkt handling*/
	if (!(xENETRxDescriptors[uxNextRxBuffer].status & RX_BD_E))/*if pkt is filled*/
	{
		if (xENETRxDescriptors[uxNextRxBuffer].status & RX_BD_L)
		{
			more_pkts = FALSE;
			if (xENETRxDescriptors[uxNextRxBuffer].status & (RX_BD_LG | RX_BD_NO | RX_BD_CR | RX_BD_OV))
			{
				/*wrong packet*/
				//LINK_STATS_INC(link.memerr);
				//LINK_STATS_INC(link.drop);
				goto exit_rx_pkt;
			}
			else
			{
				len = REVERSE16(xENETRxDescriptors[uxNextRxBuffer].length);
				//LINK_STATS_INC(link.recv);
			}
		}
		else/*if not L bit, then buffer's length*/
			len = configENET_RX_BUFFER_SIZE;

		data_temp = (uint8_t *)REVERSE32((uint32_t)xENETRxDescriptors[uxNextRxBuffer].data);
		memcpy(buf, &(data_temp[l]), len);
#if 0
		if ((first_pbuf = pbuf_alloc(PBUF_RAW, len, PBUF_POOL)) != NULL)
		{
			/*get data*/
			l = 0;
			temp_l = 0;
			/* We iterate over the pbuf chain until we have read the entire
		   * packet into the pbuf. */
			for (q = first_pbuf; q != NULL; q = q->next)
			{
				/* Read enough bytes to fill this pbuf in the chain. The
			* available data in the pbuf is given by the q->len
			* variable.
			* This does not necessarily have to be a memcpy, you can also preallocate
			* pbufs for a DMA-enabled MAC and after receiving truncate it to the
			* actually received size. In this case, ensure the tot_len member of the
			* pbuf is the sum of the chained pbuf len members.
			*/
				temp_l = LWIP_MIN(len, LWIP_MEM_ALIGN_SIZE(PBUF_POOL_BUFSIZE));
				data_temp = (u8_t *)REVERSE32((u32_t)xENETRxDescriptors[uxNextRxBuffer].data);
				memcpy((u8_t*)q->payload, &(data_temp[l]), temp_l);
				l += temp_l;
				len -= temp_l;
			}
		}
		else
		{
			/*wrong buffers*/
			LINK_STATS_INC(link.memerr);
			LINK_STATS_INC(link.drop);
			processing_error = TRUE;
		}
#endif
	exit_rx_pkt:
		xENETRxDescriptors[uxNextRxBuffer++].status |= RX_BD_E;/*consumed pkt*/
		enet->rdar = MACNET_RDAR_RDAR_MASK;
		if (uxNextRxBuffer >= configNUM_ENET_RX_BUFFERS)
			uxNextRxBuffer = 0;
	}
	else
		return -1; // (struct pbuf *)NULL;/*FSL: special NULL case*/

    /*more pkts handling*/	//cutworth, should I handle more packets like this?
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
             //LINK_STATS_INC(link.memerr);
             //LINK_STATS_INC(link.drop);
             goto exit_rx_pkt2;
          }
          else
          {
             len = REVERSE16(xENETRxDescriptors[ uxNextRxBuffer ].length);
             /*buffer with L bit has total frame's length instead of remaining bytes from frame's lenght*/
             len %= configENET_RX_BUFFER_SIZE;
             //LINK_STATS_INC(link.recv);
          }
       }
       else/*if not L bit, then buffer's length*/
          len = configENET_RX_BUFFER_SIZE;

	   data_temp = (u8_t *)REVERSE32((u32_t)xENETRxDescriptors[uxNextRxBuffer].data);
	   memcpy((u8_t*)buf, &(data_temp[l]), len);

#if 0       
       if( ((next_pbuf = pbuf_alloc(PBUF_RAW, len, PBUF_POOL)) != NULL) && (!processing_error) )
       {
        /*get data*/
        
        l = 0;
        temp_l = 0;    

        /* We iterate over the pbuf chain until we have read the entire
         * packet into the pbuf. */
        for(q = next_pbuf; q != NULL; q = q->next) 
        {
           /* Read enough bytes to fill this pbuf in the chain. The
            * available data in the pbuf is given by the q->len
            * variable.
            * This does not necessarily have to be a memcpy, you can also preallocate
            * pbufs for a DMA-enabled MAC and after receiving truncate it to the
            * actually received size. In this case, ensure the tot_len member of the
            * pbuf is the sum of the chained pbuf len members.
            */
            temp_l = LWIP_MIN(len, LWIP_MEM_ALIGN_SIZE(PBUF_POOL_BUFSIZE));
            data_temp = (u8_t *)REVERSE32((u32_t)xENETRxDescriptors[ uxNextRxBuffer ].data);
            memcpy((u8_t*)q->payload, &( data_temp[l] ), temp_l);
            l += temp_l;
            len -= temp_l;
        }
         
        /*link pbufs*/
        pbuf_cat(first_pbuf,next_pbuf);
       }
       else
       {
           /*wrong buffer: out of lwip buffers*/
           LINK_STATS_INC(link.memerr);
           LINK_STATS_INC(link.drop);
           processing_error = TRUE;
       }
#endif
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
/**
 * This function should be called when a packet is ready to be read
 * from the interface. It uses the function low_level_input() that
 * should handle the actual reception of bytes from the network
 * interface. Then the type of the received packet is determined and
 * the appropriate input function is called.
 *
 * @param netif the lwip network interface structure for this ethernetif
 */
#if 0
static void ethernetif_input(void *pParams)
{
//FSL:  struct ethernetif *ethernetif;
  struct netif *netif;
  struct eth_hdr *ethhdr;
  struct pbuf *p;

//FSL:  ethernetif = netif->state;
  netif = (struct netif*) pParams;

  for( ;; )
  {  
    do
    {
      /* move received packet into a new pbuf */
      p = low_level_input(netif);
      /* no packet could be read, silently ignore this */
      if (p == NULL)
      {
	/* No packet could be read.  Wait a for an interrupt to tell us
	   there is more data available. */
	xSemaphoreTake( xRxENETSemaphore, /*netifBLOCK_TIME_WAITING_FOR_INPUT*/portMAX_DELAY );        
      }
    }while( p == NULL );  
    /* points to packet payload, which starts with an Ethernet header */
    ethhdr = p->payload;

    switch (htons(ethhdr->type)) 
    {
      /* IP or ARP packet? */
      case ETHTYPE_IP:
      case ETHTYPE_ARP:
    #if PPPOE_SUPPORT
      /* PPPoE packet? */
      case ETHTYPE_PPPOEDISC:
      case ETHTYPE_PPPOE:
    #endif /* PPPOE_SUPPORT */
        /* full packet send to tcpip_thread to process */
        if (netif->input(p, netif)!=ERR_OK)
         { LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_input: IP input error\n"));
           pbuf_free(p);
           p = NULL;
         }
        break;
    
      default:
        pbuf_free(p);
        p = NULL;
        break;
    }
  }
}
#endif
/**
 * Should be called at the beginning of the program to set up the
 * network interface. It calls the function low_level_init() to do the
 * actual setup of the hardware.
 *
 * This function should be passed as a parameter to netif_add().
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return ERR_OK if the loopif is initialized
 *         ERR_MEM if private data couldn't be allocated
 *         any other err_t on error
 */
#if 0
err_t ethernetif_init(struct netif *netif)
{
  struct ethernetif *ethernetif;

  LWIP_ASSERT("netif != NULL", (netif != NULL));
    
  ethernetif = mem_malloc(sizeof(struct ethernetif));
  if (ethernetif == NULL) {
    LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_init: out of memory\n"));
    return ERR_MEM;
  }

#if LWIP_NETIF_HOSTNAME
  /* Initialize interface hostname */
  netif->hostname = "lwip";
#endif /* LWIP_NETIF_HOSTNAME */

  /*
   * Initialize the snmp variables and counters inside the struct netif.
   * The last argument should be replaced with your link speed, in units
   * of bits per second.
   */
  NETIF_INIT_SNMP(netif, snmp_ifType_ethernet_csmacd, LINK_SPEED_OF_YOUR_NETIF_IN_BPS);

  netif->state = ethernetif;
  netif->name[0] = IFNAME0;
  netif->name[1] = IFNAME1;
  /* We directly use etharp_output() here to save a function call.
   * You can instead declare your own function an call etharp_output()
   * from it if you have to do some checks before sending (e.g. if link
   * is available...) */
  netif->output = etharp_output;
  netif->linkoutput = low_level_output;
  
  ethernetif->ethaddr = (struct eth_addr *)&(netif->hwaddr[0]);
  
  /* initialize the hardware */
  low_level_init(netif);

  return ERR_OK;
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
unsigned long ulEvent;
portBASE_TYPE xHighPriorityTaskWoken = pdFALSE;
#if (MACNET_PORT==0)
  volatile macnet_t *enet = (macnet_t *)MACNET_BASE_PTR;
#else
  volatile macnet_t *enet = (macnet_t *)(MACNET_BASE_PTR+MAC_NET_OFFSET);
#endif

  /* Determine the cause of the interrupt. */
  ulEvent = enet->eir & enet->eimr;
  enet->eir = ulEvent;

  /*Tx Process: only aware of a complete eth frame*/
  if( /*( ulEvent & MACNET_EIR_TXB_MASK ) ||*/ ( ulEvent & MACNET_EIR_TXF_MASK ) )
  {    
    /* xmit task completed, go for next one! */
    //xSemaphoreGiveFromISR( xTxENETSemaphore, &xHighPriorityTaskWoken );
  }
  /*Rx process*/
  if( /*( ulEvent & MACNET_EIR_RXB_MASK ) ||*/ ( ulEvent & MACNET_EIR_RXF_MASK ) )
  {    
    /* A packet has been received.  Wake the handler task. */
    //xSemaphoreGiveFromISR( xRxENETSemaphore, &xHighPriorityTaskWoken );
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

  //portEND_SWITCHING_ISR( xHighPriorityTaskWoken );
}

#endif /* 0 */
