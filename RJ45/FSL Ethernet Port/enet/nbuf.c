/* Buffer Descriptors -- must be aligned on a 4-byte boundary but a 
 * 16-byte boundary is recommended. To avoid playing games with the 
 * various compilers and their different extension to ANSI C, these 
 * buffers are aligned by allocating an extra line of data and 
 * adjusting the pointers in nbuf_init().
 */

#include "common.h"
#include "nbuf.h"
#include "macnet.h"/*register definition*/
#ifdef MACNET_SWITCH_SUPPORT
#include "macsw.h"
#endif

/*FSL: to avoid overlapping, buffers must be declared at the beggining of file*/
/*then pointers can access them correctly*/

/* Data Buffers -- must be aligned on a 16-byte boundary. To avoid 
 * playing games with the various compilers and their different 
 * extension to ANSI C, these buffers are aligned by allocating an 
 * extra line of data and adjusting the pointers in nbuf_init().
 */
static uint8_t unaligned_txbd[(sizeof(NBUF) * NUM_TXBDS) + 16];
static uint8_t unaligned_rxbd[(sizeof(NBUF) * NUM_RXBDS) + 16];
#ifdef USE_DEDICATED_TX_BUFFERS
static uint8_t unaligned_txbuffer[(TX_BUFFER_SIZE * NUM_TXBDS) + 16];
#endif
#ifdef ENET_BUFFERS_OUTSIDE_OF_SRAM
#endif
static uint8_t unaligned_rxbuffer[(RX_BUFFER_SIZE * NUM_RXBDS) + 16];

/* Pointers to alligned buffer descriptors */
static NBUF *TxNBUF;
static NBUF *RxNBUF;

/* Pointers to alligned Tx/Rx data buffers */
#ifdef USE_DEDICATED_TX_BUFFERS
static uint8_t *TxBuffer;
#endif
static uint8_t *RxBuffer;

/* Next BD indicators for static BD queue */ 
static int next_txbd;
static int next_rxbd;

/********************************************************************/
void 
nbuf_alloc(int ch)
{
	int i;
	//uint32_t *temp;
        (void)ch;

        next_txbd = 0;
        next_rxbd = 0;

	TxNBUF = (NBUF *)(((uint32_t)(unaligned_txbd)) & 0xFFFFFFF0);
	RxNBUF = (NBUF *)(((uint32_t)(unaligned_rxbd)) & 0xFFFFFFF0);

	RxBuffer = (uint8_t *)(((uint32_t)(unaligned_rxbuffer)) & 0xFFFFFFF0);
#ifdef USE_DEDICATED_TX_BUFFERS
	TxBuffer = (uint8_t *)(((uint32_t)(unaligned_txbuffer)) & 0xFFFFFFF0);
#endif
	// Initialize transmit descriptor ring
	for (i = 0; i < NUM_TXBDS; i++)
	{
		TxNBUF[i].status = 0x0000;
		TxNBUF[i].length = 0;		
	        #ifdef USE_DEDICATED_TX_BUFFERS
	           TxNBUF[i].data = (uint8_t *)REVERSE32((uint32_t)&TxBuffer[i * TX_BUFFER_SIZE]);
                #endif
        
                #if (ENHANCED_BD==1)
                   TxNBUF[i].ebd_status = TX_BD_IINS | TX_BD_PINS;
                #endif
	}

	// Initialize receive descriptor ring
	for (i = 0; i < NUM_RXBDS; i++)
	{
		RxNBUF[i].status = RX_BD_E;
		RxNBUF[i].length = 0;
		RxNBUF[i].data = (uint8_t *)REVERSE32((uint32_t)&RxBuffer[i * RX_BUFFER_SIZE]);

                #if (ENHANCED_BD==1)
	        RxNBUF[i].bdu = 0x00000000;
	        RxNBUF[i].ebd_status = RX_BD_INT;
                #endif               
	}
        
	// Set the Wrap bit on the last one in the ring
	RxNBUF[NUM_RXBDS - 1].status |= RX_BD_W;
	TxNBUF[NUM_TXBDS - 1].status |= TX_BD_W;
}
/********************************************************************/
void 
nbuf_flush(int ch)
{
	int i;

	next_txbd = 0;
	next_rxbd = 0;
	
        (void)ch;
        
	// Reset enet hardware bd pointers also ??

	// Reset receive descriptor ring
	for (i = 0; i < NUM_RXBDS; i++)
	{
		RxNBUF[i].status = RX_BD_E;
		RxNBUF[i].length = 0;
		RxNBUF[i].data = (uint8_t *)REVERSE32((uint32_t)&RxBuffer[i * RX_BUFFER_SIZE]);
	}

	// Reset transmit descriptor ring
	for (i = 0; i < NUM_TXBDS; i++)
	{
		TxNBUF[i].status = 0x0000;
		TxNBUF[i].length = 0;
	}

	// Set the Wrap bit on the last one in the ring
	RxNBUF[NUM_RXBDS - 1].status |= RX_BD_W;
	TxNBUF[NUM_TXBDS - 1].status |= TX_BD_W;
}
/********************************************************************/
void 
nbuf_init(int ch)
{
#ifndef MACNET_SWITCH_SUPPORT
    volatile macnet_t *enet;
    
    if(ch)
      enet = (macnet_t *)(MACNET_BASE_PTR+MAC_NET_OFFSET);
    else
      enet = (macnet_t *)MACNET_BASE_PTR;
  
    // Set Receive Buffer Size
    enet->mrbr = (uint16_t)RX_BUFFER_SIZE;  
  
    // Point to the start of the Tx buffer descriptor queue
    enet->tdsr = (uint32_t)TxNBUF;
    // Point to the start of the circular Rx buffer descriptor queue
    enet->rdsr = (uint32_t)RxNBUF;
#else
    volatile esw_t *esw = (esw_t *)MACSW_BASE_PTR;
    (void)ch;
    // Set Receive Buffer Size
    esw->mrbr = (uint16_t)RX_BUFFER_SIZE;  
  
    // Point to the start of the Tx buffer descriptor queue
    esw->tdsr = (uint32_t)TxNBUF;
    // Point to the start of the circular Rx buffer descriptor queue
    esw->rdsr = (uint32_t)RxNBUF;
#endif
}
/********************************************************************/
void 
nbuf_start_rx(int ch)
{
#ifndef MACNET_SWITCH_SUPPORT
    volatile macnet_t *enet;
    
    if(ch)
      enet = (macnet_t *)(MACNET_BASE_PTR+MAC_NET_OFFSET);
    else
      enet = (macnet_t *)MACNET_BASE_PTR;
    
    // Indicate Empty buffers have been produced
    enet->rdar = MACNET_RDAR_RDAR_MASK;

    while( !enet->rdar )
    {
        //If RDAR cannot be test,  
        //printf("Error with internal ENET DMA engine\n");
    }
#else
    volatile esw_t *esw = (esw_t *)MACSW_BASE_PTR;
    (void)ch;
    // Indicate Empty buffers have been produced
    esw->rdar = MACSW_RDAR_RDAR_MASK;

    while( !esw->rdar )
    {
        //If RDAR cannot be test,  
        //printf("Error with internal ENET-SW DMA engine\n");
    }
#endif
}
/********************************************************************/
void 
enet_get_received_packet(int ch, NBUF * rx_packet)
{
	int last_buffer;
	uint16_t status;
	int index_rxbd;

	last_buffer = 0;
	rx_packet->length = 0;
	(void)ch;
	index_rxbd = next_rxbd;
    
	if(RxNBUF[index_rxbd].status & RX_BD_E)
	{
		printf("Under processing. SHouldnt be here\n");
		return;	
	}
        rx_packet->data = (uint8_t *)REVERSE32((uint32_t)RxNBUF[index_rxbd].data);
	// Update next_rxbd pointer and mark buffers as empty again
	while(!last_buffer)
	{
		status = RxNBUF[index_rxbd].status;
		rx_packet->length = REVERSE16(RxNBUF[index_rxbd].length);
                #if (ENHANCED_BD==1)
		rx_packet->ebd_status = RxNBUF[index_rxbd].ebd_status;
		rx_packet->timestamp = REVERSE32(RxNBUF[index_rxbd].timestamp);
		rx_packet->length_proto_type = REVERSE16(RxNBUF[index_rxbd].length_proto_type);
		rx_packet->payload_checksum = REVERSE16(RxNBUF[index_rxbd].payload_checksum);
                #endif

		last_buffer = (status & RX_BD_L);
		if(status & RX_BD_W)
		{
			RxNBUF[index_rxbd].status = (RX_BD_W | RX_BD_E);
			index_rxbd = 0;
		}
		else
		{
			RxNBUF[index_rxbd].status = RX_BD_E;
			index_rxbd++;
		}
	}
	
	// Update the global rxbd index
	next_rxbd = index_rxbd;
	
	// Put the last BD status in rx_packet->status as MISS flags and more 
	// are updated in last BD
	rx_packet->status = status;
}
/********************************************************************/
void 
enet_fill_txbds(int ch, NBUF * tx_packet)
{
	int num_txbds, i;
	int index_txbd;
        (void)ch;
	num_txbds = (tx_packet->length/TX_BUFFER_SIZE);
	
	index_txbd = next_txbd;
	
	if((num_txbds * TX_BUFFER_SIZE) < tx_packet->length)
	{
		num_txbds = num_txbds + 1;
	}
    
	// Fill Descriptors
	for (i = 0; i < num_txbds; i++)
	{
		
		TxNBUF[index_txbd].status = TX_BD_TC | TX_BD_R;
                #if (ENHANCED_BD==1)
		TxNBUF[index_txbd].bdu = 0x00000000;
		TxNBUF[index_txbd].ebd_status = TX_BD_INT | TX_BD_TS;// | TX_BD_IINS | TX_BD_PINS;
                #endif

		if(i == num_txbds - 1)
		{
		    TxNBUF[index_txbd].length = REVERSE16((tx_packet->length - (i*TX_BUFFER_SIZE)));
		    // Set the Last bit on the last BD
		    TxNBUF[index_txbd].status |= TX_BD_L;		 
		}
		else
		{
		    TxNBUF[index_txbd].length = REVERSE16(TX_BUFFER_SIZE);
		}
		
		#ifdef USE_DEDICATED_TX_BUFFERS
		   //Copy data to Tx buffers
                   memcpy((void *)REVERSE32((uint32_t)TxNBUF[index_txbd].data), (void *)(((uint32_t)(tx_packet->data)) + (i*TX_BUFFER_SIZE)),
                          REVERSE16(TxNBUF[index_txbd].length));  
                #else
                   // Just update data pointer as data is aready there
                   TxNBUF[index_txbd].data = (uint8_t *)REVERSE32((((uint32_t)(tx_packet->data)) + (i*TX_BUFFER_SIZE)));
                #endif

		// Wrap if this was last TxBD
		if(++index_txbd == NUM_TXBDS)
		{
			TxNBUF[NUM_TXBDS - 1].status |= TX_BD_W;
			index_txbd = 0;
		}
	}
	
	// Update the global txbd index
	next_txbd = index_txbd;
}

void 
enet_transmit_packet(int ch, NBUF * tx_packet)
{
#ifndef MACNET_SWITCH_SUPPORT
    volatile macnet_t *enet;
    
    if(ch)
      enet = (macnet_t *)(MACNET_BASE_PTR+MAC_NET_OFFSET);
    else
      enet = (macnet_t *)MACNET_BASE_PTR;
#else
    volatile esw_t *esw = (esw_t *)MACSW_BASE_PTR;
#endif
    
    enet_fill_txbds(ch,tx_packet);
    
#ifndef MACNET_SWITCH_SUPPORT
    // Indicate that Descriptors are ready to transmit 
    enet->tdar = MACNET_TDAR_TDAR_MASK;
#else
    // Indicate that Descriptors are ready to transmit 
    esw->tdar = MACSW_TDAR_TDAR_MASK;
#endif
}
/********************************************************************/
