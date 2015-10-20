/*!
 * \file    mii.c
 * \brief   Media Independent Interface (MII) driver
 * \version $Revision: 1.2 $
 * \author  Michael Norman
 * 
 * \warning This driver assumes that FEC0 is used for all MII management
 *          communications. For dual PHYs, etc. Insure that FEC0_MDC and
 *          FEC0_MDIO are connected to the PHY's MDC and MDIO.
 */

#include "common.h"
#include "mii.h"
#include "macnet.h"/*register definition*/
#include "macnet_hal.h"/*SoC specific*/

/********************************************************************/
/*
 * \brief   Initialize the MII interface controller
 * \param   System Clock Frequency (in MHz)
 * \warning The system clock in this case is the clock that drives
 *          the FEC logic.  This may be different from the speed at which 
 *          the CPU is operating.
 * 
 * Initialize the MII clock (EMDC) frequency. The desired MII clock is 2.5MHz:
 *
 * MII Speed Setting = System_Clock / (2.5MHz * 2)
 * (plus 1 to round up)
 */
void
mii_init(int ch, int sys_clk_mhz)
{
    volatile macnet_t *enet;
    
    if(ch)
      enet = (macnet_t *)(MACNET_BASE_PTR+MAC_NET_OFFSET);
    else
      enet = (macnet_t *)MACNET_BASE_PTR;
  
    enet->mscr = 0
                      /*TSI EVB requires a longer hold time than default 10 ns*/
                      //| MACNET_MSCR_HOLDTIME(2)    
                      | MACNET_MSCR_MII_SPEED((2*sys_clk_mhz/5)+1)
                      ;
}
/********************************************************************/
/*!
 * \brief Write a value to a PHY's MII register.
 * 
 * \param   phy_addr    Address of the PHY
 * \param   reg_addr    Address of the register in the PHY
 * \param   data        Data to be written to the PHY register
 * \return  0 if write is successful; 1 if write times out
 *
 * mii_write() polls for the FEC's MII interrupt event (which should
 * be masked from the interrupt handler) and clears it. If after a
 * suitable amount of time the event isn't triggered, a non-zero value 
 * is returned.
 */
int 
mii_write(int ch, int phy_addr, int reg_addr, int data)
{
	int timeout;
        volatile macnet_t *enet;
    
        if(ch)
          enet = (macnet_t *)(MACNET_BASE_PTR+MAC_NET_OFFSET);
        else
          enet = (macnet_t *)MACNET_BASE_PTR;
  

	/* Clear the MII interrupt bit */
	enet->eir = MACNET_EIR_MII_MASK;

	/* Initiatate the MII Management write */
	enet->mmfr = 0
		| MACNET_MMFR_ST(0x01)
		| MACNET_MMFR_OP(0x01)
		| MACNET_MMFR_PA(phy_addr)
		| MACNET_MMFR_RA(reg_addr)
		| MACNET_MMFR_TA(0x02)
		| MACNET_MMFR_DATA(data);

	/* Poll for the MII interrupt (interrupt should be masked) */
        for (timeout = 0; timeout < MII_TIMEOUT; timeout++)
	{
		if (enet->eir & MACNET_EIR_MII_MASK)
			break;
	}

	if(timeout == MII_TIMEOUT) 
		return 1;

	/* Clear the MII interrupt bit */
	enet->eir = MACNET_EIR_MII_MASK;

	return 0;
}
/********************************************************************/
/*!
 * \brief   Read a value from a PHY's MII register.
 * \param   phy_addr    Address of the PHY
 * \param   reg_addr    Address of the register in the PHY
 * \param   data        Pointer to location were read data will be stored
 * \return  0 if write is successful; 1 if write times out
 *
 * mii_read() polls for the FEC's MII interrupt event (which should
 * be masked from the interrupt handler) and clears it. If after a
 * suitable amount of time the event isn't triggered, a non-zero value 
 * is returned.
 */
int 
mii_read(int ch, int phy_addr, int reg_addr, int *data)
{
	int timeout;
        volatile macnet_t *enet;
    
        if(ch)
          enet = (macnet_t *)(MACNET_BASE_PTR+MAC_NET_OFFSET);
        else
          enet = (macnet_t *)MACNET_BASE_PTR;

	/* Clear the MII interrupt bit */
	enet->eir = MACNET_EIR_MII_MASK;

	/* Initiatate the MII Management read */
	enet->mmfr = 0
		| MACNET_MMFR_ST(0x01)
		| MACNET_MMFR_OP(0x2)
		| MACNET_MMFR_PA(phy_addr)
		| MACNET_MMFR_RA(reg_addr)
		| MACNET_MMFR_TA(0x02);

	/* Poll for the MII interrupt (interrupt should be masked) */
	for (timeout = 0; timeout < MII_TIMEOUT; timeout++)
	{
		if (enet->eir & MACNET_EIR_MII_MASK)
			break;
	}
    
	if(timeout == MII_TIMEOUT) 
		return 1;

	/* Clear the MII interrupt bit */
	enet->eir = MACNET_EIR_MII_MASK;

	*data = enet->mmfr & 0x0000FFFF;

	return 0;
}
/********************************************************************/
