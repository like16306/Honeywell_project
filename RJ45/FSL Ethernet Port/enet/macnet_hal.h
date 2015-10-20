#ifndef _MACNET_HAL_H_
#define _MACNET_HAL_H_

#define MACNET_LEGACY            0
#define MACNET_ENHANCED          1
#define MACNET_ENHANCED_V2       2

#define MICREL_PHY               0
#define NATIONAL_PHY             1

/******************************************************************************/
/***********************TWR K60 + IAR******************************************/
/******************************************************************************/

/*specific for Kinetis + IAR compiler*/
#if (TWR_K60N512 && IAR)

/*MAC-NET version used in this MCU/MPU*/
#define MACNET_VERSION           MACNET_ENHANCED_V2             

/*default MAC-NET por used*/
#define MACNET_PORT                                         0

/* ENET - Peripheral instance base addresses for Kinetis */
/* Peripheral ENET base pointer */
#define MACNET_BASE_PTR                            0x400C0000u
/*Next available MAC-NET controller his this offset*/
#define MAC_NET_OFFSET                                 0x4000u

/*compiler specific ISR header*/

#define ISR_PREFIX   /*empty*/

/*compiler+architecture NOP representation*/

#define NOP_ASM  asm( "NOP" )

#ifdef TWR_K60N512
#define ETH_PHY  MICREL_PHY
#elif (NEWTON_256 || NEWTON_144)
#define ETH_PHY  NATIONAL_PHY
#else
#error "No PHY defined"
#endif

/*MAC-NET SoC integration*/
#if (MACNET_VERSION ==MACNET_LEGACY)
  /*Choose Enhanced Buffer Descriptor or Legacy: Kinetis, Modelo, Faraday, etc*/
  #define ENHANCED_BD         0//FIXED
  #define BYTE_SWAPPING_EN    0//FIXED
  /*define Endianess for Little Endian architectures like ARM.*/
  /*Motorola/Freescale uses Big Endian or Register-Endianess*/
  #define NBUF_LITTLE_ENDIAN  0//FIXED
#elif (MACNET_VERSION ==MACNET_ENHANCED)
  /*Choose Enhanced Buffer Descriptor or Legacy: Kinetis, Modelo, Faraday, etc*/
  #define ENHANCED_BD         1//MAY CHANGE
  #define BYTE_SWAPPING_EN    0//FIXED
/*define Endianess for Little Endian architectures like ARM.*/
  /*Motorola/Freescale uses Big Endian or Register-Endianess*/
  #define NBUF_LITTLE_ENDIAN  1//FIXED

#elif (MACNET_VERSION ==MACNET_ENHANCED_V2)
  /*Choose Enhanced Buffer Descriptor or Legacy: Kinetis, Modelo, Faraday, etc*/
  #define ENHANCED_BD         1//MAY CHANGE
  #define BYTE_SWAPPING_EN    1//MAY CHANGE
  /*HW support for swapping little endian/big endian*/
  #if (BYTE_SWAPPING_EN==0)
    #define NBUF_LITTLE_ENDIAN  1//FIXED
  #else
    #define NBUF_LITTLE_ENDIAN  0//FIXED
  #endif
#else
#error "Undefined MAC-NET version"
#endif

/*FSL: IAR specific*/
#if (NBUF_LITTLE_ENDIAN==1)
#define REVERSE16           __REVSH
#define REVERSE32           __REV
#else
#define REVERSE16
#define REVERSE32
#endif

#define ENABLE_MAC_NET  SIM_SCGC2 |= SIM_SCGC2_ENET_MASK; /* Enable the ENET clock. */\
  /*FSL: allow concurrent access to MPU controller. Example: ENET uDMA to SRAM, otherwise bus error*/\
  MPU_CESR = 0;

#define ENET_INTERRUPTS_ENABLE                           \
  set_irq_priority (76, 6);/*enet_interrupt_routine*/    \
  enable_irq(76);/*ENET xmit interrupt*/                 \
  set_irq_priority (77, 6);/*enet_interrupt_routine*/    \
  enable_irq(77);/*ENET rx interrupt*/                   \
  set_irq_priority (78, 6);/*enet_interrupt_routine*/    \
  enable_irq(78);/*ENET error and misc interrupts*/

/*enable MDIO and MDC pins*/
#define ENET_MII_PINS                                          \
  PORTB_PCR0  = PORT_PCR_MUX(4);/*GPIO RMII0_MDIO/MII0_MDIO*/  \
  PORTB_PCR1  = PORT_PCR_MUX(4);/*GPIO RMII0_MDC/MII0_MDC*/

/*FSL: note RMII0_RXER/MII0_RXER is not used because shared with JTAG */

#define ENET_MII_MODE_PINS                                   \
  PORTA_PCR14 = PORT_PCR_MUX(4);/*RMII0_CRS_DV/MII0_RXDV*/   \
  /*PORTA_PCR5  = PORT_PCR_MUX(4);*//*RMII0_RXER/MII0_RXER*/   \
  PORTA_PCR12 = PORT_PCR_MUX(4);/*RMII0_RXD1/MII0_RXD1*/     \
  PORTA_PCR13 = PORT_PCR_MUX(4);/*RMII0_RXD0/MII0_RXD0*/     \
  PORTA_PCR15 = PORT_PCR_MUX(4);/*RMII0_TXEN/MII0_TXEN*/     \
  PORTA_PCR16 = PORT_PCR_MUX(4);/*RMII0_TXD0/MII0_TXD0*/     \
  PORTA_PCR17 = PORT_PCR_MUX(4);/*RMII0_TXD1/MII0_TXD1*/     \
  PORTA_PCR11 = PORT_PCR_MUX(4);/*MII0_RXCLK*/               \
  PORTA_PCR25 = PORT_PCR_MUX(4);/*MII0_TXCLK*/               \
  PORTA_PCR9  = PORT_PCR_MUX(4);/*MII0_RXD3*/                \
  PORTA_PCR10 = PORT_PCR_MUX(4);/*MII0_RXD2*/                \
  PORTA_PCR28 = PORT_PCR_MUX(4);/*MII0_TXER*/                \
  PORTA_PCR24 = PORT_PCR_MUX(4);/*MII0_TXD2*/                \
  PORTA_PCR26 = PORT_PCR_MUX(4);/*MII0_TXD3*/                \
  PORTA_PCR27 = PORT_PCR_MUX(4);/*MII0_CRS*/                 \
  PORTA_PCR29 = PORT_PCR_MUX(4);/*MII0_COL*/

#define ENET_RMII_MODE_PINS                                  \
  PORTA_PCR14 = PORT_PCR_MUX(4);/*RMII0_CRS_DV/MII0_RXDV*/   \
  /*PORTA_PCR5  = PORT_PCR_MUX(4);*//*RMII0_RXER/MII0_RXER*/   \
  PORTA_PCR12 = PORT_PCR_MUX(4);/*RMII0_RXD1/MII0_RXD1*/     \
  PORTA_PCR13 = PORT_PCR_MUX(4);/*RMII0_RXD0/MII0_RXD0*/     \
  PORTA_PCR15 = PORT_PCR_MUX(4);/*RMII0_TXEN/MII0_TXEN*/     \
  PORTA_PCR16 = PORT_PCR_MUX(4);/*RMII0_TXD0/MII0_TXD0*/     \
  PORTA_PCR17 = PORT_PCR_MUX(4);/*RMII0_TXD1/MII0_TXD1*/

/******************************************************************************/
/***********************TWR K70 + IAR******************************************/
/******************************************************************************/

#elif ((TWR_K70F120 | EDISON_256) && IAR)

/*MAC-NET version used in this MCU/MPU*/
#define MACNET_VERSION           MACNET_ENHANCED_V2             

/*default MAC-NET por used*/
#define MACNET_PORT                                         0

/* ENET - Peripheral instance base addresses for Kinetis */
/* Peripheral ENET base pointer */
#define MACNET_BASE_PTR                            0x400C0000u
/*Next available MAC-NET controller his this offset*/
#define MAC_NET_OFFSET                                 0x4000u

/*compiler specific ISR header*/

#define ISR_PREFIX   /*empty*/

/*compiler+architecture NOP representation*/

#define NOP_ASM  asm( "NOP" )

#ifdef TWR_K70F120
#define ETH_PHY  MICREL_PHY
#elif EDISON_256
#define ETH_PHY  NATIONAL_PHY
#else
#error "No PHY defined"
#endif

/*MAC-NET SoC integration*/
#if (MACNET_VERSION ==MACNET_LEGACY)
  /*Choose Enhanced Buffer Descriptor or Legacy: Kinetis, Modelo, Faraday, etc*/
  #define ENHANCED_BD         0//FIXED
  #define BYTE_SWAPPING_EN    0//FIXED
  /*define Endianess for Little Endian architectures like ARM.*/
  /*Motorola/Freescale uses Big Endian or Register-Endianess*/
  #define NBUF_LITTLE_ENDIAN  0//FIXED
#elif (MACNET_VERSION ==MACNET_ENHANCED)
  /*Choose Enhanced Buffer Descriptor or Legacy: Kinetis, Modelo, Faraday, etc*/
  #define ENHANCED_BD         1//MAY CHANGE
  #define BYTE_SWAPPING_EN    0//FIXED
/*define Endianess for Little Endian architectures like ARM.*/
  /*Motorola/Freescale uses Big Endian or Register-Endianess*/
  #define NBUF_LITTLE_ENDIAN  1//FIXED

#elif (MACNET_VERSION ==MACNET_ENHANCED_V2)
  /*Choose Enhanced Buffer Descriptor or Legacy: Kinetis, Modelo, Faraday, etc*/
  #define ENHANCED_BD         1//MAY CHANGE
  #define BYTE_SWAPPING_EN    1//MAY CHANGE
  /*HW support for swapping little endian/big endian*/
  #if (BYTE_SWAPPING_EN==0)
    #define NBUF_LITTLE_ENDIAN  1//FIXED
  #else
    #define NBUF_LITTLE_ENDIAN  0//FIXED
  #endif
#else
#error "Undefined MAC-NET version"
#endif

/*FSL: IAR specific*/
#if (NBUF_LITTLE_ENDIAN==1)
#define REVERSE16           __REVSH
#define REVERSE32           __REV
#else
#define REVERSE16
#define REVERSE32
#endif

#define ENET_BUFFERS_OUTSIDE_OF_SRAM

#define ENABLE_MAC_NET  SIM_SCGC2 |= SIM_SCGC2_ENET1_MASK; /* Enable the ENET clock. */\
  /*FSL: allow concurrent access to MPU controller. Example: ENET uDMA to SRAM, otherwise bus error*/\
  MPU_CESR = 0;

#define ENET_INTERRUPTS_ENABLE                           \
  set_irq_priority (76, 6);/*enet_interrupt_routine*/    \
  enable_irq(76);/*ENET xmit interrupt*/                 \
  set_irq_priority (77, 6);/*enet_interrupt_routine*/    \
  enable_irq(77);/*ENET rx interrupt*/                   \
  set_irq_priority (78, 6);/*enet_interrupt_routine*/    \
  enable_irq(78);/*ENET error and misc interrupts*/

/*enable MDIO and MDC pins*/
#define ENET_MII_PINS                                          \
  PORTB_PCR0  = PORT_PCR_MUX(4);/*GPIO RMII0_MDIO/MII0_MDIO*/  \
  PORTB_PCR1  = PORT_PCR_MUX(4);/*GPIO RMII0_MDC/MII0_MDC*/

/*FSL: note RMII0_RXER/MII0_RXER is not used because shared with JTAG */

#define ENET_MII_MODE_PINS                                   \
  PORTA_PCR14 = PORT_PCR_MUX(4);/*RMII0_CRS_DV/MII0_RXDV*/   \
  /*PORTA_PCR5  = PORT_PCR_MUX(4);*//*RMII0_RXER/MII0_RXER*/   \
  PORTA_PCR12 = PORT_PCR_MUX(4);/*RMII0_RXD1/MII0_RXD1*/     \
  PORTA_PCR13 = PORT_PCR_MUX(4);/*RMII0_RXD0/MII0_RXD0*/     \
  PORTA_PCR15 = PORT_PCR_MUX(4);/*RMII0_TXEN/MII0_TXEN*/     \
  PORTA_PCR16 = PORT_PCR_MUX(4);/*RMII0_TXD0/MII0_TXD0*/     \
  PORTA_PCR17 = PORT_PCR_MUX(4);/*RMII0_TXD1/MII0_TXD1*/     \
  PORTA_PCR11 = PORT_PCR_MUX(4);/*MII0_RXCLK*/               \
  PORTA_PCR25 = PORT_PCR_MUX(4);/*MII0_TXCLK*/               \
  PORTA_PCR9  = PORT_PCR_MUX(4);/*MII0_RXD3*/                \
  PORTA_PCR10 = PORT_PCR_MUX(4);/*MII0_RXD2*/                \
  PORTA_PCR28 = PORT_PCR_MUX(4);/*MII0_TXER*/                \
  PORTA_PCR24 = PORT_PCR_MUX(4);/*MII0_TXD2*/                \
  PORTA_PCR26 = PORT_PCR_MUX(4);/*MII0_TXD3*/                \
  PORTA_PCR27 = PORT_PCR_MUX(4);/*MII0_CRS*/                 \
  PORTA_PCR29 = PORT_PCR_MUX(4);/*MII0_COL*/

#define ENET_RMII_MODE_PINS                                  \
  PORTA_PCR14 = PORT_PCR_MUX(4)/*|PORT_PCR_DSE_MASK*/;/*RMII0_CRS_DV/MII0_RXDV*/   \
  /*PORTA_PCR5  = PORT_PCR_MUX(4);*//*RMII0_RXER/MII0_RXER*/   \
  PORTA_PCR12 = PORT_PCR_MUX(4);/*RMII0_RXD1/MII0_RXD1*/     \
  PORTA_PCR13 = PORT_PCR_MUX(4);/*RMII0_RXD0/MII0_RXD0*/     \
  PORTA_PCR15 = PORT_PCR_MUX(4);/*RMII0_TXEN/MII0_TXEN*/     \
  PORTA_PCR16 = PORT_PCR_MUX(4);/*RMII0_TXD0/MII0_TXD0*/     \
  PORTA_PCR17 = PORT_PCR_MUX(4);/*RMII0_TXD1/MII0_TXD1*/

/******************************************************************************/
/***********************Faraday Palladium + IAR********************************/
/******************************************************************************/

#elif (FARADAY && IAR)

#include "common_api.h"

/*MAC-NET version used in this MCU/MPU*/
#define MACNET_VERSION           MACNET_ENHANCED_V2             

/*default MAC-NET por used*/
#define MACNET_PORT                                         0

/* ENET - Peripheral instance base addresses for Faraday */
/* Peripheral ENET base pointer */
#define MACNET_BASE_PTR                            0x400D0000u
/*Next available MAC-NET controller his this offset*/
#define MAC_NET_OFFSET                                 0x1000u

/*compiler specific ISR header*/

#define ISR_PREFIX   /*empty*/

/*compiler+architecture NOP representation*/

#define NOP_ASM  asm( "NOP" )

#ifdef TWR_K70F120
#define ETH_PHY  MICREL_PHY
#elif EDISON_256
#define ETH_PHY  NATIONAL_PHY
#else
//#warning "Palladium doesnt use a PHY"
#endif

/*MAC-NET SoC integration*/
#if (MACNET_VERSION ==MACNET_LEGACY)
  /*Choose Enhanced Buffer Descriptor or Legacy: Kinetis, Modelo, Faraday, etc*/
  #define ENHANCED_BD         0//FIXED
  #define BYTE_SWAPPING_EN    0//FIXED
  /*define Endianess for Little Endian architectures like ARM.*/
  /*Motorola/Freescale uses Big Endian or Register-Endianess*/
  #define NBUF_LITTLE_ENDIAN  0//FIXED
#elif (MACNET_VERSION ==MACNET_ENHANCED)
  /*Choose Enhanced Buffer Descriptor or Legacy: Kinetis, Modelo, Faraday, etc*/
  #define ENHANCED_BD         1//MAY CHANGE
  #define BYTE_SWAPPING_EN    0//FIXED
  /*define Endianess for Little Endian architectures like ARM.*/
  /*Motorola/Freescale uses Big Endian or Register-Endianess*/
  #define NBUF_LITTLE_ENDIAN  1//FIXED
#elif (MACNET_VERSION ==MACNET_ENHANCED_V2)
  /*Choose Enhanced Buffer Descriptor or Legacy: Kinetis, Modelo, Faraday, etc*/
  #define ENHANCED_BD         1//MAY CHANGE
  #define BYTE_SWAPPING_EN    1//MAY CHANGE
  /*HW support for swapping little endian/big endian*/
  #if (BYTE_SWAPPING_EN==0)
    #define NBUF_LITTLE_ENDIAN  1//FIXED
  #else
    #define NBUF_LITTLE_ENDIAN  0//FIXED
  #endif
#else
#error "Undefined MAC-NET version"
#endif

/*FSL: IAR specific*/
#if (NBUF_LITTLE_ENDIAN==1)
#define REVERSE16           __REVSH
#define REVERSE32           __REV
#else
#define REVERSE16
#define REVERSE32
#endif

//#define ENET_BUFFERS_OUTSIDE_OF_SRAM

#define ENABLE_MAC_NET  /* Enable the ENET clock. */\
  /*FSL: allow concurrent access to MPU controller. Example: ENET uDMA to SRAM, otherwise bus error*/\
#warning "need to define if MAC in Palladium needs to disable MPU controller"

#define ENET_INTERRUPTS_ENABLE                           \
#warning "need to define interrupt handling"

#if (MACNET_PORT==0)
/*enable MDIO and MDC pins*/
#define ENET_MII_PINS                                          \
  reg32_write(IOMUXC_SW_MUX_CTL_PAD_PAD_45,(1<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_MUX_MODE_SHIFT)|(2<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_SPEED_SHIFT)|IOMUXC_SW_MUX_CTL_PAD_PAD_n_SRE_MASK|(6<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_DSE_SHIFT));/*RMII0_MDC/MII0_MDC*/\
  reg32_write(IOMUXC_SW_MUX_CTL_PAD_PAD_46,(1<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_MUX_MODE_SHIFT)|(2<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_SPEED_SHIFT)|IOMUXC_SW_MUX_CTL_PAD_PAD_n_SRE_MASK|(6<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_DSE_SHIFT));/*RMII0_MDIO/MII0_MDIO*/

/*FSL: note RMII0_RXER/MII0_RXER is not used because shared with JTAG */

#define ENET_MII_MODE_PINS                                   \
#warning "no MII implementation in Faraday"

#define ENET_RMII_MODE_PINS                                  \
  reg32_write(IOMUXC_SW_MUX_CTL_PAD_PAD_47,(1<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_MUX_MODE_SHIFT)|(2<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_SPEED_SHIFT)|IOMUXC_SW_MUX_CTL_PAD_PAD_n_SRE_MASK|(6<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_DSE_SHIFT)|IOMUXC_SW_MUX_CTL_PAD_PAD_n_IBE_MASK);/*RMII0_CRS_DV/MII0_RXDV*/\
  reg32_write(IOMUXC_SW_MUX_CTL_PAD_PAD_48,(1<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_MUX_MODE_SHIFT)|(2<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_SPEED_SHIFT)|IOMUXC_SW_MUX_CTL_PAD_PAD_n_SRE_MASK|(6<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_DSE_SHIFT)|IOMUXC_SW_MUX_CTL_PAD_PAD_n_IBE_MASK);/*RMII0_RXD1/MII0_RXD1*/\
  reg32_write(IOMUXC_SW_MUX_CTL_PAD_PAD_49,(1<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_MUX_MODE_SHIFT)|(2<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_SPEED_SHIFT)|IOMUXC_SW_MUX_CTL_PAD_PAD_n_SRE_MASK|(6<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_DSE_SHIFT)|IOMUXC_SW_MUX_CTL_PAD_PAD_n_IBE_MASK);/*RMII0_RXD0/MII0_RXD0*/\
  reg32_write(IOMUXC_SW_MUX_CTL_PAD_PAD_50,(1<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_MUX_MODE_SHIFT)|(2<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_SPEED_SHIFT)|IOMUXC_SW_MUX_CTL_PAD_PAD_n_SRE_MASK|(6<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_DSE_SHIFT)|IOMUXC_SW_MUX_CTL_PAD_PAD_n_IBE_MASK);/*RMII0_RXER/MII0_RXER*/\
  reg32_write(IOMUXC_SW_MUX_CTL_PAD_PAD_51,(1<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_MUX_MODE_SHIFT)|(2<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_SPEED_SHIFT)|IOMUXC_SW_MUX_CTL_PAD_PAD_n_SRE_MASK|(6<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_DSE_SHIFT));/*RMII0_TXD1/MII0_TXD1*/\
  reg32_write(IOMUXC_SW_MUX_CTL_PAD_PAD_52,(1<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_MUX_MODE_SHIFT)|(2<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_SPEED_SHIFT)|IOMUXC_SW_MUX_CTL_PAD_PAD_n_SRE_MASK|(6<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_DSE_SHIFT));/*RMII0_TXD0/MII0_TXD0*/\
  reg32_write(IOMUXC_SW_MUX_CTL_PAD_PAD_53,(1<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_MUX_MODE_SHIFT)|(2<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_SPEED_SHIFT)|IOMUXC_SW_MUX_CTL_PAD_PAD_n_SRE_MASK|(6<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_DSE_SHIFT));/*RMII0_TXEN/MII0_TXEN*/
  
#elif(MACNET_PORT==1)
/*enable MDIO and MDC pins*/
#define ENET_MII_PINS                                          \
  reg32_write(IOMUXC_SW_MUX_CTL_PAD_PAD_54,(1<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_MUX_MODE_SHIFT)|(2<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_SPEED_SHIFT)|IOMUXC_SW_MUX_CTL_PAD_PAD_n_SRE_MASK|(6<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_DSE_SHIFT));\//RMII0_MDC/MII0_MDC
  reg32_write(IOMUXC_SW_MUX_CTL_PAD_PAD_55,(1<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_MUX_MODE_SHIFT)|(2<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_SPEED_SHIFT)|IOMUXC_SW_MUX_CTL_PAD_PAD_n_SRE_MASK|(6<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_DSE_SHIFT));//RMII0_MDIO/MII0_MDIO

/*FSL: note RMII0_RXER/MII0_RXER is not used because shared with JTAG */

#define ENET_MII_MODE_PINS                                   \
#warning "no MII implementation in Faraday"

#define ENET_RMII_MODE_PINS                                  \
  reg32_write(IOMUXC_SW_MUX_CTL_PAD_PAD_56,(1<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_MUX_MODE_SHIFT)|(2<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_SPEED_SHIFT)|IOMUXC_SW_MUX_CTL_PAD_PAD_n_SRE_MASK|(6<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_DSE_SHIFT)|IOMUXC_SW_MUX_CTL_PAD_PAD_n_IBE_MASK);\//RMII0_CRS_DV/MII0_RXDV        
  reg32_write(IOMUXC_SW_MUX_CTL_PAD_PAD_57,(1<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_MUX_MODE_SHIFT)|(2<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_SPEED_SHIFT)|IOMUXC_SW_MUX_CTL_PAD_PAD_n_SRE_MASK|(6<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_DSE_SHIFT)|IOMUXC_SW_MUX_CTL_PAD_PAD_n_IBE_MASK);\//RMII0_RXD1/MII0_RXD1
  reg32_write(IOMUXC_SW_MUX_CTL_PAD_PAD_58,(1<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_MUX_MODE_SHIFT)|(2<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_SPEED_SHIFT)|IOMUXC_SW_MUX_CTL_PAD_PAD_n_SRE_MASK|(6<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_DSE_SHIFT)|IOMUXC_SW_MUX_CTL_PAD_PAD_n_IBE_MASK);\//RMII0_RXD0/MII0_RXD0
  reg32_write(IOMUXC_SW_MUX_CTL_PAD_PAD_59,(1<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_MUX_MODE_SHIFT)|(2<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_SPEED_SHIFT)|IOMUXC_SW_MUX_CTL_PAD_PAD_n_SRE_MASK|(6<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_DSE_SHIFT)|IOMUXC_SW_MUX_CTL_PAD_PAD_n_IBE_MASK);\//RMII0_RXER/MII0_RXER	
  reg32_write(IOMUXC_SW_MUX_CTL_PAD_PAD_60,(1<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_MUX_MODE_SHIFT)|(2<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_SPEED_SHIFT)|IOMUXC_SW_MUX_CTL_PAD_PAD_n_SRE_MASK|(6<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_DSE_SHIFT));\//RMII0_TXD1/MII0_TXD1
  reg32_write(IOMUXC_SW_MUX_CTL_PAD_PAD_61,(1<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_MUX_MODE_SHIFT)|(2<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_SPEED_SHIFT)|IOMUXC_SW_MUX_CTL_PAD_PAD_n_SRE_MASK|(6<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_DSE_SHIFT));\//RMII0_TXD0/MII0_TXD0
  reg32_write(IOMUXC_SW_MUX_CTL_PAD_PAD_62,(1<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_MUX_MODE_SHIFT)|(2<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_SPEED_SHIFT)|IOMUXC_SW_MUX_CTL_PAD_PAD_n_SRE_MASK|(6<<IOMUXC_SW_MUX_CTL_PAD_PAD_n_DSE_SHIFT));//RMII0_TXEN/MII0_TXEN
#else
#error "Error in MAC port implementation"
#endif
/******************************************************************************/
/***********************NO IMPLEMENTATION**************************************/
/******************************************************************************/
#else

#define MACNET_VERSION           MACNET_ENHANCED_V2             

/*default MAC-NET por used*/
#define MACNET_PORT                                         0

/* ENET - Peripheral instance base addresses for Kinetis */
/* Peripheral ENET base pointer */
#define MACNET_BASE_PTR                            0x400C0000u/*fix it*/
/*Next available MAC-NET controller his this offset*/
#define MAC_NET_OFFSET                                 0x4000u/*fix it*/

/*compiler specific ISR header*/

#define ISR_PREFIX   /*empty*/

/*compiler+architecture NOP representation*/

#define NOP_ASM                                               /*fill it*/

#define ETH_PHY  MICREL_PHY

/*MAC-NET SoC integration*/
#if (MACNET_VERSION ==MACNET_LEGACY)
  /*Choose Enhanced Buffer Descriptor or Legacy: Kinetis, Modelo, Faraday, etc*/
  #define ENHANCED_BD         0//FIXED
  #define BYTE_SWAPPING_EN    0//FIXED
  /*define Endianess for Little Endian architectures like ARM.*/
  /*Motorola/Freescale uses Big Endian or Register-Endianess*/
  #define NBUF_LITTLE_ENDIAN  0//FIXED
#elif (MACNET_VERSION ==MACNET_ENHANCED)
  /*Choose Enhanced Buffer Descriptor or Legacy: Kinetis, Modelo, Faraday, etc*/
  #define ENHANCED_BD         1//MAY CHANGE
  #define BYTE_SWAPPING_EN    0//FIXED
/*define Endianess for Little Endian architectures like ARM.*/
  /*Motorola/Freescale uses Big Endian or Register-Endianess*/
  #define NBUF_LITTLE_ENDIAN  1//FIXED

#elif (MACNET_VERSION ==MACNET_ENHANCED_V2)
  /*Choose Enhanced Buffer Descriptor or Legacy: Kinetis, Modelo, Faraday, etc*/
  #define ENHANCED_BD         1//MAY CHANGE
  #define BYTE_SWAPPING_EN    1//MAY CHANGE
  /*HW support for swapping little endian/big endian*/
  #if (BYTE_SWAPPING_EN==0)
    #define NBUF_LITTLE_ENDIAN  1//FIXED
  #else
    #define NBUF_LITTLE_ENDIAN  0//FIXED
  #endif
#else
#error "Undefined MAC-NET version"
#endif

#define REVERSE16                                            /*fill it*/
#define REVERSE32                                            /*fill it*/

#define ENABLE_MAC_NET                                       /*fill it*/

#define ENET_INTERRUPTS_ENABLE                               /*fill it*/

/*enable MDIO and MDC pins*/
#define ENET_MII_PINS                                        /*fill it*/

#define ENET_MII_MODE_PINS                                   /*fill it*/

#define ENET_RMII_MODE_PINS                                  /*fill it*/

#endif

#endif /*_MACNET_HAL_H_*/