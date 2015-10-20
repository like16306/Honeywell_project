#ifndef __MACNET_H__
#define __MACNET_H__

#include "macnet_hal.h"

/*data types*/
#define u32 uint32_t
#define u16 uint16_t
#define u8  uint8_t

/*MAC-NET registers*/

typedef struct macnet 
{
	u32 resv0;//
	u32 eir;
	u32 eimr;
	u32 resv1;
	u32 rdar;
	u32 tdar;
	u8 resv2[0xC];
	u32 ecr;
	u8 resv3[0x18];
	u32 mmfr;
	u32 mscr;
	u8 resv4[0x1C];
	u32 mibc;
	u8 resv5[0x1C];
	u32 rcr;
	u8 resv6[0x3C];
	u32 tcr;
	u8 resv7[0x1C];
	u32 palr;
	u32 paur;
	u32 opd;
	u8 resv8[0x28];
	u32 iaur;
	u32 ialr;
	u32 gaur;
	u32 galr;
	u8 resv9[0x1C];
	u32 tfwr;
	u32 resv10;
	u32 frbr;
	u32 frsr;
	u8 resv11[0x2C];
	u32 rdsr;
	u32 tdsr;
	u32 mrbr;
	u32 rsvd12;
	u32 rsfl;
	u32 rsem;
	u32 raem;
	u32 rafl;
	u32 tsem;
	u32 taem;
	u32 tafl;
	u32 tipg;
	u32 ftrl;
	u32 rsvd13[3];
	u32 tacc;
	u32 racc;
	u8  rsvd14[568]; /*IEEE counters defined on next section*/
	u32 atcr;        /* 0x400 */
	u32 atvr;        /* 0x404 */
	u32 atoff;       /* 0x408 */
	u32 atper;       /* 0x40C */
	u32 atcor;       /* 0x410 */
	u32 atinc;       /* 0x414 */
	u32 atstmp;      /* 0x418 */
	u8  rsvd15[228];
        u32 smacl0;      /* 0x500 */
        u32 smacu0;
        u32 smacl1;
        u32 smacu1;
        u32 smacl2;
        u32 smacu2;
        u32 smacl3;
        u32 smacu3;
        u8  rsvd16[228];
	u32 tgsr;        /* 0x604 */
	struct 
	{  /* 0x608 */
	   u32 tcsr;      /* 0x608 + index*0x8 + 0 */
	   u32 tccr;      /* 0x608 + index*0x8 + 0x4 */
	} CHANNEL[4];
	//u8 rsvd16[14812];/* 0x4000-0x624 */
} macnet_t;

typedef struct macnet_ieee_rmon 
{
	u32 rmon_t_drop;
	u32 rmon_t_packets;
	u32 rmon_t_bc_pkt;
	u32 rmon_t_mc_pkt;
	u32 rmon_t_crc_align;
	u32 rmon_t_undersize;
	u32 rmon_t_oversize;
	u32 rmon_t_frag;
	u32 rmon_t_jab;
	u32 rmon_t_col;
	u32 rmon_t_p64;
	u32 rmon_t_p65to127;
	u32 rmon_t_p128to255;
	u32 rmon_t_p256to511;
	u32 rmon_t_p512to1023;
	u32 rmon_t_p1024to2047;
	u32 rmon_t_p_gte2048;
	u32 rmon_t_octets;

	u32 ieee_t_drop;
	u32 ieee_t_frame_ok;
	u32 ieee_t_1col;
	u32 ieee_t_mcol;
	u32 ieee_t_def;
	u32 ieee_t_lcol;
	u32 ieee_t_excol;
	u32 ieee_t_macerr;
	u32 ieee_t_cserr;
	u32 ieee_t_sqe;
	u32 ieee_t_fdxfc;
	u32 ieee_t_octets_ok;
	u8 resv13[0x8];

	u32 rmon_r_drop;
	u32 rmon_r_packets;
	u32 rmon_r_bc_pkt;
	u32 rmon_r_mc_pkt;
	u32 rmon_r_crc_align;
	u32 rmon_r_undersize;
	u32 rmon_r_oversize;
	u32 rmon_r_frag;
	u32 rmon_r_jab;
	u32 rmon_r_resvd_0;
	u32 rmon_r_p64;
	u32 rmon_r_p65to127;
	u32 rmon_r_p128to255;
	u32 rmon_r_p256to511;
	u32 rmon_r_p512to1023;
	u32 rmon_r_p1024to2047;
	u32 rmon_r_p_gte2048;
	u32 rmon_r_octets;

	u32 ieee_r_drop;
	u32 ieee_r_frame_ok;
	u32 ieee_r_crc;
	u32 ieee_r_align;
	u32 ieee_r_macerr;
	u32 ieee_r_fdxfc;
	u32 ieee_r_octets_ok;
} macnet_ieee_rmon_t;

/*MAC-NET definitions*/

/* EIR Bit Fields */
#define MACNET_EIR_TS_TIMER_MASK                   0x8000u
#define MACNET_EIR_TS_TIMER_SHIFT                  15
#define MACNET_EIR_TS_AVAIL_MASK                   0x10000u
#define MACNET_EIR_TS_AVAIL_SHIFT                  16
#define MACNET_EIR_WAKEUP_MASK                     0x20000u
#define MACNET_EIR_WAKEUP_SHIFT                    17
#define MACNET_EIR_PLR_MASK                        0x40000u
#define MACNET_EIR_PLR_SHIFT                       18
#define MACNET_EIR_UN_MASK                         0x80000u
#define MACNET_EIR_UN_SHIFT                        19
#define MACNET_EIR_RL_MASK                         0x100000u
#define MACNET_EIR_RL_SHIFT                        20
#define MACNET_EIR_LC_MASK                         0x200000u
#define MACNET_EIR_LC_SHIFT                        21
#define MACNET_EIR_EBERR_MASK                      0x400000u
#define MACNET_EIR_EBERR_SHIFT                     22
#define MACNET_EIR_MII_MASK                        0x800000u
#define MACNET_EIR_MII_SHIFT                       23
#define MACNET_EIR_RXB_MASK                        0x1000000u
#define MACNET_EIR_RXB_SHIFT                       24
#define MACNET_EIR_RXF_MASK                        0x2000000u
#define MACNET_EIR_RXF_SHIFT                       25
#define MACNET_EIR_TXB_MASK                        0x4000000u
#define MACNET_EIR_TXB_SHIFT                       26
#define MACNET_EIR_TXF_MASK                        0x8000000u
#define MACNET_EIR_TXF_SHIFT                       27
#define MACNET_EIR_GRA_MASK                        0x10000000u
#define MACNET_EIR_GRA_SHIFT                       28
#define MACNET_EIR_BABT_MASK                       0x20000000u
#define MACNET_EIR_BABT_SHIFT                      29
#define MACNET_EIR_BABR_MASK                       0x40000000u
#define MACNET_EIR_BABR_SHIFT                      30
/* EIMR Bit Fields */
#define MACNET_EIMR_TS_TIMER_MASK                  0x8000u
#define MACNET_EIMR_TS_TIMER_SHIFT                 15
#define MACNET_EIMR_TS_AVAIL_MASK                  0x10000u
#define MACNET_EIMR_TS_AVAIL_SHIFT                 16
#define MACNET_EIMR_WAKEUP_MASK                    0x20000u
#define MACNET_EIMR_WAKEUP_SHIFT                   17
#define MACNET_EIMR_PLR_MASK                       0x40000u
#define MACNET_EIMR_PLR_SHIFT                      18
#define MACNET_EIMR_UN_MASK                        0x80000u
#define MACNET_EIMR_UN_SHIFT                       19
#define MACNET_EIMR_RL_MASK                        0x100000u
#define MACNET_EIMR_RL_SHIFT                       20
#define MACNET_EIMR_LC_MASK                        0x200000u
#define MACNET_EIMR_LC_SHIFT                       21
#define MACNET_EIMR_EBERR_MASK                     0x400000u
#define MACNET_EIMR_EBERR_SHIFT                    22
#define MACNET_EIMR_MII_MASK                       0x800000u
#define MACNET_EIMR_MII_SHIFT                      23
#define MACNET_EIMR_RXB_MASK                       0x1000000u
#define MACNET_EIMR_RXB_SHIFT                      24
#define MACNET_EIMR_RXF_MASK                       0x2000000u
#define MACNET_EIMR_RXF_SHIFT                      25
#define MACNET_EIMR_TXB_MASK                       0x4000000u
#define MACNET_EIMR_TXB_SHIFT                      26
#define MACNET_EIMR_TXF_MASK                       0x8000000u
#define MACNET_EIMR_TXF_SHIFT                      27
#define MACNET_EIMR_GRA_MASK                       0x10000000u
#define MACNET_EIMR_GRA_SHIFT                      28
#define MACNET_EIMR_BABT_MASK                      0x20000000u
#define MACNET_EIMR_BABT_SHIFT                     29
#define MACNET_EIMR_BABR_MASK                      0x40000000u
#define MACNET_EIMR_BABR_SHIFT                     30
/* RDAR Bit Fields */
#define MACNET_RDAR_RDAR_MASK                      0x1000000u
#define MACNET_RDAR_RDAR_SHIFT                     24
/* TDAR Bit Fields */
#define MACNET_TDAR_TDAR_MASK                      0x1000000u
#define MACNET_TDAR_TDAR_SHIFT                     24
/* ECR Bit Fields */
#define MACNET_ECR_RESET_MASK                      0x1u
#define MACNET_ECR_RESET_SHIFT                     0
#define MACNET_ECR_ETHEREN_MASK                    0x2u
#define MACNET_ECR_ETHEREN_SHIFT                   1
#define MACNET_ECR_MAGICEN_MASK                    0x4u
#define MACNET_ECR_MAGICEN_SHIFT                   2
#define MACNET_ECR_SLEEP_MASK                      0x8u
#define MACNET_ECR_SLEEP_SHIFT                     3
#define MACNET_ECR_EN1588_MASK                     0x10u
#define MACNET_ECR_EN1588_SHIFT                    4
#define MACNET_ECR_DBGEN_MASK                      0x40u
#define MACNET_ECR_DBGEN_SHIFT                     6
#define MACNET_ECR_STOPEN_MASK                     0x80u
#define MACNET_ECR_STOPEN_SHIFT                    7
#define MACNET_ECR_DBSWP_MASK                      0x100u/*new!*/
#define MACNET_ECR_DBSWP_SHIFT                     8/*new!*/
/* MMFR Bit Fields */
#define MACNET_MMFR_DATA_MASK                      0xFFFFu
#define MACNET_MMFR_DATA_SHIFT                     0
#define MACNET_MMFR_DATA(x)                        (((uint32_t)(((uint32_t)(x))<<MACNET_MMFR_DATA_SHIFT))&MACNET_MMFR_DATA_MASK)
#define MACNET_MMFR_TA_MASK                        0x30000u
#define MACNET_MMFR_TA_SHIFT                       16
#define MACNET_MMFR_TA(x)                          (((uint32_t)(((uint32_t)(x))<<MACNET_MMFR_TA_SHIFT))&MACNET_MMFR_TA_MASK)
#define MACNET_MMFR_RA_MASK                        0x7C0000u
#define MACNET_MMFR_RA_SHIFT                       18
#define MACNET_MMFR_RA(x)                          (((uint32_t)(((uint32_t)(x))<<MACNET_MMFR_RA_SHIFT))&MACNET_MMFR_RA_MASK)
#define MACNET_MMFR_PA_MASK                        0xF800000u
#define MACNET_MMFR_PA_SHIFT                       23
#define MACNET_MMFR_PA(x)                          (((uint32_t)(((uint32_t)(x))<<MACNET_MMFR_PA_SHIFT))&MACNET_MMFR_PA_MASK)
#define MACNET_MMFR_OP_MASK                        0x30000000u
#define MACNET_MMFR_OP_SHIFT                       28
#define MACNET_MMFR_OP(x)                          (((uint32_t)(((uint32_t)(x))<<MACNET_MMFR_OP_SHIFT))&MACNET_MMFR_OP_MASK)
#define MACNET_MMFR_ST_MASK                        0xC0000000u
#define MACNET_MMFR_ST_SHIFT                       30
#define MACNET_MMFR_ST(x)                          (((uint32_t)(((uint32_t)(x))<<MACNET_MMFR_ST_SHIFT))&MACNET_MMFR_ST_MASK)
/* MSCR Bit Fields */
#define MACNET_MSCR_MII_SPEED_MASK                 0x7Eu
#define MACNET_MSCR_MII_SPEED_SHIFT                1
#define MACNET_MSCR_MII_SPEED(x)                   (((uint32_t)(((uint32_t)(x))<<MACNET_MSCR_MII_SPEED_SHIFT))&MACNET_MSCR_MII_SPEED_MASK)
#define MACNET_MSCR_DIS_PRE_MASK                   0x80u
#define MACNET_MSCR_DIS_PRE_SHIFT                  7
#define MACNET_MSCR_HOLDTIME_MASK                  0x700u
#define MACNET_MSCR_HOLDTIME_SHIFT                 8
#define MACNET_MSCR_HOLDTIME(x)                    (((uint32_t)(((uint32_t)(x))<<MACNET_MSCR_HOLDTIME_SHIFT))&MACNET_MSCR_HOLDTIME_MASK)
/* MIBC Bit Fields */
#define MACNET_MIBC_MIB_CLEAR_MASK                 0x20000000u
#define MACNET_MIBC_MIB_CLEAR_SHIFT                29
#define MACNET_MIBC_MIB_IDLE_MASK                  0x40000000u
#define MACNET_MIBC_MIB_IDLE_SHIFT                 30
#define MACNET_MIBC_MIB_DIS_MASK                   0x80000000u
#define MACNET_MIBC_MIB_DIS_SHIFT                  31
/* RCR Bit Fields */
#define MACNET_RCR_LOOP_MASK                       0x1u
#define MACNET_RCR_LOOP_SHIFT                      0
#define MACNET_RCR_DRT_MASK                        0x2u
#define MACNET_RCR_DRT_SHIFT                       1
#define MACNET_RCR_MII_MODE_MASK                   0x4u
#define MACNET_RCR_MII_MODE_SHIFT                  2
#define MACNET_RCR_PROM_MASK                       0x8u
#define MACNET_RCR_PROM_SHIFT                      3
#define MACNET_RCR_BC_REJ_MASK                     0x10u
#define MACNET_RCR_BC_REJ_SHIFT                    4
#define MACNET_RCR_FCE_MASK                        0x20u
#define MACNET_RCR_FCE_SHIFT                       5
#define MACNET_RCR_RMII_MODE_MASK                  0x100u
#define MACNET_RCR_RMII_MODE_SHIFT                 8
#define MACNET_RCR_RMII_10T_MASK                   0x200u
#define MACNET_RCR_RMII_10T_SHIFT                  9
#define MACNET_RCR_PADEN_MASK                      0x1000u
#define MACNET_RCR_PADEN_SHIFT                     12
#define MACNET_RCR_PAUFWD_MASK                     0x2000u
#define MACNET_RCR_PAUFWD_SHIFT                    13
#define MACNET_RCR_CRCFWD_MASK                     0x4000u
#define MACNET_RCR_CRCFWD_SHIFT                    14
#define MACNET_RCR_CFEN_MASK                       0x8000u
#define MACNET_RCR_CFEN_SHIFT                      15
#define MACNET_RCR_MAX_FL_MASK                     0x3FFF0000u
#define MACNET_RCR_MAX_FL_SHIFT                    16
#define MACNET_RCR_MAX_FL(x)                       (((uint32_t)(((uint32_t)(x))<<MACNET_RCR_MAX_FL_SHIFT))&MACNET_RCR_MAX_FL_MASK)
#define MACNET_RCR_NLC_MASK                        0x40000000u
#define MACNET_RCR_NLC_SHIFT                       30
#define MACNET_RCR_GRS_MASK                        0x80000000u
#define MACNET_RCR_GRS_SHIFT                       31
/* TCR Bit Fields */
#define MACNET_TCR_GTS_MASK                        0x1u
#define MACNET_TCR_GTS_SHIFT                       0
#define MACNET_TCR_FDEN_MASK                       0x4u
#define MACNET_TCR_FDEN_SHIFT                      2
#define MACNET_TCR_TFC_PAUSE_MASK                  0x8u
#define MACNET_TCR_TFC_PAUSE_SHIFT                 3
#define MACNET_TCR_RFC_PAUSE_MASK                  0x10u
#define MACNET_TCR_RFC_PAUSE_SHIFT                 4
#define MACNET_TCR_ADDSEL_MASK                     0xE0u
#define MACNET_TCR_ADDSEL_SHIFT                    5
#define MACNET_TCR_ADDSEL(x)                       (((uint32_t)(((uint32_t)(x))<<MACNET_TCR_ADDSEL_SHIFT))&MACNET_TCR_ADDSEL_MASK)
#define MACNET_TCR_ADDINS_MASK                     0x100u
#define MACNET_TCR_ADDINS_SHIFT                    8
#define MACNET_TCR_CRCFWD_MASK                     0x200u
#define MACNET_TCR_CRCFWD_SHIFT                    9
/* PALR Bit Fields */
#define MACNET_PALR_PADDR1_MASK                    0xFFFFFFFFu
#define MACNET_PALR_PADDR1_SHIFT                   0
#define MACNET_PALR_PADDR1(x)                      (((uint32_t)(((uint32_t)(x))<<MACNET_PALR_PADDR1_SHIFT))&MACNET_PALR_PADDR1_MASK)
/* PAUR Bit Fields */
#define MACNET_PAUR_TYPE_MASK                      0xFFFFu
#define MACNET_PAUR_TYPE_SHIFT                     0
#define MACNET_PAUR_TYPE(x)                        (((uint32_t)(((uint32_t)(x))<<MACNET_PAUR_TYPE_SHIFT))&MACNET_PAUR_TYPE_MASK)
#define MACNET_PAUR_PADDR2_MASK                    0xFFFF0000u
#define MACNET_PAUR_PADDR2_SHIFT                   16
#define MACNET_PAUR_PADDR2(x)                      (((uint32_t)(((uint32_t)(x))<<MACNET_PAUR_PADDR2_SHIFT))&MACNET_PAUR_PADDR2_MASK)
/* OPD Bit Fields */
#define MACNET_OPD_PAUSE_DUR_MASK                  0xFFFFu
#define MACNET_OPD_PAUSE_DUR_SHIFT                 0
#define MACNET_OPD_PAUSE_DUR(x)                    (((uint32_t)(((uint32_t)(x))<<MACNET_OPD_PAUSE_DUR_SHIFT))&MACNET_OPD_PAUSE_DUR_MASK)
#define MACNET_OPD_OPCODE_MASK                     0xFFFF0000u
#define MACNET_OPD_OPCODE_SHIFT                    16
#define MACNET_OPD_OPCODE(x)                       (((uint32_t)(((uint32_t)(x))<<MACNET_OPD_OPCODE_SHIFT))&MACNET_OPD_OPCODE_MASK)
/* IAUR Bit Fields */
#define MACNET_IAUR_IADDR1_MASK                    0xFFFFFFFFu
#define MACNET_IAUR_IADDR1_SHIFT                   0
#define MACNET_IAUR_IADDR1(x)                      (((uint32_t)(((uint32_t)(x))<<MACNET_IAUR_IADDR1_SHIFT))&MACNET_IAUR_IADDR1_MASK)
/* IALR Bit Fields */
#define MACNET_IALR_IADDR2_MASK                    0xFFFFFFFFu
#define MACNET_IALR_IADDR2_SHIFT                   0
#define MACNET_IALR_IADDR2(x)                      (((uint32_t)(((uint32_t)(x))<<MACNET_IALR_IADDR2_SHIFT))&MACNET_IALR_IADDR2_MASK)
/* GAUR Bit Fields */
#define MACNET_GAUR_GADDR1_MASK                    0xFFFFFFFFu
#define MACNET_GAUR_GADDR1_SHIFT                   0
#define MACNET_GAUR_GADDR1(x)                      (((uint32_t)(((uint32_t)(x))<<MACNET_GAUR_GADDR1_SHIFT))&MACNET_GAUR_GADDR1_MASK)
/* GALR Bit Fields */
#define MACNET_GALR_GADDR2_MASK                    0xFFFFFFFFu
#define MACNET_GALR_GADDR2_SHIFT                   0
#define MACNET_GALR_GADDR2(x)                      (((uint32_t)(((uint32_t)(x))<<MACNET_GALR_GADDR2_SHIFT))&MACNET_GALR_GADDR2_MASK)
/* TFWR Bit Fields */
#define MACNET_TFWR_TFWR_MASK                      0x3Fu
#define MACNET_TFWR_TFWR_SHIFT                     0
#define MACNET_TFWR_TFWR(x)                        (((uint32_t)(((uint32_t)(x))<<MACNET_TFWR_TFWR_SHIFT))&MACNET_TFWR_TFWR_MASK)
#define MACNET_TFWR_STRFWD_MASK                    0x100u
#define MACNET_TFWR_STRFWD_SHIFT                   8
/* RDSR Bit Fields */
#define MACNET_RDSR_R_DES_START_MASK               0xFFFFFFF8u
#define MACNET_RDSR_R_DES_START_SHIFT              3
#define MACNET_RDSR_R_DES_START(x)                 (((uint32_t)(((uint32_t)(x))<<MACNET_RDSR_R_DES_START_SHIFT))&MACNET_RDSR_R_DES_START_MASK)
/* TDSR Bit Fields */
#define MACNET_TDSR_X_DES_START_MASK               0xFFFFFFF8u
#define MACNET_TDSR_X_DES_START_SHIFT              3
#define MACNET_TDSR_X_DES_START(x)                 (((uint32_t)(((uint32_t)(x))<<MACNET_TDSR_X_DES_START_SHIFT))&MACNET_TDSR_X_DES_START_MASK)
/* MRBR Bit Fields */
#define MACNET_MRBR_R_BUF_SIZE_MASK                0x3FF0u
#define MACNET_MRBR_R_BUF_SIZE_SHIFT               4
#define MACNET_MRBR_R_BUF_SIZE(x)                  (((uint32_t)(((uint32_t)(x))<<MACNET_MRBR_R_BUF_SIZE_SHIFT))&MACNET_MRBR_R_BUF_SIZE_MASK)
/* RSFL Bit Fields */
#define MACNET_RSFL_RX_SECTION_FULL_MASK           0xFFu
#define MACNET_RSFL_RX_SECTION_FULL_SHIFT          0
#define MACNET_RSFL_RX_SECTION_FULL(x)             (((uint32_t)(((uint32_t)(x))<<MACNET_RSFL_RX_SECTION_FULL_SHIFT))&MACNET_RSFL_RX_SECTION_FULL_MASK)
/* RSEM Bit Fields */
#define MACNET_RSEM_RX_SECTION_EMPTY_MASK          0xFFu
#define MACNET_RSEM_RX_SECTION_EMPTY_SHIFT         0
#define MACNET_RSEM_RX_SECTION_EMPTY(x)            (((uint32_t)(((uint32_t)(x))<<MACNET_RSEM_RX_SECTION_EMPTY_SHIFT))&MACNET_RSEM_RX_SECTION_EMPTY_MASK)
/* RAEM Bit Fields */
#define MACNET_RAEM_RX_ALMOST_EMPTY_MASK           0xFFu
#define MACNET_RAEM_RX_ALMOST_EMPTY_SHIFT          0
#define MACNET_RAEM_RX_ALMOST_EMPTY(x)             (((uint32_t)(((uint32_t)(x))<<MACNET_RAEM_RX_ALMOST_EMPTY_SHIFT))&MACNET_RAEM_RX_ALMOST_EMPTY_MASK)
/* RAFL Bit Fields */
#define MACNET_RAFL_RX_ALMOST_FULL_MASK            0xFFu
#define MACNET_RAFL_RX_ALMOST_FULL_SHIFT           0
#define MACNET_RAFL_RX_ALMOST_FULL(x)              (((uint32_t)(((uint32_t)(x))<<MACNET_RAFL_RX_ALMOST_FULL_SHIFT))&MACNET_RAFL_RX_ALMOST_FULL_MASK)
/* TSEM Bit Fields */
#define MACNET_TSEM_TX_SECTION_EMPTY_MASK          0xFFu
#define MACNET_TSEM_TX_SECTION_EMPTY_SHIFT         0
#define MACNET_TSEM_TX_SECTION_EMPTY(x)            (((uint32_t)(((uint32_t)(x))<<MACNET_TSEM_TX_SECTION_EMPTY_SHIFT))&MACNET_TSEM_TX_SECTION_EMPTY_MASK)
/* TAEM Bit Fields */
#define MACNET_TAEM_TX_ALMOST_EMPTY_MASK           0xFFu
#define MACNET_TAEM_TX_ALMOST_EMPTY_SHIFT          0
#define MACNET_TAEM_TX_ALMOST_EMPTY(x)             (((uint32_t)(((uint32_t)(x))<<MACNET_TAEM_TX_ALMOST_EMPTY_SHIFT))&MACNET_TAEM_TX_ALMOST_EMPTY_MASK)
/* TAFL Bit Fields */
#define MACNET_TAFL_TX_ALMOST_FULL_MASK            0xFFu
#define MACNET_TAFL_TX_ALMOST_FULL_SHIFT           0
#define MACNET_TAFL_TX_ALMOST_FULL(x)              (((uint32_t)(((uint32_t)(x))<<MACNET_TAFL_TX_ALMOST_FULL_SHIFT))&MACNET_TAFL_TX_ALMOST_FULL_MASK)
/* TIPG Bit Fields */
#define MACNET_TIPG_IPG_MASK                       0x1Fu
#define MACNET_TIPG_IPG_SHIFT                      0
#define MACNET_TIPG_IPG(x)                         (((uint32_t)(((uint32_t)(x))<<MACNET_TIPG_IPG_SHIFT))&MACNET_TIPG_IPG_MASK)
/* FTRL Bit Fields */
#define MACNET_FTRL_TRUNC_FL_MASK                  0x3FFFu
#define MACNET_FTRL_TRUNC_FL_SHIFT                 0
#define MACNET_FTRL_TRUNC_FL(x)                    (((uint32_t)(((uint32_t)(x))<<MACNET_FTRL_TRUNC_FL_SHIFT))&MACNET_FTRL_TRUNC_FL_MASK)
/* TACC Bit Fields */
#define MACNET_TACC_SHIFT16_MASK                   0x1u
#define MACNET_TACC_SHIFT16_SHIFT                  0
#define MACNET_TACC_IPCHK_MASK                     0x8u
#define MACNET_TACC_IPCHK_SHIFT                    3
#define MACNET_TACC_PROCHK_MASK                    0x10u
#define MACNET_TACC_PROCHK_SHIFT                   4
/* RACC Bit Fields */
#define MACNET_RACC_PADREM_MASK                    0x1u
#define MACNET_RACC_PADREM_SHIFT                   0
#define MACNET_RACC_IPDIS_MASK                     0x2u
#define MACNET_RACC_IPDIS_SHIFT                    1
#define MACNET_RACC_PRODIS_MASK                    0x4u
#define MACNET_RACC_PRODIS_SHIFT                   2
#define MACNET_RACC_LINEDIS_MASK                   0x40u
#define MACNET_RACC_LINEDIS_SHIFT                  6
#define MACNET_RACC_SHIFT16_MASK                   0x80u
#define MACNET_RACC_SHIFT16_SHIFT                  7
/* ATCR Bit Fields */
#define MACNET_ATCR_EN_MASK                        0x1u
#define MACNET_ATCR_EN_SHIFT                       0
#define MACNET_ATCR_OFFEN_MASK                     0x4u
#define MACNET_ATCR_OFFEN_SHIFT                    2
#define MACNET_ATCR_OFFRST_MASK                    0x8u
#define MACNET_ATCR_OFFRST_SHIFT                   3
#define MACNET_ATCR_PEREN_MASK                     0x10u
#define MACNET_ATCR_PEREN_SHIFT                    4
#define MACNET_ATCR_PINPER_MASK                    0x80u
#define MACNET_ATCR_PINPER_SHIFT                   7
#define MACNET_ATCR_RESTART_MASK                   0x200u
#define MACNET_ATCR_RESTART_SHIFT                  9
#define MACNET_ATCR_CAPTURE_MASK                   0x800u
#define MACNET_ATCR_CAPTURE_SHIFT                  11
#define MACNET_ATCR_SLAVE_MASK                     0x2000u
#define MACNET_ATCR_SLAVE_SHIFT                    13
/* ATVR Bit Fields */
#define MACNET_ATVR_ATIME_MASK                     0xFFFFFFFFu
#define MACNET_ATVR_ATIME_SHIFT                    0
#define MACNET_ATVR_ATIME(x)                       (((uint32_t)(((uint32_t)(x))<<MACNET_ATVR_ATIME_SHIFT))&MACNET_ATVR_ATIME_MASK)
/* ATOFF Bit Fields */
#define MACNET_ATOFF_OFFSET_MASK                   0xFFFFFFFFu
#define MACNET_ATOFF_OFFSET_SHIFT                  0
#define MACNET_ATOFF_OFFSET(x)                     (((uint32_t)(((uint32_t)(x))<<MACNET_ATOFF_OFFSET_SHIFT))&MACNET_ATOFF_OFFSET_MASK)
/* ATPER Bit Fields */
#define MACNET_ATPER_PERIOD_MASK                   0xFFFFFFFFu
#define MACNET_ATPER_PERIOD_SHIFT                  0
#define MACNET_ATPER_PERIOD(x)                     (((uint32_t)(((uint32_t)(x))<<MACNET_ATPER_PERIOD_SHIFT))&MACNET_ATPER_PERIOD_MASK)
/* ATCOR Bit Fields */
#define MACNET_ATCOR_COR_MASK                      0x7FFFFFFFu
#define MACNET_ATCOR_COR_SHIFT                     0
#define MACNET_ATCOR_COR(x)                        (((uint32_t)(((uint32_t)(x))<<MACNET_ATCOR_COR_SHIFT))&MACNET_ATCOR_COR_MASK)
/* ATINC Bit Fields */
#define MACNET_ATINC_INC_MASK                      0x7Fu
#define MACNET_ATINC_INC_SHIFT                     0
#define MACNET_ATINC_INC(x)                        (((uint32_t)(((uint32_t)(x))<<MACNET_ATINC_INC_SHIFT))&MACNET_ATINC_INC_MASK)
#define MACNET_ATINC_INC_CORR_MASK                 0x7F00u
#define MACNET_ATINC_INC_CORR_SHIFT                8
#define MACNET_ATINC_INC_CORR(x)                   (((uint32_t)(((uint32_t)(x))<<MACNET_ATINC_INC_CORR_SHIFT))&MACNET_ATINC_INC_CORR_MASK)
/* ATSTMP Bit Fields */
#define MACNET_ATSTMP_TIMESTAMP_MASK               0xFFFFFFFFu
#define MACNET_ATSTMP_TIMESTAMP_SHIFT              0
#define MACNET_ATSTMP_TIMESTAMP(x)                 (((uint32_t)(((uint32_t)(x))<<MACNET_ATSTMP_TIMESTAMP_SHIFT))&MACNET_ATSTMP_TIMESTAMP_MASK)
/* TGSR Bit Fields */
#define MACNET_TGSR_TF0_MASK                       0x1u
#define MACNET_TGSR_TF0_SHIFT                      0
#define MACNET_TGSR_TF1_MASK                       0x2u
#define MACNET_TGSR_TF1_SHIFT                      1
#define MACNET_TGSR_TF2_MASK                       0x4u
#define MACNET_TGSR_TF2_SHIFT                      2
#define MACNET_TGSR_TF3_MASK                       0x8u
#define MACNET_TGSR_TF3_SHIFT                      3
/* TCSR Bit Fields */
#define MACNET_TCSR_TDRE_MASK                      0x1u
#define MACNET_TCSR_TDRE_SHIFT                     0
#define MACNET_TCSR_TMODE_MASK                     0x3Cu
#define MACNET_TCSR_TMODE_SHIFT                    2
#define MACNET_TCSR_TMODE(x)                       (((uint32_t)(((uint32_t)(x))<<MACNET_TCSR_TMODE_SHIFT))&MACNET_TCSR_TMODE_MASK)
#define MACNET_TCSR_TIE_MASK                       0x40u
#define MACNET_TCSR_TIE_SHIFT                      6
#define MACNET_TCSR_TF_MASK                        0x80u
#define MACNET_TCSR_TF_SHIFT                       7
/* TCCR Bit Fields */
#define MACNET_TCCR_TCC_MASK                       0xFFFFFFFFu
#define MACNET_TCCR_TCC_SHIFT                      0
#define MACNET_TCCR_TCC(x)                         (((uint32_t)(((uint32_t)(x))<<MACNET_TCCR_TCC_SHIFT))&MACNET_TCCR_TCC_MASK)

#endif /*__MACNET_H__*/