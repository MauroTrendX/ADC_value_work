///*  Lynx Tecnologia Eletronica Ltda.
// *  Produto: ADS018 
// *  Arquivo: mma8653.h
// *  Descricao: Header para Acelerometro MMA8653/MMA8652 Interface TWI
// */

//#ifndef _MMA8653_H
//#define _MMA8653_H

//#include <stdbool.h>
//#include <stdint.h>

//#define MMA8653_I2C_ADDR        	   0x1D /*!< I2C DEVICE ADDRESS */
//#define MMA8653_ID                     0x5A /*!< DEVICE ID */ 
//#define MMA8X5X_PL_BF_ZCOMP_RO         0x44 /*!< Z LOCKOUT ANGLE = 30 */
//#define MMA8X5X_PL_THS_REG_RO          0x84 /*!< PL TRIP TRESHOLD = 45 */

//// STATUS bits
//#define MMA8X5X_XDR_Pos            0                                      /*!< STATUS X DATA READY Position */
//#define MMA8X5X_XDR_Msk            (0x01 << MMA8X5X_XDR_Pos)              /*!< STATUS X DATA READY Mask */
//#define MMA8X5X_YDR_Pos            1                                      /*!< STATUS Y DATA READY Position */
//#define MMA8X5X_YDR_Msk            (0x01 << MMA8X5X_YDR_Pos)              /*!< STATUS Y DATA READY Mask */
//#define MMA8X5X_ZDR_Pos            2                                      /*!< STATUS Z DATA READY Position */
//#define MMA8X5X_ZDR_Msk            (0x01 << MMA8X5X_ZDR_Pos)              /*!< STATUS Z DATA READY Mask */
//#define MMA8X5X_XYZDR_Pos          3                                      /*!< STATUS X/Y/Z DATA READY Position */
//#define MMA8X5X_XYZDR_Msk          (0x01 << MMA8X5X_XYZDR_Pos)            /*!< STATUS X/Y/Z DATA READY Mask */
//#define MMA8X5X_XOW_Pos            4                                      /*!< STATUS X OVERWRITE Position */
//#define MMA8X5X_XOW_Msk            (0x01 << MMA8X5X_XOW_Pos)              /*!< STATUS X OVERWRITE Mask */
//#define MMA8X5X_YOW_Pos            5                                      /*!< STATUS Y OVERWRITE Position */
//#define MMA8X5X_YOW_Msk            (0x01 << MMA8X5X_YOW_Pos)              /*!< STATUS Y OVERWRITE Mask */
//#define MMA8X5X_ZOW_Pos            6                                      /*!< STATUS Z OVERWRITE Position */
//#define MMA8X5X_ZOW_Msk            (0x01 << MMA8X5X_ZOW_Pos)              /*!< STATUS Z OVERWRITE Mask */
//#define MMA8X5X_XYZOW_Pos          7                                      /*!< STATUS X/Y/Z OVERWRITE Position */
//#define MMA8X5X_XYZOW_Msk          (0x01 << MMA8X5X_XYZOW_Pos)            /*!< STATUS X/Y/Z OVERWRITE Mask */

//// SYSMOD bits
//#define MMA8X5X_WAKE_Pos           0                                      /*!< SYSMOD WAKE Position */
//#define MMA8X5X_WAKE_Msk           (0x01 << MMA8X5X_WAKE_Pos)             /*!< SYSMOD WAKE Mask */
//#define MMA8X5X_SLEEP_Pos          1                                      /*!< SYSMOD SLEEP Position */
//#define MMA8X5X_SLEEP_Msk          (0x01 << MMA8X5X_SLEEP_Pos)            /*!< SYSMOD SLEEP Mask */

//// INT_SOURCE bits
//#define MMA8X5X_SRC_DRDY_Pos       0                                      /*!< INT_SOURCE DATA READY Position */
//#define MMA8X5X_SRC_DRDY_Msk       (0x01 << MMA8X5X_SRC_DRDY_Pos)         /*!< INT_SOURCE DATA READY Mask */
//#define MMA8X5X_SRC_FF_MT_Pos      2                                      /*!< INT_SOURCE FREEFALL/MOTION Position */
//#define MMA8X5X_SRC_FF_MT_Msk      (0x01 << MMA8X5X_SRC_FF_MT_Pos)        /*!< INT_SOURCE FREEFALL/MOTION Mask */
//#define MMA8X5X_SRC_LNDPRT_Pos     4                                      /*!< INT_SOURCE LANDSCAPE/PORTRAIT Position */
//#define MMA8X5X_SRC_LNDPRT_Msk     (0x01 << MMA8X5X_SRC_LNDPRT_Pos)       /*!< INT_SOURCE LANDSCAPE/PORTRAIT Mask */
//#define MMA8X5X_SRC_ASLP_Pos       7                                      /*!< INT_SOURCE AUTO SLEEP/WALK Position */
//#define MMA8X5X_SRC_ASLP_Msk       (0x01 << MMA8X5X_SRC_ASLP_Pos)         /*!< INT_SOURCE AUTO SLEEP/WALK Mask */

//// XYZ_DAT_CFG bits
//#define MMA8X5X_FS_Pos             0                                      /*!< XYZ_DAT_CFG FULL SCALE Position */
//#define MMA8X5X_FS_Msk             (0x03 << MMA8X5X_FS_Pos)               /*!< XYZ_DAT_CFG FULL SCALE Mask */

//#define MMA8X5X_FS_2G              (0x00 << MMA8X5X_FS_Pos)               /*!< XYZ_DAT_CFG SCALE +/-2G  */
//#define MMA8X5X_FS_4G              (0x01 << MMA8X5X_FS_Pos)               /*!< XYZ_DAT_CFG SCALE +/-4G  */
//#define MMA8X5X_FS_8G              (0x02 << MMA8X5X_FS_Pos)               /*!< XYZ_DAT_CFG SCALE +/-8G  */

//// PL_STATUS bits
//#define MMA8X5X_BAFRO_Pos          0                                      /*!< PL_STATUS BACK OR FRONT Position */
//#define MMA8X5X_BAFRO_Msk          (0x01 << MMA8X5X_BAFRO_Pos)            /*!< PL_STATUS BACK OR FRONT Mask */
//#define MMA8X5X_LAPO_Pos           1                                      /*!< PL_STATUS LANDSCAPE/PORTRAIT ORIENTATION Position */
//#define MMA8X5X_LAPO_Msk           (0x03 << MMA8X5X_LAPO_Pos)             /*!< PL_STATUS LANDSCAPE/PORTRAIT ORIENTATION Mask */
//#define MMA8X5X_LO_Pos             6                                      /*!< PL_STATUS Z TILT ANGLE LOCKOUT Position */
//#define MMA8X5X_LO_Msk             (0x01 << MMA8X5X_LO_Pos)               /*!< PL_STATUS Z TILT ANGLE LOCKOUT Mask */
//#define MMA8X5X_NEWLP_Pos          7                                      /*!< PL_STATUS LANDSCAPE/PORTRAIT STATUS CHANGE FLAG Position */
//#define MMA8X5X_NEWLP_Msk          (0x01 << MMA8X5X_NEWLP_Pos)            /*!< PL_STATUS LANDSCAPE/PORTRAIT STATUS CHANGE FLAG Mask */

//#define MMA8X5X_LAPO_PO_UP         (0x00 << MMA8X5X_LAPO_Pos)             /*!< PL_STATUS ORIENTATION PORTRAIT UP */
//#define MMA8X5X_LAPO_PO_DN         (0x01 << MMA8X5X_LAPO_Pos)             /*!< PL_STATUS ORIENTATION PORTRAIT DOWN */
//#define MMA8X5X_LAPO_LA_RIGHT      (0x02 << MMA8X5X_LAPO_Pos)             /*!< PL_STATUS ORIENTATION LANDSCAPE RIGHT */
//#define MMA8X5X_LAPO_LA_LEFT       (0x03 << MMA8X5X_LAPO_Pos)             /*!< PL_STATUS ORIENTATION LANDSCAPE LEFT */

//// PL_CFG bits
//#define MMA8X5X_PL_EN_Pos          6                                      /*!< PL_CFG LANDSCAPE/PORTRAIT DETECTION ENABLE Position */
//#define MMA8X5X_PL_EN_Msk          (0x01 << MMA8X5X_PL_EN_Pos)            /*!< PL_CFG LANDSCAPE/PORTRAIT DETECTION ENABLE Mask */
//#define MMA8X5X_DBCNTM_Pos         7                                      /*!< PL_CFG LANDSCAPE/PORTRAIT DEBOUNCE COUNTER MODE Position */
//#define MMA8X5X_DBCNTM_Msk         (0x01 << MMA8X5X_DBCNTM_Pos)           /*!< PL_CFG LANDSCAPE/PORTRAIT DEBOUNCE COUNTER MODE Mask */

//// FF_MT_CFG bits
//#define MMA8X5X_XEFE_Pos           3                                      /*!< FF_MT_CFG EVENT FLAG ENABLE ON X EVENT Position */
//#define MMA8X5X_XEFE_Msk           (0x01 << MMA8X5X_XEFE_Pos)             /*!< FF_MT_CFG EVENT FLAG ENABLE ON X EVENT Mask */
//#define MMA8X5X_YEFE_Pos           4                                      /*!< FF_MT_CFG EVENT FLAG ENABLE ON Y EVENT Position */
//#define MMA8X5X_YEFE_Msk           (0x01 << MMA8X5X_YEFE_Pos)             /*!< FF_MT_CFG EVENT FLAG ENABLE ON Y EVENT Mask */
//#define MMA8X5X_ZEFE_Pos           5                                      /*!< FF_MT_CFG EVENT FLAG ENABLE ON Z EVENT Position */
//#define MMA8X5X_ZEFE_Msk           (0x01 << MMA8X5X_ZEFE_Pos)             /*!< FF_MT_CFG EVENT FLAG ENABLE ON Z EVENT Mask */
//#define MMA8X5X_OAE_Pos            6                                      /*!< FF_MT_CFG MOTION DETECT/FREEFALL DETECT SELECTION Position */
//#define MMA8X5X_OAE_Msk            (0x01 << MMA8X5X_OAE_Pos)              /*!< FF_MT_CFG MOTION DETECT/FREEFALL DETECT SELECTION Mask */
//#define MMA8X5X_ELE_Pos            7                                      /*!< FF_MT_CFG EVENT LACTH ENABLE Position */
//#define MMA8X5X_ELE_Msk            (0x01 << MMA8X5X_ELE_Pos)              /*!< FF_MT_CFG EVENT LACTH ENABLE Mask */

//// FF_MT_SRC bits
//#define MMA8X5X_XHP_Pos            0                                      /*!< FF_MT_SRC X MOTION POLARITY FLAG Position */
//#define MMA8X5X_XHP_Msk            (0x01 << MMA8X5X_XHP_Pos)              /*!< FF_MT_SRC X MOTION POLARITY FLAG Mask */
//#define MMA8X5X_XHE_Pos            1                                      /*!< FF_MT_SRC X MOTION FLAG Position */
//#define MMA8X5X_XHE_Msk            (0x01 << MMA8X5X_XHE_Pos)              /*!< FF_MT_SRC X MOTION FLAG Mask */
//#define MMA8X5X_YHP_Pos            2                                      /*!< FF_MT_SRC Y MOTION POLARITY FLAG Position */
//#define MMA8X5X_YHP_Msk            (0x01 << MMA8X5X_YHP_Pos)              /*!< FF_MT_SRC Y MOTION POLARITY FLAG Mask */
//#define MMA8X5X_YHE_Pos            3                                      /*!< FF_MT_SRC Y MOTION FLAG Position */
//#define MMA8X5X_YHE_Msk            (0x01 << MMA8X5X_YHE_Pos)              /*!< FF_MT_SRC Y MOTION FLAG Mask */
//#define MMA8X5X_ZHP_Pos            4                                      /*!< FF_MT_SRC Z MOTION POLARITY FLAG Position */
//#define MMA8X5X_ZHP_Msk            (0x01 << MMA8X5X_ZHP_Pos)              /*!< FF_MT_SRC Z MOTION POLARITY FLAG Mask */
//#define MMA8X5X_ZHE_Pos            5                                      /*!< FF_MT_SRC Z MOTION FLAG Position */
//#define MMA8X5X_ZHE_Msk            (0x01 << MMA8X5X_ZHE_Pos)              /*!< FF_MT_SRC Z MOTION FLAG Mask */
//#define MMA8X5X_EA_Pos             7                                      /*!< FF_MT_SRC ACTIVE EVENT FLAG Position */
//#define MMA8X5X_EA_Msk             (0x01 << MMA8X5X_EA_Pos)               /*!< FF_MT_SRC ACTIVE EVENT FLAG Mask */

//// FF_MT_THS bits
//#define MMA8X5X_THS_Pos            0                                      /*!< FF_MT_THS FREEFALL/MOTION THRESHOLD Position */
//#define MMA8X5X_THS_Msk            (0x7F << MMA8X5X_THS_Pos)              /*!< FF_MT_THS FREEFALL/MOTION THRESHOLD Mask */
//#define MMA8X5X_DCMS_Pos           7                                      /*!< FF_MT_THS DEBOUNCE COUNTER MODE SELECTION Position */
//#define MMA8X5X_DCMS_Msk           (0x01 << MMA8X5X_DCMS_Pos)             /*!< FF_MT_THS DEBOUNCE COUNTER MODE SELECTION Mask */

//// CTRL_REG1 bits - System Control 1
//#define MMA8X5X_ACTIVE_Pos         0                                      /*!< CTRL_REG1 ACTIVE Position */
//#define MMA8X5X_ACTIVE_Msk         (0x01 << MMA8X5X_ACTIVE_Pos)           /*!< CTRL_REG1 ACTIVE Mask */
//#define MMA8X5X_F_READ_Pos         1                                      /*!< CTRL_REG1 FAST READ Position */
//#define MMA8X5X_F_READ_Msk         (0x01 << MMA8X5X_F_READ_Pos)           /*!< CTRL_REG1 FAST READ Mask */
//#define MMA8X5X_DR_Pos             3                                      /*!< CTRL_REG1 DATA RATE SELECTION Position */
//#define MMA8X5X_DR_Msk             (0x07 << MMA8X5X_DR_Pos)               /*!< CTRL_REG1 DATA RATE SELECTION Mask */
//#define MMA8X5X_ASLP_RATE_Pos      6                                      /*!< CTRL_REG1 AUTO-WAKE SAMPLE FREQUENCY IN SLEEP Position */
//#define MMA8X5X_ASLP_RATE_Msk      (0x03 << MMA8X5X_ASLP_RATE_Pos)        /*!< CTRL_REG1 AUTO-WAKE SAMPLE FREQUENCY IN SLEEP Mask */

//#define MMA8X5X_DR_800             (0X00 << MMA8X5X_DR_Pos)               /*!< CTRL_REG1 ODR 800HZ IN WALK MODE */
//#define MMA8X5X_DR_400             (0X01 << MMA8X5X_DR_Pos)               /*!< CTRL_REG1 ODR 400HZ IN WALK MODE */
//#define MMA8X5X_DR_200             (0X02 << MMA8X5X_DR_Pos)               /*!< CTRL_REG1 ODR 200HZ IN WALK MODE */
//#define MMA8X5X_DR_100             (0X03 << MMA8X5X_DR_Pos)               /*!< CTRL_REG1 ODR 100HZ IN WALK MODE */
//#define MMA8X5X_DR_50              (0X04 << MMA8X5X_DR_Pos)               /*!< CTRL_REG1 ODR 50HZ IN WALK MODE */
//#define MMA8X5X_DR_12_5            (0X05 << MMA8X5X_DR_Pos)               /*!< CTRL_REG1 ODR 12.5HZ IN WALK MODE */
//#define MMA8X5X_DR_6_25            (0X06 << MMA8X5X_DR_Pos)               /*!< CTRL_REG1 ODR 6.25HZ IN WALK MODE */
//#define MMA8X5X_DR_1_56            (0X07 << MMA8X5X_DR_Pos)               /*!< CTRL_REG1 ODR 1.56HZ IN WALK MODE */

//#define MMA8X5X_ASLP_RATE_50       (0X00 << MMA8X5X_ASLP_RATE_Pos)        /*!< CTRL_REG1 ODR 50HZ IN SLEEP MODE */
//#define MMA8X5X_ASLP_RATE_12_5     (0X01 << MMA8X5X_ASLP_RATE_Pos)        /*!< CTRL_REG1 ODR 12.5HZ IN SLEEP MODE */
//#define MMA8X5X_ASLP_RATE_6_25     (0X02 << MMA8X5X_ASLP_RATE_Pos)        /*!< CTRL_REG1 ODR 6.25HZ IN SLEEP MODE */
//#define MMA8X5X_ASLP_RATE_1_56     (0X03 << MMA8X5X_ASLP_RATE_Pos)        /*!< CTRL_REG1 ODR 1.56HZ IN SLEEP MODE */

//// CTRL_REG2 bits - System Control 2
//#define MMA8X5X_AMODS_Pos          0                                      /*!< CTRL_REG2 ACTIVE POWER SCHEME Position */
//#define MMA8X5X_AMODS_Msk          (0x03 << MMA8X5X_AMODS_Pos)            /*!< CTRL_REG2 ACTIVE POWER SCHEME Mask */
//#define MMA8X5X_SLPE_Pos           2                                      /*!< CTRL_REG2 AUTO SLEEP ENABLE Position */
//#define MMA8X5X_SLPE_Msk           (0x01 << MMA8X5X_SLPE_Pos)             /*!< CTRL_REG2 AUTO SLEEP ENABLE Mask */
//#define MMA8X5X_SMODS_Pos          3                                      /*!< CTRL_REG2 SLEEP POWER SCHEME Position */
//#define MMA8X5X_SMODS_Msk          (0x03 << MMA8X5X_SMODS_Pos)            /*!< CTRL_REG2 SLEEP POWER SCHEME Mask */
//#define MMA8X5X_RST_Pos            6                                      /*!< CTRL_REG2 SOFTWARE RESET Position */
//#define MMA8X5X_RST_Msk            (0x01 << MMA8X5X_RST_Pos)              /*!< CTRL_REG2 SOFTWARE RESET Mask */
//#define MMA8X5X_ST_Pos             7                                      /*!< CTRL_REG2 SELF TEST Position */
//#define MMA8X5X_ST_Msk             (0x01 << MMA8X5X_ST_Pos)               /*!< CTRL_REG2 SELF TEST Mask */

//#define MMA8X5X_AMODS_NORMAL        (0X00 << MMA8X5X_AMODS_Pos)           /*!< CTRL_REG2 NORMAL ACTIVE POWER SCHEME */
//#define MMA8X5X_AMODS_LNLP          (0X01 << MMA8X5X_AMODS_Pos)           /*!< CTRL_REG2 LOW NOISE LOW ACTIVE POWER POWER SCHEME */
//#define MMA8X5X_AMODS_HR            (0X02 << MMA8X5X_AMODS_Pos)           /*!< CTRL_REG2 HIGH RESOLUTION ACTIVE POWER SCHEME */
//#define MMA8X5X_AMODS_LP            (0X03 << MMA8X5X_AMODS_Pos)           /*!< CTRL_REG2 LOW POWER ACTIVE POWER SCHEME */

//#define MMA8X5X_SMODS_NORMAL        (0X00 << MMA8X5X_SMODS_Pos)           /*!< CTRL_REG2 NORMAL SLEEP POWER SCHEME */
//#define MMA8X5X_SMODS_LNLP          (0X01 << MMA8X5X_SMODS_Pos)           /*!< CTRL_REG2 LOW NOISE LOW SLEEP POWER POWER SCHEME */
//#define MMA8X5X_SMODS_HR            (0X02 << MMA8X5X_SMODS_Pos)           /*!< CTRL_REG2 HIGH RESOLUTION SLEEP POWER SCHEME */
//#define MMA8X5X_SMODS_LP            (0X03 << MMA8X5X_SMODS_Pos)           /*!< CTRL_REG2 LOW POWER SLEEP POWER SCHEME */

//// CTRL_REG3 bits - Interrupt Control
//#define MMA8X5X_PP_OD_Pos          0                                      /*!< CTRL_REG3 PUSH PULL/OPEN DRAIN Position */
//#define MMA8X5X_PP_OD_Msk          (0x01 << MMA8X5X_PP_OD_Pos)            /*!< CTRL_REG3 PUSH PULL/OPEN DRAIN Mask */
//#define MMA8X5X_IPOL_Pos           1                                      /*!< CTRL_REG3 INTERRUPT POLARITY Position */
//#define MMA8X5X_IPOL_Msk           (0x01 << MMA8X5X_IPOL_Pos)             /*!< CTRL_REG3 INTERRUPT POLARITY Mask */
//#define MMA8X5X_WAKE_FF_MT_Pos     3                                      /*!< CTRL_REG3 WAKE FROM FREEFALL/MOTION INTERRUPT Position */
//#define MMA8X5X_WAKE_FF_MT_Msk     (0x01 << MMA8X5X_WAKE_FF_MT_Pos)       /*!< CTRL_REG3 WAKE FROM FREEFALL/MOTION INTERRUPT Mask */
//#define MMA8X5X_WAKE_LAPO_Pos      5                                      /*!< CTRL_REG3 WAKE FROM LANDSCAPE/PORTRAIT INTERRUPT Position */
//#define MMA8X5X_WAKE_LAPO_Msk      (0x01 << MMA8X5X_WAKE_LAPO_Pos)        /*!< CTRL_REG3 WAKE FROM LANDSCAPE/PORTRAIT INTERRUPT Mask */

//// CTRL_REG4 bits - Interrupt Enable
//#define MMA8X5X_INT_EN_DRDY_Pos    0                                      /*!< CTRL_REG4 DATA READY INTERRUPT ENABLE Position */
//#define MMA8X5X_INT_EN_DRDY_Msk    (0x01 << MMA8X5X_INT_EN_DRDY_Pos)      /*!< CTRL_REG4 DATA READY INTERRUPT ENABLE Mask */
//#define MMA8X5X_INT_EN_FF_MT_Pos   2                                      /*!< CTRL_REG4 FREEFALL/MOTION INTERRUPT ENABLE Position */
//#define MMA8X5X_INT_EN_FF_MT_Msk   (0x01 << MMA8X5X_INT_EN_FF_MT_Pos)     /*!< CTRL_REG4 FREEFALL/MOTION INTERRUPT ENABLE Mask */
//#define MMA8X5X_INT_EN_LAPO_Pos    4                                      /*!< CTRL_REG4 LANDSCAPE/PORTRAIT INTERRUPT ENABLE Position */
//#define MMA8X5X_INT_EN_LAPO_Msk    (0x01 << MMA8X5X_INT_EN_LAPO_Pos)      /*!< CTRL_REG4 LANDSCAPE/PORTRAIT INTERRUPT ENABLE Mask */
//#define MMA8X5X_INT_EN_ASLP_Pos    7                                      /*!< CTRL_REG4 AUTO SLEEP/WAKE INTERRUPT ENABLE Position */
//#define MMA8X5X_INT_EN_ASLP_Msk    (0x01 << MMA8X5X_INT_EN_ASLP_Pos)      /*!< CTRL_REG4 AUTO SLEEP/WAKE INTERRUPT ENABLE Mask */

//// CTRL_REG5 bits - Interrupt Configuration
//#define MMA8X5X_INT_CFG_DRDY_Pos   0                                      /*!< CTRL_REG5 DATA READY INTERRUPT CONFIG Position */
//#define MMA8X5X_INT_CFG_DRDY_Msk   (0x01 << MMA8X5X_INT_CFG_DRDY_Pos)     /*!< CTRL_REG5 DATA READY INTERRUPT CONFIG Mask */
//#define MMA8X5X_INT_CFG_FF_MT_Pos  2                                      /*!< CTRL_REG5 FREEFALL/MOTION INTERRUPT CONFIG Position */
//#define MMA8X5X_INT_CFG_FF_MT_Msk  (0x01 << MMA8X5X_INT_CFG_FF_MT_Pos)    /*!< CTRL_REG5 FREEFALL/MOTION INTERRUPT CONFIG Mask */
//#define MMA8X5X_INT_CFG_LAPO_Pos   4                                      /*!< CTRL_REG5 LANDSCAPE/PORTRAIT INTERRUPT CONFIG Position */
//#define MMA8X5X_INT_CFG_LAPO_Msk   (0x01 << MMA8X5X_INT_CFG_LAPO_Pos)     /*!< CTRL_REG5 LANDSCAPE/PORTRAIT INTERRUPT CONFIG Mask */
//#define MMA8X5X_INT_CFG_ASLP_Pos   7                                      /*!< CTRL_REG5 AUTO SLEEP/WAKE INTERRUPT CONFIG Position */
//#define MMA8X5X_INT_CFG_ASLP_Msk   (0x01 << MMA8X5X_INT_CFG_ASLP_Pos)     /*!< CTRL_REG5 AUTO SLEEP/WAKE INTERRUPT CONFIG Mask */

//// MACROS
//#define MMA8X5X_TST_Z(X,M) ((X & M)==0)
//#define MMA8X5X_TST_NZ(X,M) ((X & M)!=0)
//#define MMA8X5X_SET(X,M) X=(X | M)
//#define MMA8X5X_CLR(X,M) X=(X & (~M))
//#define MMA8X5X_SET_FIELD(X,F,M) X=((X & (~F)) | M)

//// DEFAULT SETUP
//#define MMA8X5X_XYZ_DATA_CFG_DEFAULT 0x00
//// range +/-2g
//#define MMA8X5X_PL_CFG_DEFAULT       (MMA8X5X_DBCNTM_Msk | MMA8X5X_PL_EN_Msk)
//// PL: debounce counter mode 1, enabled
////#define MMA8X5X_PL_COUNT_DEFAULT     0x05
//#define MMA8X5X_PL_COUNT_DEFAULT     0x01
//// PL: no debounce delay
//#define MMA8X5X_MT_CFG_DEFAULT       0x00
//// MT: not used
//#define MMA8X5X_MT_COUNT_DEFAULT     0x00
//#define MMA8X5X_MT_THS_DEFAULT       0x00

//#define MMA8X5X_ASLP_COUNT_DEFAULT   0xff
//// if 0xc0 => WALK ODR = 50Hz => ASLP_COUNT step = 0,32s, MINIMUM TIME PERIOD OF INACTIVITY=192*0,32=61,44s,
//// if 0xff -> WALK ODR = 50Hz => ASLP_COUNT step = 0,32s, MINIMUM TIME PERIOD OF INACTIVITY=255*0,32=81,60S,

//#define MMA8X5X_CTRL_REG1_DEFAULT    (MMA8X5X_ASLP_RATE_1_56 | MMA8X5X_DR_50)
//// SLEEP ODR = 1.56Hz , WALK ODR = 50Hz 

//#define MMA8X5X_CTRL_REG2_DEFAULT    (MMA8X5X_SMODS_LP | MMA8X5X_SLPE_Msk | MMA8X5X_AMODS_LP)
//// SLEEP power mode = LOW POWER, Auto-SLEEP enabled, Active-WAKE power mode = LOW POWER

//#define MMA8X5X_CTRL_REG3_DEFAULT    (MMA8X5X_WAKE_LAPO_Msk)
//// WAKE from orientation interrupt, active low, push-pull

//#define MMA8X5X_CTRL_REG4_DEFAULT    ((MMA8X5X_INT_EN_DRDY_Msk) | (MMA8X5X_INT_EN_LAPO_Msk))
//// Enable Data Ready interrupt and X-Y Orientation interrupt

//#define MMA8X5X_CTRL_REG5_DEFAULT    (MMA8X5X_INT_CFG_LAPO_Msk)
//// X-Y Orientation interrupt on INT1 - Data Ready interrupt on INT2

///* register enum for mma8x5x registers */
//enum {
//	MMA8X5X_STATUS = 0x00,
//	MMA8X5X_OUT_X_MSB,
//	MMA8X5X_OUT_X_LSB,
//	MMA8X5X_OUT_Y_MSB,
//	MMA8X5X_OUT_Y_LSB,
//	MMA8X5X_OUT_Z_MSB,
//	MMA8X5X_OUT_Z_LSB,

//	MMA8X5X_F_SETUP = 0x09,
//	MMA8X5X_TRIG_CFG,
//	MMA8X5X_SYSMOD,
//	MMA8X5X_INT_SOURCE,
//	MMA8X5X_WHO_AM_I,
//	MMA8X5X_XYZ_DATA_CFG,
//	MMA8X5X_HP_FILTER_CUTOFF,

//	MMA8X5X_PL_STATUS,
//	MMA8X5X_PL_CFG,
//	MMA8X5X_PL_COUNT,
//	MMA8X5X_PL_BF_ZCOMP,
//	MMA8X5X_PL_THS_REG,

//	MMA8X5X_FF_MT_CFG,
//	MMA8X5X_FF_MT_SRC,
//	MMA8X5X_FF_MT_THS,
//	MMA8X5X_FF_MT_COUNT,

//	MMA8X5X_TRANSIENT_CFG = 0x1D,
//	MMA8X5X_TRANSIENT_SRC,
//	MMA8X5X_TRANSIENT_THS,
//	MMA8X5X_TRANSIENT_COUNT,

//	MMA8X5X_PULSE_CFG,
//	MMA8X5X_PULSE_SRC,
//	MMA8X5X_PULSE_THSX,
//	MMA8X5X_PULSE_THSY,
//	MMA8X5X_PULSE_THSZ,
//	MMA8X5X_PULSE_TMLT,
//	MMA8X5X_PULSE_LTCY,
//	MMA8X5X_PULSE_WIND,

//	MMA8X5X_ASLP_COUNT,
//	MMA8X5X_CTRL_REG1,
//	MMA8X5X_CTRL_REG2,
//	MMA8X5X_CTRL_REG3,
//	MMA8X5X_CTRL_REG4,
//	MMA8X5X_CTRL_REG5,

//	MMA8X5X_OFF_X,
//	MMA8X5X_OFF_Y,
//	MMA8X5X_OFF_Z,

//	MMA8X5X_REG_END,
//};

//enum {
//	STANDBY = 0,
//	ACTIVED_WAKE,
//	ACTIVED_SLEEP,
//};

//enum {
//	FS_2G = 0,
//	FS_4G,
//	FS_8G,
//};

//typedef struct {
//	int16_t xd;
//	int16_t yd;
//	int16_t zd;
//} mma8x5x_data_Type;

//typedef struct {
//	int8_t xc;
//	int8_t yc;
//	int8_t zc;
//} mma8x5x_offset_Type;

//typedef struct {
//	uint8_t status;
//	uint8_t data[6];
//	
//	uint8_t sysmod;
//	uint8_t int_source;
//	uint8_t who_am_i;
//	uint8_t xyz_data_cfg;
//	
//	uint8_t pl_status;
//	uint8_t pl_cfg;
//	uint8_t pl_count;
//	uint8_t pl_bf_zcomp;
//	uint8_t pl_ths_reg;
//	
//	uint8_t ff_mt_cfg;
//	uint8_t ff_mt_src;
//	uint8_t ff_mt_ths;
//	uint8_t ff_mt_count;

//    uint8_t aslp_count;
//	uint8_t ctrl_reg1;
//	uint8_t ctrl_reg2;
//	uint8_t ctrl_reg3;
//	uint8_t ctrl_reg4;
//	uint8_t ctrl_reg5;
//	
//	mma8x5x_offset_Type off;
//} mma8x5x_regs_Type;

//typedef struct {
//	uint8_t address;	
//	mma8x5x_regs_Type* pregs;
//} mma8x5x_setup_Type;

////typedef void (*mma8x5x_evt_handler_t)(void); /* INT app service call */

///** @file
//* @brief MMA8653 accelerometer driver.
//* @defgroup nrf_drivers_MMA8653 - MMA8x5x/MMA8653 accelerometer driver
//* @{
//* @ingroup nrf_drivers
//* @brief MMA8653 accelerometer driver.
//*/

///**
//  @brief Function for writing a MMA8x5x register contents over TWI.
//  @param[in] register_address : Register address to start writing to
//  @param[in] value : Value to write to register
//  @retval true Register write succeeded
//  @retval false Register write failed
//*/
//bool mma8x5x_write(uint8_t register_address, uint8_t value);

///**
//  @brief Function for reading MMA8x5x register contents over TWI.
//  Reads one or more consecutive registers.
//  @param[in]  register_address : Register address to start reading from
//  @param[in]  number_of_bytes : Number of bytes to read
//  @param[out] destination : Pointer to a data buffer where read data will be stored
//  @retval true Register read succeeded
//  @retval false Register read failed
//*/
//bool mma8x5x_read(uint8_t register_address, uint8_t *destination, uint8_t number_of_bytes);

///**
//  @brief Function for reading MMA8x5x with wait for Flag
//  loading registers SYSMODE, OUTX, OUTY, OUTZ, PLSTATUS on setup 
//  @retval true Register read succeeded
//  @retval false Register read failed
//*/
//bool mma8x5x_sample(void);

///**
//  @brief Function for start reading MMA8x5x without wait for flag
//  for loading registers SYSMODE, OUTX, OUTY, OUTZ, PLSTATUS on setup
//  WORK ONLY WITH INTERRUP MODE
//  @retval true start sample succeeded
//  @retval false start sample failed
//*/
//bool mma8x5x_start_sample(void);

///**
//  @brief Function for check mma8x5x read/write access is busy
//  WORK ONLY WITH INTERRUP MODE
//  @retval true : mma8x5x is busy
//  @retval false : mma8x5x is free for new access
//*/
//bool mma8x5x_is_running(void);

///**
//  @brief Function for check mma8x5x SYSMOD register is update
//  WORK ONLY WITH INTERRUP MODE
//  @retval true : mma8x5x SYSMOD register is update
//  @retval false : mma8x5x SYSMOD register is not update
//*/
//bool mma8x5x_sysmod_is_rdy(void);

///**
//  @brief Function for reading and verifying MMA8x5x register.
//  @param[in] register_address : Register address to verify
//  @param[in] value : Data to verify
//  @retval true verify OK
//  @retval false verify NOK
//*/
//bool mma8x5x_verify(uint8_t register_address, uint8_t value);

///**
//  @brief Function for writting data on register, reading back and verifying.
//  @param[in] register_address : Register address to write and verify
//  @param[in] value : Data to verify
//  @retval true verify OK
//  @retval false verify NOK
//*/
//bool mma8x5x_update(uint8_t register_address, uint8_t value);

///**
// * @brief Function for initializing MMA8x5x and verifies it's on the bus.
// * @param[in] p_setup : Pointer of type mma8x5x_setup_Type to client register image.
// * @param[in] p_poweron : pointer to flag power on.
// * @return
// * @retval true MMA8x5x found on the bus, register update and ready for operation.
// * @retval false MMA8x5x not found on the bus or communication failure.
// */
////bool mma8x5x_init(mma8x5x_setup_Type* p_setup, mma8x5x_evt_handler_t evt_handler);
//bool mma8x5x_init(mma8x5x_setup_Type* p_setup, bool* p_poweron, uint8_t ac);

///**
// * @brief Function get sample rate at Active SLEEP mode.
// * @return
// * @retval sample rate value.
// */
//float mma8x5x_Get_SLEEP_Rate(void);

///**
// * @brief Function get sample rate at active WAKE mode.
// * @return
// * @retval sample rate value.
// */
//float mma8x5x_Get_WAKE_Rate(void);


///**
// * @brief Function force to STANDBY mode.
// * @return
// * @retval true => OK.
// */
//bool mma8x5x_Standby(void);

///**
// * @brief Function set ACTIVE mode.
// * @return
// * @retval true => OK.
//*/
//bool mma8x5x_Active(void);

// /**
// * @brief Function read XYZ accel. and orientation status MMA8x5x.
// * @param[in] pdata : Pointer of mma8x5x_data_Type.
// * @return
// * @retval true MMA8x5x read ok.
// * @retval false MMA8x5x read failure.
// **/
//bool mma8x5x_read_data(mma8x5x_data_Type *pdata);

///**
// * @brief Function enable LNDPRT interrupt.
// * @return
// * @retval true => OK.
//*/
//bool mma8x5x_Enable_LP_Int(void);

///**
// * @brief Function disable LNDPRT interrupt.
// * @return
// * @retval true => OK.
//*/
//bool mma8x5x_Disable_LP_Int(void);

///**
// * @brief Function enable DRDY interrupt.
// * @return
// * @retval true => OK.
//*/
//bool mma8x5x_Enable_DR_Int(void);

///**
// * @brief Function disable DRDY interrupt.
// * @return
// * @retval true => OK.
//*/
//bool mma8x5x_Disable_DR_Int(void);

//#endif // _MMA8653_H
