/**
 * @file    tmc2209_reg.h
 * @brief   Register map TMC2209 (đối chiếu datasheet + janelia-arduino/TMC2209).
 *          Dùng macro mask/shift thay cho bitfield để không phụ thuộc compiler
 *          về thứ tự bit.
 */
#ifndef TMC2209_REG_H
#define TMC2209_REG_H

#include <stdint.h>

/* ---- Frame --------------------------------------------------------------- */
#define TMC_SYNC                   0x05U  /* 0b0101 + 4 bit reserved           */
#define TMC_REPLY_ADDR             0xFFU  /* địa chỉ master trong frame reply   */
#define TMC_RW_WRITE               0x80U
#define TMC_WRITE_FRAME_LEN        8U
#define TMC_READ_REQ_LEN           4U
#define TMC_READ_REPLY_LEN         8U
#define TMC_MAX_ADDR               3U     /* 0..3 chọn bằng chân MS1/MS2       */

/* ---- Địa chỉ register ------------------------------------------------------ */
#define TMC_REG_GCONF              0x00U  /* RW  */
#define TMC_REG_GSTAT              0x01U  /* RC  */
#define TMC_REG_IFCNT              0x02U  /* R   */
#define TMC_REG_SLAVECONF          0x03U  /* W   (SENDDELAY)                   */
#define TMC_REG_OTP_PROG           0x04U  /* W   */
#define TMC_REG_OTP_READ           0x05U  /* R   */
#define TMC_REG_IOIN               0x06U  /* R   */
#define TMC_REG_FACTORY_CONF       0x07U  /* RW  */
#define TMC_REG_IHOLD_IRUN         0x10U  /* W   */
#define TMC_REG_TPOWERDOWN         0x11U  /* W   */
#define TMC_REG_TSTEP              0x12U  /* R   */
#define TMC_REG_TPWMTHRS           0x13U  /* W   */
#define TMC_REG_TCOOLTHRS          0x14U  /* W   */
#define TMC_REG_VACTUAL            0x22U  /* W   */
#define TMC_REG_SGTHRS             0x40U  /* W   */
#define TMC_REG_SG_RESULT          0x41U  /* R   */
#define TMC_REG_COOLCONF           0x42U  /* W   */
#define TMC_REG_MSCNT              0x6AU  /* R   */
#define TMC_REG_MSCURACT           0x6BU  /* R   */
#define TMC_REG_CHOPCONF           0x6CU  /* RW  */
#define TMC_REG_DRV_STATUS         0x6FU  /* R   */
#define TMC_REG_PWMCONF            0x70U  /* RW  */
#define TMC_REG_PWM_SCALE          0x71U  /* R   */
#define TMC_REG_PWM_AUTO           0x72U  /* R   */

/* ---- Helper field ---------------------------------------------------------- */
#define TMC_FIELD_GET(reg, msk, pos)     (((reg) & (msk)) >> (pos))
#define TMC_FIELD_SET(reg, msk, pos, v)  ((reg) = ((reg) & ~(msk)) | ((((uint32_t)(v)) << (pos)) & (msk)))
#define TMC_BIT_WRITE(reg, msk, on)      ((reg) = (on) ? ((reg) | (msk)) : ((reg) & ~(msk)))

/* ---- GCONF (0x00) ---------------------------------------------------------- */
#define GCONF_I_SCALE_ANALOG_Msk   ((uint32_t)1U << 0)
#define GCONF_INTERNAL_RSENSE_Msk  ((uint32_t)1U << 1)
#define GCONF_EN_SPREADCYCLE_Msk   ((uint32_t)1U << 2)
#define GCONF_SHAFT_Msk            ((uint32_t)1U << 3)
#define GCONF_INDEX_OTPW_Msk       ((uint32_t)1U << 4)
#define GCONF_INDEX_STEP_Msk       ((uint32_t)1U << 5)
#define GCONF_PDN_DISABLE_Msk      ((uint32_t)1U << 6)
#define GCONF_MSTEP_REG_SELECT_Msk ((uint32_t)1U << 7)
#define GCONF_MULTISTEP_FILT_Msk   ((uint32_t)1U << 8)

/* ---- GSTAT (0x01) ---------------------------------------------------------- */
#define GSTAT_RESET_Msk            ((uint32_t)1U << 0)
#define GSTAT_DRV_ERR_Msk          ((uint32_t)1U << 1)
#define GSTAT_UV_CP_Msk            ((uint32_t)1U << 2)

/* ---- SLAVECONF (0x03) ------------------------------------------------------ */
#define SLAVECONF_SENDDELAY_Pos    8U
#define SLAVECONF_SENDDELAY_Msk    ((uint32_t)0xFU << 8)

/* ---- IOIN (0x06) ----------------------------------------------------------- */
#define IOIN_ENN_Msk               ((uint32_t)1U << 0)
#define IOIN_MS1_Msk               ((uint32_t)1U << 2)
#define IOIN_MS2_Msk               ((uint32_t)1U << 3)
#define IOIN_DIAG_Msk              ((uint32_t)1U << 4)
#define IOIN_PDN_UART_Msk          ((uint32_t)1U << 6)
#define IOIN_STEP_Msk              ((uint32_t)1U << 7)
#define IOIN_SPREAD_EN_Msk         ((uint32_t)1U << 8)
#define IOIN_DIR_Msk               ((uint32_t)1U << 9)
#define IOIN_VERSION_Pos           24U
#define IOIN_VERSION_Msk           ((uint32_t)0xFFU << 24)
#define TMC2209_VERSION            0x21U

/* ---- IHOLD_IRUN (0x10) ----------------------------------------------------- */
#define IHOLD_Pos                  0U
#define IHOLD_Msk                  ((uint32_t)0x1FU << 0)
#define IRUN_Pos                   8U
#define IRUN_Msk                   ((uint32_t)0x1FU << 8)
#define IHOLDDELAY_Pos             16U
#define IHOLDDELAY_Msk             ((uint32_t)0x0FU << 16)

/* ---- COOLCONF (0x42) ------------------------------------------------------- */
#define COOL_SEMIN_Pos             0U
#define COOL_SEMIN_Msk             ((uint32_t)0xFU << 0)
#define COOL_SEUP_Pos              5U
#define COOL_SEUP_Msk              ((uint32_t)0x3U << 5)
#define COOL_SEMAX_Pos             8U
#define COOL_SEMAX_Msk             ((uint32_t)0xFU << 8)
#define COOL_SEDN_Pos              13U
#define COOL_SEDN_Msk              ((uint32_t)0x3U << 13)
#define COOL_SEIMIN_Msk            ((uint32_t)1U << 15)

/* ---- CHOPCONF (0x6C) ------------------------------------------------------- */
#define CHOP_TOFF_Pos              0U
#define CHOP_TOFF_Msk              ((uint32_t)0xFU << 0)
#define CHOP_HSTRT_Pos             4U
#define CHOP_HSTRT_Msk             ((uint32_t)0x7U << 4)
#define CHOP_HEND_Pos              7U
#define CHOP_HEND_Msk              ((uint32_t)0xFU << 7)
#define CHOP_TBL_Pos               15U
#define CHOP_TBL_Msk               ((uint32_t)0x3U << 15)
#define CHOP_VSENSE_Msk            ((uint32_t)1U << 17)
#define CHOP_MRES_Pos              24U
#define CHOP_MRES_Msk              ((uint32_t)0xFU << 24)
#define CHOP_INTPOL_Msk            ((uint32_t)1U << 28)
#define CHOP_DEDGE_Msk             ((uint32_t)1U << 29)
#define CHOP_DISS2G_Msk            ((uint32_t)1U << 30)
#define CHOP_DISS2VS_Msk           ((uint32_t)1U << 31)

/* ---- DRV_STATUS (0x6F) ----------------------------------------------------- */
#define DRV_OTPW_Msk               ((uint32_t)1U << 0)
#define DRV_OT_Msk                 ((uint32_t)1U << 1)
#define DRV_S2GA_Msk               ((uint32_t)1U << 2)
#define DRV_S2GB_Msk               ((uint32_t)1U << 3)
#define DRV_S2VSA_Msk              ((uint32_t)1U << 4)
#define DRV_S2VSB_Msk              ((uint32_t)1U << 5)
#define DRV_OLA_Msk                ((uint32_t)1U << 6)
#define DRV_OLB_Msk                ((uint32_t)1U << 7)
#define DRV_T120_Msk               ((uint32_t)1U << 8)
#define DRV_T143_Msk               ((uint32_t)1U << 9)
#define DRV_T150_Msk               ((uint32_t)1U << 10)
#define DRV_T157_Msk               ((uint32_t)1U << 11)
#define DRV_CS_ACTUAL_Pos          16U
#define DRV_CS_ACTUAL_Msk          ((uint32_t)0x1FU << 16)
#define DRV_STEALTH_Msk            ((uint32_t)1U << 30)
#define DRV_STST_Msk               ((uint32_t)1U << 31)

/* ---- PWMCONF (0x70) -------------------------------------------------------- */
#define PWM_OFS_Pos                0U
#define PWM_OFS_Msk                ((uint32_t)0xFFU << 0)
#define PWM_GRAD_Pos               8U
#define PWM_GRAD_Msk               ((uint32_t)0xFFU << 8)
#define PWM_FREQ_Pos               16U
#define PWM_FREQ_Msk               ((uint32_t)0x3U << 16)
#define PWM_AUTOSCALE_Msk          ((uint32_t)1U << 18)
#define PWM_AUTOGRAD_Msk           ((uint32_t)1U << 19)
#define PWM_FREEWHEEL_Pos          20U
#define PWM_FREEWHEEL_Msk          ((uint32_t)0x3U << 20)
#define PWM_REG_Pos                24U
#define PWM_REG_Msk                ((uint32_t)0xFU << 24)
#define PWM_LIM_Pos                28U
#define PWM_LIM_Msk                ((uint32_t)0xFU << 28)

/* ---- PWM_SCALE (0x71) / PWM_AUTO (0x72) ------------------------------------ */
#define PWM_SCALE_SUM_Msk          ((uint32_t)0xFFU << 0)
#define PWM_SCALE_AUTO_Pos         16U
#define PWM_SCALE_AUTO_Msk         ((uint32_t)0x1FFU << 16)   /* 9 bit có dấu */
#define PWM_OFS_AUTO_Msk           ((uint32_t)0xFFU << 0)
#define PWM_GRAD_AUTO_Pos          16U
#define PWM_GRAD_AUTO_Msk          ((uint32_t)0xFFU << 16)

/* ---- Mặc định (giống thư viện gốc) ----------------------------------------- */
#define TMC_CHOPCONF_DEFAULT       0x10000053U
#define TMC_PWMCONF_DEFAULT        0xC10D0024U
#define TMC_IHOLD_DEFAULT          16U
#define TMC_IRUN_DEFAULT           31U
#define TMC_IHOLDDELAY_DEFAULT     1U
#define TMC_TPOWERDOWN_DEFAULT     20U
#define TMC_TOFF_DEFAULT           3U
#define TMC_TBL_DEFAULT            2U
#define TMC_HEND_DEFAULT           0U
#define TMC_HSTRT_DEFAULT          5U
#define TMC_SEIMIN_IRUN_LIMIT      20U

/* VACTUAL: v[µstep/s] = VACTUAL * fCLK / 2^24; fCLK nội 12 MHz -> ~0.715 /LSB */
#define TMC_VACTUAL_HZ_PER_LSB     (12000000.0f / 16777216.0f)
#define TMC_VACTUAL_MAX            ((int32_t)((1L << 23) - 1))

#endif /* TMC2209_REG_H */
