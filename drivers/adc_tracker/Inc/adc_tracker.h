/**
 ******************************************************************************
 * @file    adc_tracker.h
 * @brief   ADC Tracker driver.
 *          Timer-triggered ADC + circular DMA acquisition of one analog
 *          channel, converted to angle [deg] and angular velocity [deg/s],
 *          then mapped to an output axis with independent position and
 *          velocity gains.
 *
 *          See README.md for configuration and examples.
 ******************************************************************************
 */
#ifndef ADC_TRACKER_H
#define ADC_TRACKER_H

#include "main.h"      /* HAL + CMSIS device header (CubeMX project) */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================== Build options ============================== */
/** Maximum number of samples averaged per update (DMA half-buffer size). */
#ifndef ADCTRK_MAX_AVG_SAMPLES
#define ADCTRK_MAX_AVG_SAMPLES      32u
#endif

/** Maximum number of driver instances (one ADC handle per instance). */
#ifndef ADCTRK_MAX_INSTANCES
#define ADCTRK_MAX_INSTANCES        2u
#endif

/** 1: driver defines HAL_ADC_ConvHalfCpltCallback / HAL_ADC_ConvCpltCallback.
 *  0: application defines them and calls ADCTRK_ConvHalfCpltHandler /
 *     ADCTRK_ConvCpltHandler. */
#ifndef ADCTRK_USE_HAL_CALLBACKS
#define ADCTRK_USE_HAL_CALLBACKS    1
#endif

/* ================================== Types ================================== */
typedef enum
{
    ADCTRK_OK = 0,
    ADCTRK_ERR_PARAM,          /*!< NULL pointer or invalid configuration   */
    ADCTRK_ERR_TIMER,          /*!< ARR out of range / TRGO config failed   */
    ADCTRK_ERR_ADC_CONFIG,     /*!< ADC in continuous or software trigger   */
    ADCTRK_ERR_DMA_CONFIG,     /*!< DMA not configured in circular mode     */
    ADCTRK_ERR_NO_SLOT,        /*!< ADCTRK_MAX_INSTANCES exceeded           */
    ADCTRK_ERR_HAL             /*!< HAL start/stop call failed              */
} ADCTRK_StatusTypeDef;

typedef struct
{
    /* --- Acquisition --- */
    uint32_t sample_rate_hz;     /*!< ADC trigger rate                  [Hz]   */
    uint16_t avg_samples;        /*!< Samples averaged per update             */

    /* --- Input calibration --- */
    float raw_min;               /*!< ADC counts at 0 deg                      */
    float raw_max;               /*!< ADC counts at full scale (may be < min) */
    float full_scale_deg;        /*!< Mechanical range of the sensor   [deg]  */

    /* --- Filter (alpha-beta tracker) --- */
    float filter_alpha;          /*!< 0 < alpha < 1                            */
    float filter_beta;           /*!< <= 0: auto = alpha^2 / (2 - alpha)       */
    float pos_deadband_deg;      /*!< Position hysteresis              [deg]   */
    float vel_deadband_dps;      /*!< |velocity| below this is zero    [deg/s] */

    /* --- Output mapping --- */
    float pos_gain;              /*!< out_angle = pos_gain * in_angle          */
    float vel_gain;              /*!< out_speed = vel_gain * in_speed          */
    float out_vel_min_dps;       /*!< Minimum output speed, must be > 0        */
    float out_vel_max_dps;       /*!< Maximum output speed                     */
    float out_acc_max_dps2;      /*!< Max acceleration, <= 0: disabled         */
    float out_pos_min_deg;       /*!< Soft limits, min >= max: disabled        */
    float out_pos_max_deg;
} ADCTRK_ConfigTypeDef;

typedef struct
{
    float    raw_avg;              /*!< Averaged ADC counts                    */
    float    in_angle_deg;         /*!< Filtered input angle          [deg]    */
    float    in_velocity_dps;      /*!< Filtered input velocity       [deg/s]  */
    float    out_target_deg;       /*!< Final output target           [deg]    */
    float    out_vel_limit_dps;    /*!< Allowed output speed          [deg/s]  */
    float    out_setpoint_deg;     /*!< Output position setpoint      [deg]    */
    float    out_setpoint_vel_dps; /*!< Output velocity setpoint      [deg/s]  */
    uint32_t seq;                  /*!< Update counter                         */
} ADCTRK_DataTypeDef;

typedef struct
{
    /* DMA buffer first: 32-byte aligned for D-cache maintenance (F7/H7) */
    uint16_t dma_buf[2u * ADCTRK_MAX_AVG_SAMPLES] __attribute__((aligned(32)));

    ADC_HandleTypeDef   *hadc;
    TIM_HandleTypeDef   *htim;
    ADCTRK_ConfigTypeDef cfg;

    /* Timing */
    uint32_t timer_clk_hz;
    uint32_t arr;
    float    sample_rate_actual_hz;
    float    dt_s;
    float    beta_eff;

    /* Processing state (private) */
    float    x_est, v_est, x_hold;
    float    in_ref_deg, out_ref_deg;
    float    target_deg, vel_limit_dps, sp_pos_deg, sp_vel_dps, vel_peak_dps;
    uint8_t  primed;

    /* Published data (written in DMA ISR) */
    ADCTRK_DataTypeDef data;
    volatile uint8_t   data_ready;
    volatile uint32_t  data_missed;    /*!< Updates not read by application */
    uint8_t            running;
} ADCTRK_HandleTypeDef;

/* =================================== API =================================== */
void                 ADCTRK_GetDefaultConfig(ADCTRK_ConfigTypeDef *cfg);

ADCTRK_StatusTypeDef ADCTRK_Init(ADCTRK_HandleTypeDef *h,
                                 ADC_HandleTypeDef *hadc,
                                 TIM_HandleTypeDef *htim,
                                 uint32_t timer_clk_hz,
                                 const ADCTRK_ConfigTypeDef *cfg);
ADCTRK_StatusTypeDef ADCTRK_Start(ADCTRK_HandleTypeDef *h);
ADCTRK_StatusTypeDef ADCTRK_Stop(ADCTRK_HandleTypeDef *h);

uint8_t              ADCTRK_GetData(ADCTRK_HandleTypeDef *h, ADCTRK_DataTypeDef *out);
void                 ADCTRK_PeekData(ADCTRK_HandleTypeDef *h, ADCTRK_DataTypeDef *out);

void                 ADCTRK_SetGains(ADCTRK_HandleTypeDef *h, float pos_gain, float vel_gain);
void                 ADCTRK_SetZero(ADCTRK_HandleTypeDef *h, float out_pos_deg);
void                 ADCTRK_SetCalibration(ADCTRK_HandleTypeDef *h, float raw_min,
                                           float raw_max, float full_scale_deg);

float                ADCTRK_GetSampleRate(const ADCTRK_HandleTypeDef *h);
float                ADCTRK_GetUpdatePeriod(const ADCTRK_HandleTypeDef *h);

void                 ADCTRK_ProcessSample(ADCTRK_HandleTypeDef *h, float raw_avg);

void                 ADCTRK_ConvHalfCpltHandler(ADC_HandleTypeDef *hadc);
void                 ADCTRK_ConvCpltHandler(ADC_HandleTypeDef *hadc);

#ifdef __cplusplus
}
#endif
#endif /* ADC_TRACKER_H */