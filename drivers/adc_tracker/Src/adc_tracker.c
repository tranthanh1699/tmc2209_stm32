/**
 ******************************************************************************
 * @file    adc_tracker.c
 * @brief   ADC Tracker driver implementation. See adc_tracker.h / README.md.
 ******************************************************************************
 */
#include "adc_tracker.h"
#include <math.h>
#include <string.h>

/* ============================ Private variables ============================ */
static ADCTRK_HandleTypeDef *s_instances[ADCTRK_MAX_INSTANCES];

/* ============================ Private functions ============================ */
static inline float clampf(float v, float lo, float hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

static inline uint32_t enter_critical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static inline void exit_critical(uint32_t primask)
{
    __set_PRIMASK(primask);
}

static ADCTRK_HandleTypeDef *find_instance(const ADC_HandleTypeDef *hadc)
{
    for (uint32_t i = 0u; i < ADCTRK_MAX_INSTANCES; i++) {
        if ((s_instances[i] != NULL) && (s_instances[i]->hadc == hadc)) {
            return s_instances[i];
        }
    }
    return NULL;
}

static ADCTRK_StatusTypeDef register_instance(ADCTRK_HandleTypeDef *h)
{
    for (uint32_t i = 0u; i < ADCTRK_MAX_INSTANCES; i++) {
        if (s_instances[i] == h) { return ADCTRK_OK; }
    }
    for (uint32_t i = 0u; i < ADCTRK_MAX_INSTANCES; i++) {
        if (s_instances[i] == NULL) { s_instances[i] = h; return ADCTRK_OK; }
    }
    return ADCTRK_ERR_NO_SLOT;
}

/* ADC counts -> angle [deg], clamped to the mechanical range */
static float raw_to_deg(const ADCTRK_ConfigTypeDef *c, float raw)
{
    const float span = c->raw_max - c->raw_min;
    if (fabsf(span) < 1.0f) { return 0.0f; }
    return clampf((raw - c->raw_min) / span * c->full_scale_deg,
                  0.0f, c->full_scale_deg);
}

/* Moves the output setpoint toward the target at <= vel_limit, with optional
 * acceleration limit and braking curve v <= sqrt(2 a |err|). */
static void setpoint_step(ADCTRK_HandleTypeDef *h)
{
    const float dt  = h->dt_s;
    const float a   = h->cfg.out_acc_max_dps2;
    const float err = h->target_deg - h->sp_pos_deg;

    if ((fabsf(err) < 1e-4f) && (fabsf(h->sp_vel_dps) < 1e-3f)) {
        h->sp_pos_deg = h->target_deg;
        h->sp_vel_dps = 0.0f;
        return;
    }

    const float dir = (err > 0.0f) ? 1.0f : -1.0f;
    float v_allow = h->vel_limit_dps;
    if (a > 0.0f) {
        const float v_brake = sqrtf(2.0f * a * fabsf(err));
        if (v_brake < v_allow) { v_allow = v_brake; }
    }

    if (a > 0.0f) {
        h->sp_vel_dps += clampf(dir * v_allow - h->sp_vel_dps, -a * dt, a * dt);
    } else {
        h->sp_vel_dps = dir * v_allow;
    }

    h->sp_pos_deg += h->sp_vel_dps * dt;

    if ((h->target_deg - h->sp_pos_deg) * dir <= 0.0f) {   /* crossed target */
        h->sp_pos_deg = h->target_deg;
        h->sp_vel_dps = 0.0f;
    }
}

/* Averages one DMA half-buffer and runs the processing chain */
static void process_block(ADCTRK_HandleTypeDef *h, const uint16_t *blk)
{
    const uint32_t n = h->cfg.avg_samples;

#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    /* Buffer is written by DMA only -> invalidating whole 32B lines is safe */
    const uint32_t start = (uint32_t)blk & ~31u;
    const uint32_t end   = ((uint32_t)(blk + n) + 31u) & ~31u;
    SCB_InvalidateDCache_by_Addr((void *)start, (int32_t)(end - start));
#endif

    uint32_t sum = 0u;
    for (uint32_t i = 0u; i < n; i++) { sum += blk[i]; }
    ADCTRK_ProcessSample(h, (float)sum / (float)n);
}

/* ============================= Public functions ============================ */
void ADCTRK_GetDefaultConfig(ADCTRK_ConfigTypeDef *cfg)
{
    if (cfg == NULL) { return; }
    memset(cfg, 0, sizeof(*cfg));
    cfg->sample_rate_hz   = 16000u;
    cfg->avg_samples      = 16u;
    cfg->raw_min          = 0.0f;
    cfg->raw_max          = 4095.0f;
    cfg->full_scale_deg   = 270.0f;
    cfg->filter_alpha     = 0.05f;
    cfg->filter_beta      = 0.0f;
    cfg->pos_deadband_deg = 0.2f;
    cfg->vel_deadband_dps = 2.0f;
    cfg->pos_gain         = 1.0f;
    cfg->vel_gain         = 1.0f;
    cfg->out_vel_min_dps  = 5.0f;
    cfg->out_vel_max_dps  = 720.0f;
    cfg->out_acc_max_dps2 = 0.0f;
    cfg->out_pos_min_deg  = 0.0f;
    cfg->out_pos_max_deg  = 0.0f;
}

ADCTRK_StatusTypeDef ADCTRK_Init(ADCTRK_HandleTypeDef *h,
                                 ADC_HandleTypeDef *hadc,
                                 TIM_HandleTypeDef *htim,
                                 uint32_t timer_clk_hz,
                                 const ADCTRK_ConfigTypeDef *cfg)
{
    if ((h == NULL) || (hadc == NULL) || (htim == NULL) || (cfg == NULL)) {
        return ADCTRK_ERR_PARAM;
    }
    if ((timer_clk_hz == 0u) || (cfg->sample_rate_hz == 0u) ||
        (cfg->sample_rate_hz > (timer_clk_hz / 2u)) ||
        (cfg->avg_samples == 0u) || (cfg->avg_samples > ADCTRK_MAX_AVG_SAMPLES) ||
        (cfg->filter_alpha <= 0.0f) || (cfg->filter_alpha >= 1.0f) ||
        (cfg->full_scale_deg <= 0.0f) || (cfg->out_vel_min_dps <= 0.0f) ||
        (cfg->out_vel_max_dps < cfg->out_vel_min_dps)) {
        return ADCTRK_ERR_PARAM;
    }

    /* ADC / DMA are configured by CubeMX: verify the essentials */
    if (hadc->Init.ContinuousConvMode != DISABLE)          { return ADCTRK_ERR_ADC_CONFIG; }
    if (hadc->Init.ExternalTrigConv == ADC_SOFTWARE_START) { return ADCTRK_ERR_ADC_CONFIG; }
#if defined(DMA_CIRCULAR)
    if ((hadc->DMA_Handle == NULL) || (hadc->DMA_Handle->Init.Mode != DMA_CIRCULAR)) {
        return ADCTRK_ERR_DMA_CONFIG;
    }
#endif

    memset(h, 0, sizeof(*h));
    h->hadc         = hadc;
    h->htim         = htim;
    h->cfg          = *cfg;
    h->timer_clk_hz = timer_clk_hz;

    /* Timer period from timer counter clock and requested sample rate */
    const uint32_t period = (timer_clk_hz + (cfg->sample_rate_hz / 2u)) / cfg->sample_rate_hz;
    if (period < 2u) { return ADCTRK_ERR_TIMER; }
    const uint32_t arr = period - 1u;

#if defined(IS_TIM_32B_COUNTER_INSTANCE)
    if (!IS_TIM_32B_COUNTER_INSTANCE(htim->Instance) && (arr > 0xFFFFu)) {
        return ADCTRK_ERR_TIMER;
    }
#else
    if (arr > 0xFFFFu) { return ADCTRK_ERR_TIMER; }
#endif

    h->arr                   = arr;
    h->sample_rate_actual_hz = (float)timer_clk_hz / (float)period;
    h->dt_s                  = (float)cfg->avg_samples / h->sample_rate_actual_hz;

    htim->Init.Period = arr;
    __HAL_TIM_SET_AUTORELOAD(htim, arr);
    __HAL_TIM_SET_COUNTER(htim, 0u);

    TIM_MasterConfigTypeDef master;
    memset(&master, 0, sizeof(master));
    master.MasterOutputTrigger = TIM_TRGO_UPDATE;
    master.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(htim, &master) != HAL_OK) {
        return ADCTRK_ERR_TIMER;
    }

    h->beta_eff = (cfg->filter_beta > 0.0f)
                ? cfg->filter_beta
                : (cfg->filter_alpha * cfg->filter_alpha) / (2.0f - cfg->filter_alpha);

    return register_instance(h);
}

ADCTRK_StatusTypeDef ADCTRK_Start(ADCTRK_HandleTypeDef *h)
{
    if ((h == NULL) || (h->hadc == NULL) || (h->htim == NULL)) { return ADCTRK_ERR_PARAM; }

    h->primed     = 0u;
    h->data_ready = 0u;

    if (HAL_ADC_Start_DMA(h->hadc, (uint32_t *)h->dma_buf,
                          2u * (uint32_t)h->cfg.avg_samples) != HAL_OK) {
        return ADCTRK_ERR_HAL;
    }
    if (HAL_TIM_Base_Start(h->htim) != HAL_OK) {
        (void)HAL_ADC_Stop_DMA(h->hadc);
        return ADCTRK_ERR_HAL;
    }
    h->running = 1u;
    return ADCTRK_OK;
}

ADCTRK_StatusTypeDef ADCTRK_Stop(ADCTRK_HandleTypeDef *h)
{
    if (h == NULL) { return ADCTRK_ERR_PARAM; }
    ADCTRK_StatusTypeDef st = ADCTRK_OK;
    if (HAL_TIM_Base_Stop(h->htim) != HAL_OK) { st = ADCTRK_ERR_HAL; }
    if (HAL_ADC_Stop_DMA(h->hadc) != HAL_OK)  { st = ADCTRK_ERR_HAL; }
    h->running = 0u;
    return st;
}

uint8_t ADCTRK_GetData(ADCTRK_HandleTypeDef *h, ADCTRK_DataTypeDef *out)
{
    const uint32_t pm = enter_critical();
    const uint8_t ready = h->data_ready;
    *out = h->data;
    h->data_ready = 0u;
    exit_critical(pm);
    return ready;
}

void ADCTRK_PeekData(ADCTRK_HandleTypeDef *h, ADCTRK_DataTypeDef *out)
{
    const uint32_t pm = enter_critical();
    *out = h->data;
    exit_critical(pm);
}

void ADCTRK_SetGains(ADCTRK_HandleTypeDef *h, float pos_gain, float vel_gain)
{
    const uint32_t pm = enter_critical();
    h->out_ref_deg  = h->target_deg;   /* re-base: no jump of the target */
    h->in_ref_deg   = h->x_hold;
    h->cfg.pos_gain = pos_gain;
    h->cfg.vel_gain = vel_gain;
    exit_critical(pm);
}

void ADCTRK_SetZero(ADCTRK_HandleTypeDef *h, float out_pos_deg)
{
    const uint32_t pm = enter_critical();
    h->in_ref_deg   = h->x_hold;
    h->out_ref_deg  = out_pos_deg;
    h->target_deg   = out_pos_deg;
    h->sp_pos_deg   = out_pos_deg;
    h->sp_vel_dps   = 0.0f;
    h->vel_peak_dps = 0.0f;
    exit_critical(pm);
}

void ADCTRK_SetCalibration(ADCTRK_HandleTypeDef *h, float raw_min,
                           float raw_max, float full_scale_deg)
{
    const uint32_t pm = enter_critical();
    h->cfg.raw_min        = raw_min;
    h->cfg.raw_max        = raw_max;
    h->cfg.full_scale_deg = full_scale_deg;
    h->out_ref_deg        = h->sp_pos_deg;   /* keep output where it is */
    h->primed             = 0u;              /* re-prime with new scale  */
    exit_critical(pm);
}

float ADCTRK_GetSampleRate(const ADCTRK_HandleTypeDef *h)
{
    return h->sample_rate_actual_hz;
}

float ADCTRK_GetUpdatePeriod(const ADCTRK_HandleTypeDef *h)
{
    return h->dt_s;
}

void ADCTRK_ProcessSample(ADCTRK_HandleTypeDef *h, float raw_avg)
{
    const ADCTRK_ConfigTypeDef *c = &h->cfg;
    const float dt   = h->dt_s;
    const float meas = raw_to_deg(c, raw_avg);

    /* 1. First sample: prime all states, output does not jump */
    if (h->primed == 0u) {
        h->x_est        = meas;
        h->v_est        = 0.0f;
        h->x_hold       = meas;
        h->in_ref_deg   = meas;
        h->target_deg   = h->out_ref_deg;
        h->sp_pos_deg   = h->out_ref_deg;
        h->sp_vel_dps   = 0.0f;
        h->vel_peak_dps = 0.0f;
        h->primed       = 1u;
    } else {
        /* 2. Alpha-beta tracker */
        const float x_pred = h->x_est + h->v_est * dt;
        const float r      = meas - x_pred;
        h->x_est  = x_pred + c->filter_alpha * r;
        h->v_est += (h->beta_eff / dt) * r;
    }

    /* 3. Velocity deadband */
    const float v_in = (fabsf(h->v_est) < c->vel_deadband_dps) ? 0.0f : h->v_est;

    /* 4. Position hysteresis (backlash style, no step jumps) */
    const float db = c->pos_deadband_deg;
    if (h->x_est > (h->x_hold + db))      { h->x_hold = h->x_est - db; }
    else if (h->x_est < (h->x_hold - db)) { h->x_hold = h->x_est + db; }

    /* 5. Position mapping */
    float tgt = h->out_ref_deg + c->pos_gain * (h->x_hold - h->in_ref_deg);
    if (c->out_pos_max_deg > c->out_pos_min_deg) {
        tgt = clampf(tgt, c->out_pos_min_deg, c->out_pos_max_deg);
    }
    h->target_deg = tgt;

    /* 6. Velocity mapping. When the input has stopped but the output has not
     *    reached the target (vel_gain < pos_gain), finish the move at the
     *    peak speed of this move instead of dropping to out_vel_min_dps. */
    float v = fabsf(c->vel_gain * v_in);
    const uint8_t reached = (fabsf(h->target_deg - h->sp_pos_deg) <= 1e-4f) ? 1u : 0u;
    if (reached != 0u)       { h->vel_peak_dps = 0.0f; }
    if (v > h->vel_peak_dps) { h->vel_peak_dps = v; }
    if ((v_in == 0.0f) && (reached == 0u)) { v = h->vel_peak_dps; }
    h->vel_limit_dps = clampf(v, c->out_vel_min_dps, c->out_vel_max_dps);

    /* 7. Setpoint generator */
    setpoint_step(h);

    /* 8. Publish */
    if (h->data_ready != 0u) { h->data_missed++; }
    h->data.raw_avg              = raw_avg;
    h->data.in_angle_deg         = h->x_est;
    h->data.in_velocity_dps      = h->v_est;
    h->data.out_target_deg       = h->target_deg;
    h->data.out_vel_limit_dps    = h->vel_limit_dps;
    h->data.out_setpoint_deg     = h->sp_pos_deg;
    h->data.out_setpoint_vel_dps = h->sp_vel_dps;
    h->data.seq++;
    h->data_ready = 1u;
}

void ADCTRK_ConvHalfCpltHandler(ADC_HandleTypeDef *hadc)
{
    ADCTRK_HandleTypeDef *h = find_instance(hadc);
    if (h != NULL) { process_block(h, &h->dma_buf[0]); }
}

void ADCTRK_ConvCpltHandler(ADC_HandleTypeDef *hadc)
{
    ADCTRK_HandleTypeDef *h = find_instance(hadc);
    if (h != NULL) { process_block(h, &h->dma_buf[h->cfg.avg_samples]); }
}

#if (ADCTRK_USE_HAL_CALLBACKS == 1)
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    ADCTRK_ConvHalfCpltHandler(hadc);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    ADCTRK_ConvCpltHandler(hadc);
}
#endif