/**
 * @file    tmc2209_motion.c
 * @brief   Trapezoidal-ramp STEP/DIR pulse generator (fixed-tick DDS).
 */
#include "tmc2209_motion.h"
#include <string.h>

#define TWO_POW_64          18446744073709551616.0
#define Q64_MAX_DOUBLE      18446744073709549568.0   /* largest double < 2^64 */
#define Q63_DOUBLE          9223372036854775808.0
#define STOP_TOL_STEPS      2U   /* ramp quantization error when reaching target */

/* ========================================================================== */
/*  Unit conversion (thread context, uses double)                             */
/* ========================================================================== */

static uint64_t speed_to_q(const tmc_motion_t *m, float speed)
{
    double s = (double)((speed < 0.0f) ? -speed : speed);
    double q;
    if (s > (double)m->speed_limit) {
        s = (double)m->speed_limit;
    }
    q = s / (double)m->cfg.tick_hz * TWO_POW_64;
    if (q > Q64_MAX_DOUBLE) {
        q = Q64_MAX_DOUBLE;
    }
    return (uint64_t)q;
}

static uint64_t accel_to_q(const tmc_motion_t *m, float accel)
{
    double f = (double)m->cfg.tick_hz;
    double q;
    if (accel <= 0.0f) {
        return 0;
    }
    q = (double)accel / (f * f) * TWO_POW_64;
    if (q > Q63_DOUBLE) {
        q = Q63_DOUBLE;
    }
    if (q < 1.0) {
        q = 1.0;
    }
    return (uint64_t)q;
}

static double q_to_speed(const tmc_motion_t *m, uint64_t q)
{
    return (double)q / TWO_POW_64 * (double)m->cfg.tick_hz;
}

/** Number of steps needed to brake from the current velocity to start_speed: s = (v^2 - v0^2) / 2a */
static uint32_t ramp_steps_for(const tmc_motion_t *m, uint64_t vel_q, float accel)
{
    double v, v0, s;
    if (accel <= 0.0f) {
        return 0;
    }
    v  = q_to_speed(m, vel_q);
    v0 = (double)m->start_speed;
    if (v <= v0) {
        return 0;
    }
    s = (v * v - v0 * v0) / (2.0 * (double)accel);
    return (s > 4.0e9) ? 4000000000UL : (uint32_t)(s + 0.5);
}

/* ========================================================================== */
/*  Helpers used in the ISR                                                    */
/* ========================================================================== */

static void set_dir(tmc_motion_t *m, int8_t d)
{
    bool level = (d > 0) != m->cfg.dir_invert;
    m->dir = d;
    if (level) {
        TMC_PIN_HIGH(m->cfg.dir_port, m->cfg.dir_pin);
    } else {
        TMC_PIN_LOW(m->cfg.dir_port, m->cfg.dir_pin);
    }
}

static void stop_now(tmc_motion_t *m)
{
    m->vel_q      = 0;
    m->ramp_steps = 0;
    m->running    = false;
    m->mode       = TMC_MOTION_IDLE;
    m->ramp_state = TMC_RAMP_STOPPED;
    m->target     = m->position;
}

static uint64_t floor_q(const tmc_motion_t *m)
{
    if ((m->vmax_q == 0U) || (m->vmin_q < m->vmax_q)) {
        return m->vmin_q;
    }
    return m->vmax_q;
}

/* Called inside a critical section */
static void start_if_idle(tmc_motion_t *m)
{
    if (m->running) {
        return;
    }
    m->vel_q      = m->no_ramp ? m->vmax_q : floor_q(m);
    m->phase      = UINT32_MAX;         /* first step is emitted on the very next tick */
    m->ramp_steps = 0;
    m->ramp_state = TMC_RAMP_ACCEL;
    m->running    = true;
}

/* ========================================================================== */
/*  ISR                                                                        */
/* ========================================================================== */

void tmc_motion_tick(tmc_motion_t *m)
{
    int8_t   want;
    int64_t  dist = 0;
    int      action = 0;           /* +1 accelerate, -1 decelerate, 0 hold speed */
    uint64_t lo = 0;
    uint32_t inc, old;

    /* End the STEP pulse from the previous tick (pulse width = 1 tick period) */
    if (m->step_high) {
        TMC_PIN_LOW(m->cfg.step_port, m->cfg.step_pin);
        m->step_high = false;
    }
    if (!m->running) {
        return;
    }

    if (m->mode == TMC_MOTION_POSITION) {
        dist = (int64_t)m->target - (int64_t)m->position;
        want = (dist > 0) ? 1 : -1;
    } else {
        want = m->vel_dir;
    }

    /* ---- At the lowest speed: allowed to stop or reverse direction ---- */
    if (m->no_ramp || (m->vel_q <= floor_q(m))) {
        bool done = (m->mode == TMC_MOTION_POSITION) ? (dist == 0) : (m->vmax_q == 0U);
        if (done || (m->mode == TMC_MOTION_IDLE)) {
            stop_now(m);
            return;
        }
        if (want != m->dir) {
            set_dir(m, want);            /* skip a step on this tick => DIR setup time */
            m->ramp_steps = 0;
            m->phase      = UINT32_MAX;
            m->vel_q      = m->no_ramp ? m->vmax_q : floor_q(m);
            m->ramp_state = TMC_RAMP_ACCEL;
            return;
        }
    }

    /* ---- Decide whether to accelerate/decelerate ---- */
    if (m->no_ramp) {
        m->vel_q = m->vmax_q;
    } else if (m->mode == TMC_MOTION_POSITION) {
        int64_t remaining = dist * m->dir;
        if ((remaining <= 0) || (remaining <= (int64_t)m->ramp_steps)) {
            action = -1; lo = floor_q(m);                 /* brake toward the target   */
        } else if (m->vel_q < m->vmax_q) {
            action = 1;
        } else if (m->vel_q > m->vmax_q) {
            action = -1; lo = m->vmax_q;                  /* vmax was lowered mid-move */
        }
    } else {
        if (want != m->dir) {
            action = -1; lo = floor_q(m);                 /* brake to reverse direction */
        } else if (m->vel_q < m->vmax_q) {
            action = 1;
        } else if (m->vel_q > m->vmax_q) {
            action = -1; lo = (m->vmax_q > floor_q(m)) ? m->vmax_q : floor_q(m);
        }
    }

    if (action > 0) {
        uint64_t hi = m->vmax_q;
        m->vel_q = ((hi - m->vel_q) <= m->acc_q) ? hi : (m->vel_q + m->acc_q);
        m->ramp_state = TMC_RAMP_ACCEL;
    } else if (action < 0) {
        m->vel_q = ((m->vel_q <= lo) || ((m->vel_q - lo) <= m->acc_q)) ? lo : (m->vel_q - m->acc_q);
        m->ramp_state = TMC_RAMP_DECEL;
    } else {
        m->ramp_state = TMC_RAMP_CRUISE;
    }

    /* ---- DDS: phase accumulator overflow => 1 step ---- */
    inc = (uint32_t)(m->vel_q >> 32);
    old = m->phase;
    m->phase = old + inc;
    if (m->phase < old) {
        if (m->cfg.double_edge) {
            TMC_PIN_TOGGLE(m->cfg.step_port, m->cfg.step_pin);
        } else {
            TMC_PIN_HIGH(m->cfg.step_port, m->cfg.step_pin);
            m->step_high = true;
        }
        m->position += m->dir;

        if (action > 0) {
            m->ramp_steps++;
        } else if ((action < 0) && (m->ramp_steps > 0U)) {
            m->ramp_steps--;
        }

        if ((m->mode == TMC_MOTION_POSITION) && (m->position == m->target) &&
            (m->no_ramp || (m->ramp_steps <= STOP_TOL_STEPS))) {
            stop_now(m);
        }
    }
}

/* ========================================================================== */
/*  API                                                                        */
/* ========================================================================== */

tmc_status_t tmc_motion_init(tmc_motion_t *m, const tmc_motion_cfg_t *cfg, tmc2209_t *drv)
{
    if ((m == NULL) || (cfg == NULL) || (cfg->tick_hz == 0U) ||
        (cfg->step_port == NULL) || (cfg->dir_port == NULL)) {
        return TMC_ERR_PARAM;
    }
    memset(m, 0, sizeof(*m));
    m->cfg = *cfg;
    m->drv = drv;
    m->speed_limit = cfg->double_edge ? (float)cfg->tick_hz * 0.999f : (float)cfg->tick_hz * 0.5f;

    tmc_motion_set_start_speed(m, 100.0f);
    tmc_motion_set_max_speed(m, 1000.0f);
    tmc_motion_set_acceleration(m, 2000.0f);

    TMC_PIN_LOW(cfg->step_port, cfg->step_pin);
    set_dir(m, 1);
    m->ramp_state = TMC_RAMP_STOPPED;

    if (drv != NULL) {
        tmc_status_t st = tmc2209_move_using_step_dir_interface(drv);   /* VACTUAL = 0 */
        if (st != TMC_OK) {
            return st;
        }
        return cfg->double_edge ? tmc2209_enable_double_edge(drv)
                                : tmc2209_disable_double_edge(drv);
    }
    return TMC_OK;
}

void tmc_motion_set_max_speed(tmc_motion_t *m, float usteps_per_s)
{
    m->max_speed = (usteps_per_s < 0.0f) ? -usteps_per_s : usteps_per_s;
}

void tmc_motion_set_acceleration(tmc_motion_t *m, float usteps_per_s2)
{
    m->accel = (usteps_per_s2 > 0.0f) ? usteps_per_s2 : 0.0f;
}

void tmc_motion_set_start_speed(tmc_motion_t *m, float usteps_per_s)
{
    uint64_t q;
    if (usteps_per_s < 1.0f) {
        usteps_per_s = 1.0f;
    }
    m->start_speed = usteps_per_s;
    q = speed_to_q(m, usteps_per_s);
    {
        TMC_CRITICAL_ENTER();
        m->vmin_q = q;
        TMC_CRITICAL_EXIT();
    }
}

static uint64_t snapshot_vel(const tmc_motion_t *m, bool *running)
{
    uint64_t v;
    TMC_CRITICAL_ENTER();
    v = m->vel_q;
    *running = m->running;
    TMC_CRITICAL_EXIT();
    return v;
}

void tmc_motion_move_to_ex(tmc_motion_t *m, int32_t target, float max_speed, float accel)
{
    uint64_t vq = speed_to_q(m, max_speed);
    uint64_t aq = accel_to_q(m, accel);
    bool     running;
    uint64_t vel = snapshot_vel(m, &running);
    uint32_t rs  = ramp_steps_for(m, vel, accel);

    if (vq == 0U) {
        vq = 1ULL << 32;          /* minimum ~F/2^32 µstep/s, to avoid getting stuck */
    }
    {
        TMC_CRITICAL_ENTER();
        m->target  = target;
        m->vmax_q  = vq;
        m->acc_q   = aq;
        m->no_ramp = (aq == 0U);
        m->mode    = TMC_MOTION_POSITION;
        if (m->running) {
            m->ramp_steps = rs;
        } else if (target != m->position) {
            start_if_idle(m);
        } else {
            m->mode = TMC_MOTION_IDLE;
        }
        TMC_CRITICAL_EXIT();
    }
    (void)running;
}

void tmc_motion_move_ex(tmc_motion_t *m, int32_t delta, float max_speed, float accel)
{
    int32_t base;
    {
        TMC_CRITICAL_ENTER();
        base = m->running && (m->mode == TMC_MOTION_POSITION) ? m->target : m->position;
        TMC_CRITICAL_EXIT();
    }
    tmc_motion_move_to_ex(m, base + delta, max_speed, accel);
}

void tmc_motion_move_to(tmc_motion_t *m, int32_t target)
{
    tmc_motion_move_to_ex(m, target, m->max_speed, m->accel);
}

void tmc_motion_move(tmc_motion_t *m, int32_t delta)
{
    tmc_motion_move_ex(m, delta, m->max_speed, m->accel);
}

void tmc_motion_run_velocity(tmc_motion_t *m, float usteps_per_s, float accel)
{
    uint64_t vq = speed_to_q(m, usteps_per_s);
    uint64_t aq = accel_to_q(m, accel);
    bool     running;
    uint64_t vel = snapshot_vel(m, &running);
    uint32_t rs  = ramp_steps_for(m, vel, accel);
    (void)running;

    {
        TMC_CRITICAL_ENTER();
        m->vmax_q  = vq;
        m->acc_q   = aq;
        m->no_ramp = (aq == 0U);
        if (usteps_per_s > 0.0f) {
            m->vel_dir = 1;
        } else if (usteps_per_s < 0.0f) {
            m->vel_dir = -1;
        } else {
            m->vel_dir = m->dir;         /* v = 0: brake in the current direction */
        }
        if (m->running) {
            m->mode       = TMC_MOTION_VELOCITY;
            m->ramp_steps = rs;
        } else if (vq != 0U) {
            m->mode = TMC_MOTION_VELOCITY;
            start_if_idle(m);
        }
        TMC_CRITICAL_EXIT();
    }
}

void tmc_motion_stop(tmc_motion_t *m)
{
    if (m->accel <= 0.0f) {
        tmc_motion_emergency_stop(m);
        return;
    }
    tmc_motion_run_velocity(m, 0.0f, m->accel);
}

void tmc_motion_emergency_stop(tmc_motion_t *m)
{
    TMC_CRITICAL_ENTER();
    stop_now(m);
    TMC_CRITICAL_EXIT();
}

bool tmc_motion_is_running(const tmc_motion_t *m)
{
    return m->running;
}

int32_t tmc_motion_get_position(const tmc_motion_t *m)
{
    return m->position;   /* int32 read is atomic on Cortex-M */
}

int32_t tmc_motion_distance_to_go(const tmc_motion_t *m)
{
    int32_t d;
    TMC_CRITICAL_ENTER();
    d = (m->mode == TMC_MOTION_POSITION) ? (m->target - m->position) : 0;
    TMC_CRITICAL_EXIT();
    return d;
}

float tmc_motion_get_speed(const tmc_motion_t *m)
{
    uint64_t v;
    int8_t   d;
    {
        TMC_CRITICAL_ENTER();
        v = m->vel_q;
        d = m->dir;
        TMC_CRITICAL_EXIT();
    }
    return (float)(q_to_speed(m, v) * (double)d);
}

bool tmc_motion_set_position(tmc_motion_t *m, int32_t position)
{
    bool ok = false;
    TMC_CRITICAL_ENTER();
    if (!m->running) {
        m->position = position;
        m->target   = position;
        ok = true;
    }
    TMC_CRITICAL_EXIT();
    return ok;
}
