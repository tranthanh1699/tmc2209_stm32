/**
 * @file    example_position_velocity.c
 * @brief   Cookbook: position + velocity control with tmc2209_motion.
 *
 * Prerequisites (see example_main.c):
 *   - tmc_bus_init() / tmc2209_init() / tmc2209_enable() done for each driver
 *   - tmc_motion_init() done for each axis
 *   - tmc_motion_tick() called for every axis from a fixed-rate timer ISR (e.g. 20 kHz)
 *
 * Units used by the library: µsteps, µsteps/s, µsteps/s².
 * The axis_t wrapper below lets you work in mm, mm/s, mm/s² instead.
 *
 * Contents
 *   Ex 1  Point-to-point moves, each with its own speed/acceleration (blocking)
 *   Ex 2  Relative moves (jog)
 *   Ex 3  Working in millimetres with soft limits
 *   Ex 4  Non-blocking waypoint sequence with dwell times
 *   Ex 5  Changing speed in the middle of a move (speed zones)
 *   Ex 6  Following a moving set-point (e.g. potentiometer) with speed limit
 *   Ex 7  Two-axis synchronised linear move (both axes arrive together)
 *   Ex 8  Homing with a limit switch (fast seek, back off, slow seek)
 *   Ex 9  UART + STEP together: per-move current and StealthChop/SpreadCycle
 *   Ex 10 Sensorless homing with StallGuard (UART + DIAG + STEP)
 *   Ex 11 Velocity jog that stops exactly at a soft limit
 */

#include <math.h>
#include <stdlib.h>
#include "tmc2209.h"
#include "tmc2209_motion.h"

/* Timer IRQ used for tmc_motion_tick(); needed by Ex 7 to start axes together */
#ifndef MOTION_TIMER_IRQn
#define MOTION_TIMER_IRQn   TIM6_DAC_IRQn
#endif

/* ========================================================================== */
/*  Common helpers                                                            */
/* ========================================================================== */

/** Axis wrapper: motion + optional driver + mechanical scaling + soft limits. */
typedef struct {
    tmc_motion_t *m;
    tmc2209_t    *drv;             /* NULL if the axis has no UART           */
    float         usteps_per_mm;   /* e.g. 200 * 16 / 8 mm lead = 400        */
    float         min_mm;          /* soft limits                            */
    float         max_mm;
} axis_t;

static float clampf(float v, float lo, float hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

int32_t axis_mm_to_us(const axis_t *a, float mm)   { return tmc_mm_to_usteps(mm, a->usteps_per_mm); }
float   axis_us_to_mm(const axis_t *a, int32_t us) { return (float)us / a->usteps_per_mm; }
float   axis_pos_mm(const axis_t *a)               { return axis_us_to_mm(a, tmc_motion_get_position(a->m)); }
float   axis_speed_mm_s(const axis_t *a)           { return tmc_motion_get_speed(a->m) / a->usteps_per_mm; }

/** Absolute move in mm (clamped to soft limits). accel = 0 -> no ramp. */
void axis_move_to_mm(axis_t *a, float mm, float mm_s, float mm_s2)
{
    mm = clampf(mm, a->min_mm, a->max_mm);
    tmc_motion_move_to_ex(a->m, axis_mm_to_us(a, mm),
                          mm_s * a->usteps_per_mm, mm_s2 * a->usteps_per_mm);
}

/**
 * Duration of a trapezoidal (or triangular) move that starts and ends at v0.
 * Useful for timeouts and for planning.
 */
float trapezoid_time_s(float dist, float v0, float vmax, float acc)
{
    float d_ramp, vp;
    dist = fabsf(dist);
    if ((dist <= 0.0f) || (vmax <= 0.0f)) {
        return 0.0f;
    }
    if (v0 > vmax) {
        v0 = vmax;
    }
    if (acc <= 0.0f) {
        return dist / vmax;                                   /* no ramp      */
    }
    d_ramp = (vmax * vmax - v0 * v0) / (2.0f * acc);
    if (2.0f * d_ramp <= dist) {                               /* trapezoid    */
        return 2.0f * (vmax - v0) / acc + (dist - 2.0f * d_ramp) / vmax;
    }
    vp = sqrtf(v0 * v0 + acc * dist);                          /* triangle     */
    return 2.0f * (vp - v0) / acc;
}

/** Waits until the axis stops. On timeout: decelerate and return false.
 *  Note: tmc_motion_stop() brakes with the axis DEFAULT acceleration
 *  (tmc_motion_set_acceleration), not with the accel of the last command. */
bool motion_wait(tmc_motion_t *m, uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while (tmc_motion_is_running(m)) {
        if ((HAL_GetTick() - t0) > timeout_ms) {
            tmc_motion_stop(m);
            return false;
        }
        /* RTOS: osDelay(1); bare-metal: feed watchdog here if needed */
    }
    return true;
}

/** Blocking absolute move with an automatic timeout (1.5 x planned time + 200 ms). */
bool motion_move_and_wait(tmc_motion_t *m, int32_t target, float vmax, float accel)
{
    float dist = (float)(target - tmc_motion_get_position(m));
    float t    = trapezoid_time_s(dist, m->start_speed, vmax, accel);
    tmc_motion_move_to_ex(m, target, vmax, accel);
    return motion_wait(m, (uint32_t)(t * 1500.0f) + 200U);
}

/* ========================================================================== */
/*  Ex 1 - Point-to-point, per-move speed and acceleration                    */
/* ========================================================================== */
/* 16 µsteps, 200-step motor -> 3200 µsteps per revolution */
void example1_point_to_point(tmc_motion_t *m)
{
    /* 1 rev at 0.5 rev/s, ramp 1 rev/s² */
    motion_move_and_wait(m, 3200, 1600.0f, 3200.0f);

    /* 5 rev at 2 rev/s, ramp 4 rev/s² */
    motion_move_and_wait(m, 16000, 6400.0f, 12800.0f);

    /* Back to 0 slowly without ramp (only safe at low speed) */
    motion_move_and_wait(m, 0, 800.0f, 0.0f);

    /* Short move: the planner automatically produces a triangular profile */
    motion_move_and_wait(m, 200, 6400.0f, 12800.0f);
}

/* ========================================================================== */
/*  Ex 2 - Relative moves (jog)                                               */
/* ========================================================================== */
void example2_relative(tmc_motion_t *m)
{
    /* 5 increments of 1/4 rev with a pause between them */
    for (int i = 0; i < 5; i++) {
        tmc_motion_move_ex(m, 800, 2000.0f, 8000.0f);
        motion_wait(m, 2000U);
        HAL_Delay(200);
    }
    /* Return by the total distance */
    tmc_motion_move_ex(m, -4000, 4000.0f, 8000.0f);
    motion_wait(m, 3000U);
}

/* Button-driven jog: every press adds one increment.
 * While moving, tmc_motion_move_ex() adds to the current TARGET, not to the
 * current position, so fast repeated presses accumulate correctly.        */
void example2_on_jog_button(tmc_motion_t *m, bool forward)
{
    tmc_motion_move_ex(m, forward ? 400 : -400, 3000.0f, 12000.0f);
}

/* ========================================================================== */
/*  Ex 3 - Millimetres and soft limits                                        */
/* ========================================================================== */
void example3_mm(axis_t *a)
{
    /* Lead screw T8 (8 mm/rev), 16 µsteps: 3200 / 8 = 400 µsteps/mm */
    a->usteps_per_mm = 400.0f;
    a->min_mm        = 0.0f;
    a->max_mm        = 250.0f;

    axis_move_to_mm(a, 50.0f, 20.0f, 100.0f);       /* 50 mm at 20 mm/s     */
    motion_wait(a->m, 5000U);

    axis_move_to_mm(a, 120.5f, 40.0f, 200.0f);      /* 120.5 mm at 40 mm/s  */
    motion_wait(a->m, 5000U);

    axis_move_to_mm(a, 400.0f, 40.0f, 200.0f);      /* clamped to 250 mm    */
    motion_wait(a->m, 10000U);
}

/* ========================================================================== */
/*  Ex 4 - Non-blocking waypoint sequence                                     */
/* ========================================================================== */
typedef struct {
    float    mm;
    float    mm_s;
    float    mm_s2;
    uint16_t dwell_ms;
} waypoint_t;

typedef enum { SEQ_IDLE, SEQ_MOVING, SEQ_DWELL, SEQ_DONE } seq_state_t;

typedef struct {
    axis_t           *axis;
    const waypoint_t *wp;
    uint8_t           count;
    uint8_t           idx;
    bool              loop;
    seq_state_t       state;
    uint32_t          t_mark;
} sequencer_t;

const waypoint_t demo_path[] = {
    {  10.0f,  20.0f, 200.0f, 500 },   /* approach                   */
    { 100.0f,  80.0f, 400.0f,   0 },   /* fast traverse              */
    { 105.0f,   2.0f,  50.0f, 1000 },  /* slow, precise final 5 mm   */
    {  10.0f,  80.0f, 400.0f, 300 },   /* return                     */
};

void seq_start(sequencer_t *s, axis_t *a, const waypoint_t *wp, uint8_t n, bool loop)
{
    s->axis  = a;
    s->wp    = wp;
    s->count = n;
    s->idx   = 0;
    s->loop  = loop;
    s->state = SEQ_IDLE;
}

/** Call from the main loop. Returns true when the sequence is finished. */
bool seq_task(sequencer_t *s)
{
    const waypoint_t *w;

    switch (s->state) {
    case SEQ_IDLE:
        if (s->idx >= s->count) {
            if (!s->loop) {
                s->state = SEQ_DONE;
                break;
            }
            s->idx = 0;
        }
        w = &s->wp[s->idx];
        axis_move_to_mm(s->axis, w->mm, w->mm_s, w->mm_s2);
        s->state = SEQ_MOVING;
        break;

    case SEQ_MOVING:
        if (!tmc_motion_is_running(s->axis->m)) {
            s->t_mark = HAL_GetTick();
            s->state  = SEQ_DWELL;
        }
        break;

    case SEQ_DWELL:
        if ((HAL_GetTick() - s->t_mark) >= s->wp[s->idx].dwell_ms) {
            s->idx++;
            s->state = SEQ_IDLE;
        }
        break;

    case SEQ_DONE:
    default:
        break;
    }
    return s->state == SEQ_DONE;
}

/* Usage:
 *   static sequencer_t seq;
 *   seq_start(&seq, &axis_x, demo_path, 4, true);
 *   while (1) { seq_task(&seq); ...other work... }
 */

/* ========================================================================== */
/*  Ex 5 - Changing speed during a move (speed zones)                         */
/* ========================================================================== */
/* Move to 100 mm at 50 mm/s, but slow down to 5 mm/s after 80 mm.
 * Re-issuing move_to_ex() with the SAME target and a lower vmax makes the
 * axis decelerate to the new speed and continue - no stop in between.     */
typedef enum { ZONE_START, ZONE_FAST, ZONE_SLOW, ZONE_DONE } zone_state_t;

void example5_speed_zones_task(axis_t *a, zone_state_t *st)
{
    const float target_mm = 100.0f;
    const float accel     = 300.0f;   /* mm/s² */

    switch (*st) {
    case ZONE_START:
        axis_move_to_mm(a, target_mm, 50.0f, accel);
        *st = ZONE_FAST;
        break;
    case ZONE_FAST:
        if (axis_pos_mm(a) >= 80.0f) {
            axis_move_to_mm(a, target_mm, 5.0f, accel);    /* same target, lower speed */
            *st = ZONE_SLOW;
        }
        break;
    case ZONE_SLOW:
        if (!tmc_motion_is_running(a->m)) {
            *st = ZONE_DONE;
        }
        break;
    default:
        break;
    }
}

/* Speed override knob (like a CNC feed-rate override): 10..150 % */
void example5_feed_override(axis_t *a, float nominal_mm_s, float accel_mm_s2, uint8_t percent)
{
    float v = nominal_mm_s * (float)percent / 100.0f;
    if (tmc_motion_is_running(a->m) && (a->m->mode == TMC_MOTION_POSITION)) {
        tmc_motion_move_to_ex(a->m, a->m->target, v * a->usteps_per_mm, accel_mm_s2 * a->usteps_per_mm);
    }
}

/* ========================================================================== */
/*  Ex 6 - Following a moving set-point with a speed limit                     */
/* ========================================================================== */
/* Call every 10-20 ms. The axis tracks the set-point like a slow servo:
 * the ramp generator limits speed and acceleration, and a new target can be
 * given at any time (even in the opposite direction).                      */
void example6_follow_task(axis_t *a, float setpoint_mm)
{
    static int32_t last_target = INT32_MIN;
    const int32_t  deadband    = 20;           /* µsteps: ignore ADC noise */
    int32_t        target      = axis_mm_to_us(a, clampf(setpoint_mm, a->min_mm, a->max_mm));

    if ((last_target != INT32_MIN) && (abs(target - last_target) < deadband)) {
        return;
    }
    last_target = target;
    tmc_motion_move_to_ex(a->m, target, 30.0f * a->usteps_per_mm, 150.0f * a->usteps_per_mm);
}

/* Set-point from a potentiometer (12-bit ADC):
 *   float sp = a->min_mm + (a->max_mm - a->min_mm) * adc_value / 4095.0f;
 *   example6_follow_task(&axis_x, sp);
 */

/* ========================================================================== */
/*  Ex 7 - Two-axis synchronised linear move                                  */
/* ========================================================================== */
/* Scale speed, acceleration and start speed of each axis by its share of
 * the path length. All trapezoids then have the same duration, so both axes
 * start and arrive together and the tool moves on a straight line
 * (approximately - the ramp shapes are identical, only scaled).           */
void example7_linear_move_xy(axis_t *x, axis_t *y, float x_mm, float y_mm,
                             float feed_mm_s, float accel_mm_s2, float start_mm_s)
{
    int32_t tx = axis_mm_to_us(x, clampf(x_mm, x->min_mm, x->max_mm));
    int32_t ty = axis_mm_to_us(y, clampf(y_mm, y->min_mm, y->max_mm));
    float   lx = fabsf(axis_us_to_mm(x, tx - tmc_motion_get_position(x->m)));
    float   ly = fabsf(axis_us_to_mm(y, ty - tmc_motion_get_position(y->m)));
    float   len = sqrtf(lx * lx + ly * ly);
    float   kx, ky;

    if (len <= 0.0f) {
        return;
    }
    kx = lx / len;
    ky = ly / len;

    tmc_motion_set_start_speed(x->m, start_mm_s * kx * x->usteps_per_mm);
    tmc_motion_set_start_speed(y->m, start_mm_s * ky * y->usteps_per_mm);

    /* Hold the motion timer IRQ so both axes start on the same tick */
    HAL_NVIC_DisableIRQ(MOTION_TIMER_IRQn);
    if (lx > 0.0f) {
        tmc_motion_move_to_ex(x->m, tx, feed_mm_s * kx * x->usteps_per_mm, accel_mm_s2 * kx * x->usteps_per_mm);
    }
    if (ly > 0.0f) {
        tmc_motion_move_to_ex(y->m, ty, feed_mm_s * ky * y->usteps_per_mm, accel_mm_s2 * ky * y->usteps_per_mm);
    }
    HAL_NVIC_EnableIRQ(MOTION_TIMER_IRQn);
}

/* Usage: draw a square 50 x 50 mm at 30 mm/s
 *   const float pts[][2] = { {0,0}, {50,0}, {50,50}, {0,50}, {0,0} };
 *   for (int i = 0; i < 5; i++) {
 *       example7_linear_move_xy(&ax, &ay, pts[i][0], pts[i][1], 30, 300, 2);
 *       while (tmc_motion_is_running(ax.m) || tmc_motion_is_running(ay.m)) {}
 *   }
 * Note: this is point-to-point linear motion, not a continuous-path planner
 * (the axes stop at each corner).
 */

/* ========================================================================== */
/*  Ex 8 - Homing with a limit switch                                         */
/* ========================================================================== */
typedef enum {
    HOME_IDLE, HOME_FAST_SEEK, HOME_BACKOFF, HOME_SLOW_SEEK, HOME_OFFSET, HOME_DONE, HOME_FAIL
} home_state_t;

typedef struct {
    axis_t        *axis;
    GPIO_TypeDef  *sw_port;
    uint16_t       sw_pin;
    GPIO_PinState  sw_active;     /* level when the switch is pressed      */
    int8_t         dir;           /* -1: switch at the minimum end         */
    float          fast_mm_s, slow_mm_s, accel_mm_s2;
    float          backoff_mm, offset_mm, max_travel_mm;
    home_state_t   state;
    volatile bool  hit;
} homing_t;

static bool home_switch_pressed(const homing_t *h)
{
    return HAL_GPIO_ReadPin(h->sw_port, h->sw_pin) == h->sw_active;
}

void homing_start(homing_t *h)
{
    tmc_motion_t *m = h->axis->m;
    h->hit = false;
    tmc_motion_set_position(m, 0);
    if (home_switch_pressed(h)) {
        /* Already on the switch: skip the fast seek, just back off */
        h->state = HOME_BACKOFF;
        tmc_motion_move_to_ex(m, axis_mm_to_us(h->axis, -h->dir * h->backoff_mm),
                              h->slow_mm_s * 4.0f * h->axis->usteps_per_mm,
                              h->accel_mm_s2 * h->axis->usteps_per_mm);
        return;
    }
    h->state = HOME_FAST_SEEK;
    tmc_motion_run_velocity(m, h->dir * h->fast_mm_s * h->axis->usteps_per_mm,
                            h->accel_mm_s2 * h->axis->usteps_per_mm);
}

/** Call from HAL_GPIO_EXTI_Callback() for the switch pin. */
void homing_on_switch_isr(homing_t *h)
{
    if (h->state == HOME_FAST_SEEK) {
        h->hit = true;
        /* Decelerate with the HOMING acceleration (tmc_motion_stop() would use the
         * axis default set by tmc_motion_set_acceleration()). Switch needs over-travel
         * of at least v^2 / (2a).                                                  */
        tmc_motion_run_velocity(h->axis->m, 0.0f, h->accel_mm_s2 * h->axis->usteps_per_mm);
    } else if (h->state == HOME_SLOW_SEEK) {
        h->hit = true;
        tmc_motion_emergency_stop(h->axis->m);   /* slow speed: stop at once, exact edge */
    }
}

/** Call from the main loop. */
void homing_task(homing_t *h)
{
    tmc_motion_t *m   = h->axis->m;
    float         upm = h->axis->usteps_per_mm;

    /* Safety: no switch found within the maximum travel */
    if (((h->state == HOME_FAST_SEEK) || (h->state == HOME_SLOW_SEEK)) &&
        (fabsf(axis_pos_mm(h->axis)) > h->max_travel_mm)) {
        tmc_motion_emergency_stop(m);
        h->state = HOME_FAIL;
        return;
    }
    if (tmc_motion_is_running(m)) {
        return;
    }

    switch (h->state) {
    case HOME_FAST_SEEK:
        if (h->hit) {
            h->hit = false;
            tmc_motion_set_position(m, 0);
            tmc_motion_move_to_ex(m, axis_mm_to_us(h->axis, -h->dir * h->backoff_mm),
                                  h->fast_mm_s * upm, h->accel_mm_s2 * upm);
            h->state = HOME_BACKOFF;
        }
        break;
    case HOME_BACKOFF:
        if (home_switch_pressed(h)) {          /* back-off too short */
            h->state = HOME_FAIL;
            break;
        }
        tmc_motion_set_position(m, 0);
        tmc_motion_run_velocity(m, h->dir * h->slow_mm_s * upm, h->accel_mm_s2 * upm);
        h->state = HOME_SLOW_SEEK;
        break;
    case HOME_SLOW_SEEK:
        if (h->hit) {
            tmc_motion_set_position(m, 0);     /* the switch edge is the new zero */
            axis_move_to_mm(h->axis, h->offset_mm, h->fast_mm_s, h->accel_mm_s2);
            h->state = HOME_OFFSET;
        }
        break;
    case HOME_OFFSET:
        h->state = HOME_DONE;
        break;
    default:
        break;
    }
}

/* Setup example:
 *   static homing_t home_x = {
 *       .axis = &ax, .sw_port = XMIN_GPIO_Port, .sw_pin = XMIN_Pin,
 *       .sw_active = GPIO_PIN_RESET, .dir = -1,
 *       .fast_mm_s = 20, .slow_mm_s = 2, .accel_mm_s2 = 200,
 *       .backoff_mm = 3, .offset_mm = 1, .max_travel_mm = 300 };
 *   homing_start(&home_x);
 *   void HAL_GPIO_EXTI_Callback(uint16_t pin) { if (pin == XMIN_Pin) homing_on_switch_isr(&home_x); }
 *   while (home_x.state != HOME_DONE && home_x.state != HOME_FAIL) homing_task(&home_x);
 * Note: offset_mm must be >= 0 and within [min_mm, max_mm].
 */

/* ========================================================================== */
/*  Ex 9 - UART + STEP together: drive profile per move                       */
/* ========================================================================== */
/* TPWMTHRS for the StealthChop -> SpreadCycle switch-over speed.
 * TSTEP is the time between 1/256 microsteps in units of 1/fCLK (12 MHz):
 *   f_1/256 = usteps_per_s * 256 / microsteps
 *   TSTEP   = fCLK / f_1/256
 * StealthChop is used while TSTEP > TPWMTHRS, i.e. BELOW the threshold speed. */
uint32_t tpwmthrs_for_speed(float usteps_per_s, uint16_t microsteps)
{
    float t;
    if (usteps_per_s <= 0.0f) {
        return 0;                              /* 0 = StealthChop at all speeds */
    }
    t = 12000000.0f * (float)microsteps / (256.0f * usteps_per_s);
    return (t > 1048575.0f) ? 1048575UL : (uint32_t)t;
}

typedef struct {
    uint8_t run_pct;
    uint8_t hold_pct;
} drive_profile_t;

static const drive_profile_t PROFILE_PRECISE = { 50, 20 };   /* cool, quiet   */
static const drive_profile_t PROFILE_RAPID   = { 90, 30 };   /* more torque   */

/** Apply a current profile over UART, then move via STEP/DIR (blocking). */
bool move_with_profile(tmc2209_t *drv, tmc_motion_t *m, const drive_profile_t *p,
                       int32_t target, float vmax, float accel)
{
    if (tmc2209_set_run_current(drv, p->run_pct) != TMC_OK) {
        return false;                          /* do not move with unknown current */
    }
    if (tmc2209_set_hold_current(drv, p->hold_pct) != TMC_OK) {
        return false;
    }
    return motion_move_and_wait(m, target, vmax, accel);
}

void example9_combined(tmc2209_t *drv, tmc_motion_t *m)
{
    const uint16_t microsteps = 16;

    /* One-time: quiet StealthChop below 1 rev/s, SpreadCycle above (more torque) */
    tmc2209_enable_stealth_chop(drv);
    tmc2209_set_stealth_chop_duration_threshold(drv, tpwmthrs_for_speed(3200.0f, microsteps));

    move_with_profile(drv, m, &PROFILE_RAPID,   32000, 9600.0f, 19200.0f);  /* 3 rev/s   */
    move_with_profile(drv, m, &PROFILE_PRECISE, 32800,  400.0f,  1600.0f);  /* 0.125 rev/s */

    /* Check the driver after the move */
    {
        tmc2209_status_t st;
        if ((tmc2209_get_status(drv, &st) == TMC_OK) &&
            (st.over_temperature_warning || st.open_load_a || st.open_load_b)) {
            /* report / derate current / stop the machine */
        }
    }
}

/* ========================================================================== */
/*  Ex 10 - Sensorless homing with StallGuard                                 */
/* ========================================================================== */
/* Wiring: TMC2209 DIAG -> MCU EXTI input (rising edge).
 * TMC2209 StallGuard4 works in StealthChop only. DIAG goes high when
 * SG_RESULT <= 2 * SGTHRS while TCOOLTHRS >= TSTEP > TPWMTHRS.
 * SGTHRS MUST be tuned on the real machine (motor, current, speed, load):
 *   1. run at the homing speed with no load, log SG_RESULT (e.g. ~250)
 *   2. hold the axis by hand / against the end stop, log SG_RESULT (e.g. ~40)
 *   3. choose SGTHRS so that 2 * SGTHRS lies between the two (e.g. 60)
 * Homing speed must be high enough for back-EMF (too slow -> false stalls). */
typedef struct {
    tmc2209_t    *drv;
    tmc_motion_t *m;
    volatile bool stalled;
    bool          armed;
    uint32_t      t_start;
} sg_home_t;

void sg_home_start(sg_home_t *h, uint8_t sgthrs, float speed, float accel)
{
    tmc2209_enable_stealth_chop(h->drv);
    tmc2209_set_stealth_chop_duration_threshold(h->drv, 0);       /* StealthChop always */
    tmc2209_set_cool_step_duration_threshold(h->drv, 0xFFFFF);    /* SG valid at all speeds */
    tmc2209_set_stall_guard_threshold(h->drv, sgthrs);

    h->stalled = false;
    h->armed   = false;
    h->t_start = HAL_GetTick();
    tmc_motion_run_velocity(h->m, -speed, accel);
}

/** Call from HAL_GPIO_EXTI_Callback() for the DIAG pin. */
void sg_home_on_diag_isr(sg_home_t *h)
{
    if (h->armed) {
        h->stalled = true;
        tmc_motion_emergency_stop(h->m);
    }
}

/** Main loop. Returns true when homing is complete. */
bool sg_home_task(sg_home_t *h, uint32_t blank_ms)
{
    /* Ignore DIAG during acceleration: SG_RESULT is not valid yet */
    if (!h->armed && ((HAL_GetTick() - h->t_start) > blank_ms)) {
        h->armed = true;
    }
    if (h->stalled && !tmc_motion_is_running(h->m)) {
        tmc_motion_set_position(h->m, 0);
        tmc_motion_move_to_ex(h->m, 800, 2000.0f, 8000.0f);        /* move off the end stop */
        h->stalled = false;
        h->armed   = false;
        return true;
    }
    return false;
}

/* Tuning helper: print SG_RESULT while the axis runs at homing speed */
void sg_tuning_log(tmc2209_t *drv)
{
    uint16_t sg;
    if (tmc2209_get_stall_guard_result(drv, &sg) == TMC_OK) {
        /* printf("SG_RESULT=%u\r\n", sg); */
        (void)sg;
    }
}

/* ========================================================================== */
/*  Ex 11 - Velocity jog that stops exactly at a soft limit                   */
/* ========================================================================== */
/* Joystick jog in velocity mode. When the remaining distance to the limit is
 * about the braking distance, switch to a position move targeting the limit:
 * the ramp generator then brakes and stops exactly on the limit.            */
void example11_jog_task(axis_t *a, float jog_mm_s, float accel_mm_s2)
{
    tmc_motion_t *m     = a->m;
    float         pos   = axis_pos_mm(a);
    float         v     = axis_speed_mm_s(a);
    float         brake = (accel_mm_s2 > 0.0f) ? (v * v) / (2.0f * accel_mm_s2) : 0.0f;
    const float   margin = 0.5f;   /* mm, covers the call period */

    if (m->mode == TMC_MOTION_POSITION) {
        /* Already braking into a limit: only accept jog away from it */
        bool away = ((m->target >= axis_mm_to_us(a, a->max_mm)) && (jog_mm_s < 0.0f)) ||
                    ((m->target <= axis_mm_to_us(a, a->min_mm)) && (jog_mm_s > 0.0f));
        if (!away) {
            return;
        }
    }

    if ((jog_mm_s > 0.0f) && (pos + brake + margin >= a->max_mm)) {
        axis_move_to_mm(a, a->max_mm, jog_mm_s, accel_mm_s2);
    } else if ((jog_mm_s < 0.0f) && (pos - brake - margin <= a->min_mm)) {
        axis_move_to_mm(a, a->min_mm, -jog_mm_s, accel_mm_s2);
    } else {
        tmc_motion_run_velocity(m, jog_mm_s * a->usteps_per_mm, accel_mm_s2 * a->usteps_per_mm);
    }
}

/* Usage (every 20 ms):
 *   float js = (adc_joystick - 2048) / 2048.0f;           // -1 .. +1
 *   if (fabsf(js) < 0.05f) js = 0;                          // dead zone
 *   example11_jog_task(&ax, js * 50.0f, 300.0f);            // max 50 mm/s
 */
