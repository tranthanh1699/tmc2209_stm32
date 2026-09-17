/**
 * @file    tmc2209_motion.h
 * @brief   Trapezoidal-ramp STEP/DIR pulse generator, running in a timer interrupt.
 *
 * The TMC2209 has NO internal position controller (unlike TMC5160/TMC5130).
 * Over UART only velocity (VACTUAL) can be controlled. Accurate position
 * control requires generating STEP/DIR pulses from the MCU -> this module.
 *
 * Principle: a timer interrupt at a fixed frequency F (e.g. 20..50 kHz). Each tick:
 *   vel += ±acc            (Q32.32, acceleration/deceleration)
 *   phase += vel >> 32     (32-bit phase accumulator, DDS style)
 *   phase overflow  => emit 1 step
 * Braking distance: tracked with the ramp_steps counter (number of steps taken
 * while accelerating) => no division/float needed in the ISR.
 *
 * Units: microstep (µstep), µstep/s, µstep/s^2.
 * Maximum speed: F (double-edge) or F/2 (normal pulse).
 */
#ifndef TMC2209_MOTION_H
#define TMC2209_MOTION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "tmc2209.h"

typedef enum {
    TMC_MOTION_IDLE = 0,
    TMC_MOTION_POSITION,     /* run to target with vmax + accel              */
    TMC_MOTION_VELOCITY      /* run continuously to a target velocity with accel */
} tmc_motion_mode_t;

typedef enum {
    TMC_RAMP_STOPPED = 0,
    TMC_RAMP_ACCEL,
    TMC_RAMP_CRUISE,
    TMC_RAMP_DECEL
} tmc_ramp_state_t;

typedef struct {
    GPIO_TypeDef *step_port;
    uint16_t      step_pin;
    GPIO_TypeDef *dir_port;
    uint16_t      dir_pin;
    uint32_t      tick_hz;       /* frequency at which tmc_motion_tick() is called */
    bool          dir_invert;    /* invert direction in software                */
    bool          double_edge;   /* true: every STEP level change = 1 step      */
} tmc_motion_cfg_t;

typedef struct {
    tmc_motion_cfg_t cfg;
    tmc2209_t       *drv;             /* optional, may be NULL                   */

    /* Default parameters (thread context) */
    float            max_speed;       /* µstep/s                                 */
    float            accel;           /* µstep/s^2, 0 = no ramp                  */
    float            start_speed;     /* µstep/s, starting/ending speed           */
    float            speed_limit;     /* hardware limit (F or F/2)               */

    /* ---- State shared with the ISR ---- */
    volatile int32_t  position;
    volatile int32_t  target;
    volatile uint8_t  mode;           /* tmc_motion_mode_t                        */
    volatile uint8_t  ramp_state;     /* tmc_ramp_state_t                         */
    volatile bool     running;
    volatile int8_t   dir;            /* +1 / -1                                  */
    int8_t            vel_dir;        /* target direction in velocity mode        */
    bool              no_ramp;
    bool              step_high;
    uint32_t          phase;
    uint64_t          vel_q;          /* Q32.32: integer part = phase inc / tick  */
    uint64_t          vmax_q;         /* target speed (position) or |v| (velocity) */
    uint64_t          vmin_q;
    uint64_t          acc_q;          /* vel_q increment per tick                  */
    uint32_t          ramp_steps;     /* ~ distance needed to brake to start_speed */
} tmc_motion_t;

/** Initialize. drv != NULL: writes VACTUAL=0 itself and syncs the dedge bit with cfg. */
tmc_status_t tmc_motion_init(tmc_motion_t *m, const tmc_motion_cfg_t *cfg, tmc2209_t *drv);

/** Call from the timer interrupt, at exactly cfg.tick_hz frequency. */
void         tmc_motion_tick(tmc_motion_t *m);

/* ---- Default parameters --------------------------------------------------- */
void         tmc_motion_set_max_speed(tmc_motion_t *m, float usteps_per_s);
void         tmc_motion_set_acceleration(tmc_motion_t *m, float usteps_per_s2);  /* 0 = no ramp */
void         tmc_motion_set_start_speed(tmc_motion_t *m, float usteps_per_s);

/* ---- 1) Position control (uses the default max_speed/accel) --------------- */
void         tmc_motion_move_to(tmc_motion_t *m, int32_t target);
void         tmc_motion_move(tmc_motion_t *m, int32_t delta);

/* ---- 2) Position control with velocity + ramp ------------------------------ */
void         tmc_motion_move_to_ex(tmc_motion_t *m, int32_t target, float max_speed, float accel);
void         tmc_motion_move_ex(tmc_motion_t *m, int32_t delta, float max_speed, float accel);

/* ---- 3) Velocity control (signed) + ramp ----------------------------------- */
void         tmc_motion_run_velocity(tmc_motion_t *m, float usteps_per_s, float accel);

/* ---- Stop ------------------------------------------------------------------ */
void         tmc_motion_stop(tmc_motion_t *m);           /* decelerate per accel     */
void         tmc_motion_emergency_stop(tmc_motion_t *m); /* stop immediately, may skip steps */

/* ---- Status ------------------------------------------------------------------ */
bool         tmc_motion_is_running(const tmc_motion_t *m);
int32_t      tmc_motion_get_position(const tmc_motion_t *m);
int32_t      tmc_motion_distance_to_go(const tmc_motion_t *m);
float        tmc_motion_get_speed(const tmc_motion_t *m);   /* µstep/s, signed */
/** Reset the coordinate origin (homing). Only takes effect while stationary. */
bool         tmc_motion_set_position(tmc_motion_t *m, int32_t position);

/* ---- Unit conversion -------------------------------------------------------- */
static inline int32_t tmc_mm_to_usteps(float mm, float usteps_per_mm)
{
    float v = mm * usteps_per_mm;
    return (int32_t)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f));
}
static inline float tmc_rpm_to_usteps_per_s(float rpm, uint16_t full_steps_per_rev, uint16_t microsteps)
{
    return rpm * (float)full_steps_per_rev * (float)microsteps / 60.0f;
}

#ifdef __cplusplus
}
#endif
#endif /* TMC2209_MOTION_H */
