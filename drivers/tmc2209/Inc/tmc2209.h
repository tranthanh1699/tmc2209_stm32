/**
 * @file    tmc2209.h
 * @brief   TMC2209 driver over UART for STM32 HAL (C port of janelia-arduino/TMC2209).
 *
 * Two-layer architecture:
 *   - tmc2209.h/.c        : UART bus + register configuration + VACTUAL (velocity over UART)
 *   - tmc2209_motion.h/.c : STEP/DIR pulse generator with ramp (position / velocity / position+velocity)
 *
 * Multiple drivers on 1 UART: create 1 tmc_bus_t, and several tmc2209_t with addr 0..3.
 */
#ifndef TMC2209_H
#define TMC2209_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "tmc2209_port.h"
#include "tmc2209_reg.h"

/* ========================================================================== */
/*  Data types                                                                 */
/* ========================================================================== */

typedef enum {
    TMC_OK = 0,
    TMC_ERR_PARAM,      /* invalid parameter / not initialized                */
    TMC_ERR_TX,         /* HAL_UART_Transmit failed                           */
    TMC_ERR_TIMEOUT,    /* did not receive enough bytes                       */
    TMC_ERR_ECHO,       /* echo does not match the frame sent (collision / noise) */
    TMC_ERR_CRC,        /* wrong reply CRC                                     */
    TMC_ERR_REPLY,      /* wrong reply sync/addr/register                     */
    TMC_ERR_NO_RX,      /* bus is in TX-only mode, cannot read                */
    TMC_ERR_UART,       /* UART hardware error (ORE/FE/NE)                    */
    TMC_ERR_NOT_READY   /* chip not responding / wrong version                */
} tmc_status_t;

typedef enum {
    /* TX only -> PDN_UART (through a 1k resistor). Write-only, getters return the shadow register. */
    TMC_UART_TX_ONLY = 0,
    /* MCU TX and RX both wired to PDN_UART (TX through 1k). Normal full-duplex
     * UART in CubeMX. The MCU will receive its own echo back -> the library discards it. */
    TMC_UART_TX_RX,
    /* CubeMX set to "Single Wire (Half-Duplex)": one TX pin wired to PDN_UART.
     * The library calls HAL_HalfDuplex_EnableTransmitter/Receiver itself.     */
    TMC_UART_HALF_DUPLEX
} tmc_uart_mode_t;

typedef struct {
    UART_HandleTypeDef *huart;
    tmc_uart_mode_t     mode;
    uint32_t            timeout_ms;   /* reply receive timeout                */
    uint8_t             retries;      /* number of retries on a read error    */
    /* Optional for RTOS: multiple tasks sharing the bus. Leave NULL for bare-metal. */
    void              (*lock)(void *ctx);
    void              (*unlock)(void *ctx);
    void               *lock_ctx;
    /* Statistics */
    uint32_t            err_count;
    tmc_status_t        last_err;
} tmc_bus_t;

typedef enum {
    TMC_STANDSTILL_NORMAL         = 0,
    TMC_STANDSTILL_FREEWHEELING   = 1,
    TMC_STANDSTILL_STRONG_BRAKING = 2,
    TMC_STANDSTILL_BRAKING        = 3
} tmc_standstill_mode_t;

typedef enum {
    TMC_CS_INCREMENT_1 = 0, TMC_CS_INCREMENT_2 = 1,
    TMC_CS_INCREMENT_4 = 2, TMC_CS_INCREMENT_8 = 3
} tmc_cs_increment_t;

typedef enum {
    TMC_SG_COUNT_32 = 0, TMC_SG_COUNT_8 = 1,
    TMC_SG_COUNT_2  = 2, TMC_SG_COUNT_1 = 3
} tmc_sg_count_t;

/** Handle for one driver. A shadow register is required because several
 *  registers are write-only (IHOLD_IRUN, COOLCONF...) and to support TX-only mode. */
typedef struct {
    tmc_bus_t     *bus;
    uint8_t        addr;            /* 0..3 per MS1/MS2                         */

    /* Shadow registers */
    uint32_t       gconf;
    uint32_t       ihold_irun;
    uint32_t       chopconf;
    uint32_t       pwmconf;
    uint32_t       coolconf;
    uint32_t       slaveconf;
    uint32_t       tpowerdown;
    uint32_t       tpwmthrs;
    uint32_t       tcoolthrs;
    uint32_t       sgthrs;
    int32_t        vactual;
    uint8_t        toff;            /* TOFF kept so it can be re-applied on enable/disable */
    bool           cool_step_enabled;

    /* Hardware EN pin (optional, active-low) */
    GPIO_TypeDef  *en_port;
    uint16_t       en_pin;

    /* Velocity ramp over UART (VACTUAL), in microsteps/s */
    float          vr_target;
    float          vr_current;
    float          vr_accel;        /* µstep/s^2, 0 = no ramp                   */
    uint32_t       vr_last_ms;
    bool           vr_active;
} tmc2209_t;

typedef struct {
    bool     over_temperature_warning;
    bool     over_temperature_shutdown;
    bool     short_to_ground_a;
    bool     short_to_ground_b;
    bool     low_side_short_a;
    bool     low_side_short_b;
    bool     open_load_a;
    bool     open_load_b;
    bool     over_temperature_120c;
    bool     over_temperature_143c;
    bool     over_temperature_150c;
    bool     over_temperature_157c;
    uint8_t  current_scaling;       /* CS_ACTUAL 0..31                          */
    bool     stealth_chop_mode;
    bool     standstill;
    uint32_t raw;
} tmc2209_status_t;

typedef struct {
    bool     reset;
    bool     drv_err;
    bool     uv_cp;
} tmc2209_gstat_t;

typedef struct {
    bool     is_communicating;
    bool     is_setup;
    bool     software_enabled;
    uint16_t microsteps_per_step;
    bool     inverse_motor_direction_enabled;
    bool     stealth_chop_enabled;
    uint8_t  standstill_mode;
    uint8_t  irun_percent;
    uint8_t  irun_register_value;
    uint8_t  ihold_percent;
    uint8_t  ihold_register_value;
    uint8_t  iholddelay_percent;
    uint8_t  iholddelay_register_value;
    bool     automatic_current_scaling_enabled;
    bool     automatic_gradient_adaptation_enabled;
    uint8_t  pwm_offset;
    uint8_t  pwm_gradient;
    bool     cool_step_enabled;
    bool     analog_current_scaling_enabled;
    bool     internal_sense_resistors_enabled;
} tmc2209_settings_t;

/* ========================================================================== */
/*  Bus                                                                        */
/* ========================================================================== */

/** huart must already be initialized by MX_USARTx_UART_Init(). For TMC_UART_TX_RX
 *  you must enable the USART global NVIC interrupt in CubeMX (uses HAL_UART_Receive_IT). */
tmc_status_t tmc_bus_init(tmc_bus_t *bus, UART_HandleTypeDef *huart, tmc_uart_mode_t mode);
void         tmc_bus_set_lock(tmc_bus_t *bus, void (*lock)(void *), void (*unlock)(void *), void *ctx);
bool         tmc_bus_can_read(const tmc_bus_t *bus);

/* ========================================================================== */
/*  Initialization / low-level                                                */
/* ========================================================================== */

/** Equivalent to TMC2209::setup(): switches to UART mode, loads the defaults,
 *  sets minimum current, driver left in DISABLE state (must call enable afterwards). */
tmc_status_t tmc2209_init(tmc2209_t *dev, tmc_bus_t *bus, uint8_t addr);

tmc_status_t tmc2209_write_reg(tmc2209_t *dev, uint8_t reg, uint32_t value);
tmc_status_t tmc2209_read_reg(tmc2209_t *dev, uint8_t reg, uint32_t *value);

/** Re-read GCONF/CHOPCONF/PWMCONF from the chip into the shadow (only if the bus can read). */
tmc_status_t tmc2209_sync_from_chip(tmc2209_t *dev);
/** Write the entire shadow back down to the chip (use after detecting a chip reset). */
tmc_status_t tmc2209_restore_to_chip(tmc2209_t *dev);

/* ========================================================================== */
/*  Write API (usable even in TX-only) — names follow the original library    */
/* ========================================================================== */

void         tmc2209_set_hardware_enable_pin(tmc2209_t *dev, GPIO_TypeDef *port, uint16_t pin);
tmc_status_t tmc2209_enable(tmc2209_t *dev);
tmc_status_t tmc2209_disable(tmc2209_t *dev);

/** 1,2,4,...,256 (other values are rounded down to a power of two). */
tmc_status_t tmc2209_set_microsteps_per_step(tmc2209_t *dev, uint16_t microsteps);
tmc_status_t tmc2209_set_microsteps_per_step_power_of_two(tmc2209_t *dev, uint8_t exponent);
uint16_t     tmc2209_get_microsteps_per_step(const tmc2209_t *dev);   /* from shadow */

tmc_status_t tmc2209_set_run_current(tmc2209_t *dev, uint8_t percent);   /* 0..100 */
tmc_status_t tmc2209_set_hold_current(tmc2209_t *dev, uint8_t percent);  /* 0..100 */
tmc_status_t tmc2209_set_hold_delay(tmc2209_t *dev, uint8_t percent);    /* 0..100 */
tmc_status_t tmc2209_set_all_current_values(tmc2209_t *dev, uint8_t run_pct,
                                            uint8_t hold_pct, uint8_t hold_delay_pct);
/** RMS current in mA given the sense resistor (Ω); hold = run * hold_multiplier. */
tmc_status_t tmc2209_set_rms_current(tmc2209_t *dev, uint16_t mA, float r_sense, float hold_multiplier);

tmc_status_t tmc2209_enable_double_edge(tmc2209_t *dev);
tmc_status_t tmc2209_disable_double_edge(tmc2209_t *dev);
bool         tmc2209_double_edge_enabled(const tmc2209_t *dev);
tmc_status_t tmc2209_enable_vsense(tmc2209_t *dev);
tmc_status_t tmc2209_disable_vsense(tmc2209_t *dev);
tmc_status_t tmc2209_enable_inverse_motor_direction(tmc2209_t *dev);
tmc_status_t tmc2209_disable_inverse_motor_direction(tmc2209_t *dev);
tmc_status_t tmc2209_set_standstill_mode(tmc2209_t *dev, tmc_standstill_mode_t mode);

tmc_status_t tmc2209_enable_automatic_current_scaling(tmc2209_t *dev);
tmc_status_t tmc2209_disable_automatic_current_scaling(tmc2209_t *dev);
tmc_status_t tmc2209_enable_automatic_gradient_adaptation(tmc2209_t *dev);
tmc_status_t tmc2209_disable_automatic_gradient_adaptation(tmc2209_t *dev);
tmc_status_t tmc2209_set_pwm_offset(tmc2209_t *dev, uint8_t pwm_amplitude);
tmc_status_t tmc2209_set_pwm_gradient(tmc2209_t *dev, uint8_t pwm_amplitude);

tmc_status_t tmc2209_set_power_down_delay(tmc2209_t *dev, uint8_t delay);   /* >=2 for auto-tune */
/** SENDDELAY 0..15. Should be >= 2 when several addresses share a two-way bus. */
tmc_status_t tmc2209_set_reply_delay(tmc2209_t *dev, uint8_t delay);

tmc_status_t tmc2209_enable_stealth_chop(tmc2209_t *dev);
tmc_status_t tmc2209_disable_stealth_chop(tmc2209_t *dev);
tmc_status_t tmc2209_set_stealth_chop_duration_threshold(tmc2209_t *dev, uint32_t tpwmthrs);
tmc_status_t tmc2209_set_cool_step_duration_threshold(tmc2209_t *dev, uint32_t tcoolthrs);
tmc_status_t tmc2209_set_stall_guard_threshold(tmc2209_t *dev, uint8_t sgthrs);
tmc_status_t tmc2209_enable_cool_step(tmc2209_t *dev, uint8_t lower_threshold, uint8_t upper_threshold);
tmc_status_t tmc2209_disable_cool_step(tmc2209_t *dev);
tmc_status_t tmc2209_set_cool_step_current_increment(tmc2209_t *dev, tmc_cs_increment_t inc);
tmc_status_t tmc2209_set_cool_step_measurement_count(tmc2209_t *dev, tmc_sg_count_t count);

tmc_status_t tmc2209_enable_analog_current_scaling(tmc2209_t *dev);
tmc_status_t tmc2209_disable_analog_current_scaling(tmc2209_t *dev);
tmc_status_t tmc2209_use_external_sense_resistors(tmc2209_t *dev);
tmc_status_t tmc2209_use_internal_sense_resistors(tmc2209_t *dev);

/* ---- Velocity control over UART (TMC2209's internal step generator) ------- */

/** Write VACTUAL directly (raw units, ~0.715 µstep/s per LSB). Nonzero => the STEP pin is ignored. */
tmc_status_t tmc2209_move_at_velocity(tmc2209_t *dev, int32_t vactual);
/** VACTUAL = 0 => go back to using the STEP/DIR pins (required before using tmc2209_motion). */
tmc_status_t tmc2209_move_using_step_dir_interface(tmc2209_t *dev);

/** Convert µstep/s <-> VACTUAL (uses the internal 12 MHz clock; should be re-measured in practice). */
int32_t      tmc2209_usteps_per_s_to_vactual(float usteps_per_s);
float        tmc2209_vactual_to_usteps_per_s(int32_t vactual);

/** Set a target velocity with a ramp (µstep/s, µstep/s^2). accel = 0 => step change.
 *  tmc2209_velocity_ramp_task() must be called periodically (5..20 ms). */
tmc_status_t tmc2209_velocity_ramp_set(tmc2209_t *dev, float target_usteps_per_s, float accel);
tmc_status_t tmc2209_velocity_ramp_task(tmc2209_t *dev);
bool         tmc2209_velocity_ramp_reached(const tmc2209_t *dev);

/* ========================================================================== */
/*  Read API (requires a TX_RX or HALF_DUPLEX bus)                            */
/* ========================================================================== */

tmc_status_t tmc2209_get_version(tmc2209_t *dev, uint8_t *version);
bool         tmc2209_is_communicating(tmc2209_t *dev);
bool         tmc2209_is_setup_and_communicating(tmc2209_t *dev);
bool         tmc2209_is_communicating_but_not_setup(tmc2209_t *dev);
bool         tmc2209_hardware_disabled(tmc2209_t *dev);

tmc_status_t tmc2209_get_settings(tmc2209_t *dev, tmc2209_settings_t *s);
tmc_status_t tmc2209_get_status(tmc2209_t *dev, tmc2209_status_t *st);
tmc_status_t tmc2209_get_global_status(tmc2209_t *dev, tmc2209_gstat_t *gs);
tmc_status_t tmc2209_clear_reset(tmc2209_t *dev);
tmc_status_t tmc2209_clear_drive_error(tmc2209_t *dev);

tmc_status_t tmc2209_get_interface_transmission_counter(tmc2209_t *dev, uint8_t *ifcnt);
tmc_status_t tmc2209_get_interstep_duration(tmc2209_t *dev, uint32_t *tstep);
tmc_status_t tmc2209_get_stall_guard_result(tmc2209_t *dev, uint16_t *sg);
tmc_status_t tmc2209_get_pwm_scale_sum(tmc2209_t *dev, uint8_t *v);
tmc_status_t tmc2209_get_pwm_scale_auto(tmc2209_t *dev, int16_t *v);
tmc_status_t tmc2209_get_pwm_offset_auto(tmc2209_t *dev, uint8_t *v);
tmc_status_t tmc2209_get_pwm_gradient_auto(tmc2209_t *dev, uint8_t *v);
tmc_status_t tmc2209_get_microstep_counter(tmc2209_t *dev, uint16_t *mscnt);

/** Probe addresses 0..3 on the bus, return a bitmask of the addresses that responded. */
uint8_t      tmc2209_scan_bus(tmc_bus_t *bus);

const char  *tmc2209_status_str(tmc_status_t s);

#ifdef __cplusplus
}
#endif
#endif /* TMC2209_H */
