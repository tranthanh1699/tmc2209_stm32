/**
 * @file    tmc2209_motion.h
 * @brief   Bộ phát xung STEP/DIR có ramp hình thang, chạy trong ngắt timer.
 *
 * TMC2209 KHÔNG có bộ điều khiển vị trí nội (khác TMC5160/TMC5130).
 * Qua UART chỉ điều khiển được vận tốc (VACTUAL). Muốn điều khiển vị trí
 * chính xác phải phát xung STEP/DIR từ MCU -> module này.
 *
 * Nguyên lý: timer ngắt với tần số cố định F (vd 20..50 kHz). Mỗi tick:
 *   vel += ±acc            (Q32.32, tăng/giảm tốc)
 *   phase += vel >> 32     (bộ tích luỹ pha 32 bit, kiểu DDS)
 *   tràn phase  => phát 1 bước
 * Quãng đường phanh: dùng bộ đếm ramp_steps (số bước đã tăng tốc) => không
 * cần phép chia/float trong ISR.
 *
 * Đơn vị: microstep (µstep), µstep/s, µstep/s^2.
 * Tốc độ tối đa: F (double-edge) hoặc F/2 (xung thường).
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
    TMC_MOTION_POSITION,     /* chạy tới target với vmax + accel             */
    TMC_MOTION_VELOCITY      /* chạy liên tục tới vận tốc đích với accel     */
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
    uint32_t      tick_hz;       /* tần số gọi tmc_motion_tick()               */
    bool          dir_invert;    /* đảo chiều bằng phần mềm                    */
    bool          double_edge;   /* true: mỗi lần đảo mức STEP = 1 bước        */
} tmc_motion_cfg_t;

typedef struct {
    tmc_motion_cfg_t cfg;
    tmc2209_t       *drv;             /* tuỳ chọn, có thể NULL                   */

    /* Tham số mặc định (thread context) */
    float            max_speed;       /* µstep/s                                 */
    float            accel;           /* µstep/s^2, 0 = không ramp               */
    float            start_speed;     /* µstep/s, tốc độ bắt đầu/kết thúc        */
    float            speed_limit;     /* giới hạn phần cứng (F hoặc F/2)         */

    /* ---- Trạng thái dùng chung với ISR ---- */
    volatile int32_t  position;
    volatile int32_t  target;
    volatile uint8_t  mode;           /* tmc_motion_mode_t                        */
    volatile uint8_t  ramp_state;     /* tmc_ramp_state_t                         */
    volatile bool     running;
    volatile int8_t   dir;            /* +1 / -1                                  */
    int8_t            vel_dir;        /* chiều đích ở chế độ vận tốc              */
    bool              no_ramp;
    bool              step_high;
    uint32_t          phase;
    uint64_t          vel_q;          /* Q32.32: phần nguyên = phase inc / tick   */
    uint64_t          vmax_q;         /* tốc độ đích (position) hoặc |v| (velocity) */
    uint64_t          vmin_q;
    uint64_t          acc_q;          /* tăng vel_q mỗi tick                       */
    uint32_t          ramp_steps;     /* ~ quãng đường cần để phanh về start_speed */
} tmc_motion_t;

/** Khởi tạo. drv != NULL: tự ghi VACTUAL=0 và đồng bộ bit dedge với cfg. */
tmc_status_t tmc_motion_init(tmc_motion_t *m, const tmc_motion_cfg_t *cfg, tmc2209_t *drv);

/** Gọi trong ngắt timer, đúng tần số cfg.tick_hz. */
void         tmc_motion_tick(tmc_motion_t *m);

/* ---- Tham số mặc định ---------------------------------------------------- */
void         tmc_motion_set_max_speed(tmc_motion_t *m, float usteps_per_s);
void         tmc_motion_set_acceleration(tmc_motion_t *m, float usteps_per_s2);  /* 0 = không ramp */
void         tmc_motion_set_start_speed(tmc_motion_t *m, float usteps_per_s);

/* ---- 1) Điều khiển vị trí (dùng max_speed/accel mặc định) ----------------- */
void         tmc_motion_move_to(tmc_motion_t *m, int32_t target);
void         tmc_motion_move(tmc_motion_t *m, int32_t delta);

/* ---- 2) Điều khiển vị trí kèm vận tốc + ramp ------------------------------ */
void         tmc_motion_move_to_ex(tmc_motion_t *m, int32_t target, float max_speed, float accel);
void         tmc_motion_move_ex(tmc_motion_t *m, int32_t delta, float max_speed, float accel);

/* ---- 3) Điều khiển vận tốc (có dấu) + ramp -------------------------------- */
void         tmc_motion_run_velocity(tmc_motion_t *m, float usteps_per_s, float accel);

/* ---- Dừng ------------------------------------------------------------------ */
void         tmc_motion_stop(tmc_motion_t *m);           /* giảm tốc theo accel      */
void         tmc_motion_emergency_stop(tmc_motion_t *m); /* dừng ngay, có thể mất bước */

/* ---- Trạng thái ------------------------------------------------------------ */
bool         tmc_motion_is_running(const tmc_motion_t *m);
int32_t      tmc_motion_get_position(const tmc_motion_t *m);
int32_t      tmc_motion_distance_to_go(const tmc_motion_t *m);
float        tmc_motion_get_speed(const tmc_motion_t *m);   /* µstep/s, có dấu */
/** Đặt lại gốc toạ độ (homing). Chỉ có hiệu lực khi đang đứng yên. */
bool         tmc_motion_set_position(tmc_motion_t *m, int32_t position);

/* ---- Chuyển đổi đơn vị ------------------------------------------------------ */
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
