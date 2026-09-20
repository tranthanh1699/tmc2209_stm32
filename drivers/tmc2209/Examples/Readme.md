# ADC Tracker Driver for STM32

A small, flat (one `.h` + one `.c`) STM32 HAL driver that samples one analog
channel at a fixed rate using **timer-triggered ADC + circular DMA**, and turns
it into:

- a filtered **angle** (deg) and **angular velocity** (deg/s) of the input, and
- a **position / velocity setpoint** for an output axis (motor), using
  independent **position gain** and **velocity gain**.

Typical use: a rotary potentiometer or any analog angle sensor used as a
master input, driving a motor that follows it with scaled position and speed.

---

## Features

- Hardware-timed sampling (no jitter). The sample rate is derived from the
  timer counter clock you pass to `ADCTRK_Init()`.
- Circular DMA with half/full-transfer processing. The CPU only wakes once per
  block of samples.
- Block averaging (oversampling), then an **alpha-beta tracker**. This gives
  position and velocity estimates without noisy raw differentiation.
- Position hysteresis and velocity deadband to suppress jitter at rest.
- Output mapping:
  `target = ref + pos_gain * Δangle`, `speed = vel_gain * |velocity|`.
- Built-in setpoint generator with speed limit, optional acceleration limit
  and braking curve (no overshoot).
- Runtime gain change without output jump, re-zeroing, and re-calibration.
- D-cache maintenance for Cortex-M7 (F7/H7) handled internally.
- Multiple instances (one per ADC handle).
- Hardware-independent processing entry point (`ADCTRK_ProcessSample`) for
  unit tests on a PC.

---

## Files

| File            | Content                                    |
|-----------------|--------------------------------------------|
| `adc_tracker.h` | Types, configuration, public API           |
| `adc_tracker.c` | Implementation (HAL glue + processing)     |

Add both files to your CubeMX project (e.g. `Core/Inc`, `Core/Src`). The
driver includes `main.h`, so it works with any STM32 HAL family that has
`HAL_ADC_Start_DMA` and `HAL_TIMEx_MasterConfigSynchronization`.

---

## Processing chain

```
 Analog input ──► ADC ◄── TIM TRGO (sample_rate_hz)
                   │
                   ▼   DMA circular, buffer = 2 × avg_samples
        ┌──────────┴──────────┐
   Half-transfer IRQ     Full-transfer IRQ        every dt = avg_samples / fs
        └──────────┬──────────┘
                   ▼
   1. Average N samples            raw_avg
   2. Scale                        angle = (raw - raw_min)/(raw_max - raw_min) · full_scale_deg
   3. Alpha-beta tracker           in_angle_deg, in_velocity_dps
   4. Deadband / hysteresis
   5. Position mapping             out_target_deg = out_ref + pos_gain · (angle - in_ref)
   6. Velocity mapping             out_vel_limit  = clamp(|vel_gain · velocity|, v_min, v_max)
   7. Setpoint generator           out_setpoint_deg, out_setpoint_vel_dps
                   ▼
   ADCTRK_GetData()  ──►  application / motor driver
```

---

## CubeMX configuration

The driver does **not** re-initialise GPIO, ADC or DMA. Configure them in
CubeMX as follows:

| Peripheral | Setting | Value |
|---|---|---|
| GPIO | Pin mode | Analog |
| TIM  | Counter clock (after prescaler) | Any. Pass it as `timer_clk_hz` (e.g. 20 MHz) |
| TIM  | Period (ARR) / TRGO | Set by the driver. No timer interrupt needed |
| ADC  | Regular channels | 1 (the analog input) |
| ADC  | Sampling time | Long. High-impedance sources need it |
| ADC  | Continuous Conversion Mode | **Disable** |
| ADC  | External Trigger Conversion Source | **Timer x Trigger Out event** |
| ADC  | External Trigger Edge | Rising |
| ADC  | F4 / F7 / G4 / L4 | DMA Continuous Requests = **Enable** |
| ADC  | H7 | Conversion Data Management = **DMA Circular Mode** |
| DMA  | Mode / Data width | **Circular** / Half Word (both sides), memory increment |
| NVIC | DMA stream interrupt | Enabled |

`ADCTRK_Init()` checks the most common mistakes (continuous mode, software
trigger, non-circular DMA) and returns an error code instead of running
silently wrong.

> **STM32H7:** DMA1/DMA2 cannot access DTCM RAM (`0x20000000`). Place the
> `ADCTRK_HandleTypeDef` object in AXI SRAM or SRAM1-3 via your linker script
> or a section attribute. Cache invalidation is done by the driver.

> **ADC calibration:** call `HAL_ADCEx_Calibration_Start(...)` yourself before
> `ADCTRK_Start()` if your family supports it. The signature differs between
> families, so the driver does not call it.

> **Hardware tip:** a 100 nF capacitor from the ADC pin to GND close to the MCU
> reduces noise and helps the ADC sample capacitor charge.

---

## Timing

| Quantity | Formula | Example (20 MHz, 16 kHz, N = 16) |
|---|---|---|
| Timer period | `ARR = round(timer_clk_hz / sample_rate_hz) - 1` | 1249 |
| Actual sample rate | `fs = timer_clk_hz / (ARR + 1)` | 16 000 Hz |
| Update period | `dt = avg_samples / fs` | 1.0 ms |

`ADCTRK_GetSampleRate()` and `ADCTRK_GetUpdatePeriod()` return the actual
values after ARR rounding.

---

## Configuration reference (`ADCTRK_ConfigTypeDef`)

| Field | Default | Unit | Description |
|---|---|---|---|
| `sample_rate_hz` | 16000 | Hz | ADC trigger rate |
| `avg_samples` | 16 | - | Samples averaged per update (≤ `ADCTRK_MAX_AVG_SAMPLES`) |
| `raw_min` | 0 | counts | ADC value at 0° (mechanical end) |
| `raw_max` | 4095 | counts | ADC value at full scale. Use 65535 for 16-bit ADC. May be `< raw_min` for a reversed sensor |
| `full_scale_deg` | 270 | deg | Mechanical range of the sensor |
| `filter_alpha` | 0.05 | - | Tracker position gain, 0 < α < 1. Higher = faster, noisier |
| `filter_beta` | 0 (auto) | - | Tracker velocity gain. ≤ 0 selects `α² / (2 − α)` |
| `pos_deadband_deg` | 0.2 | deg | Position hysteresis |
| `vel_deadband_dps` | 2.0 | deg/s | Input speed below this counts as stopped |
| `pos_gain` | 1.0 | - | Output angle = `pos_gain` × input angle. Negative = reverse |
| `vel_gain` | 1.0 | - | Output speed = `vel_gain` × input speed |
| `out_vel_min_dps` | 5 | deg/s | Minimum output speed (must be > 0) |
| `out_vel_max_dps` | 720 | deg/s | Maximum output speed |
| `out_acc_max_dps2` | 0 | deg/s² | Acceleration limit. ≤ 0 disables it |
| `out_pos_min_deg` / `out_pos_max_deg` | 0 / 0 | deg | Output soft limits. Disabled when `min >= max` |

Build-time options (define before including, or in compiler flags):

| Macro | Default | Description |
|---|---|---|
| `ADCTRK_MAX_AVG_SAMPLES` | 32 | DMA half-buffer capacity |
| `ADCTRK_MAX_INSTANCES` | 2 | Number of driver instances |
| `ADCTRK_USE_HAL_CALLBACKS` | 1 | 1: driver defines the HAL ADC DMA callbacks. 0: you forward them |

---

## Output data (`ADCTRK_DataTypeDef`)

| Field | Unit | Description |
|---|---|---|
| `raw_avg` | counts | Averaged ADC value (useful for calibration) |
| `in_angle_deg` | deg | Filtered input angle |
| `in_velocity_dps` | deg/s | Filtered input angular velocity (signed) |
| `out_target_deg` | deg | Final output target |
| `out_vel_limit_dps` | deg/s | Speed the output is allowed to move at |
| `out_setpoint_deg` | deg | Output position setpoint for this update |
| `out_setpoint_vel_dps` | deg/s | Output velocity setpoint (signed) |
| `seq` | - | Update counter |

---

## API reference

| Function | Description |
|---|---|
| `ADCTRK_GetDefaultConfig(cfg)` | Fill `cfg` with the defaults above |
| `ADCTRK_Init(h, hadc, htim, timer_clk_hz, cfg)` | Validate config, program ARR + TRGO, register instance |
| `ADCTRK_Start(h)` | Start ADC DMA and timer |
| `ADCTRK_Stop(h)` | Stop timer and ADC DMA |
| `ADCTRK_GetData(h, out)` | Copy latest data. Returns 1 if new since last call |
| `ADCTRK_PeekData(h, out)` | Copy latest data without clearing the new flag |
| `ADCTRK_SetGains(h, pos_gain, vel_gain)` | Change gains at runtime, no output jump |
| `ADCTRK_SetZero(h, out_pos_deg)` | Current input position ⇔ `out_pos_deg` becomes the reference pair |
| `ADCTRK_SetCalibration(h, raw_min, raw_max, full_scale_deg)` | Update input scaling |
| `ADCTRK_GetSampleRate(h)` | Actual ADC sample rate [Hz] |
| `ADCTRK_GetUpdatePeriod(h)` | Update period `dt` [s] |
| `ADCTRK_ProcessSample(h, raw_avg)` | Feed one averaged sample manually (tests, non-DMA use) |
| `ADCTRK_ConvHalfCpltHandler(hadc)` / `ADCTRK_ConvCpltHandler(hadc)` | Forward HAL callbacks when `ADCTRK_USE_HAL_CALLBACKS == 0` |

Return codes (`ADCTRK_StatusTypeDef`):

| Code | Cause |
|---|---|
| `ADCTRK_OK` | Success |
| `ADCTRK_ERR_PARAM` | NULL pointer or invalid config value |
| `ADCTRK_ERR_TIMER` | ARR out of range for the timer width, or TRGO setup failed |
| `ADCTRK_ERR_ADC_CONFIG` | ADC in continuous mode or software trigger |
| `ADCTRK_ERR_DMA_CONFIG` | ADC DMA not in circular mode |
| `ADCTRK_ERR_NO_SLOT` | More than `ADCTRK_MAX_INSTANCES` instances |
| `ADCTRK_ERR_HAL` | `HAL_ADC_Start_DMA` / `HAL_TIM_Base_Start` failed |

---

## Examples

In the examples, `Motor_*` functions stand for **your own** motor driver. They
are not part of this library.

### 1. Basic usage

```c
#include "adc_tracker.h"

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;          /* counter clock = 20 MHz */

static ADCTRK_HandleTypeDef tracker;     /* H7: place in AXI SRAM */

void App_Init(void)
{
    ADCTRK_ConfigTypeDef cfg;
    ADCTRK_GetDefaultConfig(&cfg);

    cfg.sample_rate_hz = 16000u;         /* 16 kHz ADC sampling          */
    cfg.avg_samples    = 16u;            /* -> 1 kHz update rate         */
    cfg.raw_min        = 40.0f;          /* measured at mechanical ends  */
    cfg.raw_max        = 4050.0f;
    cfg.full_scale_deg = 270.0f;

    cfg.pos_gain = 2.0f;                 /* input turns a -> output 2a   */
    cfg.vel_gain = 2.0f;                 /* output speed = 2 x input     */

    if (ADCTRK_Init(&tracker, &hadc1, &htim3, 20000000u, &cfg) != ADCTRK_OK) {
        Error_Handler();
    }
    if (ADCTRK_Start(&tracker) != ADCTRK_OK) {
        Error_Handler();
    }
}

void App_Loop(void)
{
    ADCTRK_DataTypeDef d;

    if (ADCTRK_GetData(&tracker, &d)) {                 /* new every 1 ms */
        Motor_SetPosition(d.out_setpoint_deg, d.out_setpoint_vel_dps);
    }
}
```

### 2. Stepper motor output

Convert the output setpoint (degrees) into an absolute step target.

```c
#define STEPS_PER_REV   (200.0f * 16.0f)          /* 1.8° motor, 1/16 microstep */
#define STEPS_PER_DEG   (STEPS_PER_REV / 360.0f)

void App_Loop(void)
{
    ADCTRK_DataTypeDef d;

    if (ADCTRK_GetData(&tracker, &d)) {
        int32_t  step_target = (int32_t)lroundf(d.out_setpoint_deg * STEPS_PER_DEG);
        float    step_rate   = fabsf(d.out_setpoint_vel_dps) * STEPS_PER_DEG;   /* steps/s */

        Motor_StepTo(step_target, step_rate);      /* your step/dir generator */
    }
}
```

### 3. Servo motor with its own position loop

Use `out_setpoint_deg` as the position reference and `out_setpoint_vel_dps`
as velocity feed-forward.

```c
if (ADCTRK_GetData(&tracker, &d)) {
    float pos_meas = Encoder_GetAngleDeg();                 /* your encoder */
    float u = Kp * (d.out_setpoint_deg - pos_meas)
            + Kff * d.out_setpoint_vel_dps;                 /* feed-forward */
    Motor_SetVoltage(u);
}
```

### 4. Acceleration limit and soft limits

```c
cfg.out_vel_max_dps  = 360.0f;     /* max 1 rev/s                 */
cfg.out_acc_max_dps2 = 2000.0f;    /* smooth start/stop           */
cfg.out_pos_min_deg  = -180.0f;    /* output never leaves ±180°   */
cfg.out_pos_max_deg  =  180.0f;
```

### 5. Change gains at runtime

The output continues from where it is and does not jump.

```c
ADCTRK_SetGains(&tracker, 0.5f, 0.5f);    /* fine mode   */
ADCTRK_SetGains(&tracker, 4.0f, 4.0f);    /* coarse mode */
```

### 6. Re-zero after homing

```c
Motor_Home();                              /* output axis now at 0° */
ADCTRK_SetZero(&tracker, 0.0f);            /* current input <=> output 0° */
```

### 7. Calibration teach routine

Turn the input to each mechanical end and capture `raw_avg`.

```c
float Capture_Raw(void)
{
    ADCTRK_DataTypeDef d;
    float sum = 0.0f;
    for (int i = 0; i < 100; i++) {                 /* ~100 ms at 1 kHz */
        while (!ADCTRK_GetData(&tracker, &d)) { }
        sum += d.raw_avg;
    }
    return sum / 100.0f;
}

void Calibrate(void)
{
    UI_Prompt("Turn to MIN end");   Wait_Button();  float rmin = Capture_Raw();
    UI_Prompt("Turn to MAX end");   Wait_Button();  float rmax = Capture_Raw();
    ADCTRK_SetCalibration(&tracker, rmin, rmax, 270.0f);
}
```

### 8. Application already owns the HAL ADC callbacks

Compile with `ADCTRK_USE_HAL_CALLBACKS=0` and forward the callbacks:

```c
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    ADCTRK_ConvHalfCpltHandler(hadc);      /* ignored if hadc is not a tracker */
    /* ... your other ADC handling ... */
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    ADCTRK_ConvCpltHandler(hadc);
    /* ... */
}
```

### 9. Two inputs on two ADCs

```c
static ADCTRK_HandleTypeDef trk_x, trk_y;

ADCTRK_Init(&trk_x, &hadc1, &htim3, 20000000u, &cfg_x);
ADCTRK_Init(&trk_y, &hadc2, &htim3, 20000000u, &cfg_y);   /* same trigger timer */
ADCTRK_Start(&trk_x);
ADCTRK_Start(&trk_y);
```

When two instances share one timer, give both the same `sample_rate_hz`. The
last `ADCTRK_Init()` call programs the ARR.

### 10. Unit test on a PC (no hardware)

`ADCTRK_ProcessSample()` runs the full processing chain. You can feed it from
a host test with a stub `main.h`:

```c
for (int k = 0; k < 1000; k++) {
    float raw = Simulated_Raw(k);                 /* your test signal */
    ADCTRK_ProcessSample(&tracker, raw);
}
```

---

## Behaviour notes

**pos_gain vs vel_gain.** Position and velocity gains are independent:

| Case | Behaviour |
|---|---|
| `vel_gain == pos_gain` | Output follows the input naturally and arrives at the same time |
| `vel_gain > pos_gain` | Output is faster than needed and is limited by the target (catches up and waits) |
| `vel_gain < pos_gain` | Output lags. After the input stops, the output finishes the move at the **peak speed of that move** instead of crawling at `out_vel_min_dps` |

Simulation, with the input moving 0 → 90° at 180°/s and ±3 LSB noise:

| pos_gain / vel_gain | Measured input speed | Output |
|---|---|---|
| 2.0 / 2.0 | ≈ 180 °/s | ≈ 360 °/s, stops at 180° together with the input |
| 2.0 / 0.5 | ≈ 180 °/s | ≈ 90 °/s, continues after the input stops, reaches 180° |

**Start-up.** The first sample after `ADCTRK_Start()` is used as the
reference, so the output does not jump. The output reference starts at 0°. Call
`ADCTRK_SetZero()` to align it with the real axis position.

**ISR load.** Processing runs in the DMA interrupt once per update (default
1 kHz). It uses single-precision float, which is intended for FPU cores
(M4F/M7/M33).

---

## Tuning guide

| Symptom | Adjust |
|---|---|
| Output jitters at rest | Increase `pos_deadband_deg` or `avg_samples`, or add an RC filter at the pin |
| Velocity reading noisy | Decrease `filter_alpha` (e.g. 0.02) |
| Response feels laggy | Increase `filter_alpha` (e.g. 0.1–0.2) or the update rate |
| Output creeps after stop | Increase `vel_deadband_dps` |
| Jerky start/stop | Set `out_acc_max_dps2` |
| Angle not reaching 0 / full scale | Recalibrate `raw_min` / `raw_max` |
| `data_missed` keeps increasing | Main loop is slower than the update rate. Read faster or increase `avg_samples` |

---

## Troubleshooting

| Problem | Check |
|---|---|
| `ADCTRK_Init` returns `ADCTRK_ERR_ADC_CONFIG` | ADC continuous mode is on, or the trigger is still software start |
| `ADCTRK_Init` returns `ADCTRK_ERR_DMA_CONFIG` | DMA mode is Normal instead of Circular |
| `ADCTRK_Init` returns `ADCTRK_ERR_TIMER` | Sample rate too low for a 16-bit timer. Raise the rate or prescale the timer |
| No updates at all | ADC trigger source is not this timer's TRGO, or the DMA IRQ is disabled |
| DMA runs only once (F4/G4) | DMA Continuous Requests is disabled |
| Values frozen / stale on H7 | Handle placed in DTCM, or the MPU/cache configuration is wrong |
| Linker error: multiple `HAL_ADC_ConvCpltCallback` | Set `ADCTRK_USE_HAL_CALLBACKS=0` and forward the callbacks (example 8) |