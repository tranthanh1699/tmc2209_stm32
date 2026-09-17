# Configuration Guide: UART, Timer, GPIO, NVIC

This guide explains which STM32 peripherals to configure for the TMC2209 library and how to choose their parameters, especially the UART baud rate and the motion timer frequency.

The numbers in this guide come from `tools/tmc_config_calc.py`, which you can run for your own machine (see section 7).

---

## 0. TL;DR: recommended defaults

| Item | Recommended value | When to change it |
|---|---|---|
| UART baud | **115200**, 8N1 | Long cables or noise → lower it; many drivers polled often → 230400 |
| UART mode | **Asynchronous + TX/RX tied** (`TMC_UART_TX_RX`) | No RX pin available → TX-only |
| USART NVIC | **Enabled** (TX/RX mode) | Not needed for TX-only / half-duplex |
| `bus.timeout_ms` | 10 (library default) | Baud < 38400 → increase (see 2.4) |
| Motion timer | Basic timer TIM6/TIM7, **20 kHz** | From your maximum speed (see 3.1) |
| PSC / ARR | PSC = f_TIMclk[MHz] − 1, ARR = 49 | Other tick rates: ARR = 1 000 000 / tick − 1 |
| STEP mode | **Double edge** (`double_edge = true`) | Only with UART; standalone driver → normal pulse |
| Microsteps | **16** | The driver interpolates to 256 anyway |
| NVIC priority | Motion timer > USART > everything non-urgent | See section 5 |

---

## 1. What must be configured

```
                       ┌─────────────────────────────┐
                       │ Do you need exact position, │
                       │ ramps, or position counting?│
                       └───────┬───────────────┬─────┘
                            no │               │ yes
                               ▼               ▼
              ┌─────────────────────┐   ┌────────────────────────────────┐
              │ UART only (VACTUAL) │   │ STEP/DIR + motion timer        │
              │ USART               │   │ + UART (recommended)           │
              └─────────────────────┘   │ USART, TIMx, GPIO STEP/DIR/EN  │
                                        └────────────────────────────────┘
```

| Peripheral | UART-only velocity | STEP/DIR + UART | STEP/DIR only (standalone) |
|---|---|---|---|
| USART | ✔ | ✔ | – |
| Timer (TIM6/TIM7/any) | – | ✔ | ✔ |
| GPIO STEP, DIR | – | ✔ | ✔ |
| GPIO EN | optional | optional | ✔ (only way to enable the driver) |
| EXTI (DIAG, limit switch) | optional | optional | optional (limit switch) |

---

## 2. UART

### 2.1 Baud rate

The TMC2209 measures the baud rate from the sync nibble of every frame, so no baud setting is needed on the driver side.

| Baud | Byte time | Write frame | Read (req + SENDDELAY 2 + reply) | Use when |
|---|---|---|---|---|
| 9600 | 1.04 ms | 8.3 ms | ~15 ms | Very long / noisy wiring, software UART |
| 57600 | 174 µs | 1.39 ms | ~2.5 ms | Long wires |
| **115200** | **86.8 µs** | **0.69 ms** | **~1.25 ms** | **Default** |
| 230400 | 43.4 µs | 0.35 ms | ~0.63 ms | Many drivers polled frequently, short wires |
| 460800 | 21.7 µs | 0.17 ms | ~0.31 ms | Only with a UART FIFO and a short motion ISR (see 2.5) |

Higher baud rates barely help control performance. UART is only used for configuration and diagnostics; motion runs through STEP/DIR.

### 2.2 CubeMX steps

**Mode A: TX-only** (`TMC_UART_TX_ONLY`)

1. *Connectivity → USARTx → Mode:* **Asynchronous**
2. *Parameter Settings:* 115200, 8 bits, None, 1 stop
3. NVIC: not required
4. Wiring: MCU TX → 1 kΩ → PDN_UART

**Mode B: TX + RX tied** (`TMC_UART_TX_RX`, recommended)

1. *Mode:* **Asynchronous**
2. *Parameter Settings:* 115200, 8N1
3. *NVIC Settings:* **USARTx global interrupt = Enabled** (required, the library uses `HAL_UART_Receive_IT`)
4. *DMA:* none
5. Wiring: MCU TX → 1 kΩ → PDN_UART ← MCU RX (direct)

**Mode C: Half-duplex** (`TMC_UART_HALF_DUPLEX`)

1. *Mode:* **Single Wire (Half-Duplex)**
2. *Parameter Settings:* 115200, 8N1
3. *GPIO Settings:* TX pin **Alternate Function Open Drain** (ST reference manual requirement for single-wire mode), external pull-up to 3.3 V
4. NVIC: not required

**All modes:** if the project already has `HAL_UART_RxCpltCallback()` / `HAL_UART_ErrorCallback()` for other UARTs, filter them with `huart->Instance`.

### 2.3 Library bus settings

```c
tmc_bus_init(&bus, &huart2, TMC_UART_TX_RX);
bus.timeout_ms = 10;   /* default; see 2.4 */
bus.retries    = 3;    /* default: 1 try + 3 retries */
```

| Setting | Default | Notes |
|---|---|---|
| `timeout_ms` | 10 | Must be ≥ 3 × read time and ≥ 2 ms (HAL tick resolution is 1 ms) |
| `retries` | 3 | Worst-case blocking time ≈ `timeout_ms × (retries + 1)` + 1 ms per retry |
| SENDDELAY | 2 (set by `tmc2209_init` on bidirectional buses) | Recommended ≥ 2 with several addresses on the bus |

### 2.4 UART timing budget

```
Read at 115200, SENDDELAY = 2:
 MCU TX  |05|00|06|6F|                                   4 bytes  = 0.35 ms
 wait                  <-- 3 x 8 bit times -->           24 bits  = 0.21 ms
 TMC TX                                  |05|FF|06|..|CRC|  8 bytes = 0.69 ms
                                                          total ≈ 1.25 ms
```

Rules of thumb:

- **Timeout:** baud 9600 needs `timeout_ms ≥ 45`. Baud 115200 works with anything from 4 ms up (the default 10 ms is fine).
- **Polling rate:** about 400 reads/s in total at 115200, shared by all drivers on the bus. Status polling every 100–500 ms per driver is plenty.
- **Where to call:** UART functions are blocking. Call them from the main loop or an RTOS task, never from the motion ISR.

### 2.5 Interaction with the motion ISR

The motion timer usually has a higher priority than the USART. While it runs, the USART cannot empty its receive register.

- **Without a UART FIFO** (F1/F4 and similar), one byte arrives every byte-time (86.8 µs at 115200). If the motion ISR runs longer than that, the next byte overwrites the previous one and the library reports `TMC_ERR_UART`, then retries.
- **Rule:** keep the worst-case motion ISR time well below both the tick period and the UART byte time. The calculator prints this limit (30 % of the smaller value).
- **With a FIFO** (G0/G4/H7/L4+/U5 and others), you can enable it with `HAL_UARTEx_EnableFifoMode(&huartX)`, if your HAL provides it, to relax this rule.

---

## 3. Motion timer

### 3.1 Choosing the tick frequency

```
v_max [µsteps/s] = rpm_max / 60 × steps_per_rev × microsteps
                 = v_max[mm/s] / mm_per_rev × steps_per_rev × microsteps

tick_hz ≥ v_max × 1.25            (double edge)
tick_hz ≥ v_max × 2 × 1.25        (normal pulse)
```

The 1.25 factor leaves headroom. Pick the next value from the standard list **10 k, 20 k, 25 k, 40 k, 50 k, 100 k Hz**; all of them give an integer ARR with a 1 MHz timer clock.

Maximum motor speed for a 200-step motor with double edge:

| Microsteps | µsteps/rev | 10 kHz | 20 kHz | 40 kHz | 50 kHz |
|---|---|---|---|---|---|
| 4 | 800 | 750 rpm | 1500 rpm | 3000 rpm | 3750 rpm |
| 8 | 1600 | 375 rpm | 750 rpm | 1500 rpm | 1875 rpm |
| **16** | **3200** | 187 rpm | **375 rpm** | 750 rpm | 937 rpm |
| 32 | 6400 | 94 rpm | 187 rpm | 375 rpm | 469 rpm |
| 64 | 12800 | 47 rpm | 94 rpm | 187 rpm | 234 rpm |

With normal pulses (no double edge), halve these values. In practice, the motor torque at high speed (supply voltage, inductance) often limits speed before the timer does.

**Trade-offs of a higher tick rate:**

| | Higher tick | Lower tick |
|---|---|---|
| Top speed | ✔ higher | lower |
| Pulse jitter (= 1 tick) | ✔ smaller | larger (100 µs at 10 kHz) |
| CPU load | higher | ✔ lower |
| Headroom for the UART RX interrupt | less | ✔ more |

Choose the **lowest tick that covers your top speed**. The TMC2209 interpolates to 256 microsteps, so 50–100 µs of jitter is normally not audible or visible.

### 3.2 Timer clock → PSC / ARR

```
f_tick = f_TIMclk / ((PSC + 1) × (ARR + 1))
Recommended: PSC = f_TIMclk[MHz] − 1  →  1 MHz counter clock
             ARR = 1 000 000 / tick_hz − 1
```

**f_TIMclk is not the APB frequency.** Read it in CubeMX → *Clock Configuration*. On most families, if the APB prescaler is not 1, the timer clock is 2 × PCLK. Some families (for example H7 with the TIMPRE bit) have their own rules.

| f_TIMclk | PSC | ARR @ 10 kHz | ARR @ 20 kHz | ARR @ 25 kHz | ARR @ 40 kHz | ARR @ 50 kHz | ARR @ 100 kHz |
|---|---|---|---|---|---|---|---|
| 24 MHz | 23 | 99 | 49 | 39 | 24 | 19 | 9 |
| 48 MHz | 47 | 99 | 49 | 39 | 24 | 19 | 9 |
| 72 MHz | 71 | 99 | 49 | 39 | 24 | 19 | 9 |
| 84 MHz | 83 | 99 | 49 | 39 | 24 | 19 | 9 |
| 100 MHz | 99 | 99 | 49 | 39 | 24 | 19 | 9 |
| 170 MHz | 169 | 99 | 49 | 39 | 24 | 19 | 9 |
| 240 MHz | 239 | 99 | 49 | 39 | 24 | 19 | 9 |

If the timer clock is not an integer number of MHz, run the calculator. Always put the **actual** frequency into `tick_hz`; otherwise all speeds and accelerations scale by the error.

### 3.3 CubeMX steps

1. *Timers → TIM6* (or TIM7): **Activated**.
   - Some parts have no TIM6/TIM7 (e.g. STM32F103C8). Use any free general-purpose timer (TIM2/3/4) with *Clock Source = Internal Clock* and the same settings.
2. *Parameter Settings:*
   - Prescaler = from table 3.2
   - Counter Mode = Up
   - Counter Period = from table 3.2
   - auto-reload preload = Enable
3. *NVIC Settings:* **TIMx global interrupt = Enabled**.
   - The IRQ name varies by family (`TIM6_DAC_IRQn`, `TIM6_IRQn`...); check `stm32xxxx.h`.
4. *SYS → Timebase Source:* do **not** use the motion timer.
   - With FreeRTOS, CubeMX asks for a timebase other than SysTick. Pick a different timer (e.g. TIM1/TIM2), not the one used for motion.
5. The timer is **not started** by generated code. Call `HAL_TIM_Base_Start_IT(&htimX)` after `tmc_motion_init()`.

### 3.4 Hooking the ISR

**Option 1: HAL callback** (simple)

```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        tmc_motion_tick(&axis_x);
        tmc_motion_tick(&axis_y);
    }
    /* if the HAL timebase is a TIMx, CubeMX already generated this function:
       add the branch above to it instead of defining it twice */
}
```

**Option 2: direct IRQ handler** (lower overhead, for ≥ 40 kHz or many axes)

1. CubeMX → *System Core → NVIC → Code generation* tab: **untick "Generate IRQ handler"** for the motion timer.
2. Write the handler yourself:

```c
void TIM6_DAC_IRQHandler(void)          /* name from the vector table */
{
    if ((TIM6->SR & TIM_SR_UIF) != 0U) {
        TIM6->SR = ~TIM_SR_UIF;          /* SR bits are rc_w0: write 0 to clear */
        tmc_motion_tick(&axis_x);
        tmc_motion_tick(&axis_y);
    }
}
```

If the DAC shares this vector and you use it, keep calling its handler as well.

### 3.5 Measuring the ISR time

Cortex-M3/M4/M7/M33 have the DWT cycle counter:

```c
static uint32_t isr_max_cycles;

void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

/* inside the motion ISR */
uint32_t c0 = DWT->CYCCNT;
tmc_motion_tick(&axis_x);
tmc_motion_tick(&axis_y);
uint32_t dc = DWT->CYCCNT - c0;
if (dc > isr_max_cycles) isr_max_cycles = dc;

/* in the main loop, while all axes run at full speed */
float isr_max_us = (float)isr_max_cycles * 1e6f / (float)SystemCoreClock;
```

Cortex-M0/M0+ have no cycle counter. Instead, set a spare GPIO high at ISR entry and low at exit, then measure the pulse with a scope.

**Target:** worst-case ISR time < 30 % of `min(tick period, UART byte time)`. If it is higher, lower the tick rate, use option 2 in 3.4, or reduce the number of axes per timer.

---

## 4. GPIO

| Pin | Mode | Pull | Speed | Initial level | User label (example) |
|---|---|---|---|---|---|
| STEP | Output push-pull | none | High | Low | `X_STEP` |
| DIR | Output push-pull | none | Low | Low | `X_DIR` |
| EN | Output push-pull | none | Low | **High** (driver off) | `X_EN` |
| DIAG (optional) | GPIO_EXTI, rising edge | pull-down | – | – | `X_DIAG` |
| Limit switch (optional) | GPIO_EXTI, falling edge | pull-up | – | – | `X_MIN` |

- User labels make CubeMX generate `X_STEP_GPIO_Port` / `X_STEP_Pin`, which the examples use directly.
- The library writes STEP/DIR through `BSRR`, which is atomic, so several axes can share one GPIO port.
- If the MCU runs at 3.3 V, make sure the driver board's VCC_IO is 3.3 V too, so the logic levels match.

---

## 5. NVIC priorities

### Bare metal (4 priority bits)

| Interrupt | Preemption priority | Reason |
|---|---|---|
| Emergency stop EXTI | 0 | Must pre-empt everything |
| **Motion timer** | **1** | Pulse timing |
| **USART (TMC bus)** | **2** | Must empty the RX register within one byte time |
| DIAG / limit-switch EXTI | 3 | Calls `tmc_motion_*` (safe at any priority) |
| SysTick (HAL tick) | CubeMX default | Used for UART timeouts only |
| Others (debug UART, ADC...) | ≥ 4 | |

### With FreeRTOS

- ISRs with a numerically lower priority than `configMAX_SYSCALL_INTERRUPT_PRIORITY` (CubeMX default: 5) must not call FreeRTOS APIs.
  - The motion ISR and the TMC USART ISR do not call FreeRTOS APIs, so they can stay at 1 and 2 (zero-latency).
  - If an EXTI callback notifies a task (`xTaskNotifyFromISR`), give that EXTI a priority ≥ 5.
- Protect the UART bus with a mutex: `tmc_bus_set_lock()` (see `example_main.c`).

---

## 6. Worked examples

### A. Lead-screw axis on STM32F103C8 (72 MHz)

T8 lead screw (8 mm/rev), 16 µsteps, 10 mm/s maximum, 1 driver.

```
python3 tmc_config_calc.py --mm-s 10 --mm-per-rev 8 --microsteps 16 --timclk-mhz 72
```

| Result | Value |
|---|---|
| v_max | 4000 µsteps/s (75 rpm) |
| tick_hz | **10 000 Hz** (limit 25 mm/s) |
| Timer | TIM2/3/4 (no TIM6 on C8), PSC = 71, ARR = 99 |
| UART | 115200, timeout 10 ms |
| ISR budget | < 26 µs |

### B. Belt axes on STM32F411 (100 MHz), 2 axes

GT2 20T pulley (40 mm/rev), 16 µsteps, 200 mm/s maximum, 2 drivers.

```
python3 tmc_config_calc.py --mm-s 200 --mm-per-rev 40 --microsteps 16 --timclk-mhz 100 --drivers 2 --axes 2
```

| Result | Value |
|---|---|
| v_max | 16 000 µsteps/s (300 rpm) |
| tick_hz | **20 000 Hz** (limit 249.8 mm/s) |
| Timer | TIM2..TIM5 (the F411 has no TIM6/TIM7), PSC = 99, ARR = 49 |
| UART | 115200, ~222 reads/s per driver available |
| ISR budget (both axes) | < 15 µs |

### C. Three robot joints on STM32G431 (170 MHz)

16 µsteps, 600 rpm motor maximum (before the gearbox), 3 drivers on one bus.

```
python3 tmc_config_calc.py --rpm 600 --microsteps 16 --timclk-mhz 170 --drivers 3 --axes 3
```

| Result | Value |
|---|---|
| v_max | 32 000 µsteps/s |
| tick_hz | **40 000 Hz** (limit 749 rpm) |
| Timer | TIM6, PSC = 169, ARR = 24, direct IRQ handler (3.4 option 2) |
| UART | 115200 (FIFO available on G4) |
| ISR budget (3 axes) | < 7.5 µs |

At 32 µsteps the same machine would need 100 kHz and a 3 µs ISR budget. That is why 16 µsteps is the better choice here.

---

## 7. Calculator tool

```
python3 tools/tmc_config_calc.py --help

# typical
python3 tools/tmc_config_calc.py --rpm 300 --microsteps 16 --timclk-mhz 84
python3 tools/tmc_config_calc.py --mm-s 100 --mm-per-rev 8 --timclk-mhz 72 --no-dedge
python3 tools/tmc_config_calc.py --rpm 600 --timclk-mhz 170 --baud 230400 --drivers 4 --axes 3
```

| Option | Meaning |
|---|---|
| `--rpm` / `--mm-s` | Maximum speed (motor rpm or linear mm/s) |
| `--mm-per-rev` | Travel per motor revolution (lead, belt pitch × teeth) |
| `--steps-rev` | Motor full steps per revolution (200 for 1.8°) |
| `--microsteps` | 1..256 |
| `--no-dedge` | Normal STEP pulses instead of double edge |
| `--timclk-mhz` | Timer input clock from CubeMX |
| `--tick-hz` | Force a tick rate |
| `--baud`, `--senddelay`, `--drivers`, `--axes` | UART and ISR budget inputs |

---

## 8. Verification checklist

| # | Check | How | Expected |
|---|---|---|---|
| 1 | Timer frequency | Toggle a spare pin in the ISR, measure with a scope | Square wave at tick_hz / 2 |
| 2 | UART alive | `tmc2209_scan_bus()` | Bitmask matches the MS1/MS2 addresses |
| 3 | Writes accepted | `IFCNT` before/after one write | +1 |
| 4 | Error counter | `bus.err_count` while motors run at full speed | Stays at 0 (or very rare) |
| 5 | Step rate | Scope on STEP at a known speed | Double edge: square wave at v/2 Hz. Normal: pulses at v Hz |
| 6 | Distance | `move(steps_per_rev × microsteps × 10)` | Exactly 10 revolutions |
| 7 | ISR time | DWT measurement (3.5) at full speed, all axes | Below the calculator budget |
| 8 | Settings | `tmc2209_get_settings()` | Microsteps, current and chopper mode as configured |

---

## 9. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `tmc2209_init` → `TMC_ERR_NOT_READY` | VM not powered, wrong address, USART NVIC disabled | Power VM first; check MS1/MS2; enable the USART interrupt |
| `TMC_ERR_ECHO` | TX/RX not tied, missing 1 kΩ, two masters on the line | Check wiring (section 2.2) |
| `TMC_ERR_UART` during motion | Motion ISR too long → RX overrun | Lower tick rate or baud, use a direct IRQ handler, enable the UART FIFO |
| `TMC_ERR_TIMEOUT` at low baud | Timeout shorter than the read time | Increase `bus.timeout_ms` (section 2.4) |
| Motor moves half or double the distance | `double_edge` does not match the driver's `dedge` bit, or wrong microsteps | Pass the driver to `tmc_motion_init()`; check `get_microsteps_per_step()` |
| All speeds wrong by a constant factor | `tick_hz` does not match the real timer frequency | Measure (check 1); use the actual value |
| Motor does not move, UART is fine | VACTUAL ≠ 0 (STEP pin ignored), driver disabled, EN pin high | `tmc2209_move_using_step_dir_interface()`, `tmc2209_enable()` |
| Motor stalls at high speed | Torque limit, not the timer | Lower speed or acceleration, increase current/voltage, SpreadCycle above a threshold (TPWMTHRS) |
| Configuration lost at runtime | Driver lost VS (brown-out) | Poll `GSTAT.reset`, call `tmc2209_restore_to_chip()`, re-home |
