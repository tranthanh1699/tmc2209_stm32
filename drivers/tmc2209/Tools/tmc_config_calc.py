#!/usr/bin/env python3
"""
TMC2209 / STM32 configuration calculator.

Computes:
  - required motion timer frequency (tick_hz) for a target speed
  - PSC / ARR for the motion timer from the timer input clock
  - UART frame timings and a safe timeout
  - CPU/UART interaction limit (RX byte period vs. ISR time)

Examples:
  python3 tmc_config_calc.py --rpm 600 --microsteps 16 --timclk-mhz 84
  python3 tmc_config_calc.py --mm-s 100 --mm-per-rev 8 --microsteps 16 --timclk-mhz 72 --no-dedge
  python3 tmc_config_calc.py --rpm 300 --microsteps 32 --timclk-mhz 170 --baud 230400 --drivers 4
"""
import argparse
import math

STANDARD_TICKS = [10000, 20000, 25000, 40000, 50000, 100000]


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--rpm", type=float, help="max motor speed [rpm]")
    g.add_argument("--mm-s", type=float, help="max linear speed [mm/s] (needs --mm-per-rev)")
    p.add_argument("--mm-per-rev", type=float, default=8.0, help="lead / belt travel per motor rev [mm]")
    p.add_argument("--steps-rev", type=int, default=200, help="motor full steps per rev (200 = 1.8 deg)")
    p.add_argument("--microsteps", type=int, default=16, choices=[1, 2, 4, 8, 16, 32, 64, 128, 256])
    p.add_argument("--no-dedge", action="store_true", help="normal STEP pulses (no double edge)")
    p.add_argument("--margin", type=float, default=1.25, help="headroom on tick frequency")
    p.add_argument("--timclk-mhz", type=float, default=84.0, help="timer input clock [MHz] (Clock Configuration tab)")
    p.add_argument("--tick-hz", type=int, help="force a tick frequency instead of choosing one")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--senddelay", type=int, default=2, help="TMC2209 SENDDELAY (0..15)")
    p.add_argument("--drivers", type=int, default=1, help="drivers on the UART bus")
    p.add_argument("--axes", type=int, default=1, help="axes ticked by the motion ISR")
    a = p.parse_args()

    dedge = not a.no_dedge
    usteps_rev = a.steps_rev * a.microsteps
    rpm = a.rpm if a.rpm is not None else a.mm_s / a.mm_per_rev * 60.0
    v = rpm / 60.0 * usteps_rev                      # µsteps/s
    factor = 1.0 if dedge else 2.0
    need = v * factor * a.margin

    print("=== Motion (STEP/DIR) ===")
    print(f"  µsteps/rev            : {usteps_rev}")
    print(f"  max speed             : {rpm:.1f} rpm = {v:.0f} µsteps/s"
          + (f" = {a.mm_s:.1f} mm/s" if a.mm_s is not None else ""))
    print(f"  STEP mode             : {'double edge (speed limit ~ F)' if dedge else 'normal pulse (speed limit F/2)'}")
    print(f"  required tick (x{a.margin}) : >= {need:.0f} Hz")

    if a.tick_hz:
        tick = a.tick_hz
    else:
        tick = next((t for t in STANDARD_TICKS if t >= need), None)
        if tick is None:
            hint = "lower microsteps or use per-step timer scheduling" if dedge else \
                   "use double edge (halves the tick), lower microsteps, or per-step timer scheduling"
            print(f"  !! speed too high for a fixed-tick generator (> 100 kHz): {hint}.")
            tick = int(math.ceil(need / 1000.0) * 1000)

    if (not dedge) and tick > 50000:
        print("  hint: double edge would need only half this tick rate (less CPU load)")
    limit = tick * (0.999 if dedge else 0.5)
    print(f"  chosen tick_hz        : {tick} Hz  (period {1e6 / tick:.1f} µs = max pulse jitter)")
    print(f"  speed limit at tick   : {limit:.0f} µsteps/s = {limit / usteps_rev * 60:.0f} rpm")
    if a.mm_s is not None:
        print(f"                          = {limit / usteps_rev * a.mm_per_rev:.1f} mm/s")
    if not dedge:
        print(f"  STEP pulse width      : {1e6 / tick:.1f} µs (one tick)")

    print("\n=== Motion timer (basic timer, e.g. TIM6/TIM7) ===")
    clk = a.timclk_mhz * 1e6
    best = None
    for psc in range(0, 65536):
        div = clk / (psc + 1)
        arr_f = div / tick - 1
        arr = round(arr_f)
        if arr < 1 or arr > 65535:
            continue
        f = div / (arr + 1)
        err = abs(f - tick) / tick
        cand = (err, -arr, psc, arr, f)          # prefer exact, then larger ARR (finer)
        if best is None or cand < best:
            best = cand
        if err == 0 and div == 1e6:
            best = cand
            break
    err, _, psc, arr, f = best
    print(f"  timer input clock     : {a.timclk_mhz:g} MHz")
    print(f"  Prescaler (PSC)       : {psc}")
    print(f"  Counter Period (ARR)  : {arr}")
    print(f"  actual frequency      : {f:.3f} Hz (error {err * 100:.4f} %)")
    if err == 0:
        print(f"  tick_hz in code       : {tick}U")
    else:
        print(f"  tick_hz in code       : {round(f)}U  (use the ACTUAL frequency, not {tick})")

    print("\n=== UART bus ===")
    bit = 1.0 / a.baud
    byte_t = 10 * bit                                # 8N1
    sd_bits = 8 * (1 if a.senddelay <= 1 else (a.senddelay | 1))
    wr = 8 * byte_t
    rd = 12 * byte_t + sd_bits * bit
    timeout = max(2.0, math.ceil(3 * rd * 1000))
    print(f"  baud                  : {a.baud} (TMC2209 detects baud from each frame's sync)")
    print(f"  byte time             : {byte_t * 1e6:.1f} µs")
    print(f"  write frame (8 bytes) : {wr * 1e3:.2f} ms")
    print(f"  read  (4 + delay + 8) : {rd * 1e3:.2f} ms  (SENDDELAY={a.senddelay} -> {sd_bits} bit times)")
    print(f"  suggested timeout_ms  : {int(timeout)}  (>= 3 x read time, >= 2 ms for 1 ms HAL tick)")
    per_s = 1.0 / (rd + 0.001)
    print(f"  max reads/s (rough)   : ~{per_s:.0f} total, ~{per_s / a.drivers:.0f} per driver "
          f"({a.drivers} drivers, assumes ~1 ms HAL/software overhead per read)")

    print("\n=== Interrupt budget ===")
    print(f"  motion ISR period     : {1e6 / tick:.1f} µs, {a.axes} axis(es)")
    print(f"  UART RX byte period   : {byte_t * 1e6:.1f} µs")
    print(f"  -> keep worst-case motion ISR time well below {min(1e6 / tick, byte_t * 1e6) * 0.3:.1f} µs")
    print("     (30 % of the smaller value; measure with DWT_CYCCNT, see CONFIG_GUIDE.md)")
    if byte_t * 1e6 < 3 * (1e6 / tick) and a.baud > 115200:
        print("  !! high baud with a fast motion ISR: on MCUs without UART FIFO, a long motion ISR"
              " can cause RX overrun (TMC_ERR_UART). Prefer 115200 or enable the UART FIFO (G4/H7/L4+/U5).")


if __name__ == "__main__":
    main()
