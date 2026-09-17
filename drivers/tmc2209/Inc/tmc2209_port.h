/**
 * @file    tmc2209_port.h
 * @brief   Lớp port phần cứng cho TMC2209 trên STM32 HAL.
 *
 * "main.h" do CubeMX sinh ra đã include đúng stm32xxxx_hal.h cho mọi dòng
 * (F0/F1/F4/F7/G0/G4/H7/L4/U5...), nên thư viện không phụ thuộc dòng chip.
 * Có thể override các macro bên dưới trước khi include (ví dụ trong main.h).
 */
#ifndef TMC2209_PORT_H
#define TMC2209_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ---- Thời gian ------------------------------------------------------------ */
#ifndef TMC_GET_TICK_MS
#define TMC_GET_TICK_MS()            HAL_GetTick()
#endif
#ifndef TMC_DELAY_MS
#define TMC_DELAY_MS(ms)             HAL_Delay(ms)
#endif

/* ---- Critical section (lớp motion: main <-> timer ISR) -------------------- */
#ifndef TMC_CRITICAL_ENTER
#define TMC_CRITICAL_ENTER()         uint32_t _tmc_primask = __get_PRIMASK(); __disable_irq()
#endif
#ifndef TMC_CRITICAL_EXIT
#define TMC_CRITICAL_EXIT()          __set_PRIMASK(_tmc_primask)
#endif

/* ---- GPIO nhanh trong ISR (BSRR có trên các dòng STM32 hiện hành) --------- */
#ifndef TMC_PIN_HIGH
#define TMC_PIN_HIGH(port, pin)      ((port)->BSRR = (uint32_t)(pin))
#endif
#ifndef TMC_PIN_LOW
#define TMC_PIN_LOW(port, pin)       ((port)->BSRR = (uint32_t)(pin) << 16U)
#endif
#ifndef TMC_PIN_TOGGLE
#define TMC_PIN_TOGGLE(port, pin)    HAL_GPIO_TogglePin((port), (pin))
#endif

/* ---- Mặc định bus UART ---------------------------------------------------- */
#ifndef TMC_DEFAULT_TIMEOUT_MS
#define TMC_DEFAULT_TIMEOUT_MS       10U   /* 115200 baud: 12 byte ~1.1 ms     */
#endif
#ifndef TMC_DEFAULT_RETRIES
#define TMC_DEFAULT_RETRIES          3U
#endif

#ifdef __cplusplus
}
#endif
#endif /* TMC2209_PORT_H */
