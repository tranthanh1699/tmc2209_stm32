/**
 * @file    example_main.c
 * @brief   Integration example for a CubeMX-generated main.c.
 *          Copy only the snippets inside the matching USER CODE BEGIN/END blocks.
 *
 * Assumed CubeMX configuration:
 *   - USART2 : 115200 8N1, Asynchronous, NVIC USART2 global interrupt = ON
 *              TX to PDN_UART through 1k, RX directly to PDN_UART   (TMC_UART_TX_RX)
 *   - TIM6   : 20 kHz, NVIC TIM6 global interrupt = ON
 *   - GPIO   : X_STEP, X_DIR, X_EN, Y_STEP, Y_DIR, Y_EN  (Output push-pull)
 *              X_EN, Y_EN initial level = HIGH (driver disabled)
 *   - Driver X: MS1=0, MS2=0 -> address 0
 *     Driver Y: MS1=1, MS2=0 -> address 1
 *     Driver Z: MS1=0, MS2=1 -> address 2 (no STEP/DIR wiring, VACTUAL only)
 */

/* USER CODE BEGIN Includes */
#include "tmc2209.h"
#include "tmc2209_motion.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
#define MICROSTEPS        16U
#define FULL_STEPS_REV    200U
#define USTEPS_PER_REV    (FULL_STEPS_REV * MICROSTEPS)      /* 3200 */
#define USTEPS_PER_MM     (USTEPS_PER_REV / 8.0f)            /* T8 lead screw: 8 mm/rev */

static tmc_bus_t    tmc_bus;
static tmc2209_t    drv_x, drv_y, drv_z;
static tmc_motion_t axis_x, axis_y;
/* USER CODE END PV */

/* USER CODE BEGIN 0 */
static void driver_setup(tmc2209_t *d, uint8_t addr, GPIO_TypeDef *en_port, uint16_t en_pin)
{
    tmc_status_t st = tmc2209_init(d, &tmc_bus, addr);
    if (st != TMC_OK) {
        /* Most common causes: VM/VS not powered, wrong MS1/MS2 address,
         * or USART NVIC interrupt not enabled. */
        Error_Handler();
    }
    tmc2209_set_hardware_enable_pin(d, en_port, en_pin);   /* NULL if unused */
    tmc2209_set_microsteps_per_step(d, MICROSTEPS);
    tmc2209_set_run_current(d, 60);        /* % */
    tmc2209_set_hold_current(d, 30);       /* % */
    tmc2209_set_hold_delay(d, 50);         /* % */
    tmc2209_enable_automatic_current_scaling(d);   /* StealthChop auto-tuning */
    tmc2209_enable_automatic_gradient_adaptation(d);
    tmc2209_enable_stealth_chop(d);
    tmc2209_enable(d);
}

/* If CubeMX uses a TIMx as the HAL timebase, this callback already exists in
 * main.c: add the TIM6 branch to the existing function, do not redefine it. */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        tmc_motion_tick(&axis_x);
        tmc_motion_tick(&axis_y);
    }
}
/* USER CODE END 0 */

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_TIM6_Init();

    /* USER CODE BEGIN 2 */
    tmc_bus_init(&tmc_bus, &huart2, TMC_UART_TX_RX);

    uint8_t found = tmc2209_scan_bus(&tmc_bus);   /* bit0 = addr0, ... */
    (void)found;

    driver_setup(&drv_x, 0, X_EN_GPIO_Port, X_EN_Pin);
    driver_setup(&drv_y, 1, Y_EN_GPIO_Port, Y_EN_Pin);
    driver_setup(&drv_z, 2, NULL, 0);

    /* Axes X and Y: ramped STEP/DIR */
    tmc_motion_cfg_t cx = { X_STEP_GPIO_Port, X_STEP_Pin, X_DIR_GPIO_Port, X_DIR_Pin,
                            20000U, false, true /* double edge */ };
    tmc_motion_cfg_t cy = { Y_STEP_GPIO_Port, Y_STEP_Pin, Y_DIR_GPIO_Port, Y_DIR_Pin,
                            20000U, true /* invert direction */, true };
    tmc_motion_init(&axis_x, &cx, &drv_x);     /* sets VACTUAL=0 and the dedge bit */
    tmc_motion_init(&axis_y, &cy, &drv_y);

    tmc_motion_set_start_speed(&axis_x, 200);                  /* µsteps/s */
    tmc_motion_set_max_speed(&axis_x, 2.0f * USTEPS_PER_REV);   /* 2 rev/s  */
    tmc_motion_set_acceleration(&axis_x, 4.0f * USTEPS_PER_REV);

    HAL_TIM_Base_Start_IT(&htim6);

    /* 1) Position control (default speed/acceleration) */
    tmc_motion_move_to(&axis_x, tmc_mm_to_usteps(40.0f, USTEPS_PER_MM));

    /* 2) Position + velocity + ramp, specific to this command */
    tmc_motion_move_to_ex(&axis_y, -8000, 1600.0f, 3200.0f);

    /* 3) Velocity over UART (driver Z has no STEP/DIR), ramp 1600 µsteps/s^2 */
    tmc2209_velocity_ramp_set(&drv_z, tmc_rpm_to_usteps_per_s(60, FULL_STEPS_REV, MICROSTEPS), 1600.0f);
    /* USER CODE END 2 */

    uint32_t t_ramp = 0, t_diag = 0;
    uint8_t  phase  = 0;
    while (1)
    {
        /* USER CODE BEGIN 3 */
        uint32_t now = HAL_GetTick();

        /* The VACTUAL ramp must be serviced periodically */
        if (now - t_ramp >= 10U) {
            t_ramp = now;
            tmc2209_velocity_ramp_task(&drv_z);
        }

        /* Demo sequence for axis X */
        if (!tmc_motion_is_running(&axis_x)) {
            switch (phase) {
            case 0: /* return to 0 slowly, no ramp */
                tmc_motion_move_to_ex(&axis_x, 0, 800.0f, 0.0f);
                phase = 1;
                break;
            case 1: /* run at 1 rev/s in the negative direction */
                tmc_motion_run_velocity(&axis_x, -1.0f * USTEPS_PER_REV, 3200.0f);
                phase = 2;
                break;
            default:
                break;
            }
        }
        if ((phase == 2) && (tmc_motion_get_position(&axis_x) < -16000)) {
            tmc_motion_stop(&axis_x);   /* decelerate using the default acceleration */
            phase = 3;
        }

        /* 500 ms supervision: detect a driver reset (VS loss) and reload the config */
        if (now - t_diag >= 500U) {
            tmc2209_gstat_t gs;
            tmc2209_status_t st;
            t_diag = now;
            if ((tmc2209_get_global_status(&drv_x, &gs) == TMC_OK) && gs.reset) {
                tmc2209_restore_to_chip(&drv_x);
            }
            if (tmc2209_get_status(&drv_x, &st) == TMC_OK) {
                if (st.over_temperature_warning || st.short_to_ground_a || st.short_to_ground_b) {
                    tmc_motion_emergency_stop(&axis_x);
                    tmc2209_disable(&drv_x);
                }
            }
        }
        /* USER CODE END 3 */
    }
}

/* -------------------------------------------------------------------------- */
/* TX-only variant (only TX -> PDN_UART through 1k, no USART NVIC needed):    */
/*                                                                            */
/*   tmc_bus_init(&tmc_bus, &huart2, TMC_UART_TX_ONLY);                        */
/*   tmc2209_init(&drv_x, &tmc_bus, 0);      // cannot verify the chip         */
/*   ... all set_* / enable / move_at_velocity functions work as usual ...     */
/*   get_* functions return TMC_ERR_NO_RX.                                     */
/*                                                                            */
/* Half-duplex variant (CubeMX: USART Mode = Single Wire (Half-Duplex)):      */
/*   tmc_bus_init(&tmc_bus, &huart2, TMC_UART_HALF_DUPLEX);                    */
/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* FreeRTOS: several tasks sharing the bus                                    */
/*                                                                            */
/*   static SemaphoreHandle_t tmc_mtx;                                         */
/*   static void tmc_lock(void *c)   { xSemaphoreTake((SemaphoreHandle_t)c, portMAX_DELAY); } */
/*   static void tmc_unlock(void *c) { xSemaphoreGive((SemaphoreHandle_t)c); } */
/*   tmc_mtx = xSemaphoreCreateMutex();                                        */
/*   tmc_bus_set_lock(&tmc_bus, tmc_lock, tmc_unlock, tmc_mtx);                */
/* -------------------------------------------------------------------------- */
