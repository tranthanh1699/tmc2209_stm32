/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tmc2209.h"
#include "tmc2209_motion.h"
#include "adc_tracker.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/*
 * STEP/DIR/EN wiring for the TMC2209 -- NOT configured in the .ioc yet.
 * Adjust these three defines to match the actual pins on your board, then
 * add the matching GPIO clock enable in MX_GPIO_Init() if a new GPIO port
 * shows up here (GPIOC clock is enabled below for PC0/PC1/PC2).
 */
#define STEP_GPIO_Port      GPIOC
#define STEP_Pin            GPIO_PIN_0
#define DIR_GPIO_Port       GPIOC
#define DIR_Pin             GPIO_PIN_1
#define EN_GPIO_Port        GPIOC
#define EN_Pin              GPIO_PIN_2

/* MS1=0, MS2=0 -> UART slave address 0 (see TMC2209 datasheet §5) */
#define TMC_DRIVER_ADDR     0U

#define MICROSTEPS          16U
#define FULL_STEPS_REV      200U
#define USTEPS_PER_REV      (FULL_STEPS_REV * MICROSTEPS)   /* 3200 */
#define MOTION_TICK_HZ      20000U                          /* matches TIM6: 240 MHz / 240 / 50 */
#define USTEPS_PER_DEG      ((float)USTEPS_PER_REV / 360.0f)

/*
 * Analog input (potentiometer on PB1 / ADC1_IN5) -> motor angle.
 * ADC1 is triggered by TIM6 TRGO, so TIM6 is shared with the motion tick:
 * ADC sample rate = MOTION_TICK_HZ; TIM6 counter clock = 240 MHz / 240 = 1 MHz.
 * Tracker update rate = MOTION_TICK_HZ / ADC_AVG_SAMPLES = 1250 Hz.
 *
 * Absolute mapping: motor angle = MOTOR_DEG_PER_POT_DEG * pot angle, assuming
 * the motor shaft is at 0 deg (axis position 0) at power-up.
 */
#define TIM6_COUNTER_HZ         1000000u
#define ADC_AVG_SAMPLES         16u            /* 16 x uint16 = one 32 B cache line per DMA half */
#define ADC_RAW_MAX             65535.0f       /* ADC1 is configured for 16-bit resolution */
#define POT_FULL_SCALE_DEG      270.0f         /* mechanical travel of the pot */
#define MOTOR_DEG_PER_POT_DEG   1.0f           /* 1.0 = motor shaft follows the pot 1:1 */
#define MAX_SPEED_DPS           720.0f         /* 2 rev/s, same as tmc_motion max speed */
#define MIN_SPEED_DPS           30.0f          /* floor of the speed limit while tracking */
#define ALIGN_SPEED_DPS         180.0f         /* speed of the initial move to the pot angle */
#define MOTION_ACCEL_USPS2      (4.0f * USTEPS_PER_REV)   /* 4 rev/s² */

static tmc_bus_t    tmc_bus;
static tmc2209_t    tmc_drv;
static tmc_motion_t axis;

/* DMA1 cannot reach DTCM: this object holds the DMA buffer, so it is placed in
 * AXI SRAM through the .dma_buffer section (see STM32H743XX_FLASH.ld). */
__attribute__((section(".dma_buffer"))) static ADCTRK_HandleTypeDef tracker;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/** STEP/DIR/EN pins are plain GPIO outputs, not managed by CubeMX/.ioc yet. */
static void stepper_gpio_init(void)
{
    GPIO_InitTypeDef gi = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();

    HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DIR_GPIO_Port,  DIR_Pin,  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN_GPIO_Port,   EN_Pin,   GPIO_PIN_SET);   /* EN active-low -> start disabled */

    gi.Pin   = STEP_Pin | DIR_Pin | EN_Pin;
    gi.Mode  = GPIO_MODE_OUTPUT_PP;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(STEP_GPIO_Port, &gi);   /* all three pins are on GPIOC here */
}

static void driver_setup(tmc2209_t *d, uint8_t addr, GPIO_TypeDef *en_port, uint16_t en_pin)
{
    tmc_status_t st = tmc2209_init(d, &tmc_bus, addr);
    if (st != TMC_OK) {
        /* Most common causes: VM/VS not powered, wrong MS1/MS2 address,
         * or USART NVIC interrupt not enabled. */
        Error_Handler();
    }
    tmc2209_set_hardware_enable_pin(d, en_port, en_pin);
    tmc2209_set_microsteps_per_step(d, MICROSTEPS);
    tmc2209_set_run_current(d, 60);        /* % */
    tmc2209_set_hold_current(d, 30);       /* % */
    tmc2209_set_hold_delay(d, 50);         /* % */
    tmc2209_enable_automatic_current_scaling(d);   /* StealthChop auto-tuning */
    tmc2209_enable_automatic_gradient_adaptation(d);
    tmc2209_enable_stealth_chop(d);
    tmc2209_enable(d);
}

/** Degrees -> µsteps (rounded to nearest). */
static int32_t deg_to_usteps(float deg)
{
    return tmc_mm_to_usteps(deg, USTEPS_PER_DEG);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        tmc_motion_tick(&axis);
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  stepper_gpio_init();

  /* USART1: PB14/PB15 both tied to the driver's PDN_UART -> TX_RX mode, the
   * MCU sees its own echo and the library filters it out automatically. */
  tmc_bus_init(&tmc_bus, &huart1, TMC_UART_TX_RX);

  driver_setup(&tmc_drv, TMC_DRIVER_ADDR, EN_GPIO_Port, EN_Pin);

  tmc_motion_cfg_t motion_cfg = {
      STEP_GPIO_Port, STEP_Pin, DIR_GPIO_Port, DIR_Pin,
      MOTION_TICK_HZ, false /* dir_invert */, true /* double_edge */
  };
  tmc_motion_init(&axis, &motion_cfg, &tmc_drv);   /* also sets VACTUAL = 0 */

  tmc_motion_set_start_speed(&axis, 200.0f);                    /* µsteps/s */
  tmc_motion_set_max_speed(&axis, 2.0f * USTEPS_PER_REV);       /* 2 rev/s  */
  tmc_motion_set_acceleration(&axis, 4.0f * USTEPS_PER_REV);    /* 4 rev/s² */

  /* ADC tracker: TIM6 TRGO -> ADC1 -> circular DMA -> angle / speed */
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED) != HAL_OK) {
    Error_Handler();
  }

  ADCTRK_ConfigTypeDef trk_cfg;
  ADCTRK_GetDefaultConfig(&trk_cfg);
  trk_cfg.sample_rate_hz   = MOTION_TICK_HZ;
  trk_cfg.avg_samples      = ADC_AVG_SAMPLES;
  trk_cfg.raw_min          = 0.0f;
  trk_cfg.raw_max          = ADC_RAW_MAX;
  trk_cfg.full_scale_deg   = POT_FULL_SCALE_DEG;
  trk_cfg.pos_gain         = MOTOR_DEG_PER_POT_DEG;
  trk_cfg.vel_gain         = MOTOR_DEG_PER_POT_DEG;
  trk_cfg.out_vel_min_dps  = MIN_SPEED_DPS;
  trk_cfg.out_vel_max_dps  = MAX_SPEED_DPS;
  trk_cfg.out_acc_max_dps2 = 0.0f;                 /* ramp is done by tmc_motion */
  trk_cfg.out_pos_min_deg  = 0.0f;                 /* soft limits on the motor angle */
  trk_cfg.out_pos_max_deg  = POT_FULL_SCALE_DEG * MOTOR_DEG_PER_POT_DEG;
  if (ADCTRK_Init(&tracker, &hadc1, &htim6, TIM6_COUNTER_HZ, &trk_cfg) != ADCTRK_OK) {
    Error_Handler();
  }
  if (ADCTRK_Start(&tracker) != ADCTRK_OK) {       /* also starts TIM6 (HAL_TIM_Base_Start) */
    Error_Handler();
  }
  /* TIM6 is now BUSY, so HAL_TIM_Base_Start_IT() would fail: enable the
   * update interrupt for tmc_motion_tick() directly. */
  __HAL_TIM_ENABLE_IT(&htim6, TIM_IT_UPDATE);

  /* Wait for the first averaged sample (a timeout means ADC/DMA/TIM6 is misconfigured),
   * then map the pot's current angle to the motor's current position (0 deg). */
  ADCTRK_DataTypeDef trk;
  uint32_t t_start = HAL_GetTick();
  while (!ADCTRK_GetData(&tracker, &trk)) {
    if ((HAL_GetTick() - t_start) > 100U) {
      Error_Handler();
    }
  }
  ADCTRK_SetZero(&tracker, MOTOR_DEG_PER_POT_DEG * trk.in_angle_deg);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    static uint32_t t_diag = 0;
    static bool     aligned = false;
    uint32_t        now    = HAL_GetTick();

    /* New tracker update (~1250 Hz): retarget the axis to the pot angle. The speed
     * limit follows how fast the pot is turned; tmc_motion handles ramp and reversal.
     * The first move (motor 0 deg -> pot angle) runs at a fixed, moderate speed. */
    ADCTRK_DataTypeDef trk_data;
    if (ADCTRK_GetData(&tracker, &trk_data)) {
      float v_dps = aligned ? trk_data.out_vel_limit_dps : ALIGN_SPEED_DPS;
      tmc_motion_move_to_ex(&axis, deg_to_usteps(trk_data.out_target_deg),
                            v_dps * USTEPS_PER_DEG, MOTION_ACCEL_USPS2);
      if (!aligned && !tmc_motion_is_running(&axis)) {
        aligned = true;
      }
    }

    /* 500 ms supervision: detect a driver reset (VS loss) and reload the config,
     * and disable the driver on a fault condition. */
    if ((now - t_diag) >= 500U) {
      tmc2209_gstat_t  gs;
      tmc2209_status_t st;
      t_diag = now;

      if ((tmc2209_get_global_status(&tmc_drv, &gs) == TMC_OK) && gs.reset) {
        tmc2209_restore_to_chip(&tmc_drv);
      }
      if (tmc2209_get_status(&tmc_drv, &st) == TMC_OK) {
        if (st.over_temperature_shutdown || st.short_to_ground_a || st.short_to_ground_b) {
          tmc_motion_emergency_stop(&axis);
          tmc2209_disable(&tmc_drv);
        }
      }
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
