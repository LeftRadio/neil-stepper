/**
  ******************************************************************************
  * @file    stepper.c
  * @author  Vladislav Kamenev
  * @version V1.0.0
  * @date
  * @brief
  ******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "tim.h"
// #include "adc.h"
#include "usart.h"
#include "stepper.h"

/* Private define ------------------------------------------------------------*/
#define STEPPER_START_PWM           ((uint16_t)500)
#define STEPPER_MAXIMUM_PWM         ((uint16_t)5000)

#define UTICKS                      ((uint32_t)48)

/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private varibles ----------------------------------------------------------*/
/* */
volatile stepper_drv_t stepper_driver = {
  .state = MOTOR_DISABLE,
  .direction = DIRECT,
  .pwm = STEPPER_START_PWM,
  .hold_pwm = STEPPER_START_PWM,
  .hold_after_ms = 10,
  .full_step_index = 0,
  .step_subdelay_ms = 5,
  .do_step_req = 0,
  .busy = 0,
};

void (*active_step_a)(uint32_t pwm);
void (*active_step_b)(uint32_t pwm);

volatile static uint8_t adc_buf[128];


/* Private function prototypes ----------------------------------------------*/
inline static uint32_t getUs(void)  __attribute__((always_inline));
inline static void del_cnt(uint32_t cnt)  __attribute__((always_inline));

/* Functions ----------------------------------------------------------------*/

/**
  * @brief  getUs
  * @param  None
  * @retval None
  */
static uint32_t getUs(void) {
    register uint32_t ms, cycle_cnt;
    do {
        ms = HAL_GetTick();
        cycle_cnt = SysTick->VAL;
    } while (ms != HAL_GetTick());
    return (ms * 1000) + (UTICKS * 1000 - cycle_cnt) / UTICKS;
}

/**
  * @brief  del_cnt
  * @param  None
  * @retval None
  */
static void del_cnt(uint32_t cnt) {
    uint32_t start = getUs();
    while (getUs() - start < cnt) { asm("nop"); }
}

/**
  * @brief  _stepper_hold_timer_init (TIM16)
  * @param  None
  * @retval None
  */
static void _stepper_hold_timer_init(uint32_t hold_after_ms) {
    uint64_t timer_period = HAL_RCC_GetSysClockFreq() / 1000;
    uint64_t full_hold_period_ms = timer_period * hold_after_ms;
    uint64_t prescaler = 0;

    /* if cnt for 1 ms not fit to 16 bit
       get 0.1 ms timer counter value with prescaler 0 */
    if (timer_period > 65535) {
        timer_period /= 10;
    }

    prescaler = full_hold_period_ms / timer_period;
    if (prescaler > 65535) prescaler = 65535;

    htim16.Instance = TIM16;
    htim16.Init.Prescaler = (uint32_t)prescaler;
    htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim16.Init.Period = (uint32_t)timer_period;
    htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim16.Init.RepetitionCounter = 0;
    htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim16) != HAL_OK) {
      Error_Handler();
    }

    /* start hold current timer */
    __HAL_TIM_CLEAR_FLAG(&htim16, TIM_FLAG_UPDATE);
    /* Enable the TIM Update interrupt */
    __HAL_TIM_ENABLE_IT(&htim16, TIM_IT_UPDATE);
    /* Enable the Peripheral */
    __HAL_TIM_ENABLE(&htim16);
}

/**
  * @brief  _stepper_A_off
  * @param  None
  * @retval None
  */
static void _stepper_A_off(void) {
    /* ---- A+ ---- */
    GPIOA->BRR = (uint32_t)AH_1_Pin;
    GPIOF->BRR = (uint32_t)AH_0_Pin;
    TIM3->CCR1 = 0;
    TIM3->CCR2 = 0;
}

/**
  * @brief  _stepper_A_plus
  * @param  None
  * @retval None
  */
static void _stepper_A_plus(uint32_t pwm) {
    /* ---- A+ ---- */
    GPIOA->BRR = (uint32_t)AH_1_Pin;
    TIM3->CCR1 = 0;
    del_cnt(10);
    GPIOF->BSRR = (uint32_t)AH_0_Pin;
    TIM3->CCR2 = pwm;
}

/**
  * @brief  _stepper_A_minus
  * @param  None
  * @retval None
  */
static void _stepper_A_minus(uint32_t pwm) {
    /* ---- A- ---- */
    GPIOF->BRR = (uint32_t)AH_0_Pin;
    TIM3->CCR2 = 0;
    del_cnt(10);
    GPIOA->BSRR = (uint32_t)AH_1_Pin;
    TIM3->CCR1 = pwm;
}

/**
  * @brief  _stepper_B_off
  * @param  None
  * @retval None
  */
static void _stepper_B_off(void) {
    /* ---- B+ ---- */
    GPIOA->BRR = (uint32_t)BH_1_Pin;
    GPIOA->BRR = (uint32_t)BH_0_Pin;
    TIM14->CCR1 = 0;
    TIM3->CCR4 = 0;
}

/**
  * @brief  _stepper_B_plus
  * @param  None
  * @retval None
  */
static void _stepper_B_plus(uint32_t pwm) {
    /* ---- B+ ---- */
    GPIOA->BRR = (uint32_t)BH_1_Pin;
    TIM14->CCR1 = 0;
    del_cnt(10);
    GPIOA->BSRR = (uint32_t)BH_0_Pin;
    TIM3->CCR4 = pwm;
}

/**
  * @brief  _stepper_B_minus
  * @param  None
  * @retval None
  */
static void _stepper_B_minus(uint32_t pwm) {
    /* ---- B- ---- */
    GPIOA->BRR = (uint32_t)BH_0_Pin;
    TIM3->CCR4 = 0;
    del_cnt(10);
    GPIOA->BSRR = (uint32_t)BH_1_Pin;
    TIM14->CCR1 = pwm;
}

/**
  * @brief  do_step
  * @param  None
  * @retval None
  */
static void _stepper_do_step(uint16_t pwm) {
    /* */
    if (stepper_driver.state == MOTOR_DISABLE) {
        stepper_stop();
        return;
    }
    /* */
    switch (stepper_driver.full_step_index) {
      case STEP_A:
        /* ---- A+ B+ ---- */
        _stepper_A_plus(pwm);
        _stepper_B_plus(pwm);
        active_step_a = _stepper_A_plus;
        active_step_b = _stepper_B_plus;
      break;

      case STEP_B:
        /* ---- A- B+ ---- */
        _stepper_A_minus(pwm);
        _stepper_B_plus(pwm);
        active_step_a = _stepper_A_minus;
        active_step_b = _stepper_B_plus;
      break;

      case STEP_C:
        /* ---- A- B- ---- */
        _stepper_A_minus(pwm);
        _stepper_B_minus(pwm);
        active_step_a = _stepper_A_minus;
        active_step_b = _stepper_B_minus;

      break;

      case STEP_D:
        /* ---- A+ B- ---- */
        _stepper_A_plus(pwm);
        _stepper_B_minus(pwm);
        active_step_a = _stepper_A_plus;
        active_step_b = _stepper_B_minus;
      break;

      default:
        return;
      break;
    }
    /* */
    __HAL_TIM_SET_COUNTER(&htim16, 0);
    /* */
    if ( !(htim16.Instance->CR1 & TIM_CR1_CEN) ) {
        // host_send_str("T16 START");
        __HAL_TIM_CLEAR_FLAG(&htim16, TIM_FLAG_UPDATE);
        __HAL_TIM_ENABLE(&htim16);
    }
}

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  stepper_set_direction
  * @param  None
  * @retval None
  */
void stepper_set_direction(step_direction_t dir) {
    stepper_driver.direction = dir;
}

/**
  * @brief  stepper_set_pwm
  * @param  None
  * @retval None
  */
void stepper_set_pwm(uint16_t val) {
  if (val >= 5000) val = 4999;
  stepper_driver.pwm = val;
}

/**
  * @brief  stepper_set_hold_pwm
  * @param  None
  * @retval None
  */
void stepper_set_hold_pwm(uint16_t val) {
  if (val >= 5000) val = 4999;
  stepper_driver.hold_pwm = val;
}

/**
  * @brief  stepper_set_hold_delay
  * @param  None
  * @retval None
  */
void stepper_set_hold_delay(uint32_t val) {
  if (val > 1000) val = 1000;
  stepper_driver.hold_after_ms = val;
}

/**
  * @brief  steper_init
  * @param  None
  * @retval None
  */
void stepper_stop(void) {
  /*  */
  _stepper_A_off();
  _stepper_B_off();
}

/**
  * @brief  steper_init
  * @param  None
  * @retval None
  */
void stepper_start(void) {
  /* AL channels */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  /* BL channels */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
  /* */
  stepper_stop();
  /* timer for holding static current */
   _stepper_hold_timer_init(stepper_driver.hold_after_ms);
  /* */
  _stepper_do_step(stepper_driver.pwm);
}

/**
  * @brief  stepper_do_next_step
  * @param  None
  * @retval None
  */
void stepper_do_next_step(void){
    stepper_driver.busy = 1;
    /* */
    if (stepper_driver.state == MOTOR_DISABLE) {
        stepper_stop();
        return;
    }
    /* */
    if (stepper_driver.direction == DIRECT) {
        stepper_driver.full_step_index++;
        if(stepper_driver.full_step_index > STEP_D) {
            stepper_driver.full_step_index = STEP_A;
        }
    }
    else if (stepper_driver.direction == FORWARD) {
        stepper_driver.full_step_index--;
        if(stepper_driver.full_step_index < STEP_A) {
            stepper_driver.full_step_index = STEP_D;
        }
    }
    /* */
    _stepper_do_step(stepper_driver.pwm);
    /* */
    stepper_driver.busy = 0;
}

/* HAL Callbacks -------------------------------------------------------------*/

/**
  * @brief  HAL_TIM_PeriodElapsedCallback
  * @param  htim pointer
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    /* clear IT flag, stop timer */
    __HAL_TIM_CLEAR_FLAG(&htim16, TIM_FLAG_UPDATE);
    __HAL_TIM_DISABLE(&htim16);

    /* return if current stepping proccess not end */
    if (stepper_driver.busy == 1) { return; }

    /* set hold current */
    if (active_step_a != (void*)0) {
      active_step_a(stepper_driver.hold_pwm);
    }
    if (active_step_b != (void*)0) {
      active_step_b(stepper_driver.hold_pwm);
    }
}

/**
  * @brief  HAL_GPIO_EXTI_Callback
  * @param  GPIO_Pin
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    /* */
    __disable_irq();
    /* */
    switch (GPIO_Pin) {

      case EN_Pin:
          if ( HAL_GPIO_ReadPin(EN_GPIO_Port, EN_Pin) == GPIO_PIN_RESET ) {
              stepper_driver.state = MOTOR_ENABLE;
              stepper_start();
          }
          else {
              stepper_driver.state = MOTOR_DISABLE;
              stepper_stop();
          }
      break;

      case DIR_Pin:
          if ( HAL_GPIO_ReadPin(DIR_GPIO_Port, DIR_Pin) == GPIO_PIN_RESET ) {
              stepper_driver.direction = FORWARD;
          }
          else {
              stepper_driver.direction = DIRECT;
          }
      break;

      case STEP_Pin:
          stepper_driver.do_step_req = 1;
      break;

      default:
      break;
    }
    /* */
    __enable_irq();
}

/*******************************************************************************
      END FILE
*******************************************************************************/
