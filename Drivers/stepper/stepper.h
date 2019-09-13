/**
  ******************************************************************************
  * @file       stepper.h
  * @author     Left Radio
  * @version    V1.0.0
  * @date
  * @brief      header
  ******************************************************************************
**/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STEPPER__H
#define __STEPPER__H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported define -----------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported typedef ----------------------------------------------------------*/

typedef enum { MOTOR_DISABLE = 0, MOTOR_ENABLE = !MOTOR_DISABLE } stepper_state_t;
typedef enum { DIRECT = 0, FORWARD = !DIRECT } step_direction_t;
typedef enum { STEP_A = (int8_t)0, STEP_B = (int8_t)1, STEP_C = (int8_t)2, STEP_D = (int8_t)3 } step_index_t;

typedef struct _stepper_drv_t {
    stepper_state_t state;
    step_direction_t direction;
    uint8_t do_step_req;
    uint16_t pwm;
    uint16_t hold_pwm;
    uint32_t hold_after_ms;
    int8_t full_step_index;
    uint8_t step_subdelay_ms;
    uint8_t busy;
} stepper_drv_t;

/* Exported variables --------------------------------------------------------*/
extern volatile stepper_drv_t stepper_driver;

/* Exported function ---------------------------------------------------------*/
extern void stepper_set_direction(step_direction_t dir);
extern void stepper_set_pwm(uint16_t val) ;
extern void stepper_set_hold_pwm(uint16_t val) ;
extern void stepper_set_hold_delay(uint32_t val) ;
extern void stepper_stop(void) ;
extern void stepper_start(void);
extern void stepper_do_next_step(void);

#endif /* __H */
/*******************************************************************************
      END FILE
*******************************************************************************/