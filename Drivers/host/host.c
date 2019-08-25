/**
  ******************************************************************************
  * @file    host.c
  * @author  Vladislav Kamenev
  * @version V1.0.0
  * @date
  * @brief
  ******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include "host.h"
#include "stepper.h"

/* Private define ------------------------------------------------------------*/
#define _PID0            ((uint8_t)0x86)
#define _PID1            ((uint8_t)0x93)

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static int8_t _set_driver_pwm(uint8_t* data);
static int8_t _set_driver_hold_pwm(uint8_t* data);
static int8_t _set_driver_hold_after_ms(uint8_t* data);
static int8_t _set_driver_step_subdelay_ms(uint8_t* data);
static int8_t _system_voltage(uint8_t* data);
static int8_t _system_save(uint8_t* data);
static int8_t _system_reset_to_default(uint8_t* data);
static int8_t _system_firmware_ver(uint8_t* data);

/* Private variables ---------------------------------------------------------*/
const Host_Commands_TypeDef Host_Commands[HOST_CMD_CNT] = {
    { {0x0A, 0x04, _PID0, _PID1, 0x00, 0x00}, _set_driver_pwm },
    { {0x0F, 0x04, _PID0, _PID1, 0x00, 0x00}, _set_driver_hold_pwm },
    { {0x10, 0x04, _PID0, _PID1, 0x00, 0x00}, _set_driver_hold_after_ms },
    { {0x11, 0x04, _PID0, _PID1, 0x00, 0x00}, _set_driver_step_subdelay_ms },
    { {0x12, 0x02, _PID0, _PID1}, _system_voltage },
    { {0xEE, 0x02, _PID0, _PID1}, _system_save },
    { {0xCC, 0x02, _PID0, _PID1}, _system_reset_to_default },
    { {0x00, 0x02, _PID0, _PID1}, _system_firmware_ver }
};

/* Extern function ----------------------------------------------------------*/
/* Functions -----------------------------------------------------------------*/

static int8_t _set_driver_pwm(uint8_t* data) {
    stepper_set_pwm( ((uint16_t)data[0] << 8) | ((uint16_t)data[1]) );
    return 0;
}

static int8_t _set_driver_hold_pwm(uint8_t* data) {
    stepper_set_hold_pwm( ((uint16_t)data[0] << 8) | ((uint16_t)data[1]) );
    return 0;
}

static int8_t _set_driver_hold_after_ms(uint8_t* data) {
    stepper_set_hold_delay( ((uint16_t)data[0] << 8) | ((uint16_t)data[1]) );
    return 0;
}

static int8_t _set_driver_step_subdelay_ms(uint8_t* data) {
    stepper_driver.step_subdelay_ms = data[1];
    return 0;
}

static int8_t _system_voltage(uint8_t* data) {
    return 0;
}

static int8_t _system_save(uint8_t* data) {
    return 0;
}

static int8_t _system_reset_to_default(uint8_t* data) {
    return 0;
}

static int8_t _system_firmware_ver(uint8_t* data) {
    return 0;
}

/**
  * @brief  _host_send_byte
  * @param
  * @retval None
  */
static __inline void _host_send_byte(uint8_t byte) {
    /* */
    HAL_UART_Transmit(&huart1, &byte, 1, 100);
}

/**
  * @brief  Decoding command from host
  * @param  None
  * @retval None
  */
static int8_t _host_decoding_command(uint8_t cmd_index, uint8_t* data) {
    if ( (cmd_index == 255) && (cmd_index >= HOST_CMD_CNT) ){
        return -1;
    }
    /* Run respect command, return state */
    return Host_Commands[cmd_index].Handler(data);
}

/**
  * @brief  host_search_command
  * @param
  * @retval
  */
int8_t host_search_command(uint8_t val) {
    for(uint8_t i = 0; i < HOST_CMD_CNT; i++) {
        if (val == Host_Commands[i].Command[0]) {
            return i;
        }
    }
    return -1;
}

/**
  * @brief  host_search_command
  * @param
  * @retval
  */
int8_t host_verify_pid(uint8_t pid_0, uint8_t pid_1) {
    if ( (pid_0 == _PID0) && (pid_1 == _PID1) ) {
        return 0;
    }
    return -1;
}

/**
  * @brief  send string(UTF-8) to host
  * @param  string pointer
  * @retval None
  */
void host_send_str(const char *str) {
    /* */
    while(*str != 0) { _host_send_byte((uint8_t)(*str)); str++; }
    _host_send_byte(0x0D);
    _host_send_byte(0x0A);
}

/**
  * @brief  ReceivedStateMachine_CompleteCallBack
  * @param
  * @retval
  */
void ReceivedStateMachine_CompleteCallBack(uint8_t command_index, uint8_t* command_data){
    if (_host_decoding_command(command_index, command_data) == 0) {
        host_send_str("OK");
    }
    else {
        host_send_str("ER");
    }

}


/*******************************************************************************
      END FILE
*******************************************************************************/
