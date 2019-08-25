/**
  ******************************************************************************
  * @file    recive_sm.c
  * @author  LeftRadio
  * @version V1.0.0
  * @date
  * @brief
  ******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "usart.h"
#include "host.h"
#include "receive_sm.h"
#include "crc8.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO ReceivedStateMachine_TypeDef RStateMaschine;
volatile uint8_t rxfer[16];

/* Private function prototypes -----------------------------------------------*/
/* Extern function -----------------------------------------------------------*/
/* Functions -----------------------------------------------------------------*/

/**
  * @brief  ReceivedStateMachine_Reset
  * @param
  * @retval
  */
void ReceivedStateMachine_Start(void) {
	memset((uint8_t*)rxfer, 0, 16);
	HAL_UART_Receive_IT(&huart1, (uint8_t*)rxfer, 1);
}

/**
  * @brief  ReceivedStateMachine_Reset
  * @param
  * @retval
  */
void ReceivedStateMachine_Reset(void) {
    /* */
    RStateMaschine.Stage = MSG_START;
    RStateMaschine.Command_Index = 255;
    RStateMaschine.Data_Cnt = 0;
    /* */
    for(uint8_t i = 0; i < 10; i++) { RStateMaschine.Data[i] = 0; }
}

/**
  * @brief  ReceivedStateMachine_Event
  * @param
  * @retval
  */
static void ReceivedStateMachine_Event(uint8_t byte) {
	uint8_t control_crc;
	int16_t cmd_index;
	ReceivedStateMachine_TypeDef* Machine = (ReceivedStateMachine_TypeDef*)&RStateMaschine;

	switch(Machine->Stage) {

		/* Start frame, 0x5B expected byte */
		case MSG_START:
			/* */
			if (byte == 0x5B) {
				Machine->Stage = MSG_CODE;
			}
			else {
				ReceivedStateMachine_Reset();
				return;
			}
		break;

		/* Message code byte */
		case MSG_CODE:
			/* */
			cmd_index = host_search_command(byte);
			if (cmd_index != -1) {
				Machine->Stage = MSG_LEN;
				Machine->Command_Index = cmd_index;
			}
			else {
				ReceivedStateMachine_Reset();
				return;
			}
		break;

		/* Message lenght byte */
		case MSG_LEN:
			/* */
			cmd_index = Machine->Command_Index;
			if ( (cmd_index != 255) && (byte == Host_Commands[cmd_index].Command[1]) ) {
				Machine->Stage = MSG_DATA;
			}
			else {
				ReceivedStateMachine_Reset();
				return;
			}
		break;

		/* Message data */
		case MSG_DATA:
			/* */
			if (Machine->Data_Cnt >= Machine->Data[2] + 2) {
				Machine->Stage = MSG_END;
			}
		break;

		/* Message end, recive CRC byte */
		case MSG_END:
			/* crc for all recived bytes and 1 byte with 0x00 value */
			control_crc =  crc8_buffer( (uint8_t*)&Machine->Data[0], Machine->Data_Cnt + 1 );
			if ( (host_verify_pid(Machine->Data[3], Machine->Data[4])== 0) && (control_crc == byte) ) {
				Machine->Stage = MSG_COMPLETE;
			}
			else {
				ReceivedStateMachine_Reset();
				return;
			}
		break;


		/* Undefined stage, reset and return */
		default:
			ReceivedStateMachine_Reset();
		return;
	}
	/* */
	Machine->Data[Machine->Data_Cnt] = byte;
	Machine->Data_Cnt += 1;
	/* Host command recived successfull */
	if (Machine->Stage == MSG_COMPLETE) {
		ReceivedStateMachine_CompleteCallBack(Machine->Command_Index, &Machine->Data[Machine->Data[2]+1]);
		ReceivedStateMachine_Reset();
	}
}

/* Callbacks ----------------------------------------------------------------*/

/**
  * @brief  HAL_UART_RxCpltCallback
  * @param
  * @retval
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	ReceivedStateMachine_Event(rxfer[0]);
	HAL_UART_Receive_IT(&huart1, (uint8_t*)rxfer, 1);
}


/******************************************************************************
      END FILE
******************************************************************************/
