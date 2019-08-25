/**
  ******************************************************************************
  * @file    recive_sm.h
  * @author  Vladislav Kamenev
  * @version V1.0.0
  * @date
  * @brief
  ******************************************************************************
**/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RECIVE_SM_H
#define __RECIVE_SM_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* recive sm message states */
typedef enum {
    MSG_START,
    MSG_CODE,
    MSG_LEN,
    MSG_DATA,
    MSG_END,
    MSG_COMPLETE
} ReceivedStates_TypeDef;

/* recive sm type */
typedef struct {
    ReceivedStates_TypeDef Stage;
    uint8_t Data[10];
    uint8_t Data_Cnt;
    uint8_t Command_Index;
} ReceivedStateMachine_TypeDef;

/* Exported variables --------------------------------------------------------*/
/* Exported function ---------------------------------------------------------*/
extern void ReceivedStateMachine_Start(void);
extern void ReceivedStateMachine_Reset(void);

__attribute__((weak)) void ReceivedStateMachine_CompleteCallBack(uint8_t command_index, uint8_t* command_data);


#endif /* ___H */
/*******************************************************************************
      END FILE
*******************************************************************************/