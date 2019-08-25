/**
  ******************************************************************************
  * @file    host.h
  * @author  Vladislav Kamenev
  * @version V1.0.0
  * @date
  * @brief
  ******************************************************************************
**/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HOST_H
#define __HOST_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#define HOST_CMD_CNT        8

/* Exported macro ------------------------------------------------------------*/
/* Exported typedef ----------------------------------------------------------*/
typedef struct {
    uint8_t Command[10];
    int8_t (*Handler)(uint8_t* data);
} Host_Commands_TypeDef;

/* Exported variables --------------------------------------------------------*/
extern const Host_Commands_TypeDef Host_Commands[HOST_CMD_CNT];

/* Exported function ---------------------------------------------------------*/
extern void host_recive(uint8_t* data);
extern void host_send(uint8_t cmd, uint8_t *data, uint8_t dlen);
extern void host_send_str(const char *str);
extern void host_request_reset(void);
extern int8_t host_search_command(uint8_t val);
extern int8_t host_verify_pid(uint8_t pid_0, uint8_t pid_1);

#endif /* ___H */
/*******************************************************************************
      END FILE
*******************************************************************************/