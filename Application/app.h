#ifndef APP_H_
#define APP_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "stm32f405xx.h"
#include "stm32f4xx_ll_gpio.h"

// SYSTEM DRIVER //
#include "board.h"

// USER DRIVER //
#include "scheduler.h"
#include "uart.h"

// INCLUDE DRIVER //
#include "h_bridge_driver.h"
#include "v_switch_driver.h"
#include "vom_driver.h"

// INCLUDE TASK //
#include "cmd_line_task.h"
#include "h_bridge_task.h"
#include "v_switch_task.h"
#include "fsp_line_task.h"
#include "h_bridge_sequence.h"
#include "vom_task.h"

typedef enum
{
    H_BRIDGE_TASK,
    CMD_LINE_TASK,
    FSP_LINE_TASK,
    VOM_TASK,
    STATUS_LED,
} Task_List_t;


void App_Main(void);

#endif /* APP_H_ */
