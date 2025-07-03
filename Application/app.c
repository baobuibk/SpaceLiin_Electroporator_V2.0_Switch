// APP HEADER //
#include "app.h"

static void Status_Led(void*);

#define         SCHEDULER_TASK_COUNT  6
uint32_t 	g_ui32SchedulerNumTasks = SCHEDULER_TASK_COUNT;
tSchedulerTask 	g_psSchedulerTable[SCHEDULER_TASK_COUNT] =
                {
                    {
                            &VOM_OVC_Task,
                            (void *) 0,
                            20,                         //call every 500us
                            0,                          //count from start
                            true                        //is active
                    },
                    {
                            &H_Bridge_Task,
                            (void *) 0,
                            3,                          //call every 500us
                            0,                          //count from start
                            false                       //is active
                    },
                    {
                            &CMD_Line_Task,
                            (void *) 0,
                            10,                         //call every 1ms
                            0,                          //count from start
                            true                        //is active
                    },
                    {
                            &FSP_Line_Task,
                            (void *) 0,
                            20,                         //call every 1ms
                            0,                          //count from start
                            true                        //is active
                    },
                    {
                            &VOM_Task,
                            (void *) 0,
                            10,                         //call every 1ms
                            0,                          //count from start
                            false                       //is active
                    },
                    {
                            &Status_Led,
                            (void *) 0,
                            10000,                      //call every 1ms
                            0,                          //count from start
                            true                        //is active
                    },
                //     {
                //             &H_Bridge_Manual_Task,
                //             (void *) 0,
                //             10,                      	//call every 1ms
                //             0,                          //count from start
                //             false                        //is active
                //     },
                };

void App_Main(void)
{   
    // STM32F030CCT6 @ 36MHz, 
    // can run scheduler tick max @ 100us.
    SchedulerInit(10000);

    CMD_Line_Task_Init();
    FSP_Line_Task_Init();
    V_Switch_Driver_Init();
    H_Bridge_Driver_Init();
    H_Bridge_Task_Init();
    H_Bridge_Sequence_Init();
    VOM_Driver_Init();

    while (1)
    {
        SchedulerRun();
    }
}

static void Status_Led(void*)
{
    LL_GPIO_TogglePin(DEBUG_LED_PORT, DEBUG_LED_PIN);
}
