#ifndef H_BRIDGE_DRIVER_H_
#define H_BRIDGE_DRIVER_H_

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Include ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include <stdint.h>
#include <stdbool.h>

#include "stm32f4xx_ll_gpio.h"
#include "v_switch_driver.h"
#include "pwm.h"
#include "spi.h"
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Defines ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Types ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
typedef enum
{
    DELAY_PULSE_SYNC_STATE,
    ON_PULSE_SYNC_STATE,
    OFF_PULSE_SYNC_STATE,
} Sync_State_t;

typedef enum _H_Bridge_mode_typedef_
{
    H_BRIDGE_MODE_HS_ON,
    H_BRIDGE_MODE_LS_ON,
    H_BRIDGE_MODE_FLOAT,
    H_BRIDGE_MODE_PULSE,
}H_Bridge_mode;

typedef struct _H_Bridge_typdef_
{
    H_Bridge_mode   Mode;

    TIM_TypeDef     *TIMx;

    uint16_t        Delay_Prescaler;
    uint32_t        Delay_ARR;

    uint16_t        HB_Prescaler;
    uint32_t        HB_ARR;

    uint32_t        HIN_Channel;
    uint32_t        HIN_OC;

    uint32_t        LIN_Channel;
    uint32_t        LIN_OC;

    uint16_t        pulse_count;
    uint16_t        set_pulse_count;

    uint16_t        Deadtime_Delay_Prescaler;
    uint32_t        Deadtime_Delay_ARR;

    uint16_t        Deadtime_Pulse_Prescaler;
    uint32_t        Deadtime_Pulse_ARR;

    uint8_t         VOM_pulse_count;
    uint8_t         set_VOM_pulse_On_count;
    uint8_t         set_VOM_pulse_Off_count;

    uint16_t        VOM_Timer_On_Prescaler;
    uint32_t        VOM_Timer_On_ARR;

    uint16_t        VOM_Timer_Off_Prescaler;
    uint32_t        VOM_Timer_Off_ARR;

    // SPI_frame_t     ADC_Start_SPI_frame;
    // SPI_TX_data_t   ADC_Start_SPI_data[2];

} H_Bridge_typdef;

typedef struct _H_Bridge_Task_Data_typedef_
{
    bool            is_setted;

    V_Switch_mode   VS_mode;

    H_Bridge_typdef HB_pole_ls_on;
    H_Bridge_typdef HB_pole_pulse;

} H_Bridge_Task_Data_typedef;

typedef struct _H_Bridge_Task_typedef_
{
    bool                       is_setted;
    H_Bridge_Task_Data_typedef task_data[4];

} H_Bridge_Task_typedef;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
extern H_Bridge_typdef HB_pos_pole;
extern H_Bridge_typdef HB_neg_pole;

extern H_Bridge_Task_typedef HB_Task_data[10];

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Enum ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Struct ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Class ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
void H_Bridge_Driver_Init(void);

void H_Bridge_Set_Pole(H_Bridge_typdef* p_HB_pos_pole, H_Bridge_typdef* p_HB_neg_pole, uint8_t pos_pole_index, uint8_t neg_pole_index);

void H_Bridge_Set_Mode(H_Bridge_typdef* H_Bridge_x, H_Bridge_mode SetMode);

void H_Bridge_Calculate_Timing(
                               H_Bridge_Task_Data_typedef* p_HB_task_data,
                               V_Switch_mode               _VS_mode_,

                               uint16_t Set_delay_time_ms, 
                               uint16_t Set_on_time_ms, 
                               uint16_t Set_off_time_ms, 
                               uint16_t Set_pulse_count,

                               uint8_t  Set_sampling_ON_pulse_count,
                               uint8_t  Set_sampling_OFF_pulse_count
                              );

void H_Bridge_Kill(void);

void H_Bridge_TIM_2_Interupt_Handle(void);
void H_Bridge_TIM_3_Interupt_Handle(void);
void H_Bridge_TIM_4_Interupt_Handle(void);
void H_Bridge_TIM_5_Interupt_Handle(void);
void H_Bridge_TIM_8_Interupt_Handle(void);
void H_Bridge_Deadtime_IRQn_Handle(void);
void H_Bridge_VOM_Timer_IRQn_Handle(void);

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ End of the program ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#endif /* H_BRIDGE_DRIVER_H_ */

// void H_Bridge_Set_Pulse_Timing(H_Bridge_typdef* H_Bridge_x, uint16_t Set_delay_time_ms, uint16_t Set_on_time_ms, uint16_t Set_off_time_ms, uint16_t Set_pulse_count);
