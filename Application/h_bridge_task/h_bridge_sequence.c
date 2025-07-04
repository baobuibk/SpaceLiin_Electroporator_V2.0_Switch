/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Include~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include "h_bridge_sequence.h"
#include "h_bridge_task.h"
#include "h_bridge_driver.h"
#include "v_switch_driver.h"

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Defines ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#define SEQUENCE_MAX_INDEX 10

#define SD_DUTY_MIN \
((APB1_TIMER_CLK / 1000000) * 100) / (p_HB_task_data->HB_pole_pulse.PWM.Prescaler)

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
// static void H_Bridge_Calculate_Timing(
//                                       H_Bridge_Task_Data_typedef* p_HB_task_data,
//                                       V_Switch_mode               _VS_mode_,

//                                       uint16_t Set_delay_time_ms, 
//                                       uint16_t Set_on_time_ms, 
//                                       uint16_t Set_off_time_ms, 
//                                       uint16_t Set_pulse_count
//                                     );

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Enum ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Struct ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Class ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Private Types ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
typedef enum
{
    H_BRIDGE_HV_1_STATE,
    H_BRIDGE_HV_2_STATE,
    H_BRIDGE_LV_1_STATE,
    H_BRIDGE_LV_2_STATE,
} H_Bridge_Sequence_Process_State_typedef;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
static H_Bridge_Sequence_Process_State_typedef H_Bridge_Sequence_Process_State = H_BRIDGE_HV_1_STATE;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
H_Bridge_task_typedef  HB_sequence_default =
{
	.is_setted = 0,

	.sequence_delay_ms = 5,

	.pos_pole_index = 0,
	.neg_pole_index = 1,

	.pulse_delay_ms = 20,

	.hv_pos_count = 5,
	.hv_neg_count = 7,

	.hv_delay_ms = 10,

	.hv_pos_on_ms = 5,
	.hv_pos_off_ms = 5,
    .hv_neg_on_ms = 5,
	.hv_neg_off_ms = 5,

	.lv_pos_count = 9,
	.lv_neg_count = 11,

	.lv_delay_ms = 30,
    
	.lv_pos_on_ms = 5,
	.lv_pos_off_ms = 5,
    .lv_neg_on_ms = 5,
	.lv_neg_off_ms = 5,
};

H_Bridge_task_typedef  HB_sequence_array[SEQUENCE_MAX_INDEX] = {0};

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
void H_Bridge_Sequence_Init(void)
{
    for (uint8_t i = 0; i < SEQUENCE_MAX_INDEX; i++)
    {
        HB_sequence_array[i] = HB_sequence_default;
    }
}

void H_Bridge_Process_Sequence_Array(void)
{

uint8_t sequence_index;

for (sequence_index = 0; ((HB_sequence_array[sequence_index].is_setted & (1 << 7)) == (1 << 7)) && (sequence_index < SEQUENCE_MAX_INDEX) ; sequence_index++)
{

HB_Task_data[sequence_index].is_setted = true;

if (H_Bridge_Sequence_Process_State == H_BRIDGE_HV_1_STATE)
{
    if (HB_sequence_array[sequence_index].hv_pos_count != 0)
    {
        H_Bridge_Set_Pole(
                          &HB_Task_data[sequence_index].task_data[0].HB_pole_pulse,
                          &HB_Task_data[sequence_index].task_data[0].HB_pole_ls_on,
                          HB_sequence_array[sequence_index].pos_pole_index,
                          HB_sequence_array[sequence_index].neg_pole_index
                         );

        H_Bridge_Calculate_Timing(
                                   &HB_Task_data[sequence_index].task_data[0],
                                   V_SWITCH_MODE_HV_ON,

                                   HB_sequence_array[sequence_index].sequence_delay_ms, 
                                   HB_sequence_array[sequence_index].hv_pos_on_ms, 
                                   HB_sequence_array[sequence_index].hv_pos_off_ms, 
                                   HB_sequence_array[sequence_index].hv_pos_count,
                                   10, 10
                                 );

        H_Bridge_Sequence_Process_State = H_BRIDGE_HV_2_STATE;

    }
    else if (HB_sequence_array[sequence_index].hv_neg_count != 0)
    {
        H_Bridge_Set_Pole(
                          &HB_Task_data[sequence_index].task_data[1].HB_pole_pulse,
                          &HB_Task_data[sequence_index].task_data[1].HB_pole_ls_on,
                          HB_sequence_array[sequence_index].neg_pole_index,
                          HB_sequence_array[sequence_index].pos_pole_index
                         );

        H_Bridge_Calculate_Timing(
                                   &HB_Task_data[sequence_index].task_data[1],
                                   V_SWITCH_MODE_HV_ON,

                                   HB_sequence_array[sequence_index].sequence_delay_ms, 
                                   HB_sequence_array[sequence_index].hv_neg_on_ms, 
                                   HB_sequence_array[sequence_index].hv_neg_off_ms, 
                                   HB_sequence_array[sequence_index].hv_neg_count,
                                   10, 10
                                 );

        H_Bridge_Sequence_Process_State = H_BRIDGE_LV_1_STATE;
    }
    else if (HB_sequence_array[sequence_index].lv_pos_count != 0)
    {
        H_Bridge_Set_Pole(
                          &HB_Task_data[sequence_index].task_data[2].HB_pole_pulse,
                          &HB_Task_data[sequence_index].task_data[2].HB_pole_ls_on,
                          HB_sequence_array[sequence_index].pos_pole_index,
                          HB_sequence_array[sequence_index].neg_pole_index
                         );

        H_Bridge_Calculate_Timing(
                                   &HB_Task_data[sequence_index].task_data[2],
                                   V_SWITCH_MODE_LV_ON,

                                   HB_sequence_array[sequence_index].sequence_delay_ms, 
                                   HB_sequence_array[sequence_index].lv_pos_on_ms, 
                                   HB_sequence_array[sequence_index].lv_pos_off_ms, 
                                   HB_sequence_array[sequence_index].lv_pos_count,
                                   10, 10
                                 );

        H_Bridge_Sequence_Process_State = H_BRIDGE_LV_2_STATE;
    }
    else if (HB_sequence_array[sequence_index].lv_neg_count != 0)
    {
        H_Bridge_Set_Pole(
                          &HB_Task_data[sequence_index].task_data[3].HB_pole_pulse,
                          &HB_Task_data[sequence_index].task_data[3].HB_pole_ls_on,
                          HB_sequence_array[sequence_index].neg_pole_index,
                          HB_sequence_array[sequence_index].pos_pole_index
                         );

        H_Bridge_Calculate_Timing(
                                   &HB_Task_data[sequence_index].task_data[3],
                                   V_SWITCH_MODE_LV_ON,

                                   HB_sequence_array[sequence_index].sequence_delay_ms, 
                                   HB_sequence_array[sequence_index].lv_neg_on_ms, 
                                   HB_sequence_array[sequence_index].lv_neg_off_ms, 
                                   HB_sequence_array[sequence_index].lv_neg_count,
                                   10, 10
                                 );

        H_Bridge_Sequence_Process_State = H_BRIDGE_HV_1_STATE;
        continue;
    }
    else
    {   
        H_Bridge_Sequence_Process_State = H_BRIDGE_HV_1_STATE;
        continue;
    }
}

if (H_Bridge_Sequence_Process_State == H_BRIDGE_HV_2_STATE )
{
    if (HB_sequence_array[sequence_index].hv_neg_count != 0)
    {
        H_Bridge_Set_Pole(
                          &HB_Task_data[sequence_index].task_data[1].HB_pole_pulse,
                          &HB_Task_data[sequence_index].task_data[1].HB_pole_ls_on,
                          HB_sequence_array[sequence_index].neg_pole_index,
                          HB_sequence_array[sequence_index].pos_pole_index
                         );

        H_Bridge_Calculate_Timing(
                                   &HB_Task_data[sequence_index].task_data[1],
                                   V_SWITCH_MODE_HV_ON,

                                   HB_sequence_array[sequence_index].hv_delay_ms, 
                                   HB_sequence_array[sequence_index].hv_neg_on_ms, 
                                   HB_sequence_array[sequence_index].hv_neg_off_ms, 
                                   HB_sequence_array[sequence_index].hv_neg_count,
                                   10, 10
                                 );

        H_Bridge_Sequence_Process_State = H_BRIDGE_LV_1_STATE;
    }
    else if (HB_sequence_array[sequence_index].lv_pos_count != 0)
    {
        H_Bridge_Set_Pole(
                          &HB_Task_data[sequence_index].task_data[2].HB_pole_pulse,
                          &HB_Task_data[sequence_index].task_data[2].HB_pole_ls_on,
                          HB_sequence_array[sequence_index].pos_pole_index,
                          HB_sequence_array[sequence_index].neg_pole_index
                         );

        H_Bridge_Calculate_Timing(
                                   &HB_Task_data[sequence_index].task_data[2],
                                   V_SWITCH_MODE_LV_ON,

                                   HB_sequence_array[sequence_index].pulse_delay_ms, 
                                   HB_sequence_array[sequence_index].lv_pos_on_ms, 
                                   HB_sequence_array[sequence_index].lv_pos_off_ms, 
                                   HB_sequence_array[sequence_index].lv_pos_count,
                                   10, 10
                                 );

        H_Bridge_Sequence_Process_State = H_BRIDGE_LV_2_STATE;
    }
    else if (HB_sequence_array[sequence_index].lv_neg_count != 0)
    {
        H_Bridge_Set_Pole(
                          &HB_Task_data[sequence_index].task_data[3].HB_pole_pulse,
                          &HB_Task_data[sequence_index].task_data[3].HB_pole_ls_on,
                          HB_sequence_array[sequence_index].neg_pole_index,
                          HB_sequence_array[sequence_index].pos_pole_index
                         );

        H_Bridge_Calculate_Timing(
                                   &HB_Task_data[sequence_index].task_data[3],
                                   V_SWITCH_MODE_LV_ON,

                                   HB_sequence_array[sequence_index].pulse_delay_ms, 
                                   HB_sequence_array[sequence_index].lv_neg_on_ms, 
                                   HB_sequence_array[sequence_index].lv_neg_off_ms, 
                                   HB_sequence_array[sequence_index].lv_neg_count,
                                   10, 10
                                 );

        H_Bridge_Sequence_Process_State = H_BRIDGE_HV_1_STATE;
        continue;
    }
    else
    {   
        H_Bridge_Sequence_Process_State = H_BRIDGE_HV_1_STATE;
        continue;
    }
}

if (H_Bridge_Sequence_Process_State == H_BRIDGE_LV_1_STATE )
{
    if (HB_sequence_array[sequence_index].lv_pos_count != 0)
    {
        H_Bridge_Set_Pole(
                          &HB_Task_data[sequence_index].task_data[2].HB_pole_pulse,
                          &HB_Task_data[sequence_index].task_data[2].HB_pole_ls_on,
                          HB_sequence_array[sequence_index].pos_pole_index,
                          HB_sequence_array[sequence_index].neg_pole_index
                         );

        H_Bridge_Calculate_Timing(
                                   &HB_Task_data[sequence_index].task_data[2],
                                   V_SWITCH_MODE_LV_ON,

                                   HB_sequence_array[sequence_index].pulse_delay_ms, 
                                   HB_sequence_array[sequence_index].lv_pos_on_ms, 
                                   HB_sequence_array[sequence_index].lv_pos_off_ms, 
                                   HB_sequence_array[sequence_index].lv_pos_count,
                                   10, 10
                                 );

        H_Bridge_Sequence_Process_State = H_BRIDGE_LV_2_STATE;
    }
    else if (HB_sequence_array[sequence_index].lv_neg_count != 0)
    {
        H_Bridge_Set_Pole(
                          &HB_Task_data[sequence_index].task_data[3].HB_pole_pulse,
                          &HB_Task_data[sequence_index].task_data[3].HB_pole_ls_on,
                          HB_sequence_array[sequence_index].neg_pole_index,
                          HB_sequence_array[sequence_index].pos_pole_index
                         );

        H_Bridge_Calculate_Timing(
                                   &HB_Task_data[sequence_index].task_data[3],
                                   V_SWITCH_MODE_LV_ON,

                                   HB_sequence_array[sequence_index].pulse_delay_ms, 
                                   HB_sequence_array[sequence_index].lv_neg_on_ms, 
                                   HB_sequence_array[sequence_index].lv_neg_off_ms, 
                                   HB_sequence_array[sequence_index].lv_neg_count,
                                   10, 10
                                 );

        H_Bridge_Sequence_Process_State = H_BRIDGE_HV_1_STATE;
        continue;
    }
    else
    {   
        H_Bridge_Sequence_Process_State = H_BRIDGE_HV_1_STATE;
        continue;
    }
}

if (H_Bridge_Sequence_Process_State == H_BRIDGE_LV_2_STATE )
{
    if (HB_sequence_array[sequence_index].lv_neg_count != 0)
    {
        H_Bridge_Set_Pole(
                          &HB_Task_data[sequence_index].task_data[3].HB_pole_pulse,
                          &HB_Task_data[sequence_index].task_data[3].HB_pole_ls_on,
                          HB_sequence_array[sequence_index].neg_pole_index,
                          HB_sequence_array[sequence_index].pos_pole_index
                         );

        H_Bridge_Calculate_Timing(
                                   &HB_Task_data[sequence_index].task_data[3],
                                   V_SWITCH_MODE_LV_ON,

                                   HB_sequence_array[sequence_index].lv_delay_ms, 
                                   HB_sequence_array[sequence_index].lv_neg_on_ms, 
                                   HB_sequence_array[sequence_index].lv_neg_off_ms, 
                                   HB_sequence_array[sequence_index].lv_neg_count,
                                   10, 10
                                 );

        H_Bridge_Sequence_Process_State = H_BRIDGE_HV_1_STATE;
        continue;
    }
    else
    {   
        H_Bridge_Sequence_Process_State = H_BRIDGE_HV_1_STATE;
        continue;
    }
}
}

for (; sequence_index < SEQUENCE_MAX_INDEX; sequence_index++)
{
    HB_sequence_array[sequence_index].is_setted = 0;
}
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ End of the program ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */