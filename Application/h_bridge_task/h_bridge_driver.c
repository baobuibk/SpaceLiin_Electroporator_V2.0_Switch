/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Include~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
//#include "app.h"
#include "board.h"

#include "h_bridge_driver.h"
#include "vom_driver.h"

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Defines ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
// #define SD_DUTY_MIN \
// ((APB1_TIMER_CLK / 1000000) * 100) / (p_HB_task_data->HB_pole_pulse.PWM.Prescaler)
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Enum ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Struct ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Class ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Private Types ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
static SPI_TX_data_t VOM_SPI_TX_data[3] = {0};
static SPI_frame_t VOM_SPI_current_frame =
{
    .addr = 0x04,
    .p_data_array = VOM_SPI_TX_data,
    .data_size = 3,
};

static SPI_frame_t VOM_SPI_volt_frame =
{
    .addr = 0x05,
    .p_data_array = VOM_SPI_TX_data,
    .data_size = 3,
};

static SPI_TX_data_t VOM_SPI_Stop_ADC_TX_data[2] =
{
    {
        .mask = 0xFF,
        .data = 0x00,
    },
    {
        .mask = 0xFF,
        .data = 0x00,
    },
};
    
static SPI_frame_t VOM_SPI_stop_ADC_frame =
{
    .addr = 0x01,
    .p_data_array = VOM_SPI_Stop_ADC_TX_data,
    .data_size = 2,
};

static SPI_frame_t   SPI_VOM_Start_ADC_frame_temp;
static SPI_TX_data_t SPI_VOM_Start_ADC_TX_data_temp[2];

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
__STATIC_INLINE void H_Bridge_Interupt_Handle(H_Bridge_typdef* p_HB_TIM_x_IRQn);

// __STATIC_INLINE void VOM_SPI_Start_ADC(spi_stdio_typedef* p_spi, SPI_frame_t* p_SPI_frame);
__STATIC_INLINE void VOM_SPI_Start_ADC(spi_stdio_typedef* p_spi);
__STATIC_INLINE void VOM_SPI_Read_ADC(spi_stdio_typedef* p_spi);
__STATIC_INLINE void VOM_SPI_Stop_ADC(spi_stdio_typedef* p_spi);

__STATIC_INLINE void HB_Set_Duty(PWM_TypeDef *PWMx, uint32_t _Duty, bool apply_now);
__STATIC_INLINE void HB_Set_OC(TIM_TypeDef *TIMx, uint32_t _Channel, uint32_t _OC, bool apply_now);
__STATIC_INLINE void HB_Set_Freq(PWM_TypeDef *PWMx, float _Freq, bool apply_now);
__STATIC_INLINE void HB_Set_ARR(TIM_TypeDef *TIMx, uint32_t _ARR, bool apply_now);

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
TIM_TypeDef* HB_PWM_handle_array[8] =
{
    H_BRIDGE_0_HANDLE,

    H_BRIDGE_1_HANDLE,

    H_BRIDGE_2_HANDLE,

    H_BRIDGE_3_HANDLE,

    H_BRIDGE_4_HANDLE,

    H_BRIDGE_5_HANDLE,

    H_BRIDGE_6_HANDLE,

    H_BRIDGE_7_HANDLE,
};

uint32_t HB_PWM_channel_array[16] =
{
    H_BRIDGE_HIN_0_CHANNEL,
    H_BRIDGE_LIN_0_CHANNEL,

    H_BRIDGE_HIN_1_CHANNEL,
    H_BRIDGE_LIN_1_CHANNEL,

    H_BRIDGE_HIN_2_CHANNEL,
    H_BRIDGE_LIN_2_CHANNEL,

    H_BRIDGE_HIN_3_CHANNEL,
    H_BRIDGE_LIN_3_CHANNEL,

    H_BRIDGE_HIN_4_CHANNEL,
    H_BRIDGE_LIN_4_CHANNEL,

    H_BRIDGE_HIN_5_CHANNEL,
    H_BRIDGE_LIN_5_CHANNEL,

    H_BRIDGE_HIN_6_CHANNEL,
    H_BRIDGE_LIN_6_CHANNEL,

    H_BRIDGE_HIN_7_CHANNEL,
    H_BRIDGE_LIN_7_CHANNEL,
};

H_Bridge_typdef HB_pos_pole;
H_Bridge_typdef HB_neg_pole;

H_Bridge_typdef* p_HB_TIM_2_IRQn;
H_Bridge_typdef* p_HB_TIM_3_IRQn;
H_Bridge_typdef* p_HB_TIM_4_IRQn;
H_Bridge_typdef* p_HB_TIM_5_IRQn;
H_Bridge_typdef* p_HB_TIM_8_IRQn;

H_Bridge_typdef* p_Current_HB_TIM_IRQn;

H_Bridge_Task_typedef HB_Task_data[10];

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* :::::::::: H Bridge Driver Init :::::::: */
void H_Bridge_Driver_Init(void)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        LL_TIM_OC_SetMode(HB_PWM_handle_array[i], HB_PWM_channel_array[i * 2],  LL_TIM_OCMODE_FORCED_INACTIVE);
        LL_TIM_OC_SetPolarity(HB_PWM_handle_array[i], HB_PWM_channel_array[i * 2], LL_TIM_OCPOLARITY_HIGH);
        LL_TIM_OC_EnablePreload(HB_PWM_handle_array[i], HB_PWM_channel_array[i * 2]);
        LL_TIM_CC_EnableChannel(HB_PWM_handle_array[i], HB_PWM_channel_array[i * 2]);

        LL_TIM_OC_SetMode(HB_PWM_handle_array[i], HB_PWM_channel_array[(i * 2) + 1],  LL_TIM_OCMODE_FORCED_INACTIVE);
        LL_TIM_OC_SetPolarity(HB_PWM_handle_array[i], HB_PWM_channel_array[(i * 2) + 1], LL_TIM_OCPOLARITY_HIGH);
        LL_TIM_OC_EnablePreload(HB_PWM_handle_array[i], HB_PWM_channel_array[(i * 2) + 1]);
        LL_TIM_CC_EnableChannel(HB_PWM_handle_array[i], HB_PWM_channel_array[(i * 2) + 1]);

        LL_TIM_SetOffStates(HB_PWM_handle_array[i], LL_TIM_OSSI_ENABLE, LL_TIM_OSSR_ENABLE);

        LL_TIM_DisableAutomaticOutput(HB_PWM_handle_array[i]);

        LL_TIM_EnableAllOutputs(HB_PWM_handle_array[i]);

        LL_TIM_SetPrescaler(HB_PWM_handle_array[i], 1199);

        LL_TIM_EnableARRPreload(HB_PWM_handle_array[i]);

        LL_TIM_DisableIT_UPDATE(HB_PWM_handle_array[i]);
        LL_TIM_SetUpdateSource(HB_PWM_handle_array[i], LL_TIM_UPDATESOURCE_COUNTER);
        LL_TIM_GenerateEvent_UPDATE(HB_PWM_handle_array[i]);

        LL_TIM_DisableCounter(HB_PWM_handle_array[i]);
    }

    // Setting H-BRIDGE DEADTIME TIMER
    LL_TIM_DisableCounter(H_BRIDGE_DEADTIME_HANDLE);
    LL_TIM_DisableIT_UPDATE(H_BRIDGE_DEADTIME_HANDLE);
    LL_TIM_SetCounter(H_BRIDGE_DEADTIME_HANDLE, 0);

    LL_TIM_SetPrescaler(H_BRIDGE_DEADTIME_HANDLE, 1199);
    LL_TIM_EnableARRPreload(H_BRIDGE_DEADTIME_HANDLE);

    LL_TIM_SetUpdateSource(H_BRIDGE_DEADTIME_HANDLE, LL_TIM_UPDATESOURCE_COUNTER);
    LL_TIM_GenerateEvent_UPDATE(H_BRIDGE_DEADTIME_HANDLE);
    LL_TIM_ClearFlag_UPDATE(H_BRIDGE_DEADTIME_HANDLE);

    // Setting H-BRIDGE VOM TIMER
    LL_TIM_DisableCounter(H_BRIDGE_VOM_TIMER_HANDLE);
    LL_TIM_DisableIT_UPDATE(H_BRIDGE_VOM_TIMER_HANDLE);
    LL_TIM_SetCounter(H_BRIDGE_VOM_TIMER_HANDLE, 0);

    LL_TIM_SetPrescaler(H_BRIDGE_VOM_TIMER_HANDLE, 1199);
    LL_TIM_EnableARRPreload(H_BRIDGE_VOM_TIMER_HANDLE);

    LL_TIM_SetUpdateSource(H_BRIDGE_VOM_TIMER_HANDLE, LL_TIM_UPDATESOURCE_COUNTER);
    LL_TIM_GenerateEvent_UPDATE(H_BRIDGE_VOM_TIMER_HANDLE);
    LL_TIM_ClearFlag_UPDATE(H_BRIDGE_VOM_TIMER_HANDLE);
}

void H_Bridge_Set_Pole(H_Bridge_typdef* p_HB_pos_pole, H_Bridge_typdef* p_HB_neg_pole, uint8_t pos_pole_index, uint8_t neg_pole_index)
{
    //Set pole for positive pole
    p_HB_pos_pole->TIMx         = HB_PWM_handle_array[pos_pole_index];
    p_HB_pos_pole->HIN_Channel  = HB_PWM_channel_array[pos_pole_index * 2];
    p_HB_pos_pole->LIN_Channel  = HB_PWM_channel_array[(pos_pole_index * 2) + 1];

    //Set pole for negative pole
    p_HB_neg_pole->TIMx         = HB_PWM_handle_array[neg_pole_index];
    p_HB_neg_pole->HIN_Channel  = HB_PWM_channel_array[neg_pole_index * 2];
    p_HB_neg_pole->LIN_Channel  = HB_PWM_channel_array[(neg_pole_index * 2) + 1];
}

void H_Bridge_Calculate_Timing(
                               H_Bridge_Task_Data_typedef* p_HB_task_data,
                               V_Switch_mode               _VS_mode_,

                               uint16_t Set_delay_time_ms, 
                               uint16_t Set_on_time_ms, 
                               uint16_t Set_off_time_ms, 
                               uint16_t Set_pulse_count,

                               uint8_t  Set_sampling_ON_pulse_count,
                               uint8_t  Set_sampling_OFF_pulse_count
                              )
{
    float result_temp = 0.0;
    float off_time_temp = 0.0;
    p_HB_task_data->VS_mode = _VS_mode_;

    result_temp                                   = (((APB1_TIMER_CLK / 1000.0) * Set_delay_time_ms) / (2399 + 1.0)) - 1.0;
    p_HB_task_data->HB_pole_pulse.Delay_Prescaler = (uint16_t)(result_temp + 0.5f);
    p_HB_task_data->HB_pole_pulse.Delay_ARR       = 2399;
    
    result_temp                                 = (((APB1_TIMER_CLK / 1000.0) * (Set_on_time_ms + Set_off_time_ms)) / (2399 + 1.0)) - 1.0;
    p_HB_task_data->HB_pole_pulse.HB_Prescaler  = (uint16_t)(result_temp + 0.5f);
    p_HB_task_data->HB_pole_pulse.HB_Prescaler  = (p_HB_task_data->HB_pole_pulse.HB_Prescaler > 65535) ? 65535 : p_HB_task_data->HB_pole_pulse.HB_Prescaler;
    p_HB_task_data->HB_pole_pulse.HB_ARR        = 2399;

    result_temp                                 = ((APB1_TIMER_CLK / 1000.0) * Set_on_time_ms) / (p_HB_task_data->HB_pole_pulse.HB_Prescaler + 1.0);
    p_HB_task_data->HB_pole_pulse.HIN_OC        = (uint32_t)(result_temp + 0.5f);

    off_time_temp                               = (float)Set_on_time_ms + (0.1 * (float)Set_off_time_ms);
    result_temp                                 = ((APB1_TIMER_CLK / 1000.0) * off_time_temp) / (p_HB_task_data->HB_pole_pulse.HB_Prescaler + 1.0);
    p_HB_task_data->HB_pole_pulse.LIN_OC        = (uint32_t)(result_temp + 0.5f);

    p_HB_task_data->HB_pole_ls_on.HB_Prescaler = 1;

    p_HB_task_data->HB_pole_pulse.set_pulse_count = Set_pulse_count;
    p_HB_task_data->HB_pole_pulse.pulse_count     = 0;

    p_HB_task_data->is_setted = true;

    // Set timing parameter for DEADTIME TIMER
    off_time_temp                                           = (float)Set_delay_time_ms - (0.1 * (float)Set_off_time_ms);
    result_temp                                             = (((APB1_TIMER_CLK / 1000.0) * off_time_temp) / (2399 + 1.0)) - 1.0;
    p_HB_task_data->HB_pole_pulse.Deadtime_Delay_Prescaler  = (uint16_t)(result_temp + 0.5f);
    p_HB_task_data->HB_pole_pulse.Deadtime_Delay_ARR        = 2399;

    off_time_temp                                           = (float)Set_on_time_ms + (0.9 * (float)Set_off_time_ms);
    result_temp                                             = (((APB1_TIMER_CLK / 1000.0) * off_time_temp) / (2399 + 1.0)) - 1.0;
    p_HB_task_data->HB_pole_pulse.Deadtime_Pulse_Prescaler  = (uint16_t)(result_temp + 0.5f);
    p_HB_task_data->HB_pole_pulse.Deadtime_Pulse_ARR        = 2399;

    // Set timing parameter for VOM TIMER
    off_time_temp                                           = (float)Set_on_time_ms / (float)Set_sampling_ON_pulse_count;
    result_temp                                             = (((APB1_TIMER_CLK / 1000.0) * off_time_temp) / (119.0 + 1.0)) - 1.0;
    p_HB_task_data->HB_pole_pulse.VOM_Timer_On_Prescaler    = (uint16_t)(result_temp + 0.5f);
    p_HB_task_data->HB_pole_pulse.VOM_Timer_On_ARR          = 119;

    off_time_temp                                           = ((float)Set_off_time_ms * 1.0) / (float)Set_sampling_OFF_pulse_count;
    result_temp                                             = (((APB1_TIMER_CLK / 1000.0) * off_time_temp) / (119.0 + 1.0)) - 1.0;
    p_HB_task_data->HB_pole_pulse.VOM_Timer_Off_Prescaler   = (uint16_t)(result_temp + 0.5f);
    p_HB_task_data->HB_pole_pulse.VOM_Timer_Off_ARR         = 119;

    p_HB_task_data->HB_pole_pulse.set_VOM_pulse_On_count    = Set_sampling_ON_pulse_count;
    p_HB_task_data->HB_pole_pulse.set_VOM_pulse_Off_count   = Set_sampling_ON_pulse_count + Set_sampling_OFF_pulse_count;

    p_HB_task_data->HB_pole_pulse.VOM_pulse_count           = 0;

    // VOM_Config_t INA229_config =
    // {
    //     .measure_mode = VOM_BUS_SHUNT_CONT,
    //     .vsh_ct       = CT_50US,
    //     .vbus_ct      = CT_50US,
    //     .avg_vsh      = 1,
    //     .avg_vbus     = 1
    // };

    // // Tạo frame một lần duy nhất sau đó xài lại
    // VOM_Build_ADC_CONFIG_Frame(&INA229_config, &p_HB_task_data->HB_pole_pulse.ADC_Start_SPI_frame, p_HB_task_data->HB_pole_pulse.ADC_Start_SPI_data);
}

void H_Bridge_Set_Mode(H_Bridge_typdef* H_Bridge_x, H_Bridge_mode SetMode)
{
    LL_TIM_DisableIT_UPDATE(H_Bridge_x->TIMx);
    LL_TIM_DisableCounter(H_Bridge_x->TIMx);
    LL_TIM_ClearFlag_UPDATE(H_Bridge_x->TIMx);

    H_Bridge_x->Mode = SetMode;

    if (H_Bridge_x->TIMx == TIM2)
    {
        p_HB_TIM_2_IRQn = H_Bridge_x;
    }
    else if (H_Bridge_x->TIMx == TIM3)
    {
        p_HB_TIM_3_IRQn = H_Bridge_x;
    }
    else if (H_Bridge_x->TIMx == TIM4)
    {
        p_HB_TIM_4_IRQn = H_Bridge_x;
    }
    else if (H_Bridge_x->TIMx == TIM5)
    {
        p_HB_TIM_5_IRQn = H_Bridge_x;
    }
    else if (H_Bridge_x->TIMx == TIM8)
    {
        p_HB_TIM_8_IRQn = H_Bridge_x;
    }

    p_Current_HB_TIM_IRQn = H_Bridge_x;
    
    switch (SetMode)
    {
    case H_BRIDGE_MODE_PULSE:
        // Delay timing
        LL_TIM_OC_SetMode(H_Bridge_x->TIMx, H_Bridge_x->HIN_Channel, LL_TIM_OCMODE_FORCED_INACTIVE);
        LL_TIM_OC_SetMode(H_Bridge_x->TIMx, H_Bridge_x->LIN_Channel, LL_TIM_OCMODE_FORCED_ACTIVE);

        LL_TIM_SetPrescaler(H_Bridge_x->TIMx, H_Bridge_x->Delay_Prescaler);
        LL_TIM_GenerateEvent_UPDATE(H_Bridge_x->TIMx);
        HB_Set_ARR(H_Bridge_x->TIMx, H_Bridge_x->Delay_ARR, 1);

        // Pulse timing
        LL_TIM_OC_SetMode(H_Bridge_x->TIMx, H_Bridge_x->HIN_Channel, LL_TIM_OCMODE_PWM1);
        LL_TIM_OC_SetMode(H_Bridge_x->TIMx, H_Bridge_x->LIN_Channel, LL_TIM_OCMODE_PWM2);

        LL_TIM_SetPrescaler(H_Bridge_x->TIMx, H_Bridge_x->HB_Prescaler);
        HB_Set_ARR(H_Bridge_x->TIMx, H_Bridge_x->HB_ARR, 0);

        HB_Set_OC(H_Bridge_x->TIMx, H_Bridge_x->HIN_Channel, H_Bridge_x->HIN_OC, 0);
        HB_Set_OC(H_Bridge_x->TIMx, H_Bridge_x->LIN_Channel, H_Bridge_x->LIN_OC, 0);

        LL_TIM_ClearFlag_UPDATE(H_Bridge_x->TIMx);

        // Deadtime for delay timing
        LL_TIM_SetPrescaler(H_BRIDGE_DEADTIME_HANDLE, H_Bridge_x->Deadtime_Delay_Prescaler);
        LL_TIM_GenerateEvent_UPDATE(H_BRIDGE_DEADTIME_HANDLE);
        HB_Set_ARR(H_BRIDGE_DEADTIME_HANDLE, H_Bridge_x->Deadtime_Delay_ARR, 1);

        // Deadtime for pulsing timing
        LL_TIM_SetPrescaler(H_BRIDGE_DEADTIME_HANDLE, H_Bridge_x->Deadtime_Pulse_Prescaler);
        HB_Set_ARR(H_BRIDGE_DEADTIME_HANDLE, H_Bridge_x->Deadtime_Pulse_ARR, 0);

        LL_TIM_ClearFlag_UPDATE(H_BRIDGE_DEADTIME_HANDLE);

        LL_TIM_EnableCounter(H_BRIDGE_DEADTIME_HANDLE);
        LL_TIM_EnableIT_UPDATE(H_BRIDGE_DEADTIME_HANDLE);

        // VOM timer
        LL_TIM_SetPrescaler(H_BRIDGE_VOM_TIMER_HANDLE, H_Bridge_x->VOM_Timer_On_Prescaler);
        LL_TIM_GenerateEvent_UPDATE(H_BRIDGE_VOM_TIMER_HANDLE);
        HB_Set_ARR(H_BRIDGE_VOM_TIMER_HANDLE, H_Bridge_x->VOM_Timer_On_ARR, 1);

        LL_TIM_ClearFlag_UPDATE(H_BRIDGE_VOM_TIMER_HANDLE);
        LL_TIM_EnableIT_UPDATE(H_BRIDGE_VOM_TIMER_HANDLE);

        break;
    case H_BRIDGE_MODE_HS_ON:
        LL_TIM_OC_SetMode(H_Bridge_x->TIMx, H_Bridge_x->LIN_Channel, LL_TIM_OCMODE_FORCED_INACTIVE);
        LL_TIM_OC_SetMode(H_Bridge_x->TIMx, H_Bridge_x->HIN_Channel, LL_TIM_OCMODE_FORCED_ACTIVE);
        LL_TIM_GenerateEvent_UPDATE(H_Bridge_x->TIMx);

        LL_TIM_ClearFlag_UPDATE(H_Bridge_x->TIMx);

        break;

    case H_BRIDGE_MODE_LS_ON:
        LL_TIM_OC_SetMode(H_Bridge_x->TIMx, H_Bridge_x->HIN_Channel, LL_TIM_OCMODE_FORCED_INACTIVE);
        LL_TIM_OC_SetMode(H_Bridge_x->TIMx, H_Bridge_x->LIN_Channel, LL_TIM_OCMODE_FORCED_ACTIVE);
        LL_TIM_GenerateEvent_UPDATE(H_Bridge_x->TIMx);

        LL_TIM_ClearFlag_UPDATE(H_Bridge_x->TIMx);

        break;
    case H_BRIDGE_MODE_FLOAT:
        LL_TIM_OC_SetMode(H_Bridge_x->TIMx, H_Bridge_x->HIN_Channel, LL_TIM_OCMODE_FORCED_INACTIVE);
        LL_TIM_OC_SetMode(H_Bridge_x->TIMx, H_Bridge_x->LIN_Channel, LL_TIM_OCMODE_FORCED_INACTIVE);
        LL_TIM_GenerateEvent_UPDATE(H_Bridge_x->TIMx);

        LL_TIM_ClearFlag_UPDATE(H_Bridge_x->TIMx);

        break;
    
    default:
        break;
    }

    LL_TIM_EnableCounter(H_Bridge_x->TIMx);
    LL_TIM_EnableIT_UPDATE(H_Bridge_x->TIMx);
}

void H_Bridge_Kill(void)
{
    H_Bridge_Set_Mode(&HB_pos_pole, H_BRIDGE_MODE_FLOAT);
    H_Bridge_Set_Mode(&HB_neg_pole, H_BRIDGE_MODE_FLOAT);
}

/* ::::H_Bridge SD Interupt Handle:::: */
void H_Bridge_TIM_2_Interupt_Handle(void)
{
    H_Bridge_Interupt_Handle(p_HB_TIM_2_IRQn);
}

void H_Bridge_TIM_3_Interupt_Handle(void)
{
    H_Bridge_Interupt_Handle(p_HB_TIM_3_IRQn);
}

void H_Bridge_TIM_4_Interupt_Handle(void)
{
    H_Bridge_Interupt_Handle(p_HB_TIM_4_IRQn);
}

void H_Bridge_TIM_5_Interupt_Handle(void)
{
    H_Bridge_Interupt_Handle(p_HB_TIM_5_IRQn);
}

void H_Bridge_TIM_8_Interupt_Handle(void)
{
    H_Bridge_Interupt_Handle(p_HB_TIM_8_IRQn);
}

void H_Bridge_Deadtime_IRQn_Handle(void)
{
    if(LL_TIM_IsActiveFlag_UPDATE(H_BRIDGE_DEADTIME_HANDLE) == true)
    {
        LL_TIM_ClearFlag_UPDATE(H_BRIDGE_DEADTIME_HANDLE);

        if (p_Current_HB_TIM_IRQn->pulse_count == 0)
        {
            LL_TIM_OC_SetMode(p_Current_HB_TIM_IRQn->TIMx, p_Current_HB_TIM_IRQn->LIN_Channel, LL_TIM_OCMODE_FORCED_INACTIVE);

            LL_TIM_DisableCounter(H_BRIDGE_DEADTIME_HANDLE);
            LL_TIM_SetCounter(H_BRIDGE_DEADTIME_HANDLE, 0);

            VOM_SPI_Start_ADC(&VOM_SPI);

            return;
        }
        
        if (p_Current_HB_TIM_IRQn->pulse_count >= p_Current_HB_TIM_IRQn->set_pulse_count)
        {
            LL_TIM_OC_SetMode(p_Current_HB_TIM_IRQn->TIMx, p_Current_HB_TIM_IRQn->HIN_Channel, LL_TIM_OCMODE_FORCED_INACTIVE);
            LL_TIM_OC_SetMode(p_Current_HB_TIM_IRQn->TIMx, p_Current_HB_TIM_IRQn->LIN_Channel, LL_TIM_OCMODE_FORCED_ACTIVE);

            LL_TIM_DisableCounter(H_BRIDGE_DEADTIME_HANDLE);
            LL_TIM_SetCounter(H_BRIDGE_DEADTIME_HANDLE, 0);
            LL_TIM_DisableIT_UPDATE(H_BRIDGE_DEADTIME_HANDLE);

            return;
        }

        LL_TIM_OC_SetMode(p_Current_HB_TIM_IRQn->TIMx, p_Current_HB_TIM_IRQn->LIN_Channel, LL_TIM_OCMODE_FORCED_INACTIVE);

        LL_TIM_DisableCounter(H_BRIDGE_DEADTIME_HANDLE);
        LL_TIM_SetCounter(H_BRIDGE_DEADTIME_HANDLE, 0);
    }
}

void H_Bridge_VOM_Timer_IRQn_Handle(void)
{
    if(LL_TIM_IsActiveFlag_UPDATE(H_BRIDGE_VOM_TIMER_HANDLE) == true)
    {
        LL_TIM_ClearFlag_UPDATE(H_BRIDGE_VOM_TIMER_HANDLE);

        p_Current_HB_TIM_IRQn->VOM_pulse_count++;

        VOM_SPI_Read_ADC(&VOM_SPI);

        // Prep for VOM off timing
        if (p_Current_HB_TIM_IRQn->VOM_pulse_count == (p_Current_HB_TIM_IRQn->set_VOM_pulse_On_count - 2))
        {
            LL_TIM_SetPrescaler(H_BRIDGE_VOM_TIMER_HANDLE, p_Current_HB_TIM_IRQn->VOM_Timer_Off_Prescaler);
            HB_Set_ARR(H_BRIDGE_VOM_TIMER_HANDLE, p_Current_HB_TIM_IRQn->VOM_Timer_Off_ARR, 0);
            LL_TIM_ClearFlag_UPDATE(H_BRIDGE_VOM_TIMER_HANDLE);

            return;
        }
        
        if (p_Current_HB_TIM_IRQn->VOM_pulse_count == (p_Current_HB_TIM_IRQn->set_VOM_pulse_Off_count - 2))
        {
            LL_TIM_SetPrescaler(H_BRIDGE_VOM_TIMER_HANDLE, p_Current_HB_TIM_IRQn->VOM_Timer_On_Prescaler);
            HB_Set_ARR(H_BRIDGE_VOM_TIMER_HANDLE, p_Current_HB_TIM_IRQn->VOM_Timer_On_ARR, 0);
            LL_TIM_ClearFlag_UPDATE(H_BRIDGE_VOM_TIMER_HANDLE);
            return;
        }

        if (p_Current_HB_TIM_IRQn->VOM_pulse_count == (p_Current_HB_TIM_IRQn->set_VOM_pulse_Off_count - 1))
        {
            p_Current_HB_TIM_IRQn->VOM_pulse_count = 0;
            LL_TIM_DisableCounter(H_BRIDGE_VOM_TIMER_HANDLE);
            LL_TIM_SetCounter(H_BRIDGE_VOM_TIMER_HANDLE, 0);
        }
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
__STATIC_INLINE void H_Bridge_Interupt_Handle(H_Bridge_typdef* p_HB_TIM_x_IRQn)
{
    if(LL_TIM_IsActiveFlag_UPDATE(p_HB_TIM_x_IRQn->TIMx) == true)
    {
        LL_TIM_ClearFlag_UPDATE(p_HB_TIM_x_IRQn->TIMx);

        switch (p_HB_TIM_x_IRQn->Mode)
        {
        case H_BRIDGE_MODE_PULSE:
            p_HB_TIM_x_IRQn->pulse_count++;
            
            if (p_HB_TIM_x_IRQn->pulse_count >= (p_HB_TIM_x_IRQn->set_pulse_count + 1))
            {   
                LL_TIM_DisableCounter(p_HB_TIM_x_IRQn->TIMx);
                LL_TIM_SetCounter(p_HB_TIM_x_IRQn->TIMx, 0);

                p_HB_TIM_x_IRQn->VOM_pulse_count = 0;
                LL_TIM_DisableCounter(H_BRIDGE_VOM_TIMER_HANDLE);
                LL_TIM_SetCounter(H_BRIDGE_VOM_TIMER_HANDLE, 0);

                VOM_SPI_Stop_ADC(&VOM_SPI);
                return;
            }

            LL_TIM_EnableCounter(H_BRIDGE_VOM_TIMER_HANDLE);
            VOM_SPI_Read_ADC(&VOM_SPI);

            // Eveytime a pulse is finish we must reinit OCMODE_PWM2 for the LIN channel
            LL_TIM_OC_SetMode(p_HB_TIM_x_IRQn->TIMx, p_HB_TIM_x_IRQn->LIN_Channel, LL_TIM_OCMODE_PWM2);

            // Reinit the deadtime timer at the start of the puse
            LL_TIM_EnableCounter(H_BRIDGE_DEADTIME_HANDLE);
            return;

            break;
        case H_BRIDGE_MODE_HS_ON:
        case H_BRIDGE_MODE_LS_ON:
        case H_BRIDGE_MODE_FLOAT:
            
            LL_TIM_DisableIT_UPDATE(p_HB_TIM_x_IRQn->TIMx);
            break;
        
        default:
            break;
        }
    }
}

// __STATIC_INLINE void VOM_SPI_Start_ADC(spi_stdio_typedef* p_spi, SPI_frame_t* p_SPI_frame)
__STATIC_INLINE void VOM_SPI_Start_ADC(spi_stdio_typedef* p_spi)
{
    VOM_Config_t INA229_config =
    {
        .measure_mode = VOM_BUS_SHUNT_CONT,
        .vsh_ct       = CT_50US,
        .vbus_ct      = CT_50US,
        .avg_vsh      = 1,
        .avg_vbus     = 1
    };

    // Tạo frame một lần duy nhất sau đó xài lại
    VOM_Build_ADC_CONFIG_Frame(&INA229_config, &SPI_VOM_Start_ADC_frame_temp, SPI_VOM_Start_ADC_TX_data_temp);

    SPI_Overwrite(p_spi, &SPI_VOM_Start_ADC_frame_temp);
}

__STATIC_INLINE void VOM_SPI_Read_ADC(spi_stdio_typedef* p_spi)
{
    SPI_Read(p_spi, &VOM_SPI_current_frame);
    SPI_Read(p_spi, &VOM_SPI_volt_frame);
}

__STATIC_INLINE void VOM_SPI_Stop_ADC(spi_stdio_typedef* p_spi)
{
    SPI_Overwrite(p_spi, &VOM_SPI_stop_ADC_frame);
}

__STATIC_INLINE void HB_Set_Duty(PWM_TypeDef *PWMx, uint32_t _Duty, bool apply_now)
{
    LL_TIM_DisableUpdateEvent(PWMx->TIMx);

    // Limit the duty to 100
    if (_Duty > 100)
      return;

    // Set PWM DUTY for channel 1
    _Duty = (PWMx->Freq * (_Duty / 100.0));
    switch (PWMx->Channel)
    {
    case LL_TIM_CHANNEL_CH1:
        LL_TIM_OC_SetCompareCH1(PWMx->TIMx, _Duty);
        break;
    case LL_TIM_CHANNEL_CH2:
        LL_TIM_OC_SetCompareCH2(PWMx->TIMx, _Duty);
        break;
    case LL_TIM_CHANNEL_CH3:
        LL_TIM_OC_SetCompareCH3(PWMx->TIMx, _Duty);
        break;
    case LL_TIM_CHANNEL_CH4:
        LL_TIM_OC_SetCompareCH4(PWMx->TIMx, _Duty);
        break;

    default:
        break;
    }

    LL_TIM_EnableUpdateEvent(PWMx->TIMx);

    if(apply_now == 1)
    {
        if (LL_TIM_IsEnabledIT_UPDATE(PWMx->TIMx))
        {
            LL_TIM_DisableIT_UPDATE(PWMx->TIMx);
            LL_TIM_GenerateEvent_UPDATE(PWMx->TIMx);
            LL_TIM_EnableIT_UPDATE(PWMx->TIMx);
        }
        else
        {
            LL_TIM_GenerateEvent_UPDATE(PWMx->TIMx);
        }
    }
}

__STATIC_INLINE void HB_Set_OC(TIM_TypeDef *TIMx, uint32_t _Channel, uint32_t _OC, bool apply_now)
{
    LL_TIM_DisableUpdateEvent(TIMx);

    // Set PWM DUTY for channel 1
    //PWMx->Duty = _OC;
    switch (_Channel)
    {
    case LL_TIM_CHANNEL_CH1:
        LL_TIM_OC_SetCompareCH1(TIMx, _OC);
        break;
    case LL_TIM_CHANNEL_CH2:
        LL_TIM_OC_SetCompareCH2(TIMx, _OC);
        break;
    case LL_TIM_CHANNEL_CH3:
        LL_TIM_OC_SetCompareCH3(TIMx, _OC);
        break;
    case LL_TIM_CHANNEL_CH4:
        LL_TIM_OC_SetCompareCH4(TIMx, _OC);
        break;

    default:
        break;
    }

    LL_TIM_EnableUpdateEvent(TIMx);

    if(apply_now == 1)
    {
        if (LL_TIM_IsEnabledIT_UPDATE(TIMx))
        {
            LL_TIM_DisableIT_UPDATE(TIMx);
            LL_TIM_GenerateEvent_UPDATE(TIMx);
            LL_TIM_EnableIT_UPDATE(TIMx);
        }
        else
        {
            LL_TIM_GenerateEvent_UPDATE(TIMx);
        }
    }
}

__STATIC_INLINE void HB_Set_Freq(PWM_TypeDef *PWMx, float _Freq, bool apply_now)
{
    LL_TIM_DisableUpdateEvent(PWMx->TIMx);

    // Set PWM FREQ
    //PWMx->Freq = __LL_TIM_CALC_ARR(APB1_TIMER_CLK, LL_TIM_GetPrescaler(PWMx->TIMx), _Freq);
    PWMx->Freq = (float)APB1_TIMER_CLK / ((float)(LL_TIM_GetPrescaler(PWMx->TIMx) + 1) * _Freq);
    LL_TIM_SetAutoReload(PWMx->TIMx, PWMx->Freq);

    LL_TIM_EnableUpdateEvent(PWMx->TIMx);

    if(apply_now == 1)
    {
        if (LL_TIM_IsEnabledIT_UPDATE(PWMx->TIMx))
        {
            LL_TIM_DisableIT_UPDATE(PWMx->TIMx);
            LL_TIM_GenerateEvent_UPDATE(PWMx->TIMx);
            LL_TIM_EnableIT_UPDATE(PWMx->TIMx);
        }
        else
        {
            LL_TIM_GenerateEvent_UPDATE(PWMx->TIMx);
        }
    }
}

__STATIC_INLINE void HB_Set_ARR(TIM_TypeDef *TIMx, uint32_t _ARR, bool apply_now)
{
    LL_TIM_DisableUpdateEvent(TIMx);

    // Set PWM FREQ
    LL_TIM_SetAutoReload(TIMx, _ARR);

    LL_TIM_EnableUpdateEvent(TIMx);

    if(apply_now == 1)
    {
        if (LL_TIM_IsEnabledIT_UPDATE(TIMx))
        {
            LL_TIM_DisableIT_UPDATE(TIMx);
            LL_TIM_GenerateEvent_UPDATE(TIMx);
            LL_TIM_EnableIT_UPDATE(TIMx);
        }
        else
        {
            LL_TIM_GenerateEvent_UPDATE(TIMx);
        }
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ End of the program ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* PWM_TypeDef HB_PWM_SD0_3 =
{
    .TIMx       =   H_BRIDGE_SD0_3_HANDLE,
    .Prescaler  =   1199,
    .Mode       =   LL_TIM_OCMODE_FORCED_INACTIVE,
    .Polarity   =   LL_TIM_OCPOLARITY_HIGH,
    .Duty       =   3, //100us
    .Freq       =   0,
};

PWM_TypeDef HB_PWM_SD4_7 =
{
    .TIMx       =   H_BRIDGE_SD4_7_HANDLE,
    .Prescaler  =   1199,
    .Mode       =   LL_TIM_OCMODE_FORCED_INACTIVE,
    .Polarity   =   LL_TIM_OCPOLARITY_HIGH,
    .Duty       =   3, //100us
    .Freq       =   0,
}; */

/* ::::H_Bridge SD0_3 Interupt Handle:::: */
/* void H_Bridge_SD0_3_Interupt_Handle()
{
    if(LL_TIM_IsActiveFlag_UPDATE(H_BRIDGE_SD0_3_HANDLE) == true)
    {
        LL_TIM_ClearFlag_UPDATE(H_BRIDGE_SD0_3_HANDLE);

        switch (p_HB_SD_0_3_IRQn->Mode)
        {
        case H_BRIDGE_MODE_PULSE:
            p_HB_SD_0_3_IRQn->pulse_count++;

            if (p_HB_SD_0_3_IRQn->pulse_count >= ((p_HB_SD_0_3_IRQn->set_pulse_count * 2) + 1))
            {
                LL_TIM_DisableCounter(H_BRIDGE_SD0_3_HANDLE);
                return;
            }
            
            if (*p_HB_SD_0_3_IRQn->Pin_State == 1)
            {
                LL_GPIO_SetOutputPin(p_HB_SD_0_3_IRQn->Port, *p_HB_SD_0_3_IRQn->Pin);
                HB_Set_Freq(&p_HB_SD_0_3_IRQn->PWM, 1000.0 / (float)p_HB_SD_0_3_IRQn->off_time_ms, 0);
                *p_HB_SD_0_3_IRQn->Pin_State = 0;
            }
            else
            {
                LL_GPIO_ResetOutputPin(p_HB_SD_0_3_IRQn->Port, *p_HB_SD_0_3_IRQn->Pin);
                HB_Set_Freq(&p_HB_SD_0_3_IRQn->PWM, 1000.0 / (float)p_HB_SD_0_3_IRQn->on_time_ms, 0);
                *p_HB_SD_0_3_IRQn->Pin_State = 1;
            }

            break;
        case H_BRIDGE_MODE_HS_ON:
            LL_GPIO_SetOutputPin(p_HB_SD_0_3_IRQn->Port, *p_HB_SD_0_3_IRQn->Pin);

            LL_TIM_OC_SetMode(p_HB_SD_0_3_IRQn->PWM.TIMx, p_HB_SD_0_3_IRQn->PWM.Channel, LL_TIM_OCMODE_FORCED_ACTIVE);
            LL_TIM_GenerateEvent_UPDATE(p_HB_SD_0_3_IRQn->PWM.TIMx);
            LL_TIM_ClearFlag_UPDATE(p_HB_SD_0_3_IRQn->PWM.TIMx);
            LL_TIM_DisableIT_UPDATE(p_HB_SD_0_3_IRQn->PWM.TIMx);
            break;
        case H_BRIDGE_MODE_LS_ON:
            LL_GPIO_ResetOutputPin(p_HB_SD_0_3_IRQn->Port, *p_HB_SD_0_3_IRQn->Pin);

            LL_TIM_OC_SetMode(p_HB_SD_0_3_IRQn->PWM.TIMx, p_HB_SD_0_3_IRQn->PWM.Channel, LL_TIM_OCMODE_FORCED_ACTIVE);
            LL_TIM_GenerateEvent_UPDATE(p_HB_SD_0_3_IRQn->PWM.TIMx);
            LL_TIM_ClearFlag_UPDATE(p_HB_SD_0_3_IRQn->PWM.TIMx);
            LL_TIM_DisableIT_UPDATE(p_HB_SD_0_3_IRQn->PWM.TIMx);
            break;
        case H_BRIDGE_MODE_FLOAT:
            LL_TIM_DisableIT_UPDATE(p_HB_SD_0_3_IRQn->PWM.TIMx);
            break;
        
        default:
            break;
        }

        p_HB_SD_0_3_IRQn->is_setted = true;
    }
} */

/* ::::H_Bridge SD4_7 Interupt Handle:::: */
/* void H_Bridge_SD4_7_Interupt_Handle()
{
    if(LL_TIM_IsActiveFlag_UPDATE(H_BRIDGE_SD4_7_HANDLE) == true)
    {
        LL_TIM_ClearFlag_UPDATE(H_BRIDGE_SD4_7_HANDLE);

        switch (p_HB_SD_4_7_IRQn->Mode)
        {
        case H_BRIDGE_MODE_PULSE:
            p_HB_SD_4_7_IRQn->pulse_count++;

            if (p_HB_SD_4_7_IRQn->pulse_count >= ((p_HB_SD_4_7_IRQn->set_pulse_count * 2) + 1))
            {
                LL_TIM_DisableCounter(H_BRIDGE_SD4_7_HANDLE);
                return;
            }

            if (*p_HB_SD_4_7_IRQn->Pin_State == 1)
            {
                LL_GPIO_SetOutputPin(p_HB_SD_4_7_IRQn->Port, *p_HB_SD_4_7_IRQn->Pin);
                HB_Set_Freq(&p_HB_SD_4_7_IRQn->PWM, 1000.0 / (float)p_HB_SD_4_7_IRQn->off_time_ms, 0);
                *p_HB_SD_4_7_IRQn->Pin_State = 0;
            }
            else
            {
                LL_GPIO_ResetOutputPin(p_HB_SD_4_7_IRQn->Port, *p_HB_SD_4_7_IRQn->Pin);
                HB_Set_Freq(&p_HB_SD_4_7_IRQn->PWM, 1000.0 / (float)p_HB_SD_4_7_IRQn->on_time_ms, 0);
                *p_HB_SD_4_7_IRQn->Pin_State = 1;
            }

            break;
        case H_BRIDGE_MODE_HS_ON:
            LL_GPIO_SetOutputPin(p_HB_SD_4_7_IRQn->Port, *p_HB_SD_4_7_IRQn->Pin);

            LL_TIM_OC_SetMode(p_HB_SD_4_7_IRQn->PWM.TIMx, p_HB_SD_4_7_IRQn->PWM.Channel, LL_TIM_OCMODE_FORCED_ACTIVE);
            LL_TIM_GenerateEvent_UPDATE(p_HB_SD_4_7_IRQn->PWM.TIMx);
            LL_TIM_ClearFlag_UPDATE(p_HB_SD_4_7_IRQn->PWM.TIMx);
            LL_TIM_DisableIT_UPDATE(p_HB_SD_4_7_IRQn->PWM.TIMx);
            break;
        case H_BRIDGE_MODE_LS_ON:
            LL_GPIO_ResetOutputPin(p_HB_SD_4_7_IRQn->Port, *p_HB_SD_4_7_IRQn->Pin);

            LL_TIM_OC_SetMode(p_HB_SD_4_7_IRQn->PWM.TIMx, p_HB_SD_4_7_IRQn->PWM.Channel, LL_TIM_OCMODE_FORCED_ACTIVE);
            LL_TIM_GenerateEvent_UPDATE(p_HB_SD_4_7_IRQn->PWM.TIMx);
            LL_TIM_ClearFlag_UPDATE(p_HB_SD_4_7_IRQn->PWM.TIMx);
            LL_TIM_DisableIT_UPDATE(p_HB_SD_4_7_IRQn->PWM.TIMx);
            break;
        case H_BRIDGE_MODE_FLOAT:
            LL_TIM_DisableIT_UPDATE(p_HB_SD_4_7_IRQn->PWM.TIMx);
            break;
        
        default:
            break;
        }

        p_HB_SD_4_7_IRQn->is_setted = true;
    }
} */

// void H_Bridge_Set_Pulse_Timing(H_Bridge_typdef* H_Bridge_x, uint16_t Set_delay_time_ms, uint16_t Set_on_time_ms, uint16_t Set_off_time_ms, uint16_t Set_pulse_count)
// {
//     LL_TIM_DisableIT_UPDATE(H_Bridge_x->PWM.TIMx);
//     LL_TIM_DisableCounter(H_Bridge_x->PWM.TIMx);
//     LL_TIM_ClearFlag_UPDATE(H_Bridge_x->PWM.TIMx);

//     /* 
//     * @brief Calculate the maximum of Set_delay_time_ms, Set_on_time_ms, and Set_off_time_ms.
//     * @details 
//     *   - If 'Set_delay_time_ms' is greater than 'Set_on_time_ms', check if 'Set_delay_time_ms' is also greater than 'Set_off_time_ms'.
//     *     If true, 'Set_delay_time_ms' is the maximum; otherwise, 'Set_off_time_ms' is the maximum.
//     *   - If 'Set_delay_time_ms' is not greater than 'Set_on_time_ms', then check if 'Set_on_time_ms' is greater than 'Set_off_time_ms'.
//     *     If true, 'Set_on_time_ms' is the maximum; otherwise, 'Set_off_time_ms' is the maximum.
//     */
//     uint16_t max_time_ms =  (Set_delay_time_ms > Set_on_time_ms) ? 
//                             ((Set_delay_time_ms > Set_off_time_ms) ? Set_delay_time_ms : Set_off_time_ms) : 
//                             ((Set_on_time_ms > Set_off_time_ms) ? Set_on_time_ms : Set_off_time_ms);

//     uint16_t min_time_ms =  (Set_on_time_ms < Set_off_time_ms) ? Set_on_time_ms : Set_off_time_ms;
     
//     H_Bridge_x->PWM.Prescaler   = (((APB1_TIMER_CLK / 1000) * max_time_ms) / (UINT16_MAX)) + 1;
//     H_Bridge_x->PWM.Prescaler   = (H_Bridge_x->PWM.Prescaler > UINT16_MAX) ? UINT16_MAX : H_Bridge_x->PWM.Prescaler;

//     H_Bridge_x->PWM.Duty        = ((APB1_TIMER_CLK / 1000) * min_time_ms) / (H_Bridge_x->PWM.Prescaler * 100);
//     H_Bridge_x->PWM.Duty        = (H_Bridge_x->PWM.Duty > SD_DUTY_MIN) ? H_Bridge_x->PWM.Duty : SD_DUTY_MIN;

//     H_Bridge_x->delay_time_ms   = Set_delay_time_ms;

//     H_Bridge_x->on_time_ms      = Set_on_time_ms;
//     H_Bridge_x->off_time_ms     = Set_off_time_ms;

//     H_Bridge_x->set_pulse_count = Set_pulse_count;
//     H_Bridge_x->pulse_count     = 0;

//     LL_TIM_SetPrescaler(H_Bridge_x->PWM.TIMx, H_Bridge_x->PWM.Prescaler);
//     LL_TIM_GenerateEvent_UPDATE(H_Bridge_x->PWM.TIMx);
//     LL_TIM_ClearFlag_UPDATE(H_Bridge_x->PWM.TIMx);

//     LL_TIM_EnableIT_UPDATE(H_Bridge_x->PWM.TIMx);
//     LL_TIM_EnableCounter(H_Bridge_x->PWM.TIMx);
// }
