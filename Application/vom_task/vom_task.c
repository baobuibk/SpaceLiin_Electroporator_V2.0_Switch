/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Include~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include "app.h"

#include "vom_task.h"

#include "crc.h"

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Defines ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Enum ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Struct ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Class ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Private Types ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
typedef enum
{
    VOM_INITIAL_SET_STATE,
    VOM_CALCULATE_IMPEDANCE,
} VOM_State_t;

typedef enum
{
    VOM_OVC_FIRST_SIGNAL,
    VOM_OVC_RESET_ACTION,
    VOM_OVC_DATA_PROCESS,
    VOM_OVC_WAIT_FOR_RESET,
} VOM_OVC_State_t;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
static VOM_State_t VOM_State = VOM_INITIAL_SET_STATE;
static VOM_OVC_State_t VOM_OVC_State = VOM_OVC_FIRST_SIGNAL;

//static bool is_OVC_signal_elapse = false;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
__STATIC_INLINE void VOM_SPI_Start_ADC(spi_stdio_typedef* p_spi, SPI_frame_t* p_SPI_frame);
__STATIC_INLINE void VOM_SPI_Read_ADC(spi_stdio_typedef* p_spi);
__STATIC_INLINE void VOM_SPI_Stop_ADC(spi_stdio_typedef* p_spi);

static void     VOM_Task_Data_Process(spi_stdio_typedef* p_spi);
static uint8_t  VOM_OVC_Data_Process(spi_stdio_typedef* p_spi);
static void     fsp_print(uint8_t packet_length);

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
bool is_Measure_Impedance = false;
uint16_t Current_Sense_Period = 100;

H_Bridge_Task_typedef VOM_HB_Task_data;

bool OVC_flag_signal   = false;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* :::::::::: VOM Task :::::::: */
void VOM_Task(void*)
{
    if(is_Measure_Impedance == false)
    {
        V_Switch_Set_Mode(V_SWITCH_MODE_ALL_OFF);

        H_Bridge_Set_Mode(&VOM_HB_Task_data.task_data[0].HB_pole_pulse, H_BRIDGE_MODE_FLOAT);
        H_Bridge_Set_Mode(&VOM_HB_Task_data.task_data[0].HB_pole_ls_on, H_BRIDGE_MODE_FLOAT);
        LL_GPIO_ResetOutputPin(PULSE_LED_PORT,PULSE_LED_PIN);

        // H_Bridge_Set_Mode(&HB_pos_pole, H_BRIDGE_MODE_FLOAT);
        // H_Bridge_Set_Mode(&HB_neg_pole, H_BRIDGE_MODE_FLOAT);
        // LL_GPIO_ResetOutputPin(PULSE_LED_PORT,PULSE_LED_PIN);

        SchedulerTaskDisable(VOM_TASK);
        return;
    }

    switch (VOM_State)
    {
    case VOM_INITIAL_SET_STATE:
    {
        if (VOM_HB_Task_data.is_setted == false)
        {
            is_Measure_Impedance = false;
            break;
        }
        
        LL_GPIO_SetOutputPin(PULSE_LED_PORT,PULSE_LED_PIN);

        V_Switch_Set_Mode(VOM_HB_Task_data.task_data[0].VS_mode);

        H_Bridge_Set_Mode(&VOM_HB_Task_data.task_data[0].HB_pole_ls_on, H_BRIDGE_MODE_LS_ON);
        H_Bridge_Set_Mode(&VOM_HB_Task_data.task_data[0].HB_pole_pulse, H_BRIDGE_MODE_PULSE);

        VOM_State = VOM_CALCULATE_IMPEDANCE;

        break;
    }

    case VOM_CALCULATE_IMPEDANCE:
    {
        if(VOM_HB_Task_data.task_data[0].HB_pole_pulse.pulse_count < (VOM_HB_Task_data.task_data[0].HB_pole_pulse.set_pulse_count + 1))
        {
            break;
        }

        VOM_Task_Data_Process(&VOM_SPI);

        is_Measure_Impedance = false;

        VOM_State = VOM_INITIAL_SET_STATE;
        break;
    }
    default:
        break;
    }
}

void VOM_OVC_Task(void*)
{
    switch (VOM_OVC_State)
    {
    case VOM_OVC_FIRST_SIGNAL:
    {
        if (OVC_flag_signal == false)
        {
            return;
        }

        VOM_OVC_State = VOM_OVC_RESET_ACTION;

        break;
    }
        
    case VOM_OVC_RESET_ACTION:
    {
        uint32_t VOM_OVC_port = LL_GPIO_ReadInputPort(VOM_OVC_PORT);
        VOM_OVC_port = ((VOM_OVC_port & VOM_OVC_PIN) != 0) ? SET : RESET;
        if (VOM_OVC_port == SET)
        {
            OVC_flag_signal = false;

            VOM_OVC_State = VOM_OVC_FIRST_SIGNAL;

            break;
        }

        is_h_bridge_enable = false;
        
        V_Switch_Set_Mode(V_SWITCH_MODE_ALL_OFF);

        H_Bridge_Set_Mode(&HB_pos_pole, H_BRIDGE_MODE_FLOAT);
        H_Bridge_Set_Mode(&HB_neg_pole, H_BRIDGE_MODE_FLOAT);
        LL_GPIO_ResetOutputPin(PULSE_LED_PORT,PULSE_LED_PIN);

        UART_Send_String(&RS232_UART, "OVER CURRENT DETECTED\n> ");
        
        ps_FSP_TX->CMD                 = FSP_CMD_OVER_CURRENT_DETECT;
        ps_FSP_TX->Payload.ovc_current_detect.OVC_flag_signal = true;
        fsp_print(2);

        SchedulerTaskDisable(H_BRIDGE_TASK);

        // VOM_OVC_State = VOM_OVC_DATA_PROCESS;
        VOM_OVC_State = VOM_OVC_WAIT_FOR_RESET;

        break;
    }

    case VOM_OVC_DATA_PROCESS:
    {
        if (VOM_OVC_Data_Process(&VOM_SPI) == 1)
        {
            VOM_OVC_State = VOM_OVC_WAIT_FOR_RESET;
            break;
        }

        if (OVC_flag_signal == false)
        {
            VOM_OVC_State = VOM_OVC_WAIT_FOR_RESET;
            break;
        }

        break;
    }

    case VOM_OVC_WAIT_FOR_RESET:
    {
        if (OVC_flag_signal == true)
        {
            return;
        }

        VOM_Reset_OVC_Flag(&VOM_SPI);
        
        UART_Send_String(&RS232_UART, "OVER CURRENT FLAG RESET\n> ");

        SchedulerTaskEnable(H_BRIDGE_TASK, true);

        VOM_OVC_State = VOM_OVC_FIRST_SIGNAL;

        break;
    }
    
    default:
        break;
    }
}

void VOM_OVC_IRQHandler(void)
{
    OVC_flag_signal = true;

    // VOM_SPI_Read_ADC(&VOM_SPI);
    // VOM_SPI_Stop_ADC(&VOM_SPI);
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
__STATIC_INLINE void VOM_SPI_Start_ADC(spi_stdio_typedef* p_spi, SPI_frame_t* p_SPI_frame)
{
    SPI_Write(p_spi, p_SPI_frame);
}

__STATIC_INLINE void VOM_SPI_Read_ADC(spi_stdio_typedef* p_spi)
{
    SPI_frame_t SPI_frame;
    
    SPI_frame.addr = 0x04,
    SPI_frame.data_size = 3,

    SPI_Read(p_spi, &SPI_frame);

    SPI_frame.addr = 0x05,
    SPI_frame.data_size = 3,

    SPI_Read(p_spi, &SPI_frame);
}

__STATIC_INLINE void VOM_SPI_Stop_ADC(spi_stdio_typedef* p_spi)
{
    SPI_TX_data_t SPI_TX_data[2] = {0};
    SPI_TX_data[0].mask = 0xFF;
    SPI_TX_data[1].mask = 0xFF;
    
    SPI_frame_t SPI_frame =
    {
        .addr = 0x01,
        .p_data_array = SPI_TX_data,
        .data_size = 2,
    };

    SPI_Write(p_spi, &SPI_frame);
}

static void VOM_Task_Data_Process(spi_stdio_typedef* p_spi)
{
    float current[10] = {0};
    float volt[10] = {0};

    float    impedance_sum = 0.0;
    uint16_t impedance_average = 0;

    int32_t current_temp = 0;
    int32_t volt_temp = 0;

    for (uint8_t Idx = 0; Idx < 10; Idx++)
    {
        SPI_ADVANCE_RX_READ_INDEX(p_spi);

        current_temp = (int32_t)p_spi->p_RX_buffer[p_spi->RX_read_index] << 12;
        SPI_ADVANCE_RX_READ_INDEX(p_spi);

        current_temp |= (int32_t)p_spi->p_RX_buffer[p_spi->RX_read_index] << 4;
        SPI_ADVANCE_RX_READ_INDEX(p_spi);

        current_temp |= (int32_t)p_spi->p_RX_buffer[p_spi->RX_read_index] >> 4;
        SPI_ADVANCE_RX_READ_INDEX(p_spi);

        SPI_ADVANCE_RX_READ_INDEX(p_spi);

        volt_temp = (int32_t)p_spi->p_RX_buffer[p_spi->RX_read_index] << 12;
        SPI_ADVANCE_RX_READ_INDEX(p_spi);

        volt_temp |= (int32_t)p_spi->p_RX_buffer[p_spi->RX_read_index] << 4;
        SPI_ADVANCE_RX_READ_INDEX(p_spi);

        volt_temp |= (int32_t)p_spi->p_RX_buffer[p_spi->RX_read_index] >> 4;
        SPI_ADVANCE_RX_READ_INDEX(p_spi);

        // Shift range from [-8,388,608, +8,388,607] → [0, 16,777,215]
        current[Idx] = (float)current_temp;
        //current[Idx] = (float)current[Idx] * 0.16384;
        current[Idx] = current[Idx] * 0.000022321428;
        
        volt[Idx]    = (float)volt_temp;
        // 0.000703438335 * 1.030622388
        volt[Idx]    = volt[Idx] * 0.0007249792963;

        current_temp = 0;
        volt_temp    = 0;
    }

    for (uint8_t i = 0; i < 80; i++)
    {
        if (SPI_RX_BUFFER_EMPTY(p_spi))
        {
            break;
        }
        
        SPI_ADVANCE_RX_READ_INDEX(p_spi);
    }

    for (uint8_t Idx = 0; Idx < 10; Idx++)
    {
        impedance_sum += volt[Idx] / current[Idx];
    }
    
    impedance_average = impedance_sum / 10;

    ps_FSP_TX->CMD = FSP_CMD_MEASURE_IMPEDANCE;
    ps_FSP_TX->Payload.measure_impedance.Value_low  =  impedance_average;
    ps_FSP_TX->Payload.measure_impedance.Value_high = (impedance_average >> 8);

    fsp_print(7);
}

static uint8_t VOM_OVC_Data_Process(spi_stdio_typedef* p_spi)
{
    if (SPI_RX_BUFFER_EMPTY(p_spi))
    {
        return 0;
    }

    float current = 0.0;
    float volt = 0.0;

    uint16_t impedance_average = 0;

    int32_t current_temp = 0;
    int32_t volt_temp = 0;

    SPI_ADVANCE_RX_READ_INDEX(p_spi);

    current_temp = (int32_t)p_spi->p_RX_buffer[p_spi->RX_read_index] << 12;
    SPI_ADVANCE_RX_READ_INDEX(p_spi);

    current_temp |= (int32_t)p_spi->p_RX_buffer[p_spi->RX_read_index] << 4;
    SPI_ADVANCE_RX_READ_INDEX(p_spi);

    current_temp |= (int32_t)p_spi->p_RX_buffer[p_spi->RX_read_index] >> 4;
    SPI_ADVANCE_RX_READ_INDEX(p_spi);

    SPI_ADVANCE_RX_READ_INDEX(p_spi);

    volt_temp = (int32_t)p_spi->p_RX_buffer[p_spi->RX_read_index] << 12;
    SPI_ADVANCE_RX_READ_INDEX(p_spi);

    volt_temp |= (int32_t)p_spi->p_RX_buffer[p_spi->RX_read_index] << 4;
    SPI_ADVANCE_RX_READ_INDEX(p_spi);

    volt_temp |= (int32_t)p_spi->p_RX_buffer[p_spi->RX_read_index] >> 4;
    SPI_ADVANCE_RX_READ_INDEX(p_spi);

    // Shift range from [-8,388,608, +8,388,607] → [0, 16,777,215]
    current = (float)current_temp;
    current = current * 0.000022321428;
    
    volt    = (float)volt_temp;
    volt    = volt * 0.0007249792963;

    current_temp = 0;
    volt_temp    = 0;

    for (uint8_t i = 0; i < 80; i++)
    {
        if (SPI_RX_BUFFER_EMPTY(p_spi))
        {
            break;
        }
        
        SPI_ADVANCE_RX_READ_INDEX(p_spi);
    }
    
    impedance_average = volt / current;

    return 1;
}

//static uint32_t Convert_2_complement_to_unsign(int32_t val)
//{
//    // Giới hạn giá trị trong khoảng 24-bit 2's complement: [-8388608, +8388607]
//    if (val >  8388607)
//    {
//        val =  8388607;
//    }
//
//    if (val < -8388608)
//    {
//        val = -8388608;
//    }
//
//    // Mask chỉ lấy 24-bit thấp nhất
//    return (((uint32_t)val) & 0xFFFFFF);
//}

static void fsp_print(uint8_t packet_length)
{
   s_FSP_TX_Packet.sod 		= FSP_PKT_SOD;
   s_FSP_TX_Packet.src_adr 	= fsp_my_adr;
   s_FSP_TX_Packet.dst_adr 	= FSP_ADR_GPC;
   s_FSP_TX_Packet.length 	= packet_length;
   s_FSP_TX_Packet.type     = FSP_PKT_TYPE_CMD_W_DATA;
   s_FSP_TX_Packet.eof 		= FSP_PKT_EOF;
   s_FSP_TX_Packet.crc16 	= crc16_CCITT(FSP_CRC16_INITIAL_VALUE, &s_FSP_TX_Packet.src_adr, s_FSP_TX_Packet.length + 4);

   uint8_t encoded_frame[20] = { 0 };
   uint8_t frame_len;
   fsp_encode(&s_FSP_TX_Packet, encoded_frame, &frame_len);

   UART_FSP(&GPC_UART, (char*)encoded_frame, frame_len);
}
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ End of the program ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
