/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Include~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include "vom_driver.h"

#include "app.h"
#include "stm32f4xx_ll_spi.h"

#include "spi.h"
#include "crc.h"
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Defines ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#define SPI_BUFFER_SIZE 64

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Enum ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Struct ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Class ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Private Types ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
// static void fsp_print(uint8_t packet_length);

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
spi_stdio_typedef VOM_SPI;

SPI_TX_buffer_t g_VOM_SPI_TX_buffer[2048];
uint8_t g_VOM_SPI_RX_buffer[5012];
uint8_t g_VOM_temp_SPI_RX_buffer[6];

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* :::::::::: VOM Driver Init :::::::: */
void VOM_Driver_Init(void)
{
    SPI_Init(&VOM_SPI, VOM_SPI_HANDLE, VOM_SPI_IRQ,
            g_VOM_temp_SPI_RX_buffer, 41,
			g_VOM_SPI_TX_buffer, 2048,
			g_VOM_SPI_RX_buffer, 5012,
            VOM_SPI_CS_PORT, VOM_SPI_CS_PIN);
    
    SPI_TX_data_t SPI_data_array[5] = {0};
    SPI_data_array[0].data = 0b00000000;
    SPI_data_array[0].mask = 0b00000000;
    SPI_data_array[1].data = 0b00010000;
    SPI_data_array[1].mask = 0b00010000;

    SPI_frame_t SPI_frame =
    {
        .addr = 0x02,
        .p_data_array = SPI_data_array,
        .data_size = 2,
    };
    
    SPI_Write(&VOM_SPI, &SPI_frame);

    SPI_data_array[0].data = 0b10110000;
    SPI_data_array[0].mask = 0xFF;
    SPI_data_array[1].data = 0b00110110;
    SPI_data_array[1].mask = 0xFF;

    SPI_frame.addr = 0x0C;
    SPI_frame.p_data_array = SPI_data_array;
    SPI_frame.data_size = 2;
    
    SPI_Write(&VOM_SPI, &SPI_frame);

    SPI_data_array[0].data = 0b00100000;
    SPI_data_array[0].mask = 0xFF;
    SPI_data_array[1].data = 0b00000000;
    SPI_data_array[1].mask = 0xFF;

    SPI_frame.addr = 0x0E;
    SPI_frame.p_data_array = SPI_data_array;
    SPI_frame.data_size = 2;
    
    SPI_Write(&VOM_SPI, &SPI_frame);

    SPI_data_array[0].data = 0b00000000;
    SPI_data_array[0].mask = 0xFF;
    SPI_data_array[1].data = 0b00010100;
    SPI_data_array[1].mask = 0xFF;

    SPI_frame.addr = 0x10;
    SPI_frame.p_data_array = SPI_data_array;
    SPI_frame.data_size = 2;
    
    SPI_Write(&VOM_SPI, &SPI_frame);

    SPI_data_array[0].data = 0b00000000;
    SPI_data_array[0].mask = 0xFF;
    SPI_data_array[1].data = 0b00010100;
    SPI_data_array[1].mask = 0xFF;

    SPI_frame.addr = 0x11;
    SPI_frame.p_data_array = SPI_data_array;
    SPI_frame.data_size = 2;
    
    SPI_Write(&VOM_SPI, &SPI_frame);

    // SPI_frame.addr = 0x0C;
    // SPI_frame.p_data_array = SPI_data_array;
    // SPI_frame.data_size = 2;
    
    // SPI_Read(&VOM_SPI, &SPI_frame);
}

/* :::::::::: VOM Build ADC_CONFIG Frame :::::::: */
bool VOM_Build_ADC_CONFIG_Frame(const VOM_Config_t* config, SPI_frame_t* out_frame, SPI_TX_data_t* out_data_array)
{
    if (!config || !out_frame || !out_data_array || config->measure_mode == 0)
    {
        return false;
    }

    uint8_t avg = (config->avg_vsh > config->avg_vbus) ? config->avg_vsh : config->avg_vbus;
    if (avg > 127)
    {
        avg = 0;
    }

    uint16_t value = 0;
    value |= (config->measure_mode << 12); // MODE
    value |= (config->vbus_ct      << 9);  // VBUS_CT
    value |= (config->vsh_ct       << 6);  // VSH_CT
    value |= (avg << 0);                   // AVG

    // Build data array (MSB first)
    out_data_array[0].data = value & 0xFF;
    out_data_array[0].mask = 0xC7;
    out_data_array[1].data = (value >> 8) & 0xFF;
    out_data_array[1].mask = 0xFF;

    // Fill frame
    out_frame->addr = 0x01; // CONFIG register
    out_frame->p_data_array = out_data_array;
    out_frame->data_size = 2;

    return true;
}

/* :::::::::: VOM Data Process :::::::: */
void VOM_Data_Process(spi_stdio_typedef* p_spi)
{
    uint8_t VOM_data_size = 4;

    uint8_t sequence_count = 0, task_data_count = 0, pulse_count = 0, sampling_on_count = 0, sampling_off_count = 0;

    for (sequence_count = 0; (HB_Task_data[sequence_count].is_setted == true) && (sequence_count < 10); sequence_count++)
    {
        for (task_data_count = 0; (HB_Task_data[sequence_count].task_data[task_data_count].is_setted == true) && (task_data_count < 4); task_data_count++)
        {
            uint8_t VOM_on_state, VOM_off_state;
            if ((task_data_count == 0) || (task_data_count == 1))
            {
                VOM_on_state = VOM_HV_ON;
                VOM_off_state = VOM_HV_OFF;
            }
            else
            {
                VOM_on_state = VOM_LV_ON;
                VOM_off_state = VOM_LV_OFF;
            }

            for (pulse_count = 0; pulse_count < HB_Task_data[sequence_count].task_data[task_data_count].HB_pole_pulse.set_pulse_count; pulse_count++)
            {   
                for (sampling_on_count = 0; sampling_on_count < 20; sampling_on_count++)
                {
                    p_spi->p_RX_buffer[p_spi->RX_read_index] = (p_spi->p_RX_buffer[p_spi->RX_read_index] & ~(0x03)) | (VOM_on_state & 0x03);

                    for (uint8_t i = 0; i < VOM_data_size; i++)
                    {
                        if (SPI_RX_BUFFER_EMPTY(p_spi))
                        {
                            return;
                        }
                        
                        SPI_ADVANCE_RX_READ_INDEX(p_spi);
                    }
                }
                
                for (sampling_off_count = 0; sampling_off_count < 20; sampling_off_count++)
                {
                    p_spi->p_RX_buffer[p_spi->RX_read_index] = (p_spi->p_RX_buffer[p_spi->RX_read_index] & ~(0x03)) | (VOM_off_state & 0x03);

                    for (uint8_t i = 0; i < VOM_data_size; i++)
                    {
                        if (SPI_RX_BUFFER_EMPTY(p_spi))
                        {
                            return;
                        }
                        
                        SPI_ADVANCE_RX_READ_INDEX(p_spi);
                    }
                }
            }
        }
    }
}

/* :::::::::: VOM SPI Interupt Handler ::::::::::::: */
void VOM_driver_SPI_IRQHandler(void)
{
    if (LL_SPI_IsActiveFlag_RXNE(VOM_SPI.handle) == true)
	{
        VOM_SPI.p_temp_RX_buffer[VOM_SPI.temp_RX_index] = LL_SPI_ReceiveData8(VOM_SPI.handle);

        if (VOM_SPI.p_TX_buffer[VOM_SPI.TX_read_index].command == SPI_READ)
        {
            if (VOM_SPI.p_TX_buffer[VOM_SPI.TX_read_index].data_type == SPI_HEADER)
            {
                VOM_SPI.p_temp_RX_buffer[VOM_SPI.temp_RX_index] = VOM_SPI.p_TX_buffer[VOM_SPI.TX_read_index].data;
            }
            
            VOM_SPI.temp_RX_index++;
        }

        if (VOM_SPI.p_TX_buffer[VOM_SPI.TX_read_index].data_type == SPI_ENDER)
        {
            LL_GPIO_SetOutputPin(VOM_SPI.cs_port, VOM_SPI.cs_pin);
        }
        
        SPI_ADVANCE_TX_READ_INDEX(&VOM_SPI);

		if (SPI_TX_BUFFER_EMPTY(&VOM_SPI))
		{
			//LL_GPIO_SetOutputPin(VOM_SPI.cs_port, VOM_SPI.cs_pin);

			SPI_flush_temp_to_RX_buffer(&VOM_SPI);

			// Buffer empty, so disable interrupts
            LL_SPI_DisableIT_RXNE(VOM_SPI.handle);
		}
		else
		{
			// There is more data in the output buffer. Send the next byte
			SPI_Prime_Transmit(&VOM_SPI);
		}
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
//static void fsp_print(uint8_t packet_length)
//{
//    s_FSP_TX_Packet.sod 		= FSP_PKT_SOD;
//    s_FSP_TX_Packet.src_adr 	= fsp_my_adr;
//    s_FSP_TX_Packet.dst_adr 	= FSP_ADR_GPC;
//    s_FSP_TX_Packet.length 	    = packet_length;
//    s_FSP_TX_Packet.type 		= FSP_PKT_TYPE_CMD_W_DATA;
//    s_FSP_TX_Packet.eof 		= FSP_PKT_EOF;
//    s_FSP_TX_Packet.crc16 		= crc16_CCITT(FSP_CRC16_INITIAL_VALUE, &s_FSP_TX_Packet.src_adr, s_FSP_TX_Packet.length + 4);
//
//    uint8_t encoded_frame[20] = { 0 };
//    uint8_t frame_len;
//    fsp_encode(&s_FSP_TX_Packet, encoded_frame, &frame_len);
//
//    UART_FSP(&GPC_UART, (char*)encoded_frame, frame_len);
//}
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ End of the program ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

// void VOM_driver_SPI_IRQHandler(void)
// {
//     if ((VOM_SPI.current_state == SPI_READ_HEADER) || (VOM_SPI.current_state == SPI_READ_DATA))
//     {
//         VOM_SPI.RX_irq_char = LL_SPI_ReceiveData8(VOM_SPI.handle);

//         if (!SPI_RX_BUFFER_FULL(&VOM_SPI))
//         {
//             VOM_SPI.p_RX_buffer[VOM_SPI.RX_write_index]         = VOM_SPI.RX_irq_char;
//             VOM_SPI.p_RX_state_buffer[VOM_SPI.RX_write_index]   = VOM_SPI.current_state;
//             SPI_ADVANCE_RX_WRITE_INDEX(&VOM_SPI);
//         }
//     }

//     if (LL_SPI_IsActiveFlag_TXE(VOM_SPI.handle) == true)
//     {
// 		if (SPI_TX_BUFFER_EMPTY(&VOM_SPI))
//         {
//             LL_GPIO_SetOutputPin(VOM_SPI.cs_port, VOM_SPI.cs_pin);

// 			// Buffer empty, so disable interrupts
// 			LL_SPI_DisableIT_TXE(VOM_SPI.handle);
// 		}
//         else
//         {
// 			// There is more data in the output buffer. Send the next byte
// 			SPI_Prime_Transmit(&VOM_SPI);
// 		}
// 	}
// }
