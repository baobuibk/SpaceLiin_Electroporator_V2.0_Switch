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
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
spi_stdio_typedef VOM_SPI;

SPI_TX_buffer_t g_VOM_SPI_TX_buffer[1024];
uint8_t g_VOM_SPI_RX_buffer[96000];
uint8_t g_VOM_temp_SPI_RX_buffer[6];

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* :::::::::: VOM Driver Init :::::::: */
void VOM_Driver_Init(void)
{
    SPI_Init(&VOM_SPI, VOM_SPI_HANDLE, VOM_SPI_IRQ,
            g_VOM_temp_SPI_RX_buffer, 6,
			g_VOM_SPI_TX_buffer, 1024,
			g_VOM_SPI_RX_buffer, 96000,
            VOM_SPI_CS_PORT, VOM_SPI_CS_PIN);
    
    SPI_TX_data_t SPI_data_array[5] = {0};
    SPI_frame_t SPI_frame;

    SPI_data_array[0].data = 0b00000000;
    SPI_data_array[0].mask = 0xFF;
    SPI_data_array[1].data = 0b00010000;
    SPI_data_array[1].mask = 0xFF;

    SPI_frame.addr = 0x02;
    SPI_frame.data_size = 2;
    SPI_frame.p_data_array = SPI_data_array;
    
    SPI_Overwrite(&VOM_SPI, &SPI_frame);

    VOM_Shunt_Overvoltage_Threshold(&VOM_SPI, 5.0);

    VOM_Shunt_Undervoltage_Threshold(&VOM_SPI, -5.0);

    VOM_Bus_Overvoltage_Threshold(&VOM_SPI, 306.0);

    VOM_Bus_Undervoltage_Threshold(&VOM_SPI, 0.0);

    SPI_data_array[0].data = 0b00000000;
    SPI_data_array[0].mask = 0x00;
    SPI_data_array[1].data = 0b10000000;
    SPI_data_array[1].mask = 0xFF;

    SPI_frame.addr = 0x0B;
    SPI_frame.data_size = 2;
    SPI_frame.p_data_array = SPI_data_array;
    
    SPI_Write(&VOM_SPI, &SPI_frame);
}

/* :::::::::: VOM Build ADC_CONFIG Frame :::::::: */
bool VOM_Build_ADC_CONFIG_Frame(const VOM_Config_t* config, SPI_frame_t* out_frame, SPI_TX_data_t* out_data_array)
{
    if (!config || !out_frame || !out_data_array || config->measure_mode == 0)
    {
        return false;
    }

    uint16_t value = 0;
    value |= (config->measure_mode << 12); // MODE
    value |= (config->vbus_ct      << 9);  // VBUS_CT
    value |= (config->vsh_ct       << 6);  // VSH_CT
    value |= (config->avg          << 0);  // AVG

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
__STATIC_INLINE bool VOM_Raw_Data_Process(spi_stdio_typedef* p_spi, uint16_t* p_current_mA, uint16_t* p_volt_mV)
{
    const uint8_t SAMPLE_BYTE_SIZE = 3;

    uint8_t current_raw_temp[3] = {0};
    uint8_t volt_raw_temp[3] = {0};

    int32_t current_raw = 0;
    int32_t volt_raw = 0;

    float current_float = 0.0;
    float volt_float = 0.0;

    uint8_t* p_RX_buffer_temp = p_spi->p_RX_buffer;

    for (uint8_t Idx = 0; Idx < SAMPLE_BYTE_SIZE; ++Idx)
    {
        if (SPI_RX_BUFFER_EMPTY(p_spi))
        {
            return false; // not enough data → abort
        }

        current_raw_temp[Idx] = p_RX_buffer_temp[p_spi->RX_read_index];
        SPI_ADVANCE_RX_READ_INDEX(p_spi);
    }

    for (uint8_t Idx = 0; Idx < SAMPLE_BYTE_SIZE; ++Idx)
    {
        if (SPI_RX_BUFFER_EMPTY(p_spi))
        {
            return false; // not enough data → abort
        }

        volt_raw_temp[Idx] = p_RX_buffer_temp[p_spi->RX_read_index];
        SPI_ADVANCE_RX_READ_INDEX(p_spi);
    }

    // Merge each 8 bits raw data to a 24 bits int raw data
    current_raw =  (int32_t)current_raw_temp[0] << 12;
    current_raw |= (int32_t)current_raw_temp[1] << 4;
    current_raw |= (int32_t)current_raw_temp[2] >> 4;

    // Merge each 8 bits raw data to a 24 bits int raw data
    volt_raw =  (int32_t)volt_raw_temp[0] << 12;
    volt_raw |= (int32_t)volt_raw_temp[1] << 4;
    volt_raw |= (int32_t)volt_raw_temp[2] >> 4;

    // Transform from raw data to current (A)
    // raw_data: Differential VOLTAGE measured across the shunt output. Two's complement value.
    // 312.5E-9: Conversion factor 312.5 nV/LSB when ADCRANGE = 0.
    // 14E-3   : INA229 shunt value
    // raw_data * (312.5E-9 / 14E-3) = raw_data * 0.000022321428.
    current_float = (float)current_raw * 0.000022321428;
    
    // Transform from raw data to volt (V)
    // raw_data   : Bus voltage output. Two's complement value, however always positive.
    // 195.3125E-6: Conversion factor 195.3125 µV/LSB.
    // 1347 / 374 : (37.4 + 3.3 + 47 + 47) / 37.4: Volt div circuit (Resistor in K.Ohm).
    // 1.030622387: Magic calib number.
    // raw_data * 195.3125E-6 * (1347 / 374) * 1.030622387 = raw_data * 0.0007249792963.
    volt_float = (float)volt_raw * 0.0007249792963;

    *p_current_mA = current_float * 1000;
    *p_volt_mV    = volt_float * 1000;

    return true;
}

// Helper: process a block of samples in a given state (ON or OFF)
// Tags each sample with `state` (2 LSBs) and advances the SPI RX read index
// Returns false if the SPI buffer runs empty (early exit)
__STATIC_INLINE bool VOM_Process_Raw_Sample(
                                        spi_stdio_typedef* p_spi,
                                        VOM_Pulse_State_t  pulse_state,
                                        VOM_Task_State_t   task_state,  

                                        uint8_t sample_count,
                                        uint8_t current_sequence)
{
    uint16_t current_mA, volt_mV;

    for (uint8_t sample_Idx = 0; sample_Idx < sample_count; ++sample_Idx)
    {
        if (!VOM_Raw_Data_Process(p_spi, &current_mA, &volt_mV))
        {
            return false; // not enough data → abort
        }

        p_spi->p_RX_buffer[p_spi->RX_write_index] =  pulse_state;
        p_spi->p_RX_buffer[p_spi->RX_write_index] |= task_state << 1;
        p_spi->p_RX_buffer[p_spi->RX_write_index] |= VOM_CURRENT_TYPE << 2;
        p_spi->p_RX_buffer[p_spi->RX_write_index] |= current_sequence << 3;
        SPI_ADVANCE_RX_WRITE_INDEX(p_spi);

        p_spi->p_RX_buffer[p_spi->RX_write_index] = current_mA;
        SPI_ADVANCE_RX_WRITE_INDEX(p_spi);

        p_spi->p_RX_buffer[p_spi->RX_write_index] = current_mA >> 8;
        SPI_ADVANCE_RX_WRITE_INDEX(p_spi);

        p_spi->p_RX_buffer[p_spi->RX_write_index] =  pulse_state;
        p_spi->p_RX_buffer[p_spi->RX_write_index] |= task_state << 1;
        p_spi->p_RX_buffer[p_spi->RX_write_index] |= VOM_VOLT_TYPE << 2;
        p_spi->p_RX_buffer[p_spi->RX_write_index] |= current_sequence << 3;
        SPI_ADVANCE_RX_WRITE_INDEX(p_spi);

        p_spi->p_RX_buffer[p_spi->RX_write_index] = volt_mV;
        SPI_ADVANCE_RX_WRITE_INDEX(p_spi);

        p_spi->p_RX_buffer[p_spi->RX_write_index] = volt_mV >> 8;
        SPI_ADVANCE_RX_WRITE_INDEX(p_spi);
    }

    return true;
}

// Main data-processing routine: labels and consumes VOM samples from INA229
void VOM_Data_Process(spi_stdio_typedef* p_spi)
{
    const uint8_t SAMPLES_PER_PULSE = 10;
    const uint8_t SAMPLE_BYTE_SIZE  = 6;   // bytes per sample from INA229

    const uint8_t MAX_SEQUENCE_COUNT = 6;
    const uint8_t MAX_TASK_COUNT     = 4;
    const uint8_t MAX_PULSE_COUNT    = 20;

    // Loop through up to 6 sequences
    for (uint8_t sequence_Idx = 0; (sequence_Idx < MAX_SEQUENCE_COUNT) && (HB_Task_data[sequence_Idx].is_setted); ++sequence_Idx)
    {
        // Loop through up to 4 task_data entries per sequence
        for (uint8_t task_Idx = 0; (task_Idx < MAX_TASK_COUNT) && (HB_Task_data[sequence_Idx].task_data[task_Idx].is_setted); ++task_Idx)
        {
            // Number of pulses to process for this task
            uint8_t set_pulse_count = HB_Task_data[sequence_Idx].task_data[task_Idx].HB_pole_pulse.set_pulse_count;

            //
            // In the first pulse of every task:
            // - ON pulse: only have 9 samples (6 bytes per) or 18 sample (3 bytes per).
            // - OFF pulse: have 10 samples (6 bytes per) or 18 sample (3 bytes per).
            //
            // Process ON_pulse samples; abort if buffer underflow
            if (!VOM_Process_Raw_Sample(p_spi, VOM_PULSE_ON, (VOM_Task_State_t)task_Idx, (SAMPLES_PER_PULSE - 1), sequence_Idx))
            {
                return;
            }

            // Process OFF_pulse samples; abort if buffer underflow
            if (!VOM_Process_Raw_Sample(p_spi, VOM_PULSE_OFF, (VOM_Task_State_t)task_Idx, SAMPLES_PER_PULSE, sequence_Idx))
            {
                return;
            }

            //
            // After the first pulse of every task:
            // - ON pulse: have 10 samples (6 bytes per) or 18 sample (3 bytes per).
            // - OFF pulse: have 10 samples (6 bytes per) or 18 sample (3 bytes per).
            //
            // For each pulse: do SAMPLES_PER_PULSE for ON_pulse, then OFF_pulse
            for (uint8_t pulse_Idx = 1; pulse_Idx < set_pulse_count; ++pulse_Idx)
            {
                // Process ON_pulse samples; abort if buffer underflow
                if (!VOM_Process_Raw_Sample(p_spi, VOM_PULSE_ON, (VOM_Task_State_t)task_Idx, SAMPLES_PER_PULSE, sequence_Idx))
                {
                    return;
                }

                // Process OFF_pulse samples; abort if buffer underflow
                if (!VOM_Process_Raw_Sample(p_spi, VOM_PULSE_OFF, (VOM_Task_State_t)task_Idx, SAMPLES_PER_PULSE, sequence_Idx))
                {
                    return;
                }
            }
        }
    }
}

void VOM_Reset_OVC_Flag(spi_stdio_typedef* p_spi)
{
    SPI_frame_t SPI_frame;
    
    SPI_frame.addr = 0x0B,
    SPI_frame.data_size = 2,

    SPI_Read(p_spi, &SPI_frame, 1);
}

void VOM_Shunt_Overvoltage_Threshold(spi_stdio_typedef* p_spi, float current_A)
{
    SPI_TX_data_t s_SPI_data_array[2];
    SPI_frame_t s_SPI_frame;

    // 2.8 * 1000 = 14E-3 / 5E-6 = (14 / 5) * (1 / 10E-3)
    // R_SHUNT = 14E-3
    // CONVERSION FACTOR = 5E-6
    int16_t threshold = current_A * 2.8 * 1000.0;

    s_SPI_data_array[0].data = threshold;
    s_SPI_data_array[0].mask = 0xff;
    s_SPI_data_array[1].data = threshold >> 8;
    s_SPI_data_array[1].mask = 0xff;

    s_SPI_frame.addr = 0x0C;
    s_SPI_frame.data_size = 2;
    s_SPI_frame.p_data_array = s_SPI_data_array;

    SPI_Overwrite(p_spi, &s_SPI_frame);
}

void VOM_Shunt_Undervoltage_Threshold(spi_stdio_typedef* p_spi, float current_A)
{
    SPI_TX_data_t s_SPI_data_array[2];
    SPI_frame_t s_SPI_frame;

    // 2.8 * 1000 = 14E-3 / 5E-6 = (14 / 5) * (1 / 10E-3)
    // R_SHUNT = 14E-3
    // CONVERSION FACTOR = 5E-6
    int16_t threshold = current_A * 2.8 * 1000.0;

    s_SPI_data_array[0].data = threshold;
    s_SPI_data_array[0].mask = 0xff;
    s_SPI_data_array[1].data = threshold >> 8;
    s_SPI_data_array[1].mask = 0xff;

    s_SPI_frame.addr = 0x0D;
    s_SPI_frame.data_size = 2;
    s_SPI_frame.p_data_array = s_SPI_data_array;

    SPI_Overwrite(p_spi, &s_SPI_frame);
}

void VOM_Bus_Overvoltage_Threshold(spi_stdio_typedef* p_spi, float volt_V)
{
    SPI_TX_data_t s_SPI_data_array[2];
    SPI_frame_t s_SPI_frame;

    // 2.8 * 1000 = 14E-3 / 5E-6 = (14 / 5) * (1 / 10E-3)
    // R_SHUNT = 14E-3
    // CONVERSION FACTOR = 5E-6
    float   divided_volt_V = volt_V * 37.4 * (10.0 / 1347.0);
    uint16_t threshold = (divided_volt_V / 3.125) * 1000.0;

    s_SPI_data_array[0].data = threshold;
    s_SPI_data_array[0].mask = 0xff;
    s_SPI_data_array[1].data = threshold >> 8;
    s_SPI_data_array[1].mask = 0xff;

    s_SPI_frame.addr = 0x0E;
    s_SPI_frame.data_size = 2;
    s_SPI_frame.p_data_array = s_SPI_data_array;

    SPI_Overwrite(p_spi, &s_SPI_frame);
}

void VOM_Bus_Undervoltage_Threshold(spi_stdio_typedef* p_spi, float volt_V)
{
    SPI_TX_data_t s_SPI_data_array[2];
    SPI_frame_t s_SPI_frame;

    // 2.8 * 1000 = 14E-3 / 5E-6 = (14 / 5) * (1 / 10E-3)
    // R_SHUNT = 14E-3
    // CONVERSION FACTOR = 5E-6
    float   divided_volt_V = volt_V * 37.4 * (10.0 / 1347.0);
    uint16_t threshold = (divided_volt_V / 3.125) * 1000.0;

    s_SPI_data_array[0].data = threshold;
    s_SPI_data_array[0].mask = 0xff;
    s_SPI_data_array[1].data = threshold >> 8;
    s_SPI_data_array[1].mask = 0xff;

    s_SPI_frame.addr = 0x0F;
    s_SPI_frame.data_size = 2;
    s_SPI_frame.p_data_array = s_SPI_data_array;

    SPI_Overwrite(p_spi, &s_SPI_frame);
}

/* :::::::::: VOM SPI Interupt Handler ::::::::::::: */
__STATIC_INLINE void SPI_modified_raw_data(spi_stdio_typedef* p_spi, uint32_t current_TX_read_index)
{
    if (p_spi->temp_RX_index == 0)
        return;

    uint32_t tx_buffer_size = p_spi->TX_size;
    uint32_t buf_idx;
    SPI_TX_buffer_t* p_TX_buffer_temp;

    for (int8_t data_idx = 1; data_idx < p_spi->temp_RX_index; data_idx++)
    {
        buf_idx = (current_TX_read_index + data_idx) % tx_buffer_size;
        p_TX_buffer_temp = &p_spi->p_TX_buffer[buf_idx];

        p_TX_buffer_temp->data = ((p_spi->p_temp_RX_buffer[data_idx]) & ~(p_TX_buffer_temp->mask)) | ((p_TX_buffer_temp->data) & (p_TX_buffer_temp->mask));
    }

    p_spi->temp_RX_index = 0;
}

void VOM_driver_SPI_IRQHandler(void)
{
    // Xử lý chỉ khi có data
    if (LL_SPI_IsActiveFlag_RXNE(VOM_SPI_HANDLE))
    {
        // Cache index & pointer cho nhanh, hạn chế truy RAM nhiều lần
        SPI_TX_buffer_t *p_tx       = &VOM_SPI.p_TX_buffer[VOM_SPI.TX_read_index];
        uint8_t rx_temp             = LL_SPI_ReceiveData8(VOM_SPI_HANDLE);

        if (p_tx->data_type != SPI_HEADER)
        {
            // Nếu là READ, chuyển thẳng sang RX_buffer
            if (p_tx->command == SPI_READ)
            {
                VOM_SPI.p_RX_buffer[VOM_SPI.RX_write_index] = rx_temp;
                SPI_ADVANCE_RX_WRITE_INDEX(&VOM_SPI);
            }
            // Nếu là READ_TO_TEMP, chỉ advance index tạm
            else if (p_tx->command == SPI_READ_TO_TEMP)
            {
                VOM_SPI.p_temp_RX_buffer[VOM_SPI.temp_RX_index] = rx_temp;
                VOM_SPI.temp_RX_index++;
            }
        }

        // Còn lại (WRITE, WRITE_MODIFY) KHÔNG LÀM GÌ - vì không có nhận data

        // Nếu kết thúc 1 frame
        if (p_tx->data_type == SPI_ENDER)
        {
            LL_GPIO_SetOutputPin(VOM_SPI_CS_PORT, VOM_SPI_CS_PIN);
        }

        // Advance TX index sang byte tiếp theo
        SPI_ADVANCE_TX_READ_INDEX(&VOM_SPI);

        // Nếu hết buffer truyền, tắt ngắt
        if (SPI_TX_BUFFER_EMPTY(&VOM_SPI))
        {
            LL_SPI_DisableIT_RXNE(VOM_SPI_HANDLE);
            return;
        }

        // Tiếp tục gửi byte tiếp theo
        // SPI_Prime_Transmit(&VOM_SPI);
        p_tx = &VOM_SPI.p_TX_buffer[VOM_SPI.TX_read_index];

        if (p_tx->data_type == SPI_HEADER)
        {
            if(p_tx->command == SPI_WRITE_MODIFY)
            {
                SPI_modified_raw_data(&VOM_SPI, VOM_SPI.TX_read_index);
            }

            LL_GPIO_ResetOutputPin(VOM_SPI.cs_port, VOM_SPI.cs_pin);
        }

        LL_SPI_TransmitData8(VOM_SPI_HANDLE, p_tx->data);
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ End of the program ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
