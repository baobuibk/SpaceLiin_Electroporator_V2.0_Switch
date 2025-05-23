/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Include~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdbool.h>

#include "stm32f405xx.h"

#include "spi.h"
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Defines ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Enum ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Struct ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Class ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Private Types ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
__STATIC_INLINE uint8_t SPI_make_header(uint8_t reg_addr, SPI_command_t command_type);
__STATIC_INLINE void SPI_modified_raw_data(spi_stdio_typedef* p_spi, uint16_t current_TX_read_index);

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/**
 * The function `SPIConfig` initializes a SPI configuration with a specified receive buffer size and
 * sets up interrupt-driven reception.
 * 
 * @param uart_p The `uart_p` parameter is a pointer to a structure of type `SPI_HandleTypeDef`, which
 * contains configuration settings for a SPI (Universal Asynchronous Receiver-Transmitter) peripheral.
 * @param rx_buffer_size The `rx_buffer_size` parameter specifies the size of the receive buffer used
 * for storing incoming data in the SPIConfig function. This buffer will be initialized using the
 * QUEUE_Init function to manage incoming data efficiently.
 */
void SPI_Init( spi_stdio_typedef* p_spi, SPI_TypeDef* _handle,
                IRQn_Type _irqn, SPI_RX_buffer_t* _p_temp_RX_buffer,
                uint16_t _temp_RX_size, SPI_TX_buffer_t* _p_TX_buffer,
                uint16_t _TX_size, SPI_RX_buffer_t* _p_RX_buffer,
                uint16_t _RX_size, GPIO_TypeDef* _cs_port, uint32_t _cs_pin)
{
    p_spi->handle  = _handle;
    p_spi->irqn    = _irqn;

    p_spi->p_temp_RX_buffer = _p_temp_RX_buffer;

    p_spi->temp_RX_index = 0;
    p_spi->temp_RX_size  = _temp_RX_size;

    p_spi->p_TX_buffer = _p_TX_buffer;

    p_spi->TX_write_index = 0;
    p_spi->TX_read_index  = 0;
    p_spi->TX_size        = _TX_size;

    p_spi->p_RX_buffer = _p_RX_buffer;

    p_spi->RX_write_index = 0;
    p_spi->RX_read_index  = 0;
    p_spi->RX_size        = _RX_size;

    p_spi->cs_port = _cs_port;
    p_spi->cs_pin  = _cs_pin;

    LL_SPI_Enable(p_spi->handle);

    // Disable the TX IT
    //LL_SPI_DisableIT_TXE(p_spi->handle);

    LL_SPI_DisableIT_RXNE(p_spi->handle);
    LL_GPIO_SetOutputPin(p_spi->cs_port, p_spi->cs_pin);
}

//*****************************************************************************
//
//! Send the string to the SPI.
//!
//! \param pcBuf points to a buffer containing the string to transmit.
//
//*****************************************************************************
void SPI_Write(spi_stdio_typedef* p_spi, SPI_frame_t* p_frame)
{
    SPI_Add_to_TX_buffer(p_spi, p_frame, SPI_WRITE);
}

//*****************************************************************************
//
//! Send the string to the SPI.
//!
//! \param pcBuf points to a buffer containing the string to transmit.
//
//*****************************************************************************
void SPI_Read(spi_stdio_typedef* p_spi, SPI_frame_t* p_frame)
{
    SPI_Add_to_TX_buffer(p_spi, p_frame, SPI_READ);
}

//*****************************************************************************
//
//! Send the string to the SPI.
//!
//! \param pcBuf points to a buffer containing the string to transmit.
//
//*****************************************************************************
// void SPI_Overwrite(spi_stdio_typedef* p_spi, SPI_frame_t* p_frame)
// {
//     SPI_Add_to_TX_buffer(p_spi, p_frame, SPI_WRITE);
// }

//*****************************************************************************
//
//! Writes a string of characters to the SPI output.
//!
//!
//! \return Returns the count of characters written.
//
//*****************************************************************************
uint16_t SPI_Add_to_TX_buffer(spi_stdio_typedef* p_spi, SPI_frame_t* p_frame, SPI_command_t command)
{

    //uint8_t uIdx;

    for(int8_t Idx = p_frame->data_size; Idx >= 0; Idx--)
    {
        if (SPI_TX_BUFFER_FULL(p_spi))
        {
            p_spi->p_TX_buffer[p_spi->TX_write_index].data_type = SPI_ENDER;
            break;
        }
        
        p_spi->p_TX_buffer[p_spi->TX_write_index].command = SPI_READ;

        if (Idx == p_frame->data_size)
        {
            p_spi->p_TX_buffer[p_spi->TX_write_index].data_type = SPI_HEADER;
            p_spi->p_TX_buffer[p_spi->TX_write_index].mask      = 0xFF;
            p_spi->p_TX_buffer[p_spi->TX_write_index].data      = SPI_make_header(p_frame->addr, SPI_READ);
        }
        else if (Idx == 0)
        {
            p_spi->p_TX_buffer[p_spi->TX_write_index].data_type = SPI_ENDER;
            p_spi->p_TX_buffer[p_spi->TX_write_index].mask = 0xFF;
            p_spi->p_TX_buffer[p_spi->TX_write_index].data = 0xFF;
        }
        else
        {
            p_spi->p_TX_buffer[p_spi->TX_write_index].data_type = SPI_DATA;
            p_spi->p_TX_buffer[p_spi->TX_write_index].mask = 0xFF;
            p_spi->p_TX_buffer[p_spi->TX_write_index].data = 0xFF;
        }

        SPI_ADVANCE_TX_WRITE_INDEX(p_spi);
    }

    if (command == SPI_WRITE)
    {   
        for(int8_t Idx = p_frame->data_size; Idx >= 0; Idx--)
        {
            if (SPI_TX_BUFFER_FULL(p_spi))
            {
                p_spi->p_TX_buffer[p_spi->TX_write_index].data_type = SPI_ENDER;
                break;
            }

            p_spi->p_TX_buffer[p_spi->TX_write_index].command = SPI_WRITE;

            if (Idx == p_frame->data_size)
            {
                p_spi->p_TX_buffer[p_spi->TX_write_index].data_type = SPI_HEADER;
                p_spi->p_TX_buffer[p_spi->TX_write_index].mask      = 0xFF;
                p_spi->p_TX_buffer[p_spi->TX_write_index].data      = SPI_make_header(p_frame->addr, SPI_WRITE);
            }
            else if (Idx == 0)
            {
                p_spi->p_TX_buffer[p_spi->TX_write_index].data_type = SPI_ENDER;
                p_spi->p_TX_buffer[p_spi->TX_write_index].mask = p_frame->p_data_array[Idx].mask;
                p_spi->p_TX_buffer[p_spi->TX_write_index].data = p_frame->p_data_array[Idx].data;
            }
            else
            {
                p_spi->p_TX_buffer[p_spi->TX_write_index].data_type = SPI_DATA;
                p_spi->p_TX_buffer[p_spi->TX_write_index].mask = p_frame->p_data_array[Idx].mask;
                p_spi->p_TX_buffer[p_spi->TX_write_index].data = p_frame->p_data_array[Idx].data;
            }

            SPI_ADVANCE_TX_WRITE_INDEX(p_spi);
        }
    }

    //
    // If the usart txe irq is disable, this mean an usart phase is finished
    // we need to enable the txe irq and kick start the transmit process.
    //
    //if (LL_SPI_IsEnabledIT_TXE(p_spi->handle) == false)
    if (LL_SPI_IsEnabledIT_RXNE(p_spi->handle) == false)
    {
        //LL_GPIO_ResetOutputPin(p_spi->cs_port, p_spi->cs_pin);

        SPI_Prime_Transmit(p_spi);
        //LL_SPI_EnableIT_TXE(p_spi->handle);
        LL_SPI_EnableIT_RXNE(p_spi->handle);
    }

    //
    // Return the number of characters written.
    //
    return(0);
}

//*****************************************************************************
//
//! Determines whether the ring buffer whose pointers and size are provided
//! is full or not.
//!
//! \param pui16Read points to the read index for the buffer.
//! \param pui16Write points to the write index for the buffer.
//! \param ui16Size is the size of the buffer in bytes.
//!
//! This function is used to determine whether or not a given ring buffer is
//! full.  The structure of the code is specifically to ensure that we do not
//! see warnings from the compiler related to the order of volatile accesses
//! being undefined.
//!
//! \return Returns \b 1 if the buffer is full or \b 0 otherwise.
//
//*****************************************************************************
uint8_t SPI_is_buffer_full(volatile uint16_t *pui16Read,
             volatile uint16_t *pui16Write, uint16_t ui16Size)
{
    uint16_t ui16Write;
    uint16_t ui16Read;

    ui16Write = *pui16Write;
    ui16Read = *pui16Read;

    return((((ui16Write + 1) % ui16Size) == ui16Read) ? 1 : 0);
}


//*****************************************************************************
//
//! Determines whether the ring buffer whose pointers and size are provided
//! is empty or not.
//!
//! \param pui16Read points to the read index for the buffer.
//! \param pui16Write points to the write index for the buffer.
//!
//! This function is used to determine whether or not a given ring buffer is
//! empty.  The structure of the code is specifically to ensure that we do not
//! see warnings from the compiler related to the order of volatile accesses
//! being undefined.
//!
//! \return Returns \b 1 if the buffer is empty or \b 0 otherwise.
//
//*****************************************************************************
uint8_t SPI_is_buffer_empty(volatile uint16_t *pui16Read,
              volatile uint16_t *pui16Write)
{
    uint16_t ui16Write;
    uint16_t ui16Read;

    ui16Write = *pui16Write;
    ui16Read = *pui16Read;

    return((ui16Read == ui16Write) ? 1 : 0);
}


//*****************************************************************************
//
//! Determines the number of bytes of data contained in a ring buffer.
//!
//! \param pui16Read points to the read index for the buffer.
//! \param pui16Write points to the write index for the buffer.
//! \param ui16Size is the size of the buffer in bytes.
//!
//! This function is used to determine how many bytes of data a given ring
//! buffer currently contains.  The structure of the code is specifically to
//! ensure that we do not see warnings from the compiler related to the order
//! of volatile accesses being undefined.
//!
//! \return Returns the number of bytes of data currently in the buffer.
//
//*****************************************************************************
uint16_t SPI_get_buffer_count(volatile uint16_t *pui16Read,
               volatile uint16_t *pui16Write, uint16_t ui16Size)
{
    uint16_t ui16Write;
    uint16_t ui16Read;

    ui16Write = *pui16Write;
    ui16Read = *pui16Read;

    return((ui16Write >= ui16Read) ? (ui16Write - ui16Read) :
           (ui16Size - (ui16Read - ui16Write)));
}

//*****************************************************************************
//
//! Adding +1 to the index
//!
//! \param pui16Read points to the read index for the buffer.
//! \param pui16Write points to the write index for the buffer.
//! \param ui16Size is the size of the buffer in bytes.
//!
//! This function is use to advance the index by 1, if the index
//! already hit the uart size then it will reset back to 0.
//!
//! \return Returns the number of bytes of data currently in the buffer.
//
//*****************************************************************************
uint16_t SPI_advance_buffer_index(volatile uint16_t* pui16Index, uint16_t ui16Size)
{
    *pui16Index = (*pui16Index + 1) % ui16Size;

    return(*pui16Index);
}

//*****************************************************************************
//
//! Get +n index value according to its size
//!
//! \param pui16Read points to the read index for the buffer.
//! \param pui16Write points to the write index for the buffer.
//! \param ui16Size is the size of the buffer in bytes.
//!
//! This function is use to advance the index by 1, if the index
//! already hit the uart size then it will reset back to 0.
//!
//! \return Returns the number of bytes of data currently in the buffer.
//
//*****************************************************************************
uint16_t SPI_get_next_buffer_index(uint16_t ui16Index, uint16_t addition, uint16_t ui16Size)
{
    ui16Index = (ui16Index + addition) % ui16Size;

    return(ui16Index);
}

void SPI_flush_temp_to_RX_buffer(spi_stdio_typedef* p_spi)
{
    if (p_spi->temp_RX_index == 0)
    {
        return;
    }

    p_spi->p_RX_buffer[p_spi->RX_write_index].data_type = p_spi->p_temp_RX_buffer[0].data_type;
    p_spi->p_RX_buffer[p_spi->RX_write_index].data = p_spi->p_temp_RX_buffer[0].data;

    SPI_ADVANCE_RX_WRITE_INDEX(p_spi);
    
    for (int8_t Idx = (p_spi->temp_RX_index - 1); Idx >= 1; Idx--)
    {
        p_spi->p_RX_buffer[p_spi->RX_write_index].data_type = p_spi->p_temp_RX_buffer[Idx].data_type;
        p_spi->p_RX_buffer[p_spi->RX_write_index].data = p_spi->p_temp_RX_buffer[Idx].data;

        SPI_ADVANCE_RX_WRITE_INDEX(p_spi);
    }

    p_spi->temp_RX_index = 0;
}

//*****************************************************************************
//
// Take as many bytes from the transmit buffer as we have space for and move
// them into the SPI transmit FIFO.
//
//*****************************************************************************
void SPI_Prime_Transmit(spi_stdio_typedef* p_spi)
{
    if (SPI_TX_BUFFER_EMPTY(p_spi))
    {
        return;
    }

    if (p_spi->p_TX_buffer[p_spi->TX_read_index].data_type == SPI_HEADER)
    {
        if(p_spi->p_TX_buffer[p_spi->TX_read_index].command == SPI_READ)
        {
            SPI_flush_temp_to_RX_buffer(p_spi);
        }

        if(p_spi->p_TX_buffer[p_spi->TX_read_index].command == SPI_WRITE)
        {
            SPI_modified_raw_data(p_spi, p_spi->TX_read_index);
        }

        LL_GPIO_ResetOutputPin(p_spi->cs_port, p_spi->cs_pin);
    }
    
    NVIC_DisableIRQ(p_spi->irqn);

    LL_SPI_TransmitData8(p_spi->handle, p_spi->p_TX_buffer[p_spi->TX_read_index].data);

    NVIC_EnableIRQ(p_spi->irqn);
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
__STATIC_INLINE uint8_t SPI_make_header(uint8_t reg_addr, SPI_command_t command_type)
{
    uint8_t spi_header = 0;
    spi_header  = reg_addr << 2;
    spi_header |= command_type;

    return spi_header;
}

__STATIC_INLINE void SPI_modified_raw_data(spi_stdio_typedef* p_spi, uint16_t current_TX_read_index)
{
    if (p_spi->temp_RX_index == 0)
    {
        return;
    }

    uint8_t current_value = 0, value = 0, mask = 0, modified_value = 0;
    uint16_t tx_buffer_size = p_spi->TX_size;

    for (int8_t i = 1; i < p_spi->temp_RX_index; i++)
    {
        current_value  = p_spi->p_temp_RX_buffer[i].data;
        value          = p_spi->p_TX_buffer[SPI_get_next_buffer_index(current_TX_read_index, i, tx_buffer_size)].data;
        mask           = p_spi->p_TX_buffer[SPI_get_next_buffer_index(current_TX_read_index, i, tx_buffer_size)].mask;
        modified_value = (current_value & ~mask) | (value & mask);

        p_spi->p_TX_buffer[SPI_get_next_buffer_index(current_TX_read_index, i, tx_buffer_size)].data = modified_value;
    }

    p_spi->temp_RX_index = 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ End of the program ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
