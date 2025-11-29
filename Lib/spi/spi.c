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
__STATIC_INLINE void SPI_populate_TX_buffer(spi_stdio_typedef* p_spi, SPI_frame_t* p_frame, SPI_command_t command);
__STATIC_INLINE void SPI_Add_to_TX_buffer(spi_stdio_typedef* p_spi, SPI_frame_t* p_frame, uint8_t frame_count, SPI_command_t command);
__STATIC_INLINE void SPI_modified_raw_data(spi_stdio_typedef* p_spi, uint32_t current_TX_read_index);

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
                IRQn_Type _irqn, uint8_t* _p_temp_RX_buffer,
                uint32_t _temp_RX_size, SPI_TX_buffer_t* _p_TX_buffer,
                uint32_t _TX_size, uint8_t* _p_RX_buffer,
                uint32_t _RX_size, GPIO_TypeDef* _cs_port, uint32_t _cs_pin)
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
//! Request SPI write request with R-M-W operation.
//!
//! \param pcBuf points to a buffer containing the string to transmit.
//
//*****************************************************************************
void SPI_Write(spi_stdio_typedef* p_spi, SPI_frame_t* p_frame)
{
    SPI_Add_to_TX_buffer(p_spi, p_frame, 1, SPI_READ_TO_TEMP);
    SPI_Add_to_TX_buffer(p_spi, p_frame, 1, SPI_WRITE_MODIFY);
}

//*****************************************************************************
//
//! Request SPI read request.
//!
//! \param pcBuf points to a buffer containing the string to transmit.
//
//*****************************************************************************
void SPI_Read(spi_stdio_typedef* p_spi, SPI_frame_t* p_frame, uint8_t frame_count)
{   
    SPI_Add_to_TX_buffer(p_spi, p_frame, frame_count, SPI_READ);
}

//*****************************************************************************
//
//! Request SPI write request with overwrite operation.
//!
//! \param pcBuf points to a buffer containing the string to transmit.
//
//*****************************************************************************
void SPI_Overwrite(spi_stdio_typedef* p_spi, SPI_frame_t* p_frame)
{
    SPI_Add_to_TX_buffer(p_spi, p_frame, 1, SPI_WRITE);
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
uint8_t SPI_is_buffer_full(volatile uint32_t *pui16Read,
             volatile uint32_t *pui16Write, uint32_t ui16Size)
{
    uint32_t ui16Write;
    uint32_t ui16Read;

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
uint8_t SPI_is_buffer_empty(volatile uint32_t *pui16Read,
              volatile uint32_t *pui16Write)
{
    uint32_t ui16Write;
    uint32_t ui16Read;

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
uint32_t SPI_get_buffer_count(volatile uint32_t *pui16Read,
               volatile uint32_t *pui16Write, uint32_t ui16Size)
{
    uint32_t ui16Write;
    uint32_t ui16Read;

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
uint32_t SPI_advance_buffer_index(volatile uint32_t* pui16Index, uint32_t ui16Size)
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
uint32_t SPI_get_next_buffer_index(uint32_t ui16Index, uint32_t addition, uint32_t ui16Size)
{
    ui16Index = (ui16Index + addition) % ui16Size;

    return(ui16Index);
}

void SPI_flush_buffer(volatile uint32_t *pui16Read,
              volatile uint32_t *pui16Write)
{
    *pui16Read = *pui16Write;
}

//*****************************************************************************
//
// Take as many bytes from the transmit buffer as we have space for and move
// them into the SPI transmit FIFO.
//
//*****************************************************************************
void SPI_Prime_Transmit(spi_stdio_typedef* p_spi)
{
    SPI_TX_buffer_t *p_tx = &p_spi->p_TX_buffer[p_spi->TX_read_index];

    if (p_tx->data_type == SPI_HEADER)
    {
        if(p_tx->command == SPI_WRITE_MODIFY)
        {
            SPI_modified_raw_data(p_spi, p_spi->TX_read_index);
        }

        LL_GPIO_ResetOutputPin(p_spi->cs_port, p_spi->cs_pin);
    }

    LL_SPI_TransmitData8(p_spi->handle, p_tx->data);
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
__STATIC_INLINE uint8_t SPI_make_header(uint8_t reg_addr, SPI_command_t command_type)
{
    // Tạo header: [reg_addr(6b) | command(2b)]
    return (uint8_t)((reg_addr << 2) | (command_type & 0x03));
}

__STATIC_INLINE void SPI_populate_TX_buffer(
                                            spi_stdio_typedef*  p_spi,
                                            SPI_frame_t*        p_frame,
                                            SPI_command_t       command)
{
    SPI_TX_buffer_t* p_TX_buffer_temp = &p_spi->p_TX_buffer[p_spi->TX_write_index];

    if (SPI_TX_BUFFER_FULL(p_spi))
    {
        p_TX_buffer_temp->data_type = SPI_ENDER;
        return;
    }
    
    // Add the header to the TX buffer
    p_TX_buffer_temp->data_type = SPI_HEADER;
    p_TX_buffer_temp->command   = command;
    p_TX_buffer_temp->mask      = 0xFF;
    p_TX_buffer_temp->data      = SPI_make_header(p_frame->addr, command);

    SPI_ADVANCE_TX_WRITE_INDEX(p_spi);

    // Loop through data bytes from last to first
    for (int8_t data_idx = (p_frame->data_size - 1); data_idx >= 0; data_idx--)
    {
        p_TX_buffer_temp = &p_spi->p_TX_buffer[p_spi->TX_write_index];

        if (SPI_TX_BUFFER_FULL(p_spi))
        {
            p_TX_buffer_temp->data_type = SPI_ENDER;
            return;
        }

        // Set data type: END for last byte, DATA for others
        p_TX_buffer_temp->data_type =   (data_idx == 0) 
                                        ? SPI_ENDER
                                        : SPI_DATA;

        p_TX_buffer_temp->command   =   command;
        p_TX_buffer_temp->mask      =   p_frame->p_data_array[data_idx].mask;

        // For SPI_READ or SPI_READ_TO_TEMP, set data to 0xFF; otherwise use provided data
        p_TX_buffer_temp->data      =   ((command & 0x03) == SPI_READ) 
                                        ? 0xFF 
                                        : p_frame->p_data_array[data_idx].data;

        SPI_ADVANCE_TX_WRITE_INDEX(p_spi);
    }
}


__STATIC_INLINE void SPI_Add_to_TX_buffer(
                                        spi_stdio_typedef*  p_spi,
                                        SPI_frame_t*        p_frame,
                                        uint8_t             frame_count,
                                        SPI_command_t       command)
{
    for (uint8_t Idx = 0; Idx < frame_count; Idx++)
    {
        SPI_populate_TX_buffer(p_spi, &p_frame[Idx], command);
    }

    if (!LL_SPI_IsEnabledIT_RXNE(p_spi->handle))
    {
        SPI_Prime_Transmit(p_spi);
        LL_SPI_EnableIT_RXNE(p_spi->handle);
    }
}

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

        // Read-modify-write
        // uint8_t  current_value = p_spi->p_temp_RX_buffer[data_idx];
        // uint8_t  tx_data       = p_TX_buffer_temp->data;
        // uint8_t  tx_mask       = p_TX_buffer_temp->mask;

        // p_TX_buffer_temp->data = (current_value & ~tx_mask) | (tx_data & tx_mask);
        p_TX_buffer_temp->data = ((p_spi->p_temp_RX_buffer[data_idx]) & ~(p_TX_buffer_temp->mask)) | ((p_TX_buffer_temp->data) & (p_TX_buffer_temp->mask));
    }

    p_spi->temp_RX_index = 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ End of the program ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
