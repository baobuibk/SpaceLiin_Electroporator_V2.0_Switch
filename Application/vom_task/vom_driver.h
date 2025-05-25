#ifndef VOM_DRIVER_H_
#define VOM_DRIVER_H_

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Include ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "spi.h"

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Defines ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
typedef enum
{
    CT_50US   = 0x00,
    CT_84US   = 0x01,
    CT_150US  = 0x02,
    CT_280US  = 0x03,
    CT_540US  = 0x04,
    CT_1052US = 0x05,
    CT_2074US = 0x06,
    CT_4120US = 0x07
} VOM_Conv_Time_t;

typedef enum
{
    VOM_SHUTDOWN         = 0x00,

    VOM_SHUNT_SINGLE     = 0x01,
    VOM_BUS_SINGLE       = 0x02,
    VOM_BUS_SHUNT_SINGLE = 0x03,

    VOM_SHUNT_CONT       = 0x09,
    VOM_BUS_CONT         = 0x0A,
    VOM_BUS_SHUNT_CONT   = 0x0B,
} VOM_Measure_Mode_t;

typedef struct
{
    VOM_Measure_Mode_t measure_mode;
    VOM_Conv_Time_t    vsh_ct;
    VOM_Conv_Time_t    vbus_ct;
    uint8_t            avg_vsh;
    uint8_t            avg_vbus;
} VOM_Config_t;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Types ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
extern spi_stdio_typedef VOM_SPI;

//extern uint16_t g_Feedback_Voltage[ADC_CHANNEL_COUNT];
extern bool        is_Measure_Impedance;
extern uint16_t    Current_Sense_Period;
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Enum ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Struct ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Class ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* :::::::::: VOM Driver Init :::::::: */
void VOM_Driver_Init(void);

/* :::::::::: VOM Build ADC_CONFIG Frame :::::::: */
bool VOM_Build_ADC_CONFIG_Frame(const VOM_Config_t* config, SPI_frame_t* out_frame, SPI_TX_data_t* out_data_array);

/* :::::::::: VOM SPI Interupt Handler ::::::::::::: */
void VOM_driver_SPI_IRQHandler(void);

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ End of the program ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#endif /* VOM_DRIVER_H_ */
