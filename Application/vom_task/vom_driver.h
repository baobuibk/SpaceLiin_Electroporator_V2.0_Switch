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
    VOM_PULSE_ON,
    VOM_PULSE_OFF,
} VOM_Pulse_State_t;

typedef enum
{
    VOM_TASK_HV_POS,
    VOM_TASK_HV_NEG,
    VOM_TASK_LV_POS,
    VOM_TASK_LV_NEG,
} VOM_Task_State_t;

typedef enum
{
    VOM_CURRENT_TYPE,
    VOM_VOLT_TYPE,
} VOM_Data_Type_t;

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
    AVG_1    = 0x00,
    AVG_4    = 0x01,
    AVG_16   = 0x02,
    AVG_64   = 0x03,
    AVG_128  = 0x04,
    AVG_256  = 0x05,
    AVG_512  = 0x06,
    AVG_1024 = 0x07
} VOM_Avg_Count_t;

typedef struct
{
    VOM_Measure_Mode_t measure_mode;
    VOM_Conv_Time_t    vsh_ct;
    VOM_Conv_Time_t    vbus_ct;
    VOM_Avg_Count_t    avg;
} VOM_Config_t;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Types ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
extern spi_stdio_typedef VOM_SPI;

//extern uint16_t g_Feedback_Voltage[ADC_CHANNEL_COUNT];

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Enum ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Struct ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Class ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* :::::::::: VOM Driver Init :::::::: */
void VOM_Driver_Init(void);

/* :::::::::: VOM Build ADC_CONFIG Frame :::::::: */
bool VOM_Build_ADC_CONFIG_Frame(const VOM_Config_t* config, SPI_frame_t* out_frame, SPI_TX_data_t* out_data_array);

/* :::::::::: VOM Data Process :::::::: */
void VOM_Data_Process(spi_stdio_typedef* p_spi);

void VOM_Reset_OVC_Flag(spi_stdio_typedef* p_spi);

void VOM_Shunt_Overvoltage_Threshold(spi_stdio_typedef* p_spi, float current_A);
void VOM_Shunt_Undervoltage_Threshold(spi_stdio_typedef* p_spi, float current_A);

void VOM_Bus_Overvoltage_Threshold(spi_stdio_typedef* p_spi, float volt_A);
void VOM_Bus_Undervoltage_Threshold(spi_stdio_typedef* p_spi, float volt_A);

/* :::::::::: VOM SPI Interupt Handler ::::::::::::: */
void VOM_driver_SPI_IRQHandler(void);
void VOM_OVC_IRQHandler(void);

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ End of the program ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#endif /* VOM_DRIVER_H_ */
