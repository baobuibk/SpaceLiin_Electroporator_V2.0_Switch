/*
 * HEE_board.h
 *
 *  Created on: Jul 27, 2024
 *      Author: duong
 */

#ifndef BOARD_H_
#define BOARD_H_

/*********************H BRIDGE******************/
#define H_BRIDGE_SD0_3_HANDLE   TIM1

#define H_BRIDGE_SD0_CHANNEL    LL_TIM_CHANNEL_CH1

#define H_BRIDGE_SD1_CHANNEL    LL_TIM_CHANNEL_CH2

#define H_BRIDGE_SD2_CHANNEL    LL_TIM_CHANNEL_CH3

#define H_BRIDGE_SD3_CHANNEL    LL_TIM_CHANNEL_CH4

#define H_BRIDGE_SD4_7_HANDLE   TIM8

#define H_BRIDGE_SD4_CHANNEL    LL_TIM_CHANNEL_CH4

#define H_BRIDGE_SD5_CHANNEL    LL_TIM_CHANNEL_CH3

#define H_BRIDGE_SD6_CHANNEL    LL_TIM_CHANNEL_CH2

#define H_BRIDGE_SD7_CHANNEL    LL_TIM_CHANNEL_CH1

//________________________________________________
#define H_BRIDGE_HIN0_7_PORT    GPIOB

#define H_BRIDGE_HIN0_PIN       LL_GPIO_PIN_1

#define H_BRIDGE_HIN1_PIN       LL_GPIO_PIN_2

#define H_BRIDGE_HIN2_PIN       LL_GPIO_PIN_10

#define H_BRIDGE_HIN3_PIN       LL_GPIO_PIN_11

#define H_BRIDGE_HIN4_PIN       LL_GPIO_PIN_15

#define H_BRIDGE_HIN5_PIN       LL_GPIO_PIN_14

#define H_BRIDGE_HIN6_PIN       LL_GPIO_PIN_13

#define H_BRIDGE_HIN7_PIN       LL_GPIO_PIN_12

/***********************************************/

/*********************H BRIDGE******************/
// CHANNEL 0
#define H_BRIDGE_0_HANDLE       TIM3
#define H_BRIDGE_HIN_0_CHANNEL  LL_TIM_CHANNEL_CH2
#define H_BRIDGE_LIN_0_CHANNEL  LL_TIM_CHANNEL_CH1

// CHANNEL 1
#define H_BRIDGE_1_HANDLE       TIM4
#define H_BRIDGE_HIN_1_CHANNEL  LL_TIM_CHANNEL_CH2
#define H_BRIDGE_LIN_1_CHANNEL  LL_TIM_CHANNEL_CH1

// CHANNEL 2
#define H_BRIDGE_2_HANDLE       TIM4
#define H_BRIDGE_HIN_2_CHANNEL  LL_TIM_CHANNEL_CH4
#define H_BRIDGE_LIN_2_CHANNEL  LL_TIM_CHANNEL_CH3

// CHANNEL 3
#define H_BRIDGE_3_HANDLE       TIM2
#define H_BRIDGE_HIN_3_CHANNEL  LL_TIM_CHANNEL_CH2
#define H_BRIDGE_LIN_3_CHANNEL  LL_TIM_CHANNEL_CH1

// CHANNEL 4
#define H_BRIDGE_4_HANDLE       TIM8
#define H_BRIDGE_HIN_4_CHANNEL  LL_TIM_CHANNEL_CH4
#define H_BRIDGE_LIN_4_CHANNEL  LL_TIM_CHANNEL_CH3

// CHANNEL 5
#define H_BRIDGE_5_HANDLE       TIM2
#define H_BRIDGE_HIN_5_CHANNEL  LL_TIM_CHANNEL_CH4
#define H_BRIDGE_LIN_5_CHANNEL  LL_TIM_CHANNEL_CH3

// CHANNEL 6
#define H_BRIDGE_6_HANDLE       TIM3
#define H_BRIDGE_HIN_6_CHANNEL  LL_TIM_CHANNEL_CH4
#define H_BRIDGE_LIN_6_CHANNEL  LL_TIM_CHANNEL_CH3

// CHANNEL 7
#define H_BRIDGE_7_HANDLE       TIM5
#define H_BRIDGE_HIN_7_CHANNEL  LL_TIM_CHANNEL_CH4
#define H_BRIDGE_LIN_7_CHANNEL  LL_TIM_CHANNEL_CH3

// H_BRIDGE_DELAY
#define H_BRIDGE_SYNC_HANDLE   TIM6

/***********************************************/

/*****************Voltage Switching*************/
#define V_SWITCH_LIN1_HANDLE    TIM1
#define V_SWITCH_LIN1_CHANNEL   LL_TIM_CHANNEL_CH3

#define V_SWITCH_HIN1_PORT      GPIOA
#define V_SWITCH_HIN1_PIN       LL_GPIO_PIN_11

#define V_SWITCH_LIN2_HANDLE    TIM1
#define V_SWITCH_LIN2_CHANNEL   LL_TIM_CHANNEL_CH2

#define V_SWITCH_HIN2_PORT      GPIOA
#define V_SWITCH_HIN2_PIN       LL_GPIO_PIN_8
/***********************************************/

/**************CURRENT MONITOR*****************/
#define VOM_SPI_HANDLE          SPI1
#define VOM_SPI_IRQ             SPI1_IRQn

#define VOM_SPI_CS_PORT         GPIOC
#define VOM_SPI_CS_PIN          LL_GPIO_PIN_4
/***********************************************/

/*********************UART**********************/
#define RS232_UART_HANDLE       UART5
#define RS232_UART_IRQ          UART5_IRQn

#define GPC_UART_HANDLE         USART6
#define GPC_UART_IRQ            USART6_IRQn
/***********************************************/

/*******************DEBUG LED*******************/
#define DEBUG_LED_PORT          GPIOC
#define DEBUG_LED_PIN           LL_GPIO_PIN_3

#define PULSE_LED_PORT          GPIOC
#define PULSE_LED_PIN           LL_GPIO_PIN_1
/***********************************************/

#define GPP_TX_SIZE			64
#define	GPP_RX_SIZE			64
#define FSP_BUF_LEN			64

#endif /* BOARD_H_ */
