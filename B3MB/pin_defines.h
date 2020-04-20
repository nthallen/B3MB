/* ****************************************************************************
 * B3MB Atmel Start Pin NAME Definitions
 * 
 * Rev 1.0	04/13/2020	Litch	Initial Assignment
 *
 * Code generated from Atmel Start, comments added
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy to a separate file to avoid losing it when reconfiguring with
 * new atmel Start Files
 *
 ************************************************************************** */
#ifndef PIN_DEFINES_H_INCLUDED
#define PIN_DEFINES_H_INCLUDED

#include <hal_gpio.h>

// SAMC21 has 9 pin functions
#define GPIO_PIN_FUNCTION_A 0
#define GPIO_PIN_FUNCTION_B 1
#define GPIO_PIN_FUNCTION_C 2
#define GPIO_PIN_FUNCTION_D 3
#define GPIO_PIN_FUNCTION_E 4
#define GPIO_PIN_FUNCTION_F 5
#define GPIO_PIN_FUNCTION_G 6
#define GPIO_PIN_FUNCTION_H 7
#define GPIO_PIN_FUNCTION_I 8

// Communication Channels
#define BATT_SDA   GPIO(GPIO_PORTA, 8)   // A2D I2C comm's, Battery Monitoring 
#define BATT_SCL   GPIO(GPIO_PORTA, 9)
#define LOAD_SDA   GPIO(GPIO_PORTA, 12)  // A2D I2C comm's, Load Monitoring
#define LOAD_SCL   GPIO(GPIO_PORTA, 13)
#define TMPR_SDA   GPIO(GPIO_PORTA, 16)  // A2D I2C comm's, Temperature Monitoring
#define TMPR_SCL   GPIO(GPIO_PORTA, 17)

#define CAN_TX     GPIO(GPIO_PORTA, 24)  // Host comm's
#define CAN_RX     GPIO(GPIO_PORTA, 25)

#define USART_TX   GPIO(GPIO_PORTA, 22)  // Debug Port comm's
#define USART_RX   GPIO(GPIO_PORTA, 23)

// On/Off Control (Outputs)
#define BATT1_ON   GPIO(GPIO_PORTB, 2)
#define BATT2_ON   GPIO(GPIO_PORTB, 3)
#define BATT3_ON   GPIO(GPIO_PORTB, 8)
#define BATT4_ON   GPIO(GPIO_PORTB, 9)

#define LOAD1_ON   GPIO(GPIO_PORTB, 10)
#define LOAD2_ON   GPIO(GPIO_PORTB, 11)
#define LOAD3_ON   GPIO(GPIO_PORTB, 22)
#define LOAD4_ON   GPIO(GPIO_PORTB, 23)

// Resettable Fuse Status (Inputs)
#define BATT1_FLT  GPIO(GPIO_PORTA, 14)
#define BATT2_FLT  GPIO(GPIO_PORTA, 15)
#define BATT3_FLT  GPIO(GPIO_PORTA, 10)
#define BATT4_FLT  GPIO(GPIO_PORTA, 11)

#define LOAD1_FLT  GPIO(GPIO_PORTA, 18)
#define LOAD2_FLT  GPIO(GPIO_PORTA, 19)
#define LOAD3_FLT  GPIO(GPIO_PORTA, 20)
#define LOAD4_FLT  GPIO(GPIO_PORTA, 21)

// Miscellaneous I/O
#define FAULT_O    GPIO(GPIO_PORTA, 28)
#define ID_CPU     GPIO(GPIO_PORTA, 5)
#define ALERT      GPIO(GPIO_PORTA, 6)
#define STATUS_O   GPIO(GPIO_PORTA, 7)

#endif // ATMEL_START_PINS_H_INCLUDED
