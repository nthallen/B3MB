/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */
#ifndef ATMEL_START_PINS_H_INCLUDED
#define ATMEL_START_PINS_H_INCLUDED

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

#define ID_CPU GPIO(GPIO_PORTA, 5)
#define ALERT GPIO(GPIO_PORTA, 6)
#define STATUS_O GPIO(GPIO_PORTA, 7)
#define BATT_SDA GPIO(GPIO_PORTA, 8)
#define BATT_SCL GPIO(GPIO_PORTA, 9)
#define BATT3_FLT GPIO(GPIO_PORTA, 10)
#define BATT4_FLT GPIO(GPIO_PORTA, 11)
#define LOAD_SDA GPIO(GPIO_PORTA, 12)
#define LOAD_SCL GPIO(GPIO_PORTA, 13)
#define BATT1_FLT GPIO(GPIO_PORTA, 14)
#define BATT2_FLT GPIO(GPIO_PORTA, 15)
#define TEMP_SDA GPIO(GPIO_PORTA, 16)
#define TEMP_SCL GPIO(GPIO_PORTA, 17)
#define LOAD1_FLT GPIO(GPIO_PORTA, 18)
#define LOAD2_FLT GPIO(GPIO_PORTA, 19)
#define LOAD3_FLT GPIO(GPIO_PORTA, 20)
#define LOAD4_FLT GPIO(GPIO_PORTA, 21)
#define USART_TX GPIO(GPIO_PORTA, 22)
#define USART_RX GPIO(GPIO_PORTA, 23)
#define CAN_TX GPIO(GPIO_PORTA, 24)
#define CAN_RX GPIO(GPIO_PORTA, 25)
#define FAULT_O GPIO(GPIO_PORTA, 28)
#define BATT1_ON GPIO(GPIO_PORTB, 2)
#define BATT2_ON GPIO(GPIO_PORTB, 3)
#define BATT3_ON GPIO(GPIO_PORTB, 8)
#define BATT4_ON GPIO(GPIO_PORTB, 9)
#define LOAD1_ON GPIO(GPIO_PORTB, 10)
#define LOAD2_ON GPIO(GPIO_PORTB, 11)
#define LOAD3_ON GPIO(GPIO_PORTB, 22)
#define LOAD4_ON GPIO(GPIO_PORTB, 23)

#endif // ATMEL_START_PINS_H_INCLUDED
