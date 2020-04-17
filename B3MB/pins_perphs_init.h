/* ****************************************************************************
 * B3MB Atmel Start ASF4 Driver Assignments / Initialization functions
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
#ifndef PINS_PERPHS_INIT_INCLUDED
#define PINS_PERPHS_INIT_INCLUDED

#include "pin_defines.h"

#ifdef __cplusplus
extern "C" {
#endif

// Base functionality from Atmel Start 
#include <hal_atomic.h>
#include <hal_delay.h>
#include <hal_gpio.h>
#include <hal_init.h>
#include <hal_io.h>
#include <hal_sleep.h>

// User Selected Driver specific from Atmel Start
#include <hal_i2c_m_async.h>
#include <hal_usart_async.h>
#include <hal_timer.h>
#include <hpl_tc_base.h>
#include <hal_can_async.h>

extern struct i2c_m_async_desc		 I2C_BATT;   // I2C peripheral to A2D's monitoring Batteries
extern struct i2c_m_async_desc		 I2C_Temp;   // I2C peripheral to A2D's monitoring Temperatures
extern struct i2c_m_async_desc       I2C_Load;   // I2C peripheral to A2D's monitoring Loads
extern struct usart_async_descriptor USART_0;    // 
extern struct timer_descriptor       TIMER_0;    // Timer Counter for 10 usec timing
extern struct can_async_descriptor   CAN_0;      // Control Module comm's with Host

void I2C_BATT_PORT_init(void);
void I2C_BATT_CLOCK_init(void);
void I2C_BATT_init(void);

void I2C_Temp_PORT_init(void);
void I2C_Temp_CLOCK_init(void);
void I2C_Temp_init(void);

void I2C_Load_PORT_init(void);
void I2C_Load_CLOCK_init(void);
void I2C_Load_init(void);

void USART_0_PORT_init(void);
void USART_0_CLOCK_init(void);
void USART_0_init(void);

// initialize pins and clocks for micro-controller peripherals
void system_init(void);

#ifdef __cplusplus
}
#endif
#endif // DRIVER_INIT_INCLUDED
