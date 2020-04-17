/* *******************************************************************************************************
 * Use TC0 Hardware resource to generate a 32 bit counter that increments once 
 * every 1000 uSec (once every 8000 CPU clocks, 8MHz CPU, Div by 1 Prescaler on counter)
 * ~ 3 usec of overhead associated with ISR so set ticks to 7976 *0.125us = 997us
 *
 */

#include <hal_gpio.h>
#include "pin_defines.h"
#include "pins_perphs_init.h"
#include "Timer_Setup.h"

volatile uint32_t count_1msec = 0;
static struct timer_task TIMER_0_task1;

static void TIMER_0_task1_cb(const struct timer_task *const timer_task) {
	count_1msec++;
}

void TIMER_0_go(void) {
	uint32_t clock_cycles = 7976;                              // # of Timer Clocks (effected by Prescaler)       7976 = 256.20ms
	timer_set_clock_cycles_per_tick(&TIMER_0, clock_cycles);   // between each interrupt.                        797 =  25.64ms
	TIMER_0_task1.interval = 1;                  // Number of Timer Interrupts prior to calling callback task     79 =   2.57ms
	TIMER_0_task1.cb       = TIMER_0_task1_cb;   // the task to call on task.interval-th interrupt
	TIMER_0_task1.mode     = TIMER_TASK_REPEAT;  // Re-queue task or do it only once

	timer_add_task(&TIMER_0, &TIMER_0_task1);
	timer_start(&TIMER_0);
}


