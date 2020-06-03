/* ****************************************************************************
 * B3MB Atmel Start Pin and Driver Peripherals Initialization functions
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

#include "pins_perphs_init.h"
#include <peripheral_clk_config.h>
#include <utils.h>
#include <hal_init.h>


#define USART_0_BUFFER_SIZE 16    // USART Buffer size, in bytes
static uint8_t USART_0_buffer[USART_0_BUFFER_SIZE];

struct usart_async_descriptor USART_0;
struct timer_descriptor       TIMER_0;
struct can_async_descriptor   CAN_0;

struct i2c_m_async_desc I2C_BATT;   // Comm peripheral to A2D's monitoring Batteries
struct i2c_m_async_desc I2C_TMPR;   // Comm peripheral to A2D's monitoring Temperatures
struct i2c_m_async_desc I2C_LOAD;   // Comm peripheral to A2D's monitoring Loads

/* ***********************************************************************************
 * Initialize I2C comm's to A2D's monitoring Switchable Batteries Voltage's and Current's
 */
void I2C_BATT_PORT_init(void) {
  gpio_set_pin_pull_mode(BATT_SDA, GPIO_PULL_OFF);
  gpio_set_pin_function(BATT_SDA, PINMUX_PA08C_SERCOM0_PAD0);
  gpio_set_pin_pull_mode(BATT_SCL, GPIO_PULL_OFF);
  gpio_set_pin_function(BATT_SCL, PINMUX_PA09C_SERCOM0_PAD1);
}

void I2C_BATT_CLOCK_init(void) {
  hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM0_GCLK_ID_CORE, CONF_GCLK_SERCOM0_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
  hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM0_GCLK_ID_SLOW, CONF_GCLK_SERCOM0_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
  hri_mclk_set_APBCMASK_SERCOM0_bit(MCLK);
}

void I2C_BATT_init(void) {
  I2C_BATT_CLOCK_init();
  i2c_m_async_init(&I2C_BATT, SERCOM0);
  I2C_BATT_PORT_init();
}

/* ************************************************************************* 
 * Initialize I2c comm's to A2D's monitoring Battery Box Temperatures
 */
void I2C_tmpr_PORT_init(void) {
  gpio_set_pin_pull_mode(TMPR_SDA, GPIO_PULL_OFF);
  gpio_set_pin_function(TMPR_SDA, PINMUX_PA16C_SERCOM1_PAD0);
  gpio_set_pin_pull_mode(TMPR_SCL, GPIO_PULL_OFF);
  gpio_set_pin_function(TMPR_SCL, PINMUX_PA17C_SERCOM1_PAD1);
}

void I2C_tmpr_CLOCK_init(void) {
  hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM1_GCLK_ID_CORE, CONF_GCLK_SERCOM1_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
  hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM1_GCLK_ID_SLOW, CONF_GCLK_SERCOM1_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
  hri_mclk_set_APBCMASK_SERCOM1_bit(MCLK);
}

void I2C_tmpr_init(void) {
  I2C_tmpr_CLOCK_init();
  i2c_m_async_init(&I2C_TMPR, SERCOM1);
  I2C_tmpr_PORT_init();
}

/* ******************************************************************************** 
 * Initialize I2C comm's to A2D's monitoring Switchable Loads Voltage's and Current's
 */
void I2C_load_PORT_init(void) {
  gpio_set_pin_pull_mode(LOAD_SDA, GPIO_PULL_OFF);
  gpio_set_pin_function(LOAD_SDA, PINMUX_PA12C_SERCOM2_PAD0);
  gpio_set_pin_pull_mode(LOAD_SCL, GPIO_PULL_OFF);
  gpio_set_pin_function(LOAD_SCL, PINMUX_PA13C_SERCOM2_PAD1);
}

void I2C_load_CLOCK_init(void) {
  hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM2_GCLK_ID_CORE, CONF_GCLK_SERCOM2_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
  hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM2_GCLK_ID_SLOW, CONF_GCLK_SERCOM2_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
  hri_mclk_set_APBCMASK_SERCOM2_bit(MCLK);
}

void I2C_load_init(void) {
  I2C_load_CLOCK_init();
  i2c_m_async_init(&I2C_LOAD, SERCOM2);
  I2C_load_PORT_init();
}

/* ******************************************************************************** 
 * Initialize USART, used as a debug port only
 */
void USART_0_CLOCK_init() {
  hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM3_GCLK_ID_CORE, CONF_GCLK_SERCOM3_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
  hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM3_GCLK_ID_SLOW, CONF_GCLK_SERCOM3_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
  hri_mclk_set_APBCMASK_SERCOM3_bit(MCLK);
}

void USART_0_PORT_init() {
  gpio_set_pin_function(USART_TX, PINMUX_PA22C_SERCOM3_PAD0);
  gpio_set_pin_function(USART_RX, PINMUX_PA23C_SERCOM3_PAD1);
}

void USART_0_init(void) {
  USART_0_CLOCK_init();
  usart_async_init(&USART_0, SERCOM3, USART_0_buffer, USART_0_BUFFER_SIZE, (void *)NULL);
  USART_0_PORT_init();
}

/* ******************************************************************************** 
 * Initialize Timer Counter, used to establish at 10 micro-second resolution 
 * counter. No interrupts, just counts, 32 bits deep, 10 micro-second clock.
 * Will wrap after ~ 11.93 hours of continuous counting.
 */
static void TIMER_0_init(void) {
  hri_mclk_set_APBCMASK_TC0_bit(MCLK);
  hri_gclk_write_PCHCTRL_reg(GCLK, TC0_GCLK_ID, CONF_GCLK_TC0_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
  timer_init(&TIMER_0, TC0, _tc_get_timer());
}

/* ******************************************************************************** 
 * Initialize CAN comm's to Host Application for Commands / Data exchange
 */
void CAN_0_PORT_init(void) {
  gpio_set_pin_function(CAN_RX, PINMUX_PA25G_CAN0_RX);
  gpio_set_pin_function(CAN_TX, PINMUX_PA24G_CAN0_TX);
}

void CAN_0_init(void) {
  hri_mclk_set_AHBMASK_CAN0_bit(MCLK);
  hri_gclk_write_PCHCTRL_reg(GCLK, CAN0_GCLK_ID, CONF_GCLK_CAN0_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
  can_async_init(&CAN_0, CAN0);
  CAN_0_PORT_init();
}

/* ******************************************************************************** 
 * Initialize Hardware Peripherals and GPIO pins
 */
void system_init(void) {
	
  init_mcu();
	
  // *************************************************************
  // GPIO for Digital Monitoring of re-settable Fuses
  //
  // GPIO on PA14 - Fuse Tripped on Battery 1 Input to Bus
  gpio_set_pin_direction(BATT1_FLT, GPIO_DIRECTION_IN);
  gpio_set_pin_pull_mode(BATT1_FLT, GPIO_PULL_OFF);
  gpio_set_pin_function(BATT1_FLT, GPIO_PIN_FUNCTION_OFF);

  // GPIO on PA15 - Fuse Tripped on Battery 2 Input to Bus  
  gpio_set_pin_direction(BATT2_FLT, GPIO_DIRECTION_IN);
  gpio_set_pin_pull_mode(BATT2_FLT, GPIO_PULL_OFF);
  gpio_set_pin_function(BATT2_FLT, GPIO_PIN_FUNCTION_OFF);

  // GPIO on PA10 - Fuse Tripped on Battery 3 Input to Bus
  gpio_set_pin_direction(BATT3_FLT, GPIO_DIRECTION_IN);
  gpio_set_pin_pull_mode(BATT3_FLT, GPIO_PULL_OFF);
  gpio_set_pin_function(BATT3_FLT, GPIO_PIN_FUNCTION_OFF);

  // GPIO on PA11 - Fuse Tripped on Battery 4 Input to Bus
  gpio_set_pin_direction(BATT4_FLT, GPIO_DIRECTION_IN);
  gpio_set_pin_pull_mode(BATT4_FLT, GPIO_PULL_OFF);
  gpio_set_pin_function(BATT4_FLT, GPIO_PIN_FUNCTION_OFF);

  // GPIO on PA18 - Fuse Tripped on Load 1
  gpio_set_pin_direction(LOAD1_FLT, GPIO_DIRECTION_IN);
  gpio_set_pin_pull_mode(LOAD1_FLT, GPIO_PULL_OFF);
  gpio_set_pin_function(LOAD1_FLT, GPIO_PIN_FUNCTION_OFF);

  // GPIO on PA19 - Fuse Tripped on Load 2
  gpio_set_pin_direction(LOAD2_FLT, GPIO_DIRECTION_IN);
  gpio_set_pin_pull_mode(LOAD2_FLT, GPIO_PULL_OFF);
  gpio_set_pin_function(LOAD2_FLT, GPIO_PIN_FUNCTION_OFF);

  // GPIO on PA20 - Fuse Tripped on Load 3 
  gpio_set_pin_direction(LOAD3_FLT, GPIO_DIRECTION_IN);
  gpio_set_pin_pull_mode(LOAD3_FLT, GPIO_PULL_OFF);
  gpio_set_pin_function(LOAD3_FLT, GPIO_PIN_FUNCTION_OFF);

  // GPIO on PA21 - Fuse Tripped on Load 4
  gpio_set_pin_direction(LOAD4_FLT, GPIO_DIRECTION_IN);
  gpio_set_pin_pull_mode(LOAD4_FLT, GPIO_PULL_OFF);
  gpio_set_pin_function(LOAD4_FLT, GPIO_PIN_FUNCTION_OFF);

  // ********************************************************************
  // GPIO for Digital Control of Battery and Load Connection Switches
  //     also used for resetting any tripped fuses
  //
  // GPIO on PB02 - Battery 1 on/off Bus
  gpio_set_pin_level(BATT1_ON, false);
  gpio_set_pin_direction(BATT1_ON, GPIO_DIRECTION_OUT);
  gpio_set_pin_function(BATT1_ON, GPIO_PIN_FUNCTION_OFF);

  // GPIO on PB03 - Battery 2 on/off Bus
  gpio_set_pin_level(BATT2_ON, false);
  gpio_set_pin_direction(BATT2_ON, GPIO_DIRECTION_OUT);
  gpio_set_pin_function(BATT2_ON, GPIO_PIN_FUNCTION_OFF);

  // GPIO on PB08 - Battery 3 on/off Bus
  gpio_set_pin_level(BATT3_ON, false);
  gpio_set_pin_direction(BATT3_ON, GPIO_DIRECTION_OUT);
  gpio_set_pin_function(BATT3_ON, GPIO_PIN_FUNCTION_OFF);

  // GPIO on PB09 - Battery 4 on/off Bus
  gpio_set_pin_level(BATT4_ON, false);
  gpio_set_pin_direction(BATT4_ON, GPIO_DIRECTION_OUT);
  gpio_set_pin_function(BATT4_ON, GPIO_PIN_FUNCTION_OFF);

  // GPIO on PB10 - Load 1 on/off
  gpio_set_pin_level(LOAD1_ON, false);
  gpio_set_pin_direction(LOAD1_ON, GPIO_DIRECTION_OUT);
  gpio_set_pin_function(LOAD1_ON, GPIO_PIN_FUNCTION_OFF);

  // GPIO on PB11 - Load 2 on/off
  gpio_set_pin_level(LOAD2_ON, false);
  gpio_set_pin_direction(LOAD2_ON, GPIO_DIRECTION_OUT);
  gpio_set_pin_function(LOAD2_ON, GPIO_PIN_FUNCTION_OFF);

  // GPIO on PB22 - Load 3 on/off
  gpio_set_pin_level(LOAD3_ON, false);
  gpio_set_pin_direction(LOAD3_ON, GPIO_DIRECTION_OUT);
  gpio_set_pin_function(LOAD3_ON, GPIO_PIN_FUNCTION_OFF);

  // GPIO on PB23 - Load 4 on/off
  gpio_set_pin_level(LOAD4_ON, false);
  gpio_set_pin_direction(LOAD4_ON, GPIO_DIRECTION_OUT);
  gpio_set_pin_function(LOAD4_ON, GPIO_PIN_FUNCTION_OFF);

  // *************************************************************
  // Miscellaneous GPIO		
  //
  // GPIO on PA05 - Used to Indicate CPU ?????
  gpio_set_pin_direction(ID_CPU, GPIO_DIRECTION_IN);
  gpio_set_pin_pull_mode(ID_CPU, GPIO_PULL_OFF);
  gpio_set_pin_function(ID_CPU, GPIO_PIN_FUNCTION_OFF);

  // GPIO on PA06 - Alert Signal From A2D's
  gpio_set_pin_direction(ALERT, GPIO_DIRECTION_IN);
  gpio_set_pin_pull_mode(ALERT,GPIO_PULL_OFF);
  gpio_set_pin_function(ALERT, GPIO_PIN_FUNCTION_OFF);

  // GPIO on PA07 - uController Status Output, Hooked up to LED
  gpio_set_pin_level(STATUS_O, false);
  gpio_set_pin_direction(STATUS_O, GPIO_DIRECTION_OUT);
  gpio_set_pin_function(STATUS_O, GPIO_PIN_FUNCTION_OFF);
		
  // GPIO on PA28
  gpio_set_pin_level(FAULT_O, false);
  gpio_set_pin_direction(FAULT_O, GPIO_DIRECTION_OUT);
  gpio_set_pin_function(FAULT_O, GPIO_PIN_FUNCTION_OFF);
		
  // ***************************************************************
  // Driver Peripheral Initialization calls
  //	
  I2C_BATT_init();
  I2C_tmpr_init();
  I2C_load_init();
  USART_0_init();
  TIMER_0_init();
  CAN_0_init();
}
