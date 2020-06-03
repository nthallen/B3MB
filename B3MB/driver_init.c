/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */

#include "driver_init.h"
#include <peripheral_clk_config.h>
#include <utils.h>
#include <hal_init.h>

/*! The buffer size for USART */
#define USART_0_BUFFER_SIZE 16

struct usart_async_descriptor USART_0;
struct timer_descriptor       TIMER_0;
struct can_async_descriptor   CAN_0;

static uint8_t USART_0_buffer[USART_0_BUFFER_SIZE];

struct i2c_m_async_desc I2C_Batt;

struct i2c_m_async_desc I2C_Temp;

struct i2c_m_async_desc I2C_Load;

void I2C_Batt_PORT_init(void)
{

	gpio_set_pin_pull_mode(BATT_SDA,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(BATT_SDA, PINMUX_PA08C_SERCOM0_PAD0);

	gpio_set_pin_pull_mode(BATT_SCL,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(BATT_SCL, PINMUX_PA09C_SERCOM0_PAD1);
}

void I2C_Batt_CLOCK_init(void)
{
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM0_GCLK_ID_CORE, CONF_GCLK_SERCOM0_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM0_GCLK_ID_SLOW, CONF_GCLK_SERCOM0_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_mclk_set_APBCMASK_SERCOM0_bit(MCLK);
}

void I2C_Batt_init(void)
{
	I2C_Batt_CLOCK_init();
	i2c_m_async_init(&I2C_Batt, SERCOM0);
	I2C_Batt_PORT_init();
}

void I2C_Temp_PORT_init(void)
{

	gpio_set_pin_pull_mode(TEMP_SDA,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(TEMP_SDA, PINMUX_PA16C_SERCOM1_PAD0);

	gpio_set_pin_pull_mode(TEMP_SCL,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(TEMP_SCL, PINMUX_PA17C_SERCOM1_PAD1);
}

void I2C_Temp_CLOCK_init(void)
{
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM1_GCLK_ID_CORE, CONF_GCLK_SERCOM1_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM1_GCLK_ID_SLOW, CONF_GCLK_SERCOM1_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_mclk_set_APBCMASK_SERCOM1_bit(MCLK);
}

void I2C_Temp_init(void)
{
	I2C_Temp_CLOCK_init();
	i2c_m_async_init(&I2C_Temp, SERCOM1);
	I2C_Temp_PORT_init();
}

void I2C_Load_PORT_init(void)
{

	gpio_set_pin_pull_mode(LOAD_SDA,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(LOAD_SDA, PINMUX_PA12C_SERCOM2_PAD0);

	gpio_set_pin_pull_mode(LOAD_SCL,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(LOAD_SCL, PINMUX_PA13C_SERCOM2_PAD1);
}

void I2C_Load_CLOCK_init(void)
{
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM2_GCLK_ID_CORE, CONF_GCLK_SERCOM2_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM2_GCLK_ID_SLOW, CONF_GCLK_SERCOM2_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_mclk_set_APBCMASK_SERCOM2_bit(MCLK);
}

void I2C_Load_init(void)
{
	I2C_Load_CLOCK_init();
	i2c_m_async_init(&I2C_Load, SERCOM2);
	I2C_Load_PORT_init();
}

/**
 * \brief USART Clock initialization function
 *
 * Enables register interface and peripheral clock
 */
void USART_0_CLOCK_init()
{

	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM3_GCLK_ID_CORE, CONF_GCLK_SERCOM3_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM3_GCLK_ID_SLOW, CONF_GCLK_SERCOM3_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_mclk_set_APBCMASK_SERCOM3_bit(MCLK);
}

/**
 * \brief USART pinmux initialization function
 *
 * Set each required pin to USART functionality
 */
void USART_0_PORT_init()
{

	gpio_set_pin_function(USART_TX, PINMUX_PA22C_SERCOM3_PAD0);

	gpio_set_pin_function(USART_RX, PINMUX_PA23C_SERCOM3_PAD1);
}

/**
 * \brief USART initialization function
 *
 * Enables USART peripheral, clocks and initializes USART driver
 */
void USART_0_init(void)
{
	USART_0_CLOCK_init();
	usart_async_init(&USART_0, SERCOM3, USART_0_buffer, USART_0_BUFFER_SIZE, (void *)NULL);
	USART_0_PORT_init();
}

/**
 * \brief Timer initialization function
 *
 * Enables Timer peripheral, clocks and initializes Timer driver
 */
static void TIMER_0_init(void)
{
	hri_mclk_set_APBCMASK_TC0_bit(MCLK);
	hri_gclk_write_PCHCTRL_reg(GCLK, TC0_GCLK_ID, CONF_GCLK_TC0_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));

	timer_init(&TIMER_0, TC0, _tc_get_timer());
}

void CAN_0_PORT_init(void)
{

	gpio_set_pin_function(CAN_RX, PINMUX_PA25G_CAN0_RX);

	gpio_set_pin_function(CAN_TX, PINMUX_PA24G_CAN0_TX);
}
/**
 * \brief CAN initialization function
 *
 * Enables CAN peripheral, clocks and initializes CAN driver
 */
void CAN_0_init(void)
{
	hri_mclk_set_AHBMASK_CAN0_bit(MCLK);
	hri_gclk_write_PCHCTRL_reg(GCLK, CAN0_GCLK_ID, CONF_GCLK_CAN0_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	can_async_init(&CAN_0, CAN0);
	CAN_0_PORT_init();
}

void system_init(void)
{
	init_mcu();

	// GPIO on PA05

	// Set pin direction to input
	gpio_set_pin_direction(ID_CPU, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(ID_CPU,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(ID_CPU, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA06

	// Set pin direction to input
	gpio_set_pin_direction(ALERT, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(ALERT,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(ALERT, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA07

	gpio_set_pin_level(STATUS_O,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(STATUS_O, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(STATUS_O, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA10

	// Set pin direction to input
	gpio_set_pin_direction(BATT3_FLT, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(BATT3_FLT,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(BATT3_FLT, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA11

	// Set pin direction to input
	gpio_set_pin_direction(BATT4_FLT, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(BATT4_FLT,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(BATT4_FLT, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA14

	// Set pin direction to input
	gpio_set_pin_direction(BATT1_FLT, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(BATT1_FLT,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(BATT1_FLT, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA15

	// Set pin direction to input
	gpio_set_pin_direction(BATT2_FLT, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(BATT2_FLT,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(BATT2_FLT, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA18

	// Set pin direction to input
	gpio_set_pin_direction(LOAD1_FLT, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(LOAD1_FLT,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(LOAD1_FLT, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA19

	// Set pin direction to input
	gpio_set_pin_direction(LOAD2_FLT, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(LOAD2_FLT,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(LOAD2_FLT, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA20

	// Set pin direction to input
	gpio_set_pin_direction(LOAD3_FLT, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(LOAD3_FLT,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(LOAD3_FLT, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA21

	// Set pin direction to input
	gpio_set_pin_direction(LOAD4_FLT, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(LOAD4_FLT,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(LOAD4_FLT, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA28

	gpio_set_pin_level(FAULT_O,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(FAULT_O, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(FAULT_O, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB02

	gpio_set_pin_level(BATT1_ON,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(BATT1_ON, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(BATT1_ON, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB03

	gpio_set_pin_level(BATT2_ON,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(BATT2_ON, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(BATT2_ON, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB08

	gpio_set_pin_level(BATT3_ON,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(BATT3_ON, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(BATT3_ON, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB09

	gpio_set_pin_level(BATT4_ON,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(BATT4_ON, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(BATT4_ON, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB10

	gpio_set_pin_level(LOAD1_ON,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(LOAD1_ON, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(LOAD1_ON, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB11

	gpio_set_pin_level(LOAD2_ON,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(LOAD2_ON, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(LOAD2_ON, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB22

	gpio_set_pin_level(LOAD3_ON,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(LOAD3_ON, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(LOAD3_ON, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB23

	gpio_set_pin_level(LOAD4_ON,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(LOAD4_ON, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(LOAD4_ON, GPIO_PIN_FUNCTION_OFF);

	I2C_Batt_init();

	I2C_Temp_init();

	I2C_Load_init();
	USART_0_init();

	TIMER_0_init();
	CAN_0_init();
}
