/* 
 * i2c_tmpr.c
 * 
 * An I2C interfaces servicing 3 Dual Differential Input ADS1115's = 6 Channels:
 * 	Bus Voltage, 5 Internal Thermistor Temperatures
 *  tmpr_sb_i2c - subbus driver
 * 
 * Rev_00: Marco Rivero 9:40 AM 10/11/2020
 * 	Derived from i2c_batt.c: Replaced "batt" with "tmpr"
 * Rev_01: Marco Rivero 
 * 	Modified for B3MB
 * 
 */ 

#include <utils.h>
#include <hal_init.h>
#include <hal_i2c_m_async.h>
#include "pins_perphs_init.h"
#include "pin_defines.h"
#include "I2C_ADS1115.h"
#include "Timer_Setup.h"
#include "subbus.h"

static bool             tmpr_enabled       = TEMP_ENABLE_DEFAULT; // is I2C hardware wired to Battery Monitors enabled?
static volatile bool    tmpr_txfr_complete = true;                // is this I2C hardware busy with a transaction?                                 
static volatile bool    tmpr_error_seen    = false;               // was there an error during the last transaction?
static volatile int32_t tmpr_error         = I2C_OK;              // if so, what is the error code?

static struct io_descriptor *I2C_TEMP_IO;                         // ASF4 HAL i2c_async_driver io descriptor structure

/* ********************************************************************************************
 * Host Accessible Cache Address space for Battery Monitoring information
 *
 */
static subbus_cache_word_t i2c_tmpr_cache[I2C_TEMP_HIGH_ADDR - I2C_TEMP_BASE_ADDR+1] = {
  { 0, 0, true,  false, false, false, false }, // Offset 0: R: Bus Voltage	ADS1115_A_AIN_0
  { 0, 0, true,  false, false, false, false }, // Offset 1: R: Thermistor 1	ADS1115_A_AIN_1
  { 0, 0, true,  false, false, false, false }, // Offset 2: R: Thermistor 2	ADS1115_B_AIN_0
  { 0, 0, true,  false, false, false, false }, // Offset 3: R: Thermistor 3	ADS1115_B_AIN_1
  { 0, 0, true,  false, false, false, false }, // Offset 4: R: Thermistor 4	ADS1115_C_AIN_0
  { 0, 0, true,  false, false, false, false }, // Offset 5: R: Thermistor 5	ADS1115_C_AIN_1
//  { 0, 0, true,  false, false, false, false }, // Offset 8: R: batt_ic2_status
};

/*
static void tmpr_record_i2c_error(enum ads_state_t ads_poll_state, int32_t i2c_error) {
  uint16_t word = ((ads_poll_state & 0x7) << 4) | (i2c_error & 0xF);     // error in which state and what was code
  i2c_tmpr_cache[8].cache = (i2c_tmpr_cache[8].cache & 0xFF00) | word;   // save it to cache
}
*/

/* **********************************************************************************************
 * State Machine for Reading Battery Voltages and Currents from a pair of ADS1115's 
 * Attached to Peripheral SERCOMx via ASF4's i2c_async_driver 
 *    micro-controller Hardware instance   = I2C_TEMP
 *    micro-controller ASF4 io descriptor  = i2c_tmpr_io
 *    Host cache structure                 = i2c_tmpr_cache
 *
 * Each ADS1115 is configured as Quad Single Ended Input
 *
 */

// States
enum tmpr_state_t {tmpr_config,     tmpr_ptr_cnvr_reg, 
	               tmpr_wait_cnvrt, tmpr_read_cnvrt, 
				   tmpr_cache_cnvrt};
static enum tmpr_state_t tmpr_state = tmpr_config;

// ADS1115 Configuration and Read commands

/** Write to config register [01] 0x83 0x83:
 *   0x01: Pointer register value specifying config register
 *   0x8383:
 *     OS[15] = 1: Single Conversion
 *     MUX[14:12] = 000: AIN0/AIN1 => Vbus, Temp2, Temp4
 *     PGA[11:9] = 001: FSR = +/- 4.096V
 *     MODE[8] = 1: Single shot conversion
 *     DR[7:5] = 100: 128 SPS
 *     COMP_MODE[4] = 0: Default/Don't Care
 *     COMP_POL[3] = 0: Default/Don't Care
 *     COMP_LAT[2] = 0: Default/Don't Care
 *     COMP_QUE[1:0] = 11: Disable comparator and set ALERT to high impedance
 */
static uint8_t  tmpr_cnfg_reg[3] = { 0x01, 0x83, 0x83 }; 

// .. Will also need next command for second channel 
//** Write to config register [01] 0xB3 0x83:
//*     MUX[14:12] = 011: AIN2/AIN3 => Temp1, Temp3, Temp5

//

// cleanup
														 // 0x01 = sets Addr_Pointer to configuration register
                                                         // 0xC7 = MSByte Config Reg, AINp = AIN0, AINn = GND
														 //        this will cycle from 0xC9 to 0xF9 to select
														 //        all four inputs, all single ended
														 //        and set FSR = +/- 1.024v
														 //        and set Mode = one-shot
														 // 0x83 = LBYte Config Reg, sets 128 SPS and ALRT/DRDY = Hi-Z
														 //        MSNibble sets Conversion Speed 
														 //        0=8, 2=15, 4=32, 6=64, 8=128, A=250, C=475, E=860  
// cleanup

static uint8_t  tmpr_conver_reg[1] = { 0x00 };           // 0x00 = ADS1115's internal address for where to read conversion data
static uint8_t  tmpr_ain_cmd[4] = { 0xC7, 0xD7, 0xE7, 0xF7 }; // 2nd of 3 bytes in tmpr_cnfg_reg[3], choose which tmpr_ain_x #
static uint8_t  tmpr_ads_ibuf[2];                        // buffer for read back of converted data

// Three I2C Slave devices on TEMP_MASTER, all ADS1115's, at I2C device address = 0x48, 0x49, and 0x4A.
static uint8_t  tmpr_dev_addr = I2C_A_DEV_ADDR;
static uint8_t  tmpr_dev_x = 0;                          // Two ADS1115s on I2C bus, indicates which device is up to bat
static uint8_t  tmpr_ain_x = 0;                          // 4 Single Ended Ana_ins on ADS1115, which a_in   is up to bat
#define NUM_CHANNELS         4                           // total # of single ended a_in's on the ADS1115

#define CONVERT_TIME 8                                   // 7.8ms / sample @ 128 Samples per Second, time in milliseconds
static uint32_t  tmpr_start_time = 0;                    // timer value at start convert command issued
 
void i2c_tmpr_poll(void) {
  if ( !tmpr_enabled || !tmpr_txfr_complete ) {          // if not enabled or transfer in process, nothing to do
	return;
  }
  switch (tmpr_state) {    // I2C enabled and no transfer in process so do current state's work, evaluate next state
	  
    case tmpr_config:                        // Write Config Reg, selects which channel to convert and starts conversion
      tmpr_txfr_complete = false;
	  tmpr_dev_addr = (tmpr_dev_x == 0) ? I2C_A_DEV_ADDR : I2C_B_DEV_ADDR;
      tmpr_cnfg_reg[1] = tmpr_ain_cmd[tmpr_ain_x];   // pick the correct tmpr_ain_x when issuing the convert command
      i2c_m_async_set_slaveaddr(&I2C_TEMP, tmpr_dev_addr, I2C_M_SEVEN);
      io_write(I2C_TEMP_IO, tmpr_cnfg_reg, 3);  
      if (tmpr_dev_x == 0) {
        tmpr_dev_x = 1;
        tmpr_state = tmpr_config;                    // only one set to correct channel, set 2nd
	  } else {
	    tmpr_dev_x = 0;
        tmpr_state = tmpr_ptr_cnvr_reg;              // both chips set to correct channel, update Addr_Pointer
	  }
    break;
	  
    case tmpr_ptr_cnvr_reg:                  // Set Addr_Pointer to point to conversion data register
      tmpr_txfr_complete = false;
      tmpr_dev_addr = (tmpr_dev_x == 0) ? I2C_A_DEV_ADDR : I2C_B_DEV_ADDR;
      i2c_m_async_set_slaveaddr(&I2C_TEMP, tmpr_dev_addr, I2C_M_SEVEN);
      io_write(I2C_TEMP_IO, tmpr_conver_reg, 1);
      if (tmpr_dev_x == 0) {
	    tmpr_dev_x = 1;
	    tmpr_state = tmpr_ptr_cnvr_reg;              // only one points to conversion register, point 2nd
	  } else {
	    tmpr_dev_x = 0;
	    tmpr_start_time = count_1msec;
	    tmpr_state = tmpr_wait_cnvrt;                // both chips points to conversion register, wait for conversion
      }
    break;
	  
	case tmpr_wait_cnvrt:                    // Wait for conversion to complete
	  tmpr_state = ( (count_1msec - tmpr_start_time) > CONVERT_TIME ) ? tmpr_read_cnvrt : tmpr_wait_cnvrt;
    break;
	  
    case tmpr_read_cnvrt:                    // Read converted data
      tmpr_txfr_complete = false;
      tmpr_dev_addr = (tmpr_dev_x == 0) ? I2C_A_DEV_ADDR : I2C_B_DEV_ADDR;
      i2c_m_async_set_slaveaddr(&I2C_TEMP, tmpr_dev_addr, I2C_M_SEVEN);
      io_read(I2C_TEMP_IO, tmpr_ads_ibuf, 2);
	  tmpr_state = tmpr_cache_cnvrt;
    break;
	  
    case tmpr_cache_cnvrt:                   // Check read contents: MSByte's MSBit = status of conversion
      if (tmpr_dev_x == 0) {
		i2c_tmpr_cache[tmpr_ain_x].cache = (tmpr_ads_ibuf[0] << 8) | tmpr_ads_ibuf[1];
	    tmpr_dev_x = 1;
	    tmpr_state = tmpr_read_cnvrt;                // only one read from, get 2nd
	  } else {
		i2c_tmpr_cache[tmpr_ain_x + NUM_CHANNELS].cache = (tmpr_ads_ibuf[0] << 8) | tmpr_ads_ibuf[1];
	    tmpr_dev_x = 0;
		tmpr_ain_x = (tmpr_ain_x+1) % NUM_CHANNELS;  // Valid channels are 0, 1, 2, and 3, NUM_CHANNELS = 4
	    tmpr_state = tmpr_config;                    // both chips cached, bump channel select and go select it
      }
    break;
	  
    default:
      assert(false, __FILE__, __LINE__);
  }
  return;
}

void tmpr_enable(bool value) {
  tmpr_enabled = value;
}

#define I2C_INTFLAG_ERROR (1<<7)
static void i2c_tmpr_async_error(struct i2c_m_async_desc *const i2c, int32_t error) {
  tmpr_txfr_complete = true;
  tmpr_error_seen = true;
  tmpr_error = error;
  if (error == I2C_ERR_BUS) {
    hri_sercomi2cm_write_STATUS_reg(&I2C_TEMP, SERCOM_I2CM_STATUS_BUSERR);
    hri_sercomi2cm_clear_INTFLAG_reg(&I2C_TEMP, I2C_INTFLAG_ERROR);
  }
}

static void tmpr_txfr_completed(struct i2c_m_async_desc *const i2c) {
  tmpr_txfr_complete = true;
}

static void i2c_tmpr_reset() {
  if (!sb_i2c_tmpr.initialized) {
    i2c_m_async_get_io_descriptor(&I2C_TEMP, &I2C_TEMP_IO);
    i2c_m_async_enable(&I2C_TEMP);
    i2c_m_async_register_callback(&I2C_TEMP, I2C_M_ASYNC_ERROR,       (FUNC_PTR)i2c_tmpr_async_error);
    i2c_m_async_register_callback(&I2C_TEMP, I2C_M_ASYNC_TX_COMPLETE, (FUNC_PTR)tmpr_txfr_completed);
    i2c_m_async_register_callback(&I2C_TEMP, I2C_M_ASYNC_RX_COMPLETE, (FUNC_PTR)tmpr_txfr_completed);
    sb_i2c_tmpr.initialized = true;
  }
}

subbus_driver_t sb_i2c_tmpr = {
  I2C_TEMP_BASE_ADDR, I2C_TEMP_HIGH_ADDR, // address range for Battery Monitoring
  i2c_tmpr_cache,                 // Host accessible cache structure
  i2c_tmpr_reset,                 // reset function
  i2c_tmpr_poll,                  // poll function
  0,                              // No Dynamic function
  false                           // initialized ?
};