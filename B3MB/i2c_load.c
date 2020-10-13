/*
 * i2c_load.c
 *
 * Created: 4/18/2020 12:30 PM
 * Derived from i2c_batt.c
 *		Modified by replace all batt_ with load_,
 * Rev_01 Litch - created
 *
 * An I2C interfaces servicing a total of 2 Quad Input ADS1115's
 *    load_sb_i2c - 2 ADS1115's, single ended, (Voltage and Current) x (LOAD1 - LOAD4)
 */ 

#include <utils.h>
#include <hal_init.h>
#include <hal_i2c_m_async.h>
#include "pins_perphs_init.h"
#include "pin_defines.h"
#include "I2C_ADS1115.h"
#include "Timer_Setup.h"
#include "subbus.h"

static bool             load_enabled       = LOAD_ENABLE_DEFAULT; // is I2C hardware wired to Load Monitors enabled?
static volatile bool    load_txfr_complete = true;                // is this I2C hardware busy with a transaction?                                 
static volatile bool    load_error_seen    = false;               // was there an error during the last transaction?
static volatile int32_t load_error         = I2C_OK;              // if so, what is the error code?

static struct io_descriptor *I2C_LOAD_IO;                         // ASF4 HAL i2c_async_driver io descriptor structure

/* ********************************************************************************************
 * Host Accessible Cache Address space for Load Monitoring information
 *
 */
static subbus_cache_word_t i2c_load_cache[I2C_LOAD_HIGH_ADDR - I2C_LOAD_BASE_ADDR+1] = {
  { 0, 0, true,  false, false, false, false }, // Offset 0: R: load_1_v    ADS1115_A_AIN_0
  { 0, 0, true,  false, false, false, false }, // Offset 1: R: load_1_i    ADS1115_A_AIN_1
  { 0, 0, true,  false, false, false, false }, // Offset 2: R: load_2_v    ADS1115_A_AIN_2
  { 0, 0, true,  false, false, false, false }, // Offset 3: R: load_2_i    ADS1115_A_AIN_3
  { 0, 0, true,  false, false, false, false }, // Offset 4: R: load_3_v    ADS1115_B_AIN_0
  { 0, 0, true,  false, false, false, false }, // Offset 5: R: load_3_i    ADS1115_B_AIN_1
  { 0, 0, true,  false, false, false, false }, // Offset 6: R: load_4_v    ADS1115_B_AIN_2
  { 0, 0, true,  false, false, false, false }, // Offset 7: R: load_4_i    ADS1115_B_AIN_3
//  { 0, 0, true,  false, false, false, false }, // Offset 8: R: load_ic2_status
};

/*
static void load_record_i2c_error(enum ads_state_t ads_poll_state, int32_t i2c_error) {
  uint16_t word = ((ads_poll_state & 0x7) << 4) | (i2c_error & 0xF);     // error in which state and what was code
  i2c_load_cache[8].cache = (i2c_load_cache[8].cache & 0xFF00) | word;   // save it to cache
}
*/

/* **********************************************************************************************
 * State Machine for Reading Load Voltages and Currents from a pair of ADS1115's 
 * Attached to Peripheral SERCOMx via ASF4's i2c_async_driver 
 *    micro-controller Hardware instance   = I2C_LOAD
 *    micro-controller ASF4 io descriptor  = i2c_load_io
 *    Host cache structure                 = i2c_load_cache
 *
 * Each ADS1115 is configured as Quad Single Ended Input
 *
 */

// States
enum load_state_t {load_config,     load_ptr_cnvr_reg, 
	               load_wait_cnvrt, load_read_cnvrt, 
				   load_cache_cnvrt};
static enum load_state_t load_state = load_config;

// I2C MASTER write Packets and Read buffer
static uint8_t  load_cnfg_reg[3] = { 0x01, 0xC7, 0x83};  // 0x01 = sets Addr_Pointer to configuration register
                                                         // 0xC7 = MSByte Config Reg, AINp = AIN0, AINn = GND
														 //        this will cycle from 0xC9 to 0xF9 to select
														 //        all four inputs, all single ended
														 //        and set FSR = +/- 1.024v
														 //        and set Mode = one-shot
														 // 0x83 = LBYte Config Reg, sets 128 SPS and ALRT/DRDY = Hi-Z
static uint8_t  load_conver_reg[1] = { 0x00 };           // 0x00 = ADS1115's internal address for where to read conversion data
static uint8_t  load_ain_cmd[4] = { 0xC7, 0xD7, 0xE7, 0xF7 }; // 2nd of 3 bytes in load_cnfg_reg[3], choose which load_ain_x #
static uint8_t  load_ads_ibuf[2];                        // buffer for read back of converted data

// Two I2C Slave devices on LOAD_MASTER, both ADS1115's, one at I2C device address = 0x48,the other at 0x49.
static uint8_t  load_dev_addr = I2C_A_DEV_ADDR;
static uint8_t  load_dev_x = 0;                          // Two ADS1115s on I2C bus, indicates which device is up to bat
static uint8_t  load_ain_x = 0;                          // 4 Single Ended Ana_ins on ADS1115, which a_in   is up to bat
#define NUM_CHANNELS         4                           // total # of single ended a_in's on the ADS1115

#define CONVERT_TIME 8                                   // 7.8ms / sample @ 128 Samples per Second, time in milliseconds
static uint32_t  load_start_time = 0;                    // timer value at start convert command issued
 
void i2c_load_poll(void) {
  if ( !load_enabled || !load_txfr_complete ) {          // if not enabled or transfer in process, nothing to do
	return;
  }
  switch (load_state) {    // I2C enabled and no transfer in process so do current state's work, evaluate next state
	  
    case load_config:                        // Write Config Reg, selects which channel to convert and starts conversion
      load_txfr_complete = false;
	  load_dev_addr = (load_dev_x == 0) ? I2C_A_DEV_ADDR : I2C_B_DEV_ADDR;
      load_cnfg_reg[1] = load_ain_cmd[load_ain_x];   // pick the correct load_ain_x when issuing the convert command
      i2c_m_async_set_slaveaddr(&I2C_LOAD, load_dev_addr, I2C_M_SEVEN);
      io_write(I2C_LOAD_IO, load_cnfg_reg, 3);  
      if (load_dev_x == 0) {
        load_dev_x = 1;
        load_state = load_config;                    // only one set to correct channel, set 2nd
	  } else {
	    load_dev_x = 0;
        load_state = load_ptr_cnvr_reg;              // both chips set to correct channel, update Addr_Pointer
	  }
    break;
	  
    case load_ptr_cnvr_reg:                  // Set Addr_Pointer to point to conversion data register
      load_txfr_complete = false;
      load_dev_addr = (load_dev_x == 0) ? I2C_A_DEV_ADDR : I2C_B_DEV_ADDR;
      i2c_m_async_set_slaveaddr(&I2C_LOAD, load_dev_addr, I2C_M_SEVEN);
      io_write(I2C_LOAD_IO, load_conver_reg, 1);
      if (load_dev_x == 0) {
	    load_dev_x = 1;
	    load_state = load_ptr_cnvr_reg;              // only one points to conversion register, point 2nd
	  } else {
	    load_dev_x = 0;
	    load_start_time = count_1msec;
	    load_state = load_wait_cnvrt;                // both chips points to conversion register, wait for conversion
      }
    break;
	  
	case load_wait_cnvrt:                    // Wait for conversion to complete
	  load_state = ( (count_1msec - load_start_time) > CONVERT_TIME ) ? load_read_cnvrt : load_wait_cnvrt;
    break;
	  
    case load_read_cnvrt:                    // Read converted data
      load_txfr_complete = false;
      load_dev_addr = (load_dev_x == 0) ? I2C_A_DEV_ADDR : I2C_B_DEV_ADDR;
      i2c_m_async_set_slaveaddr(&I2C_LOAD, load_dev_addr, I2C_M_SEVEN);
      io_read(I2C_LOAD_IO, load_ads_ibuf, 2);
	  load_state = load_cache_cnvrt;
    break;
	  
    case load_cache_cnvrt:                   // Check read contents: MSByte's MSBit = status of conversion
      if (load_dev_x == 0) {
		i2c_load_cache[load_ain_x].cache = (load_ads_ibuf[0] << 8) | load_ads_ibuf[1];
	    load_dev_x = 1;
	    load_state = load_read_cnvrt;                // only one read from, get 2nd
	  } else {
		i2c_load_cache[load_ain_x + NUM_CHANNELS].cache = (load_ads_ibuf[0] << 8) | load_ads_ibuf[1];
	    load_dev_x = 0;
		load_ain_x = (load_ain_x+1) % NUM_CHANNELS;  // Valid channels are 0, 1, 2, and 3, NUM_CHANNELS = 4
	    load_state = load_config;                    // both chips cached, bump channel select and go select it
      }
    break;
	  
    default:
      assert(false, __FILE__, __LINE__);
  }
  return;
}

void load_enable(bool value) {
  load_enabled = value;
}

#define I2C_INTFLAG_ERROR (1<<7)
static void i2c_load_async_error(struct i2c_m_async_desc *const i2c, int32_t error) {
  load_txfr_complete = true;
  load_error_seen = true;
  load_error = error;
  if (error == I2C_ERR_BUS) {
    hri_sercomi2cm_write_STATUS_reg(&I2C_LOAD, SERCOM_I2CM_STATUS_BUSERR);
    hri_sercomi2cm_clear_INTFLAG_reg(&I2C_LOAD, I2C_INTFLAG_ERROR);
  }
}

static void load_txfr_completed(struct i2c_m_async_desc *const i2c) {
  load_txfr_complete = true;
}

static void i2c_load_reset() {
  if (!sb_i2c_load.initialized) {
    i2c_m_async_get_io_descriptor(&I2C_LOAD, &I2C_LOAD_IO);
    i2c_m_async_enable(&I2C_LOAD);
    i2c_m_async_register_callback(&I2C_LOAD, I2C_M_ASYNC_ERROR,       (FUNC_PTR)i2c_load_async_error);
    i2c_m_async_register_callback(&I2C_LOAD, I2C_M_ASYNC_TX_COMPLETE, (FUNC_PTR)load_txfr_completed);
    i2c_m_async_register_callback(&I2C_LOAD, I2C_M_ASYNC_RX_COMPLETE, (FUNC_PTR)load_txfr_completed);
    sb_i2c_load.initialized = true;
  }
}

subbus_driver_t sb_i2c_load = {
  I2C_LOAD_BASE_ADDR, I2C_LOAD_HIGH_ADDR, // address range for Load Monitoring
  i2c_load_cache,                 // Host accessible cache structure
  i2c_load_reset,                 // reset function
  i2c_load_poll,                  // poll function
  0,                              // No Dynamic function
  false                           // initialized ?
};