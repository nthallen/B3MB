/*
 * ADS1115_i2c.c
 *
 * Created: 4/14/2020 10:57:26 AM
 * Derived from GitHub master branch of BMM_A0_01\i2c.h
 * Modified for use on B3MB board,
 * Rev_01 Litch - created
 *
 * Three I2C interfaces servicing a total of 7 Quad Input ADS1115's
 *    batt_sb_i2c - 2 ADS1115's, single ended, (Voltage and Current) x (BATT1 - BATT4)
 *    load_sb_i2c - 2 ADS1115's, single ended, (Voltage and Current) x (LOAD1 - LOAD4)
 *    temp_sb_i2c - 3 ADS1115's, differential, Batt_Bus V_monitor, TEMP1 - TEMP5 (temperatures)
 */ 

#include <utils.h>
#include <hal_init.h>
#include <hal_i2c_m_async.h>
#include "pins_perphs_init.h"
#include "pin_defines.h"
#include "ADS1115_i2c.h"
#include "Timer_Setup.h"
#include "subbus.h"

static bool             batt_enabled       = BATT_ENABLE_DEFAULT; // is I2C hardware wired to Battery Monitors enabled?
static volatile bool    batt_txfr_complete = true;                // is this I2C hardware busy with a transaction?                                 
static volatile bool    batt_error_seen    = false;               // was there an error during the last transaction?
static volatile int32_t batt_error         = I2C_OK;              // if so, what is the error code?

static struct io_descriptor *I2C_BATT_IO;                         // ASF4 HAL i2c_async_driver io descriptor structure

/* ********************************************************************************************
 * Host Accessible Cache Address space for Battery Monitoring information
 *
 */
static subbus_cache_word_t i2c_batt_cache[BATT_HIGH_ADDR - BATT_BASE_ADDR+1] = {
  { 0, 0, true,  false, false, false, false }, // Offset 0: R: batt_1_v    ADS1115_A_AIN_0
  { 0, 0, true,  false, false, false, false }, // Offset 1: R: batt_1_i    ADS1115_A_AIN_1
  { 0, 0, true,  false, false, false, false }, // Offset 2: R: batt_2_v    ADS1115_A_AIN_2
  { 0, 0, true,  false, false, false, false }, // Offset 3: R: batt_2_i    ADS1115_A_AIN_3
  { 0, 0, true,  false, false, false, false }, // Offset 4: R: batt_3_v    ADS1115_B_AIN_0
  { 0, 0, true,  false, false, false, false }, // Offset 5: R: batt_3_i    ADS1115_B_AIN_1
  { 0, 0, true,  false, false, false, false }, // Offset 6: R: batt_4_v    ADS1115_B_AIN_2
  { 0, 0, true,  false, false, false, false }, // Offset 7: R: batt_4_i    ADS1115_B_AIN_3
  { 0, 0, true,  false, false, false, false }, // Offset 8: R: batt_ic2_status

};

/*
static void batt_record_i2c_error(enum ads_state_t ads_poll_state, int32_t i2c_error) {
  uint16_t word = ((ads_poll_state & 0x7) << 4) | (i2c_error & 0xF);     // error in which state and what was code
  i2c_batt_cache[8].cache = (i2c_batt_cache[8].cache & 0xFF00) | word;   // save it to cache
}
*/

/* **********************************************************************************************
 * State Machine for Reading Battery Voltages and Currents from a pair of ADS1115's 
 * Attached to Peripheral SERCOMx via ASF4's i2c_async_driver 
 *    micro-controller Hardware instance   = I2C_BATT
 *    micro-controller ASF4 io descriptor  = i2c_batt_io
 *    Host cache structure                 = i2c_batt_cache
 *
 * Each ADS1115 is configured as Quad Single Ended Input
 *
 */

// States
enum ads_state_t {channel_config, point_to_cnvr_reg, wait_cnvrt, read_cnvrt, cache_cnvrt};
static enum ads_state_t ads_state = channel_config;

// I2C MASTER write Packets and Read buffer
static uint8_t  wr_config_reg[3] = { 0x01, 0xC9, 0x83};  // 0x01 = sets Addr_Pointer to configuration register
                                                         // 0xC9 = MSByte Config Reg, AINp = AIN0, AINn = GND
														 //        this will cycle from 0xC9 to 0xF9 to select
														 //        all four inputs, all single ended
														 //        and set FSR = +/- 0.512v
														 //        and set Mode = one-shot
														 // 0x83 = LBYte Config Reg, sets 128 SPS and ALRT/DRDY = Hi-Z
static uint8_t  ain_cmd[4] = { 0xC9, 0xD9, 0xE9, 0xF9 }; // 2nd of 3 bytes in wr_config_reg[3], choose which ain_x #
static uint8_t  ptr_conver_reg[1] = { 0x00 };			 // 0x00 = Sets Addr_Pointer to conversion Data register
static uint8_t  ads_ibuf[2];                             // buffer for read back of converted data

// Two I2C Slave devices on BATT_MASTER, both ADS1115's, one at I2C device address = 0x48,the other at 0x49.
#define BATT_A_SLAVE_ADDR 0x48                           // I2C device addresses for ADS1115s on Battery Monitor channel
#define BATT_B_SLAVE_ADDR 0x49
static uint8_t  slave_addr = BATT_A_SLAVE_ADDR;
static uint8_t  dev_x      = 0;                          // Two ADS1115s on I2C bus, indicates which device is up to bat
static uint8_t  ain_x      = 0;                          // 4 Single Ended Ana_ins on ADS1115, which a_in   is up to bat
#define NUM_CHANNELS         4                           // total # of single ended a_in's on the ADS1115

#define CONVERT_TIME 8                                   // 7.8ms / sample @ 128 Samples per Second, time in milliseconds
static uint32_t  batt_start_time = 0;                    // timer value at start convert command issued
 
void i2c_batt_poll(void) {
  if ( !batt_enabled || !batt_txfr_complete ) {          // if not enabled or transfer in process, nothing to do
	return;
  }
  switch (ads_state) {    // I2C enabled and no transfer in process so do current state's work, evaluate next state
	  
    case channel_config:                     // Write Config Reg, selects which channel to convert and starts conversion
      batt_txfr_complete = false;
	  slave_addr = (dev_x == 0) ? BATT_A_SLAVE_ADDR : BATT_B_SLAVE_ADDR;
      wr_config_reg[1] = ain_cmd[ain_x];         // pick the correct ain_x when issuing the convert command
      i2c_m_async_set_slaveaddr(&I2C_BATT, slave_addr, I2C_M_SEVEN);
      io_write(I2C_BATT_IO, wr_config_reg, 3);  
      if (dev_x == 0) {
        dev_x = 1;
        ads_state = channel_config;              // only one set to correct channel, set 2nd
	  } else {
	    dev_x = 0;
        ads_state = point_to_cnvr_reg;           // both chips set to correct channel, update Addr_Pointer
	  }
    break;
	  
    case point_to_cnvr_reg:                  // Set Addr_Pointer to point to conversion data register
      batt_txfr_complete = false;
      slave_addr = (dev_x == 0) ? BATT_A_SLAVE_ADDR : BATT_B_SLAVE_ADDR;
      i2c_m_async_set_slaveaddr(&I2C_BATT, slave_addr, I2C_M_SEVEN);
      io_write(I2C_BATT_IO, ptr_conver_reg, 1);
      if (dev_x == 0) {
	    dev_x = 1;
	    ads_state = point_to_cnvr_reg;           // only one points to conversion register, point 2nd
	  } else {
	    dev_x = 0;
	    batt_start_time = count_1msec;
	    ads_state = wait_cnvrt;                  // both chips points to conversion register, wait for conversion
      }
    break;
	  
	case wait_cnvrt:                         // Wait for conversion to complete
	  ads_state = ( (count_1msec - batt_start_time) > CONVERT_TIME ) ? read_cnvrt : wait_cnvrt;
    break;
	  
    case read_cnvrt:                         // Read converted data
      batt_txfr_complete = false;
      slave_addr = (dev_x == 0) ? BATT_A_SLAVE_ADDR : BATT_B_SLAVE_ADDR;
      i2c_m_async_set_slaveaddr(&I2C_BATT, slave_addr, I2C_M_SEVEN);
      io_read(I2C_BATT_IO, ads_ibuf, 2);
	  ads_state = cache_cnvrt;
    break;
	  
    case cache_cnvrt:                        // Check read contents: MSByte's MSBit = status of conversion
      if (dev_x == 0) {
		i2c_batt_cache[ain_x].cache = (ads_ibuf[0] << 8) | ads_ibuf[1];
	    dev_x = 1;
	    ads_state = read_cnvrt;                  // only one read from, get 2nd
	  } else {
		i2c_batt_cache[ain_x + NUM_CHANNELS].cache = (ads_ibuf[0] << 8) | ads_ibuf[1];
	    dev_x = 0;
		ain_x = (ain_x + 1) % NUM_CHANNELS;      // Valid channels are 0, 1, 2, and 3, NUM_CHANNELS = 4
	    ads_state = channel_config;              // both chips cached, bump channel select and go select it
      }
    break;
	  
    default:
      assert(false, __FILE__, __LINE__);
  }
  return;
}

void batt_enable(bool value) {
  batt_enabled = value;
}


#define I2C_INTFLAG_ERROR (1<<7)
static void i2c_batt_async_error(struct i2c_m_async_desc *const i2c, int32_t error) {
  batt_txfr_complete = true;
  batt_error_seen = true;
  batt_error = error;
  if (error == I2C_ERR_BUS) {
    hri_sercomi2cm_write_STATUS_reg(&I2C_BATT, SERCOM_I2CM_STATUS_BUSERR);
    hri_sercomi2cm_clear_INTFLAG_reg(&I2C_BATT, I2C_INTFLAG_ERROR);
  }
}

static void batt_txfr_completed(struct i2c_m_async_desc *const i2c) {
  batt_txfr_complete = true;
}

static void i2c_batt_reset() {
  if (!sb_i2c_batt.initialized) {
    i2c_m_async_get_io_descriptor(&I2C_BATT, &I2C_BATT_IO);
    i2c_m_async_enable(&I2C_BATT);
    i2c_m_async_register_callback(&I2C_BATT, I2C_M_ASYNC_ERROR,       (FUNC_PTR)i2c_batt_async_error);
    i2c_m_async_register_callback(&I2C_BATT, I2C_M_ASYNC_TX_COMPLETE, (FUNC_PTR)batt_txfr_completed);
    i2c_m_async_register_callback(&I2C_BATT, I2C_M_ASYNC_RX_COMPLETE, (FUNC_PTR)batt_txfr_completed);
    sb_i2c_batt.initialized = true;
  }
}

subbus_driver_t sb_i2c_batt = {
  BATT_BASE_ADDR, BATT_HIGH_ADDR, // address range for Battery Monitoring
  i2c_batt_cache,                 // Host accessible cache structure
  i2c_batt_reset,                 // reset function
  i2c_batt_poll,                  // poll function
  0,                              // No Dynamic function
  false                           // initialized ?
};