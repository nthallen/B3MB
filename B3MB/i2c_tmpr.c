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

static struct io_descriptor *I2C_TMPR_IO;                         // ASF4 HAL i2c_async_driver io descriptor structure

static void i2c_write(int16_t i2c_addr, const uint8_t *obuf, int16_t nbytes);
static void i2c_read(int16_t i2c_addr, uint8_t *ibuf, int16_t nbytes);

/* ********************************************************************************************
 * Host Accessible Cache Address space for Battery Monitoring information
 *
 */
static subbus_cache_word_t i2c_tmpr_cache[I2C_TMPR_HIGH_ADDR - I2C_TMPR_BASE_ADDR+1] = {
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
 *    micro-controller Hardware instance   = I2C_TMPR
 *    micro-controller ASF4 io descriptor  = i2c_tmpr_io
 *    Host cache structure                 = i2c_tmpr_cache
 *
 * Each ADS1115 is configured as Dual Differential Input
 *
 */

// States
enum tmpr_state_t {tmpr_t1_init, tmpr_t1_init_tx, tmpr_t1_read_cfg,
                  tmpr_t1_read_cfg_tx, tmpr_t1_reg0, tmpr_t1_read_adc,
                  tmpr_t1_read_adc_tx,
                  tmpr_t2_init, tmpr_t2_init_tx, tmpr_t2_read_cfg,
                  tmpr_t2_read_cfg_tx, tmpr_t2_reg0, tmpr_t2_read_adc,
                  tmpr_t2_read_adc_tx};
static enum tmpr_state_t tmpr_state = tmpr_t1_init;

// ADS1115 Configuration and Read commands

/** Write to config register for channel t1: [01] 0x83 0x83:
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
static uint8_t  tmpr_t1_cmd[3] = { 0x01, 0x83, 0x83 }; 

/** Write to config register for channel t2: [01] 0xB3 0x83:
 *   0xB383:
 *   ...
 *     MUX[14:12] = 011: AIN2/AIN3 => Temp1, Temp3, Temp5
 *   ...
 */
static uint8_t tmpr_t2_cmd[3] = { 0x01, 0xB3, 0x83 };


static uint8_t tmpr_r0_prep[1] = { 0x00 };          // 0x00 = ADS1115's internal address to read conversion data
static uint8_t tmpr_ibuf[2];                        // buffer for read back of converted data

static uint8_t tmpr_slave_addr[3] = { 0x48, 0x49, 0x4A }; // 3 x 7-bit ADS1115 device address. (4th, 0x4B )
static int tmpr_slave_index = 0 ;					// Tracks active device on I2C bus

void i2c_tmpr_poll(void) {
  switch (tmpr_state) {
    case tmpr_t1_init: // Start to convert 1st signal
//      ads_n_reads = 0;
      i2c_write(tmpr_slave_addr[tmpr_slave_index], tmpr_t1_cmd, 3);
      tmpr_state = tmpr_t1_init_tx;
      return;
    case tmpr_t1_init_tx: // Release bus after starting write
//	  flexible index will change the <2 to <len(tmpr_ain_cmd[])
	  if (tmpr_slave_index <2 ) {
		  ++tmpr_slave_index;
		  tmpr_state = tmpr_t1_init;
	  } else {
		  tmpr_slave_index = 0;
		  tmpr_state = tmpr_t1_read_cfg;
	  }
      return;
    case tmpr_t1_read_cfg: // Start read from config register
      i2c_read(tmpr_slave_addr[tmpr_slave_index], tmpr_ibuf, 2);
      tmpr_state = tmpr_t1_read_cfg_tx;
      return;
    case tmpr_t1_read_cfg_tx: // If high bit is set, conversion is complete
      if (tmpr_ibuf[0] & 0x80) {
        tmpr_state = tmpr_t1_reg0;
      } else {
//        ++ads_n_reads;
        tmpr_state = tmpr_t1_read_cfg;
      }
      return;
    case tmpr_t1_reg0: // Write pointer register to read from conversion reg [0]
      i2c_write(tmpr_slave_addr[tmpr_slave_index], tmpr_r0_prep, 1);
      tmpr_state = tmpr_t1_read_adc;
      return;
    case tmpr_t1_read_adc: // Start read from conversion reg
      i2c_read(tmpr_slave_addr[tmpr_slave_index], tmpr_ibuf, 2);
      tmpr_state = tmpr_t1_read_adc_tx;
      return;
    case tmpr_t1_read_adc_tx: // Save converted value
//          cache index will change with flexible # inputs : replace the  " * 2 "
//          i2c_tmpr_cache[ tmpr_slave_index * (len(tmpr_ain_cmd[]) - 1) ] = ...
	  i2c_tmpr_cache[tmpr_slave_index * 2].cache = (tmpr_ibuf[0] << 8) | tmpr_ibuf[1];
	  
//      sb_cache_update(i2c_tmpr_cache, tmpr_slave_index * 2 + (tmpr_slave_index >= 2 ? 1 : 0), 
//		(tmpr_ibuf[0] << 8) | tmpr_ibuf[1]);
//      sb_cache_update(i2c_tmpr_cache, I2C_ADS_OFFSET+8, ads_n_reads);

//	  flexible index will change the <2 to <len(tmpr_ain_cmd[])
	  if (tmpr_slave_index <2 ) {
		  ++tmpr_slave_index;
		  tmpr_state = tmpr_t1_read_cfg;
	  } else {
		  tmpr_slave_index = 0;
		  tmpr_state = tmpr_t2_init;
	  }
      return;
    case tmpr_t2_init:
//      ads_n_reads = 0;
      i2c_write(tmpr_slave_addr[tmpr_slave_index], tmpr_t2_cmd, 4);
      tmpr_state = tmpr_t2_init_tx;
      return;
    case tmpr_t2_init_tx:
	  if (tmpr_slave_index <2 ) {
		  ++tmpr_slave_index;
		  tmpr_state = tmpr_t2_init;
	  } else {
		  tmpr_slave_index = 0;
		  tmpr_state = tmpr_t2_read_cfg;
	  }
      return;
    case tmpr_t2_read_cfg:
      i2c_read(tmpr_slave_addr[tmpr_slave_index], tmpr_ibuf, 2);
      tmpr_state = tmpr_t2_read_cfg_tx;
      return;
    case tmpr_t2_read_cfg_tx:
      if (tmpr_ibuf[0] & 0x80) {
        tmpr_state = tmpr_t2_reg0;
      } else {
//        ++ads_n_reads;
        tmpr_state = tmpr_t2_read_cfg;
      }
      return;
    case tmpr_t2_reg0:
      i2c_write(tmpr_slave_addr[tmpr_slave_index], tmpr_r0_prep, 1);
      tmpr_state = tmpr_t2_read_adc;
      return;
    case tmpr_t2_read_adc:
      i2c_read(tmpr_slave_addr[tmpr_slave_index], tmpr_ibuf, 2);
      tmpr_state = tmpr_t2_read_adc_tx;
      return;
    case tmpr_t2_read_adc_tx:
//          cache index will change with flexible # inputs : replace the  " * 2 "
//          i2c_tmpr_cache[ tmpr_slave_index * (len(tmpr_ain_cmd[]) - 1) ] = ...
	  i2c_tmpr_cache[tmpr_slave_index * 2 + 1].cache = (tmpr_ibuf[0] << 8) | tmpr_ibuf[1];
	  
// May need adjusting cache addressing to match desired order
//      sb_cache_update(i2c_tmpr_cache, tmpr_slave_index * 2 + (tmpr_slave_index < 2 ? 1 : 0), 
//		(tmpr_ibuf[0] << 8) | tmpr_ibuf[1]);
//      sb_cache_update(i2c_tmpr_cache, I2C_ADS_OFFSET+8, ads_n_reads);

//	  flexible index will change the <2 to <len(tmpr_ain_cmd[])
	  if (tmpr_slave_index <2 ) {
		  ++tmpr_slave_index;
		  tmpr_state = tmpr_t2_read_cfg;
	  } else {
		  tmpr_slave_index = 0;
		  tmpr_state = tmpr_t1_init;
	  }
      return;
    default:
      assert(false, __FILE__, __LINE__);
  }
  return;
}

static void i2c_write(int16_t i2c_addr, const uint8_t *obuf, int16_t nbytes) {
  assert(tmpr_txfr_complete, __FILE__, __LINE__);
  tmpr_txfr_complete = false;
  i2c_m_async_set_slaveaddr(&I2C_TMPR, i2c_addr, I2C_M_SEVEN);
  io_write(I2C_TMPR_IO, obuf, nbytes);
}

static void i2c_read(int16_t i2c_addr, uint8_t *ibuf, int16_t nbytes) {
  assert(tmpr_txfr_complete, __FILE__, __LINE__);
  tmpr_txfr_complete = false;
  i2c_m_async_set_slaveaddr(&I2C_TMPR, i2c_addr, I2C_M_SEVEN);
  io_read(I2C_TMPR_IO, ibuf, nbytes);
}

void tmpr_enable(bool value) {
  tmpr_enabled = value;
}

#define I2C_INTFLAG_ERROR (1<<7)
static void i2c_tmpr_async_error(struct i2c_m_async_desc *const i2c, int32_t error) {
  tmpr_txfr_complete = true;
  tmpr_error_seen = true;
  tmpr_error = error;
/* 
 *  Need to introduce an I2C_STATUS register and correspoinding 
 *  I2C_STATUS_OFFSET to include this error handling.
 *
  if (sb_cache_was_read(i2c_tmpr_cache, I2C_STATUS_OFFSET)) {
    sb_cache_update(i2c_tmpr_cache, I2C_STATUS_OFFSET, 0);
  }
  if (tmpr_error >= -7 && tmpr_error <= -2) {
    uint16_t val = i2c_tmpr_cache[I2C_STATUS_OFFSET].cache;
    val |= (1 << (7+tmpr_error));
    sb_cache_update(i2c_tmpr_cache, I2C_STATUS_OFFSET, val);
  }
 */
  if (error == I2C_ERR_BUS) {
    hri_sercomi2cm_write_STATUS_reg(&I2C_TMPR, SERCOM_I2CM_STATUS_BUSERR);
    hri_sercomi2cm_clear_INTFLAG_reg(&I2C_TMPR, I2C_INTFLAG_ERROR);
  }
}

static void tmpr_txfr_completed(struct i2c_m_async_desc *const i2c) {
  tmpr_txfr_complete = true;
}

static void i2c_tmpr_reset() {
  if (!sb_i2c_tmpr.initialized) {
    i2c_m_async_get_io_descriptor(&I2C_TMPR, &I2C_TMPR_IO);
    i2c_m_async_enable(&I2C_TMPR);
    i2c_m_async_register_callback(&I2C_TMPR, I2C_M_ASYNC_ERROR,       (FUNC_PTR)i2c_tmpr_async_error);
    i2c_m_async_register_callback(&I2C_TMPR, I2C_M_ASYNC_TX_COMPLETE, (FUNC_PTR)tmpr_txfr_completed);
    i2c_m_async_register_callback(&I2C_TMPR, I2C_M_ASYNC_RX_COMPLETE, (FUNC_PTR)tmpr_txfr_completed);
    sb_i2c_tmpr.initialized = true;
  }
}

subbus_driver_t sb_i2c_tmpr = {
  I2C_TMPR_BASE_ADDR, I2C_TMPR_HIGH_ADDR, // address range for Battery Monitoring
  i2c_tmpr_cache,                 // Host accessible cache structure
  i2c_tmpr_reset,                 // reset function
  i2c_tmpr_poll,                  // poll function
  0,                              // No Dynamic function
  false                           // initialized ?
};