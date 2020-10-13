/* ***************************************************************************************
 * i2c_ADS1115.h
 *
 * Created: 4/14/2020 10:41:14 AM, 
 * Derived from GitHub master branch of BMM_A0_01\i2c.h
 * Modified for use on B3MB board,
 * Rev_01 Litch - created
 *
 * Three I2C interfaces, subbus drivers servicing a total of 7 Quad Input ADS1115's
 *    sb_i2c_batt - 2 ADS1115's, single ended, (Voltage and Current) x (BATT1 - BATT4)
 *    sb_i2c_load - 2 ADS1115's, single ended, (Voltage and Current) x (LOAD1 - LOAD4)
 *    sb_i2c_tmpr - 3 ADS1115's, differential, Batt_Bus V_monitor, TEMP1 - TEMP5 (temperatures)
 *
 */ 

#ifndef I2C_ADS1115_H_
#define I2C_ADS1115_H_

#include "subbus.h"

#define BATT_ENABLE_DEFAULT true
#define LOAD_ENABLE_DEFAULT true
#define TEMP_ENABLE_DEFAULT true

#define I2C_A_DEV_ADDR 0x48    // I2C device addresses for ADS1115s  Location A
#define I2C_B_DEV_ADDR 0x49    //                                    Location B
#define I2C_C_DEV_ADDR 0x4A    //                                    Location C
#define CONVERSION_REG 0       // ADS1115s internal conversion register address (where to read data)

// Add Board Specific Cache Addresses here
#define I2C_BATT_BASE_ADDR   0x20
#define I2C_BATT_HIGH_ADDR   0x27
#define I2C_LOAD_BASE_ADDR   0x28
#define I2C_LOAD_HIGH_ADDR   0x2F
#define I2C_TMPR_BASE_ADDR   0x30
#define I2C_TMPR_HIGH_ADDR   0x35

void batt_enable(bool value);  // I2C Comm to A2Ds for Ideal Diode Isolated Battery Voltage and Current
void load_enable(bool value);  // I2C_Comm to A2Ds for                         Load Voltage and Current
void tmpr_enable(bool value);  // I2C Comm to A2Ds for Battery Bus Voltage and 5 Temperature Sensors

#endif /* ADS1115_I2C_H_ */