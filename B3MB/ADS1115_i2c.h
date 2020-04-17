/* ***************************************************************************************
 * ADS1115_i2c.h
 *
 * Created: 4/14/2020 10:41:14 AM, 
 * Derived from GitHub master branch of BMM_A0_01\i2c.h
 * Modified for use on B3MB board,
 * Rev_01 Litch - created
 *
 * Three I2C interfaces, subbus drivers servicing a total of 7 Quad Input ADS1115's
 *    batt_a2d - 2 ADS1115's, single ended, (Voltage and Current) x (BATT1 - BATT4)
 *    load_a2d - 2 ADS1115's, single ended, (Voltage and Current) x (LOAD1 - LOAD4)
 *    temp_a2d - 3 ADS1115's, differential, Batt_Bus V_monitor, TEMP1 - TEMP5 (temperatures)
 *
 */ 

#ifndef ADS1115_I2C_H_
#define ADS1115_I2C_H_

#include "subbus.h"

#define BATT_ENABLE_DEFAULT true
#define LOAD_ENABLE_DEFAULT true
#define TEMP_ENABLE_DEFAULT true

extern subbus_driver_t sb_i2c_batt;
extern subbus_driver_t load_a2d;
extern subbus_driver_t temp_a2d;

void batt_enable(bool value);  // I2C Comm to A2Ds for Ideal Diode Isolated Battery Voltage and Current
void load_enable(bool value);  // I2C_Comm to A2Ds for                         Load Voltage and Current
void temp_enable(bool value);  // I2C Comm to  A2Ds for Battery Bus Voltage and 5 Temperature Sensors

#endif /* ADS1115_I2C_H_ */