#include "pins_perphs_init.h"
#include "subbus.h"
#include "can_control.h"
#include "Timer_Setup.h"
#include "I2C_ADS1115.h"

int main(void) {
  system_init();
  TIMER_0_go();    // Start up the 1 millisecond counter (available at global count_1ms)

  // Add in all needed drives and test for success
  if ( subbus_add_driver(&sb_base)       ||
       subbus_add_driver(&sb_can_desc)   ||
  	   subbus_add_driver(&sb_i2c_batt)   || 
       subbus_add_driver(&sb_i2c_load)   ||
       subbus_add_driver(&sb_i2c_tmpr)   ||
       subbus_add_driver(&sb_on_off)     ||
	   subbus_add_driver(&sb_can)          ) { 
         while(true);                          // if True => some driver is mis configured.
  }		
  	
  // reset all resettable drivers and spin forever
  subbus_reset();
  
  // Make sure all loads are off and Only BATT1 is on
  gpio_set_pin_level(LOAD1_ON, false);
  gpio_set_pin_level(LOAD2_ON, false);
  gpio_set_pin_level(LOAD3_ON, false);
  gpio_set_pin_level(LOAD4_ON, false);
  gpio_set_pin_level(BATT1_ON, true);   
  gpio_set_pin_level(BATT2_ON, false);
  gpio_set_pin_level(BATT3_ON, false);
  gpio_set_pin_level(BATT4_ON, false);
  
  // loop forever, do its thing
  while (1) {
	subbus_poll();
    gpio_toggle_pin_level(STATUS_O);
  }
}
