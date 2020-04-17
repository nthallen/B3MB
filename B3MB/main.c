#include "pins_perphs_init.h"
#include "subbus.h"
#include "Timer_Setup.h"
#include "ADS1115_i2c.h"

int main(void) {
  system_init();
  TIMER_0_go();    // Start up the 1 millisecond counter (available at global count_1ms)

  // Add in all needed drives and test for success
  if ( subbus_add_driver(&sb_base)  ||
  	   subbus_add_driver(&sb_i2c_batt) ) { 
         while(true);                          // if True => some driver is mis configured.
  }		
  	
  // reset all resettable drivers and spin forever
  subbus_reset();
  
  gpio_set_pin_level(STATUS_O, true);
  uint32_t current_time = count_1msec;
  uint32_t elapsed_time = current_time;
  
  while (1) {
	elapsed_time = count_1msec - current_time;
	subbus_poll();
    if (elapsed_time > 1) {
      current_time = count_1msec;
      gpio_toggle_pin_level(STATUS_O);
    }
  }
}
