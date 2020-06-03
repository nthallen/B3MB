#include "pins_perphs_init.h"
#include "subbus.h"
#include "Timer_Setup.h"
#include "I2C_ADS1115.h"

#define EVENT0 0
#define EVENT1 2000
#define EVENT2 8000
#define EVENT3 50000
#define EVENT4 60000
#define EVENT5 70000
#define EVENT6 80000
#define EVENT7 90000
#define EVENT8 100000

int main(void) {
  system_init();
  TIMER_0_go();    // Start up the 1 millisecond counter (available at global count_1ms)

  // Add in all needed drives and test for success
  if ( subbus_add_driver(&sb_base)     ||
  	   subbus_add_driver(&sb_i2c_batt) || 
       subbus_add_driver(&sb_i2c_load)   ) { 
         while(true);                          // if True => some driver is mis configured.
  }		
  	
  // reset all resettable drivers and spin forever
  subbus_reset();
  
  gpio_set_pin_level(STATUS_O, true);
  uint32_t current_time = count_1msec;
  uint32_t elapsed_time = current_time;
  
  gpio_set_pin_level(BATT1_ON, true);
  while (1) {
	elapsed_time = count_1msec - current_time;
	subbus_poll();
    if ( EVENT0   <  elapsed_time && elapsed_time <  EVENT1 ) { gpio_set_pin_level(LOAD1_ON, true);  }
	if ( EVENT1+1 <= elapsed_time && elapsed_time <  EVENT2 ) { gpio_set_pin_level(LOAD1_ON, false); }
	if ( EVENT2+1 <= elapsed_time && elapsed_time ) { current_time = count_1msec; }
		
		
//	if ( EVENT5 < elapsed_time && elapsed_time <  EVENT5 + 2 ) { gpio_set_pin_level(BATT3_ON, true);  }
//	if ( EVENT6 < elapsed_time && elapsed_time <  EVENT6 + 2 ) { gpio_set_pin_level(BATT3_ON, false); }
//	if ( EVENT7 < elapsed_time && elapsed_time <  EVENT7 + 2 ) { gpio_set_pin_level(BATT4_ON, true);  }
//  if ( EVENT8 < elapsed_time && elapsed_time <  EVENT8 + 2 ) { gpio_set_pin_level(BATT4_ON, false); }
		
    gpio_toggle_pin_level(STATUS_O);
	
  }
}
