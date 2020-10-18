/* ********************************************************************************************
 * commands.c
 * 
 * Command Interface for 8 switches and 8 Alert/Status bits
 * 
 * Rev_00: Marco Rivero 4:55 PM 10/15/2020
 * 	Derived from WI-ICOS-MAINS/commands.c 
 * 
 */ 
#include "pins_perphs_init.h"
#include "commands.h"
#include "subbus.h"

static void update_status(uint16_t *status, uint8_t pin, uint16_t bit) {
  if (gpio_get_pin_level(pin)) {
    *status |= bit;
  } else {
    *status &= ~bit;
  }
}

/**
 * CMD_BASE_ADDR 0x40
 * Command Word  Definition: 
 *  Cmd	Function    	RW	Notes
 *  0	BATT1 OFF		W	Battery 1 Switch Off
 *  1	BATT1 ON		W	Battery 1 Switch On
 *  2	BATT2 OFF		W	Battery 2 Switch Off
 *  3	BATT2 ON		W	Battery 2 Switch On
 *  4	BATT3 OFF		W	Battery 3 Switch Off
 *  5	BATT3 ON		W	Battery 3 Switch On
 *  6	BATT4 OFF		W	Battery 4 Switch Off
 *  7	BATT4 ON		W	Battery 4 Switch On
 *  8	LOAD1 OFF		W	Load 1 Switch Off
 *  9	LOAD1 ON		W	Load 1 Switch On
 *  10	LOAD2 OFF		W	Load 2 Switch Off
 *  11	LOAD2 ON		W	Load 2 Switch On
 *  12	LOAD3 OFF		W	Load 3 Switch Off
 *  13	LOAD3 ON		W	Load 3 Switch On
 *  14	LOAD4 OFF		W	Load 4 Switch Off
 *  15	LOAD4 ON		W	Load 4 Switch On
 *  16	ALL LOADS OFF 	W	ALL LOADS OFF
 *  17	ALL LOADS ON	W	ALL LOADS ON
 *  18	ALL BATTS OFF	W	ALL BATTS OFF
 *  19	ALL BATTS ON 	W	ALL BATTS ON 
 *  20	STATUS_LED OFF	W	STATUS_LED OFF	
 *  21	STATUS_LED ON	W	STATUS_LED ON	
 *  22	ALERT_LED OFF	W	ALERT_LED OFF	
 *  23	ALERT_LED ON	W	ALERT_LED ON	
 * 
 * CMD_BASE_ADDR 0x40
 * Status Word 1 Definition: 
 *  Bit	Function    	RW	Notes
 *  0	BATT1 ON/OFF	R	Battery 1 Switch Status
 *  1	BATT2 ON/OFF	R	Battery 2 Switch Status
 *  2	BATT3 ON/OFF	R	Battery 3 Switch Status
 *  3	BATT4 ON/OFF	R	Battery 4 Switch Status
 *  4	LOAD1 ON/OFF	R	Load 1 Switch Status
 *  5	LOAD2 ON/OFF	R	Load 2 Switch Status
 *  6	LOAD3 ON/OFF	R	Load 3 Switch Status
 *  7	LOAD4 ON/OFF	R	Load 4 Switch Status
 *  8	BATT1 FAULT 	R	Battery 1 Fault Status
 *  9	BATT2 FAULT 	R	Battery 2 Fault Status
 *  10	BATT3 FAULT 	R	Battery 3 Fault Status
 *  11	BATT4 FAULT 	R	Battery 4 Fault Status
 *  12	LOAD1 FAULT 	R	Load 1 Fault Status
 *  13	LOAD2 FAULT 	R	Load 2 Fault Status
 *  14	LOAD3 FAULT 	R	Load 3 Fault Status
 *  15	LOAD4 FAULT 	R	Load 4 Fault Status
 *
 * CMD_BASE_ADDR 0x41
 * Status Word 2 Definition: 
 *  Bit	Function   	RW	Notes
 *  0	STATUS  	R	STATUS LED State
 *  1	FAULT   	R	FAULT LED State
 *  2	ID_CPU  	R	ID_CPU Pin State
 *  3	ALERT   	R	ALERT Pin State

 */
static subbus_cache_word_t cmd_cache[CMD_HIGH_ADDR-CMD_BASE_ADDR+1] = {
  { 0, 0, true, false,  true, false, false }, // Offset 0: R: Command/Fault Status. W: Command value
  { 0, 0, true, false, false, false, false }  // Offset 1: R: LED/Misc IO Status
};

static void cmd_poll(void) {
  uint16_t cmd;
  uint16_t status = cmd_cache[0].cache;
  if (subbus_cache_iswritten(&sb_cmd, CMD_BASE_ADDR, &cmd)) {
    switch (cmd) {
      case 0:  gpio_set_pin_level(BATT1_ON, false); break;	// Batt 1 OFF
      case 1:  gpio_set_pin_level(BATT1_ON, true); break;	// Batt 1 ON
      case 2:  gpio_set_pin_level(BATT2_ON, false); break;	// Batt 2 OFF
      case 3:  gpio_set_pin_level(BATT2_ON, true); break;	// Batt 2 ON
      case 4:  gpio_set_pin_level(BATT3_ON, false); break;	// Batt 3 OFF
      case 5:  gpio_set_pin_level(BATT3_ON, true); break;	// Batt 3 ON
      case 6:  gpio_set_pin_level(BATT4_ON, false); break;	// Batt 4 OFF
      case 7:  gpio_set_pin_level(BATT4_ON, true); break;	// Batt 4 ON
      case 8:  gpio_set_pin_level(LOAD1_ON, false); break;	// Load 1 OFF
      case 9:  gpio_set_pin_level(LOAD1_ON, true); break;	// Load 1 ON
      case 10: gpio_set_pin_level(LOAD2_ON, false); break;	// Load 2 OFF
      case 11: gpio_set_pin_level(LOAD2_ON, true); break;	// Load 2 ON
      case 12: gpio_set_pin_level(LOAD3_ON, false); break;	// Load 3 OFF
      case 13: gpio_set_pin_level(LOAD3_ON, true); break;	// Load 3 ON
      case 14: gpio_set_pin_level(LOAD4_ON, false); break;	// Load 4 OFF
      case 15: gpio_set_pin_level(LOAD4_ON, true); break;	// Load 4 ON
      case 16: // All Loads OFF
        gpio_set_pin_level(LOAD1_ON, false);
        gpio_set_pin_level(LOAD2_ON, false);
        gpio_set_pin_level(LOAD3_ON, false);
        gpio_set_pin_level(LOAD4_ON, false);
        break;
      case 17: // All Loads ON
        gpio_set_pin_level(LOAD1_ON, true);
        gpio_set_pin_level(LOAD2_ON, true);
        gpio_set_pin_level(LOAD3_ON, true);
        gpio_set_pin_level(LOAD4_ON, true);
        break;
      case 18: // All Batts OFF
        gpio_set_pin_level(BATT1_ON, false);
        gpio_set_pin_level(BATT2_ON, false);
        gpio_set_pin_level(BATT3_ON, false);
        gpio_set_pin_level(BATT4_ON, false);
        break;
      case 19: // All Batts ON
        gpio_set_pin_level(BATT1_ON, true);
        gpio_set_pin_level(BATT2_ON, true);
        gpio_set_pin_level(BATT3_ON, true);
        gpio_set_pin_level(BATT4_ON, true);
        break;
      case 20:  gpio_set_pin_level(STATUS_O, false); break;	// STATUS_LED OFF
      case 21:  gpio_set_pin_level(STATUS_O, true); break;	// STATUS_LED ON
      case 22:  gpio_set_pin_level(FAULT_O, false); break;	// ALERT_LED OFF
      case 23:  gpio_set_pin_level(FAULT_O, true); break;	// ALERT_LED ON
      default:
        break;
    }
  }
// Update current Batt/Load Switch and Alert status
  status = 0;
  update_status(&status, BATT1_ON, 0x0001);
  update_status(&status, BATT2_ON, 0x0002);
  update_status(&status, BATT3_ON, 0x0004);
  update_status(&status, BATT4_ON, 0x0008);
  update_status(&status, LOAD1_ON, 0x0010);
  update_status(&status, LOAD2_ON, 0x0020);
  update_status(&status, LOAD3_ON, 0x0040);
  update_status(&status, LOAD4_ON, 0x0080);
  update_status(&status, BATT1_FLT, 0x0100);
  update_status(&status, BATT2_FLT, 0x0200);
  update_status(&status, BATT3_FLT, 0x0400);
  update_status(&status, BATT4_FLT, 0x0800);
  update_status(&status, LOAD1_FLT, 0x1000);
  update_status(&status, LOAD2_FLT, 0x2000);
  update_status(&status, LOAD3_FLT, 0x4000);
  update_status(&status, LOAD4_FLT, 0x8000);
  subbus_cache_update(&sb_cmd, CMD_BASE_ADDR, status);
 // Update current LED status
  status = 0;
  update_status(&status, STATUS_O, 0x0001);
  update_status(&status, FAULT_O, 0x0002);
  update_status(&status, ID_CPU, 0x0004);
  update_status(&status, ALERT, 0x0008);
  subbus_cache_update(&sb_cmd, CMD_BASE_ADDR+1, status);
}

static void cmd_reset(void) {
  if (!sb_cmd.initialized) {
    sb_cmd.initialized = true;
  }
}

subbus_driver_t sb_cmd = {
  CMD_BASE_ADDR, CMD_HIGH_ADDR, // address range
  cmd_cache,
  cmd_reset,
  cmd_poll,
  0, // dynamic driver
  false
};
