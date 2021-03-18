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
#include "Timer_Setup.h"

static void update_status(uint16_t *status, uint8_t pin, uint16_t bit) {
  if (gpio_get_pin_level(pin)) {
    *status |= bit;
  } else {
    *status &= ~bit;
  }
}

static subbus_cache_word_t cmd_cache[CMD_HIGH_ADDR-CMD_BASE_ADDR+1] = {
  { 0, 0, true, false,  true, false, false }, // Offset 0: R: Command/Fault Status. W: Command value
  { 0, 0, true, false, false, false, false }  // Offset 1: R: LED/Misc IO Status
};

static int startup_commands[] = { STARTUP_COMMANDS };
static int n_startup_commands_executed = 0;
static uint32_t startup_sequence_time = 0;

// * CMD_BASE_ADDR 0x40
static void cmd_poll(void) {
  uint16_t cmd;
  bool have_cmd = false;
  uint16_t status = cmd_cache[0].cache;
  if (startup_sequence_time == 0) {
    startup_sequence_time = count_1msec + 1000;
  }
  if ((n_startup_commands_executed < sizeof(startup_commands)/sizeof(int)) &&
       (count_1msec >= startup_sequence_time)) {
    cmd = startup_commands[n_startup_commands_executed++];
    startup_sequence_time = count_1msec + 1000;
    have_cmd = true;
  } else if (subbus_cache_iswritten(&sb_cmd, CMD_BASE_ADDR, &cmd)) {
    have_cmd = true;
  }
  if (have_cmd) {
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
//  * CMD_BASE_ADDR 0x40
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
// Turn LEDs on according to status
  gpio_set_pin_level(STATUS_O, (status & 0x00FF ));
  gpio_set_pin_level(FAULT_O, (~status & 0xFF00 ));

// Update current LED status
// * CMD_BASE_ADDR 0x41
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
