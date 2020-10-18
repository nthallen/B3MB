#ifndef COMMANDS_H_INCLUDED
#define COMMANDS_H_INCLUDED

#include "pin_defines.h"
#include "subbus.h"

#define CMD_BASE_ADDR 0x40
#define CMD_HIGH_ADDR 0x41

extern subbus_driver_t sb_cmd;

#endif
