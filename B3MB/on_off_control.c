/*
 * on_of_control.c
 *
 * Created: 6/18/2020 14:57:26
 * Rev_01 Litch - created
 *
 * An driver for controlling B3MB Battery and Load on off switches via subbus writes from Host
 * 8 Control Points, Bat1 - 4 On/Off Battery Packs to Battery Bus, Load 1 - 4 On/Off Battery Bus to Load
 * 
 */ 

#include <string.h>
#include "pin_defines.h"
#include "subbus.h"
