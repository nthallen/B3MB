#ifndef SUBBUS_H_INCLUDED
#define SUBBUS_H_INCLUDED

#include <stdint.h>
#include <stdbool.h>
#include "serial_num.h"

// All boards must support these subbus Baseline Cache Addresses / Information
#define SUBBUS_FAIL_RESERVED        0xF000
#define SUBBUS_INTA_ADDR            0x0001
#define SUBBUS_BDID_ADDR            0x0002  // Board ID, 6 = B3MB, 28v
#define SUBBUS_BLDNO_ADDR           0x0003  // Firmware Build Number
#define SUBBUS_BDSN_ADDR            0x0004  // Board Serial Number
#define SUBBUS_INSTID_ADDR          0x0005  // Instrument ID
#define SUBBUS_FAIL_ADDR            0x0006  // Aircraft Interface Fail Indicator
#define SUBBUS_SWITCHES_ADDR        0x0007  // What's this ????
#define SUBBUS_DESC_FIFO_SIZE_ADDR  0x0008  // Large Blocks of data can be made available via FIFO
#define SUBBUS_DESC_FIFO_ADDR       0x0009  // Host Side read Address of FIFO output
#define SUBBUS_ADDR_CMDS			0x0018  // Query Address?		What's this ????

#define SUBBUS_MAX_DRIVERS          7

// Define the subbus's Host Interface Cache structure
typedef struct {
  uint16_t cache;    // The current value of this word as written by hardware
  uint16_t wvalue;   // The value written by Host. Allows the driver code to do checks for validity
  bool readable;     // True if this cache word location is readable by the Host
  bool was_read;     // True if this cache word location has been read by the Host
  bool writable;     // True if this cache word location is writable by the Host
  bool written;      // True if this cache word location has been written by the Host
  bool dynamic;      // True if the driver supports sb_action function immediately on Host action
                     //    rather than waiting for next poll to come around for driver.
} subbus_cache_word_t;

// Define the subbus's Driver structure
typedef struct {
  uint16_t low, high;                 // Base and Upper Address spanned in Cache address space
  subbus_cache_word_t *cache;         // the cache structured memory locations themselves
  void (*reset)(void);                // driver function to invoke (can be null) during subbus resets
  void (*poll)(void);                 // driver function to invoke (can be null) during subbus polling
  void (*sb_action)(uint16_t offset); // driver function to invoke (can be null) on Host action dynamic cache
  bool initialized;                   // Flag, driver initialized ?
} subbus_driver_t;

// cache maintenance functions available to all invoked subbus drivers
/**
 * Fully qualified check that the specified subbus address is both
 * valid and has been written. If so, copies the written value into
 * the value argument and resets the iswritten flag.
 * @param drv pointer to the subbus_driver_t struct
 * @param addr the subbus address
 * @param value pointer where the new value should be stored
 * @return true if the address is valid and the iswritten flag was set
 */
bool subbus_cache_iswritten(subbus_driver_t *drv, uint16_t addr, uint16_t *value);

/**
 * Unqualified check of the local cache similar to subbus_cache_iswritten, but
 * using the offset within the local cache instead of the global address.
 * @param cache pointer to the local cache
 * @param offset of the register in the local cache
 * @param value pointer where the new value should be stored
 * @return true if the iswritten flag was set
 */
bool sb_cache_iswritten(subbus_cache_word_t *cache, uint16_t offset, uint16_t *value);

/**
 * @param drv Pointer to the subbus_driver_t structure defining the driver.
 * @param addr The Absolute address of the desired cache register.
 * @return true if the specified cache register has been read since it was last updated.
 */
bool subbus_cache_was_read(subbus_driver_t *drv, uint16_t addr);

/**
 * @param cache Pointer to the driver's cache array.
 * @param offset The relative offset of the desired cache register within this cache array.
 * @return true if the specified cache register has been read since it was last updated.
 */
bool sb_cache_was_read(subbus_cache_word_t *cache, uint16_t offset);

/**
 * This function differs from subbus_write() in that it directly
 * updates the cache value. subbus_write() is specifically for
 * writes originating from the control port.
 *
 * subbus_cache_update() is used by internal functions for storing data
 * acquired from peripherals, or for storing values written from the control
 * port after verifying them.
 *
 * @param drv The driver structure
 * @param addr The cache address
 * @param data The value to be written
 * @return true on success
 */
bool subbus_cache_update(subbus_driver_t *drv, uint16_t addr, uint16_t data);

/**
 * This function differs from subbus_write() in that it directly
 * updates the cache value.
 *
 * This function differes from subbus_cache_update() in that it
 * directly addresses the local cache without checking the range.
 *
 * On success, returns true and clears the was_read flag.
 *
 * @param cache pointer to the local cache
 * @param offset of the register in the local cache
 * @param data The value to be written
 * @return true on success
 */
bool sb_cache_update(subbus_cache_word_t *cache, uint16_t offset, uint16_t data);

int subbus_read( uint16_t addr, uint16_t *rv );
int subbus_write( uint16_t addr, uint16_t data);
void subbus_reset(void);
void subbus_poll(void);

// List default / mandatory subbus drivers here
bool subbus_add_driver(subbus_driver_t *driver);
extern subbus_driver_t sb_base;
extern subbus_driver_t sb_board_desc;

// subbus enabled interrupt functions available to all invoked subbus drivers
#define SUBBUS_INTERRUPTS           0
#if SUBBUS_INTERRUPTS
extern volatile uint8_t subbus_intr_req;
void init_interrupts(void);
int  intr_attach(int id, uint16_t addr);
int  intr_detach( uint16_t addr );
void intr_service(void);
#endif
#endif
