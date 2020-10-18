/* *********************************************************************************
 * The Subbus concept is central to the Norton Architecture - Embedded code side.
 *
 * It consists of a collection of hardware "drivers" defined by a structure.
 * Each driver structure can implement up to three functions:
 *		Reset
 *		Poll (write latest data from hardware / read latest info from host)
 *      Action (do "now" when host reads or writes to or from driver's caches).
 *
 * The poll function utilizes an addressable "cached" memory whose address
 * space is defined within the driver structure by address_low and address_high 
 * members.
 *
 * The main function's while(1) loop acts as a state clock, during each
 * state clock cycle a driver's poll function (if it exists) is invoked to
 * evaluate the current state, it's inputs, take appropriate steps under those
 * determined conditions and set its next state .. forever.
 *
 * Any state in any driver's poll_state_machines should not require more than
 * ~ 20uSec to execute (add more states, use non-blocking async hardware resources).
 * This helps to "guarantee" "real-time" responsiveness (no latency anywhere 
 * in excess of ~ 200usec = 20us max state duration * 10(SUBBUS_MAX_DRIVERS)   
 *
 * The driver's cache is an "intelligent memory" structure used as 
 * coherent, bi-directional, shared memory between the drive (local embedded code)
 * and the instrument's host.  
 * 
 * Communications between the two may occur on any of many possible physical
 * layers (i.e. RS282, RS485, USB, CAN, MODBUS, other).
 *
 * Each cached memory location has seven properties as follows:
 *
 *   1          2        3           4           5           6             7
 * value,     value,  boolean,    boolean,    boolean,    boolean,       boolean
 * hardware,  host,   readable?,  was_read?,  writable?,  was_written?,  dynamic?
 *
 * There exists a series of cache functions that can be exploited by the driver
 * to read from or write to the cache and maintain coherency between Host and driver
 * some are described in this heading here:
 * 
 *  int subbus_read(uint16_t addr, uint16_t *rv)  --- Host reads value from cache
 *           returns (via pointer rv) value hardware at cache addr,
 *           marks was read as true
 *           if driver needs to act immediately (dynamic = true) invokes
 *              driver's dynamic method then and there.
 *           updates location pointed at by *rv with host written value
 *           returns 1 if successful, 0 if handed in address is out of range.
 *
 *  int subbus_write(uint16_t addr, uint16_t data) --- Host writes value into cache
 *           writes data to value host at cache addr,
 *           marks was written as true
 *           if driver needs to act immediately (dynamic = true) invokes
 *              driver's dynamic method then and there.
 *           updates location pointed at by *rv with host written value 
 */
#include <string.h>
#include "pin_defines.h"  // for sb_on_off only
#include "subbus.h"

static subbus_driver_t *drivers[SUBBUS_MAX_DRIVERS];
static int n_drivers = 0;

/* *******************************************************************************
 * Add Subbus drivers, @return true on error.
 *		Possible errors include too many drivers or drivers address space 
 *		not in ascending order.
 */
bool subbus_add_driver(subbus_driver_t *driver) {
  if ((n_drivers >= SUBBUS_MAX_DRIVERS) ||
      ((n_drivers > 0) && (drivers[n_drivers-1]->high > driver->low)) ||
      driver->high < driver->low ) { return true; }
  drivers[n_drivers++] = driver;
  return false;
}

/* ********************************************************************************
 * Reset all added drivers that support a reset method
 *
 */
void subbus_reset(void) {
  for (int i = 0; i < n_drivers; ++i) {
    if (drivers[i]->reset) { (*(drivers[i]->reset))(); }
  }
}

/* ********************************************************************************
 * Invoke poll method for all added drivers that instantiate a polling method
 *
 */
void subbus_poll(void) {
  for (int i = 0; i < n_drivers; ++i) {
    if (drivers[i]->poll) { (*drivers[i]->poll)(); }
  }
}

/* *********************************************************************
 * Host side Read from cache, 
 * invoked via physical layer's control module detecting a read request
 * for any driver, any new data / commands
 *		@return non-zero on success (acknowledge)
 */
int subbus_read(uint16_t addr, uint16_t *rv) {
  int i;
  for (i = 0; i < n_drivers; ++i) {
    if (addr < drivers[i]->low) return 0;
    if (addr <= drivers[i]->high) {
      uint16_t offset = addr-drivers[i]->low;
      subbus_cache_word_t *cache = &drivers[i]->cache[offset];
      if (cache->readable) {
        *rv = cache->cache;
        cache->was_read = true;
        if (cache->dynamic && drivers[i]->sb_action)
          drivers[i]->sb_action(offset);
        return 1;
      }
    }
  }
  *rv = 0;
  return 0;
}

/* ***********************************************************************
 * Host side Write into cache, 
 * invoked via physical layer's control module detecting a write request
 * for any driver, any new data / commands
 *		@return non-zero on success (acknowledge)
 */
int subbus_write(uint16_t addr, uint16_t data) {
  int i;
  for (i = 0; i < n_drivers; ++i) {
    if (addr < drivers[i]->low) return 0;
    if (addr <= drivers[i]->high) {
      uint16_t offset = addr-drivers[i]->low;
      subbus_cache_word_t *cache = &drivers[i]->cache[offset];
      if (cache->writable) {
        cache->wvalue = data;
        cache->written = true;
        if (cache->dynamic && drivers[i]->sb_action)
          drivers[i]->sb_action(offset);
        return 1;
      }
    }
  }
  return 0;
}

#if SUBBUS_INTERRUPTS
extern volatile uint8_t subbus_intr_req;
void init_interrupts(void);
int intr_attach(int id, uint16_t addr);
int intr_detach( uint16_t addr );
void intr_service(void);
#endif

/* **************************************************************************************
 *Board Info Driver
 */
static subbus_cache_word_t sb_base_cache[SUBBUS_INSTID_ADDR+1] = {
  { 0, 0, 0, 0, 0, 0, 0 },                          // Reserved zero address
  { 0, 0, 0, 0, 0, 0, 0} ,                          // INTA
  { SUBBUS_BOARD_ID, 0, 1, 0, 0, 0, 0 },            // Board ID               (SUBBUS_BDID_ADDR)
  { SUBBUS_BOARD_BUILD_NUM, 0, 1, 0, 0, 0, 0 },     // Software Build number  (SUBBUS_BLDNO_ADDR)
  { SUBBUS_BOARD_SN, 0, 1, 0, 0, 0, 0 },            // Board    Serial number (SUBBUS_BDSN_ADDR)
  { SUBBUS_BOARD_INSTRUMENT_ID, 0, 1, 0, 0, 0, 0 }  // Instrument ID          (SUBBUS_INSTID_ADDR)
};

subbus_driver_t sb_base = { 
	0, SUBBUS_INSTID_ADDR, 
	sb_base_cache, 
	0,                  // no reset  function
	0,					// no poll   function
	0,					// no action function
	false
};

/* ***********************************************************************
 * If a value has been written by the Host to the specified address since
 * the last call to this function, the new Host written value is updated at
 * the value address.
 *
 * @param addr The cache address
 * @param value Pointer where value may be written
 * @return true if a value has been written to this address.
 */
bool subbus_cache_iswritten(subbus_driver_t *drv, uint16_t addr, uint16_t *value) {
  if (addr >= drv->low && addr <= drv->high) {
    subbus_cache_word_t *word = &drv->cache[addr-drv->low];
    if (word->writable && word->written) {
      *value = word->wvalue;
      word->written = false;
      return true;
    }
  }
  return false;
}

/* ********************************************************************
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
bool subbus_cache_update(subbus_driver_t *drv, uint16_t addr, uint16_t data) {
  if (addr >= drv->low && addr <= drv->high) {
    subbus_cache_word_t *word = &drv->cache[addr-drv->low];
    if (word->readable) {
      word->cache = data;
      word->was_read = false;
      return true;
    }
  }
  return false;
}

bool subbus_cache_was_read(subbus_driver_t *drv, uint16_t addr) {
  if (addr >= drv->low && addr <= drv->high) {
    subbus_cache_word_t *word = &drv->cache[addr-drv->low];
    return word->was_read;
  }
  return false;
}

// *******************************************************************
// Board Description subbus Driver
//
static subbus_cache_word_t board_desc_cache[2] = {
  { 0, 0, true, false, false, false, false },
  { 0, 0, true, false, false, false, true }
};

static struct board_desc_t {
  const char *desc;
  int cp;
  int nc;
} board_desc;

static void board_desc_init(void) {
  board_desc.desc = SUBBUS_BOARD_REV;
  board_desc.cp = 0;
  board_desc.nc = strlen(board_desc.desc)+1; // Include the trailing NUL
  subbus_cache_update(&sb_board_desc, SUBBUS_DESC_FIFO_SIZE_ADDR, (board_desc.nc+1)/2);
  subbus_cache_update(&sb_board_desc, SUBBUS_DESC_FIFO_ADDR,
  (board_desc.desc[0] & 0xFF) + (board_desc.desc[1]<<8));
}

static void board_desc_action(uint16_t addr) {
  if (board_desc_cache[1].was_read) {
    board_desc.cp += 2;
    if (board_desc.cp >= board_desc.nc) {
      board_desc.cp = 0;
    }
  }
  subbus_cache_update(&sb_board_desc, SUBBUS_DESC_FIFO_SIZE_ADDR,
  ((board_desc.nc-board_desc.cp)+1)/2);
  subbus_cache_update(&sb_board_desc, SUBBUS_DESC_FIFO_ADDR,
  (board_desc.desc[board_desc.cp] & 0xFF) + (board_desc.desc[board_desc.cp+1]<<8));
}

subbus_driver_t sb_board_desc = {
  SUBBUS_DESC_FIFO_SIZE_ADDR, SUBBUS_DESC_FIFO_ADDR,
  board_desc_cache,         // Host accessible cache
  board_desc_init,          // reset function
  0,						// no polling function
  board_desc_action,        // action function
  false 
};
