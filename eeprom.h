#ifndef __EEPROM_H__
#define __EEPROM_H__

// Library to handle data storage to EEPROM inside the TM4c123GH6PMI chip

// Structure to contain the configuration
// size is 4, MUST BE MULTIPLE OF 4
#define EEPROM_ECONFIG_SIZE 4
typedef struct __attribute__((__packed__)) _eConfig {
	uint8_t bubbleLevel;					// Threshold for bubbling sensor. NOTE! If this is 255, then the config was not stored previously!
	uint8_t storeInterval;					// Interval to save to eeprom in minutes
	uint8_t flags;							// Print flags
	uint8_t reserved;
} eConfig;

// Structure that contains the data to be stored
// Size is 16 bytes, MUST BE MULTIPLE OF 4
// Packing should be done properly if possible, to have everything lay out nicely on 4 byte boundaries...
// N is written last, so that if write is interrupted, the segment will be reused next time
#define EEPROM_EDATA_SIZE 16
typedef struct __attribute__((__packed__)) _eData {
	int32_t weight;							// Weight in ADC units
	uint16_t temperature;					// External temperature
	uint16_t ethanol;						// Ethanol sensor reading
	uint32_t bubble;						// Bubbling sensor integral
	uint16_t co2;							// Co2 volume sensor integral (ie. times flushed)
	uint8_t reserved;						// Spare
	uint8_t n;								// Number of the data segment that was stored, max is 254, since 255 denotes empty EEPROM
} eData;


#define EEPROM_SIZE				2048		// Eeprom size in bytes
#define EEPROM_CONF_LOC			0			// Configuration struct location in bytes, multiple of 4!
#define EEPROM_DATA_LOC			(EEPROM_CONF_LOC + EEPROM_ECONFIG_SIZE)							// Start of data area in bytes, mutiple of 4!
#define EEPROM_DATA_BLOCKS		((EEPROM_SIZE - EEPROM_DATA_LOC) / EEPROM_EDATA_SIZE)			// (eeprom_size - config_size) / data_block_size
#define EEPROM_DATA_END			( EEPROM_DATA_LOC + EEPROM_DATA_BLOCKS * EEPROM_EDATA_SIZE)		// Address where data storage ends

#define EEPROM_MAX_N			0xFE		// Largest segment number to store before rollover 


// Verify that addresses are on correct boundaries (i.e. 4 bytes)
#if (EEPROM_CONF_LOC % 4) != 0
#error "EEPROM: Configuration word address not divisible by 4"
#endif
#if (EEPROM_DATA_LOC % 4) != 0
#error "EEPROM: Data start address not divisible by 4"
#endif
#if (EEPROM_CONF_LOC + EEPROM_ECONFIG_SIZE + EEPROM_DATA_SIZE*EEPROM_DATA_BLOCKS) > EEPROM_SIZE
#error "EEPROM: Total storage exceeds EEPROM size"
#endif

// Initialize EEPROM and recover from failures
// Return 0 if everything ok, nonzero if there is a failure
uint8_t eInit(void);

// Read configuration
// Return 0 on success
uint8_t eReadConfig(eConfig *config);

// Write configuration to eeprom
// Returns 0 on success
uint8_t eWriteConfig(eConfig *config);

// Write next data
// Returns 0 on ok, otherwise error
uint8_t eWriteData(eData *data);

// Read data from index "addr"
uint8_t eReadData(eData *data, uint8_t addr);

// Read byte from EEPROM, returns 0 if ok, 1 if eeprom is not ready, 2 if outside bounds, 3 if address is not divisible by 4
// Data must be at least 4 byte array, addr must be multiple of 4
uint8_t eDumpData(uint8_t *data, uint16_t addr);

// Reset EEPROM to factory defaults
void eReset(void);

// Get number of data blocks
uint8_t eGetNumBlocks(void);

// Get number of next data block to be stored
uint8_t eGetNextNum(void);

uint8_t eIsOK(void);
uint32_t eGetSize(void);
uint32_t eGetBlocks(void);


#endif