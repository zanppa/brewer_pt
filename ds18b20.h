#ifndef __DS18B20_H__
#define __DS18B20_H__


// Library to interface with DS18B20 temperature sensor using 1-wire
// communication protocol.
// DS18B20 returns the temperature in celsius

// This version only works as single-drop, i.e. only one sensor on the bus.

// With external 5k pull-up on data signal and third wire for powering.
// Normally an open-drain output is used, but on some commands with power parameter,
// push-pull mode is activated, which is same as strong pull-up for parasitic powering
// of the IC.

// Rough calculations of timing
// Reset 1000 us, before every command
// Write 85 us / bit, ~700 us / byte
// Read 75 us / bit, ~600 us / byte
// Typical sequence is: reset, skip rom, write command = ~2500 us
// or reset, skip rom, write command, read byte = ~3000 us
// So for 1 ms system tick, need another method to handle this with ~10us accuracy...

#define DS_TIME_INTERVAL			20000			// ms, read every 20 seconds

// Reset timing
#define DS_RESET_PULSE				500				// us, reset pulse min duration (> 480 us)
#define DS_RESET_DELAY				65				// us, max time after pull-up that device must have responded (15 us then 60 us min)
#define DS_RESET_WAIT				430				// us, min wait time for device response after reset (> 480 us - reset_delay)

// Write timing
#define DS_WRITE_1					5				// us, write 1 pulse duration (< 15 us)
#define DS_WRITE_1_WAIT				75				// us, wait after write 1 (total pulse 60 ... 120 us)
#define DS_WRITE_0					70				// us, write 0 pulse duration (> 60 us)
#define DS_WRITE_0_WAIT				10				// us, wait after write 0 (total pulse 60 .... 120 us);

// Read timing
#define DS_READ_PULSE				3				// us, pull-down time to initate read cycle (> 1 us)
#define DS_READ_DELAY				10				// us, sample after low pulse (within 15 us from falling edge)
#define DS_READ_WAIT				53				// us, wait time after sampling. Total read cycle > 60 us

// Flags
#define DS_DATA_VALID				0x01
#define DS_NEW_DATA					0x02
#define DS_DEVICE_FOUND				0x04
#define DS_CONVERSION_DONE			0x08


// Struct containing the ROM code of 1-wire devices
typedef union _dsROMCode {
	uint8_t raw[8];
	struct __attribute__((__packed__)) _dsROMCodeData {
		uint8_t CRC;
		uint8_t serno[6];
		uint8_t family;								// Should be 28h for DS18B20
	} d;
} dsROMCode;

// Struct containing the data from the DS18B20
typedef union _dsScratchpad {
	uint8_t raw[9];
	struct __attribute__((__packed__)) _dsScratchpadData {
		uint8_t temperature[2];
		uint8_t Th;									// High temperature alarm limit
		uint8_t Tl;									// Low temperature alarm limit
		uint8_t conf;
		uint8_t reserved[3];
		uint8_t crc;
	} d;
} dsScratchpad;

// Setup the pins and ports
void dsSetup(void);

// Common 1-wire commands

// Perform reset and wait for acknowledge from slave(s)
// Returns 1 on success and 0 if nothing was found on the bus
uint8_t dsReset(void);

// Search ROM command F0h
// TODO: Implement...
uint8_t dsSearchRom(void);

// Read ROM Command 33h
// Can be used when only 1 slave on the bus
// Parameter must be a 64-bit buffer to store the ROM code
// Return 1 on success, 0 on error
uint8_t dsReadRom(dsROMCode *pROMCode);

// Match ROM Command 55h
// Address a specific slave on the bus
// Parameter must be a 64-bit buffer containing the target ROM code
// Return 1 on success, 0 on error
uint8_t dsMatchRom(dsROMCode *pROMCode);

// Skip ROM command CCh
// Can be used to address ALL slaves on the bus, e.g. trigger a conversion
// Only on single-drop systems may a read command follow this command
void dsSkipRom(void);

// Alarm Search ECh
// Same as search ROM but only devices with alarm flag set will respond
// Returns 1 on success, 0 on error
uint8_t dsAlarmSearch(dsROMCode *pROMCode);


// DS18B20 specific commands

// Convert T 44h
// Initiate a temperature conversion
// Set power = 1 if parasitic powering is used, otherwise data pin will be left pulled-up (TODO: not done)
// Set power = 2 to wait until conversion is done (TODO: Not done)
// TODO: Blocking until done if power=1?
// Returns 0 on error (no device on bus), otherwise nonzero
uint8_t dsConvertT(uint8_t power);

// Check if conversion is done; dsConvertT must be called before this, otherwise behaviour is undefined
// returns 0xFF when conversion is done, 0 otherwise
uint8_t dsConversionDone(void);

// Write scratchpad 4Eh
// Write Th and Tl and conf registers to the chip
void dsWrite(uint8_t Th, uint8_t Tl, uint8_t conf);

// Read scratchpad BEh
// Read contents of the memory
// Parameter must be a 9-byte buffer containing following data in byte-order:
// 0 - Temperature LSB, MSB, Th, Tl, conf, res, res, res, CRC
// Returns 1 on success and 0 on error
uint8_t dsRead(dsScratchpad *pData);

// Copy scratchpad 48h
// Copies Th, Tl and conf registers to eeprom
// power = 1 keeps output high afterwards for parasitic powered devices.
// TODO: Blocking for 10 ms until copy is done if power=1?
void dsCopy(uint8_t power);

// Recall EEPROM B8h
// Copy Th, Tl and conf from EEPROM to scratchpad
void dsRecall(void);


// Read power supply B4h
// Check whether chip is powered parasitically or not
// Returns 1 if power supply is used, 2 if parasitically and 0 if no devices on the bus
uint8_t dsReadPower(void);

// Other functions
uint8_t dsCalculateCRC(void);

// Main loop
PT_THREAD(dsLoop(struct pt *pt));

// Returns 1 if data is valid (i.e. conversion not running)
uint8_t dsDataValid();
// Returns 1 if new data since last reset (value reset)
uint8_t dsNewData();
// Reset new data flag
void dsResetNewData();
// Return the latest conversion result in fixed point 12.4 (i.e. divide by 16.0 to get temp in Celsius)
uint16_t dsGetLastValue();
// Return address of the scratcpad
dsScratchpad * dsGetScratchpad();

#endif
