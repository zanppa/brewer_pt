/**
 * Driver for Dallas/Maxim DS18B20 One-Wire temperature sensor(s)
 *
 * To be used TI Tiva TM4C123 microcontrollers
 *
 * This driver uses protothreads (by Adam Dunkels, http://dunkels.com/adam/pt/)
 * Some parts use timed functions for Tiva by me.
 *
 * Currently this driver supports only externally powered devices, and 
 * only one sensor on the bus, i.e. no search is done and skip rom
 * command is used a lot.
 *
 * Copyright (C) 2016 Lauri Peltonen
 */
#include <stdint.h>
typedef uint8_t bool;

#include "inc/tm4c123gh6pm.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"

#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"

#include "pt.h"

#include "ds18b20.h"
#include "common.h"


// Port & pin mappings
const uint32_t dsPeripheral = SYSCTL_PERIPH_GPIOB;
const uint32_t dsPort = GPIO_PORTB_BASE;
const uint32_t dsPin = GPIO_PIN_2;					// PB2

// Variables
//uint8_t dsTimedFound = 0;							// Device found in bus, used in timed function
//volatile uint8_t dsConvDoneTimed = 0;				// Conversion done flag, used in timed function
static uint8_t dsFlags = 0;
static dsScratchpad dsData;

// Timer
uint32_t *dsTimer;


/**
 * Setup peripherals needed for DS18B20
 *
 * I.e. pins and ports, all communications are done on software
 */
void dsSetup(void)
{
	bool bInt;

	// Enable output GPIO peripheral if not yet enabled
	if(!SysCtlPeripheralReady(dsPeripheral))
	{
		SysCtlPeripheralEnable(dsPeripheral);
		while(!SysCtlPeripheralReady(dsPeripheral));
	}

	// Initialize pin types and set values
	bInt = IntMasterDisable();
	GPIOPinTypeGPIOOutputOD(dsPort, dsPin);
	GPIOPinWrite(dsPort, dsPin, dsPin);				// Pin = 1 => ext pull-up
	if(bInt) IntMasterEnable();

	dsTimer = getFreeTimer();
	if(dsTimer)
		*dsTimer = DS_TIME_INTERVAL;
}

/**
 * Write bit to bus
 *
 * Data can be zero for 0 or nonzero for 1
 * This function blocks until write is done.
 */
inline void _dsWriteBit(uint8_t data)
{
	//bool bInt;

	//bInt = IntMasterDisable();
	GPIOPinWrite(dsPort, dsPin, 0);					// Data low
	if(data) {
		delayMicrosec(DS_WRITE_1);
		GPIOPinWrite(dsPort, dsPin, dsPin);			// Back up
		delayMicrosec(DS_WRITE_1_WAIT);
	} else {
		delayMicrosec(DS_WRITE_0);
		GPIOPinWrite(dsPort, dsPin, dsPin);			// Back up
		delayMicrosec(DS_WRITE_0_WAIT);
	}
	//if(bInt) IntMasterEnable();
}

/**
 * Read a bit from bus
 *
 * Return 0xFF (all bits on) for 1 or 0x00 for 0
 * This function blocks until read is done.
 */
inline uint8_t _dsReadBit()
{
	//bool bInt;
	uint8_t data;

	//bInt = IntMasterDisable();

	GPIOPinWrite(dsPort, dsPin, 0);					// Data low
	delayMicrosec(DS_READ_PULSE);

	GPIOPinWrite(dsPort, dsPin, dsPin);				// Back up
	GPIOPinTypeGPIOInput(dsPort, dsPin);			// Change to input and after wait read status
	delayMicrosec(DS_READ_DELAY);

	data = GPIOPinRead(dsPort, dsPin);
	GPIOPinTypeGPIOOutputOD(dsPort, dsPin);			// Re-configure as output for next cycle
	delayMicrosec(DS_READ_WAIT);

	//if(bInt) IntMasterEnable();

	return data ? 0xFF : 0;
}

/**
 * Write byte (8 bits) to bus, blocking
 */
void _dsWriteByte(uint8_t data)
{
	uint8_t i = 0b00000001;							// LSB first

	// Loop until i overflows after MSB
	while(i) {
		_dsWriteBit(data & i);
		i <<= 1;
	}
}

/**
 * Read a byte (8 bits) from the bus, blocking
 */
uint8_t _dsReadByte(void)
{
	uint8_t i = 0b00000001;							// Read LSB first
	uint8_t data = 0;

	// Loop until overflow after MSB
	while(i) {
		data |= _dsReadBit() & i;
		i <<= 1;
	}

	return data;
}

/**
 * Send a reset command to bus, blocking
 */
uint8_t dsReset(void)
{
	//bool bInt;
	uint8_t found = 0;

	//bInt = IntMasterDisable();

	GPIOPinWrite(dsPort, dsPin, 0);					// Pull data low
	delayMicrosec(DS_RESET_PULSE);

	GPIOPinWrite(dsPort, dsPin, dsPin);				// Let float high
	GPIOPinTypeGPIOInput(dsPort, dsPin);			// Change to input and after wait read status
	delayMicrosec(DS_RESET_DELAY);
	found = GPIOPinRead(dsPort, dsPin);

	delayMicrosec(DS_RESET_WAIT);

	GPIOPinTypeGPIOOutputOD(dsPort, dsPin);
	//delayMicrosec(DS_RESET_DELAY2);

	//if(bInt) IntMasterEnable();

	return found ? 0 : 1;
}

/**
 * Rom search command, not implemented
 */
uint8_t dsSearchRom(void)
{
	return 0;
}

/**
 * Read ROM code from device on the bus, blocking
 */
uint8_t dsReadRom(dsROMCode *pROMCode)
{
	uint8_t i;

	if(!pROMCode) return 0;
	if(!dsReset()) return 0;						// No device on the bus
	dsSkipRom();
	_dsWriteByte(0x33);

	// Read 8 bytes
	for(i=0;i<8;i++)
		pROMCode->raw[i] = _dsReadByte();

	return 1;
}

/**
 * Send a Match ROM command, blocking
 */
uint8_t dsMatchRom(dsROMCode *pROMCode)
{
	uint8_t i;

	if(!pROMCode) return 0;

	_dsWriteByte(0x55);
	for(i=0;i<8;i++)								// TODO: High byte or low byte first?!
		_dsWriteByte(pROMCode->raw[i]);

	return 1;
}

/**
 * Send Skip ROM command, blocking
 */
void dsSkipRom(void)
{
	_dsWriteByte(0xCC);
}

/**
 * Do an alarm search, not implemented
 */
uint8_t dsAlarmSearch(dsROMCode *pROMCode)
{
	return 0;
}

/**
 * Start temperature conversion, blocking
 */
uint8_t dsConvertT(uint8_t power)
{
	if(!dsReset()) return;								// No device on the bus
	dsSkipRom();
	_dsWriteByte(0x44);

	return 1;
}

/**
 * Check whether temperature conversion is done
 * Only works with externally powered chips
 */
uint8_t dsConversionDone(void)
{
	return _dsReadBit();
}

/**
 * Write temperature limits and config to device
 * Skips ROM, blocking
 */
void dsWrite(uint8_t Th, uint8_t Tl, uint8_t conf)
{
	if(!dsReset()) return;								// No device on the bus
	dsSkipRom();
	_dsWriteByte(0x4E);
	_dsWriteByte(Th);
	_dsWriteByte(Tl);
	_dsWriteByte(conf);
}

/**
 * Read the scrathpad memory from the chip
 * Skips ROM, blocking
 */
uint8_t dsRead(dsScratchpad *pData)
{
	uint8_t i;

	if(!pData) return 0;								// NULL pointer exception
	if(!dsReset()) return 0;							// No device on the bus
	dsSkipRom();
	_dsWriteByte(0xBE);

	// Read 9 bytes
	for(i=0;i<9;i++) {
		pData->raw[i] = _dsReadByte();
	}
	return 1;
}

void dsCopy(uint8_t power)
{
	if(!dsReset()) return;							// No device on the bus
	dsSkipRom();
	_dsWriteByte(0x48);
}

void dsRecall(void)
{
	if(!dsReset()) return;							// No device on the bus
	dsSkipRom();
	_dsWriteByte(0x48);
}

uint8_t dsReadPower(void)
{
	if(!dsReset()) return 0;						// No device on the bus
	dsSkipRom();
	_dsWriteByte(0xB4);

	return _dsReadBit() ? 1 : 2;
}

/**
 * Reset the one-wire bus
 * This is a non-blocking (timed) function
 */
TIMED_FUNCTION(dsResetTimed)
{
	TIMED_BEGIN();

	// Wait until timer is free, then lock it
	LOCK_TIMER();

	dsFlags &= ~DS_DEVICE_FOUND;

	GPIOPinWrite(dsPort, dsPin, 0);							// Pull data low
	TIMER_WAIT(dsResetTimed, DS_RESET_PULSE);				// Wait here

	GPIOPinWrite(dsPort, dsPin, dsPin);						// Let float high
	GPIOPinTypeGPIOInput(dsPort, dsPin);					// Change to input and after wait read status
	TIMER_WAIT(dsResetTimed, DS_RESET_DELAY);

	if(!GPIOPinRead(dsPort, dsPin))							// Slave pulls down if present, thus inverted!
		dsFlags |= DS_DEVICE_FOUND;

	TIMER_WAIT(dsResetTimed, DS_RESET_WAIT);

	GPIOPinTypeGPIOOutputOD(dsPort, dsPin);
	//TIMER_WAIT(dsResetTimed, DS_RESET_DELAY2);

	RELEASE_TIMER();

	TIMED_END();
}

/**
 * Start temperature conversion
 * Skips ROM
 * This is a non-blocking (timed) function
 */
TIMED_FUNCTION(dsConvertTTimed)
{
	static uint8_t i = 1;
	static uint8_t data = 0x00;

	TIMED_BEGIN();

	LOCK_TIMER();

	// Skip ROM: Write 0xCC (Only 1 device allowed on the bus!)
	i = 0b00000001;											// LSB first
	data = 0xCC;
	while(i) {
		GPIOPinWrite(dsPort, dsPin, 0);						// Data low
		if(data & i) {
			TIMER_WAIT(dsConvertTTimed, DS_WRITE_1);
			GPIOPinWrite(dsPort, dsPin, dsPin);				// Back up
			TIMER_WAIT(dsConvertTTimed, DS_WRITE_1_WAIT);
		} else {
			TIMER_WAIT(dsConvertTTimed, DS_WRITE_0);
			GPIOPinWrite(dsPort, dsPin, dsPin);				// Back up
			TIMER_WAIT(dsConvertTTimed, DS_WRITE_0_WAIT);
		}
		i <<= 1;
	}

	// Convert T command, 0x44
	i = 0b00000001;	 // LSB first
	data = 0x44;
	while(i) {
		GPIOPinWrite(dsPort, dsPin, 0);						// Data low
		if(data & i) {
			TIMER_WAIT(dsConvertTTimed, DS_WRITE_1);
			GPIOPinWrite(dsPort, dsPin, dsPin);				// Back up
			TIMER_WAIT(dsConvertTTimed, DS_WRITE_1_WAIT);
		} else {
			TIMER_WAIT(dsConvertTTimed, DS_WRITE_0);
			GPIOPinWrite(dsPort, dsPin, dsPin);				// Back up
			TIMER_WAIT(dsConvertTTimed, DS_WRITE_0_WAIT);
		}
		i <<= 1;
	}

	dsFlags &= ~DS_CONVERSION_DONE;							// Mark conversion started

	RELEASE_TIMER();

	TIMED_END();
}

/**
 * Check whether the temperature conversion is done
 * Works only with externally powered chips
 * This is a non-blocking (timed) function
 */
TIMED_FUNCTION(dsConversionDoneTimed)
{
	TIMED_BEGIN();

	LOCK_TIMER();

	GPIOPinWrite(dsPort, dsPin, 0);							// Data low
	TIMER_WAIT(dsConversionDoneTimed, DS_READ_PULSE);

	GPIOPinWrite(dsPort, dsPin, dsPin);						// Back up
	GPIOPinTypeGPIOInput(dsPort, dsPin);					// Change to input and after wait read status
	TIMER_WAIT(dsConversionDoneTimed, DS_READ_DELAY);

	if(GPIOPinRead(dsPort, dsPin))							// If read 1 conversion is done!
		dsFlags |= DS_CONVERSION_DONE;

	GPIOPinTypeGPIOOutputOD(dsPort, dsPin);					// Re-configure as output for next cycle
	TIMER_WAIT(dsConversionDoneTimed, DS_READ_WAIT);

	RELEASE_TIMER();

	TIMED_END();
}

/**
 * Read scratchpad memory
 * Skips ROM
 * Sets the data valid flag after read is done
 * This is a non-blocking (timed) function
 */
TIMED_FUNCTION(dsReadTimed)
{
	static uint8_t i = 1;
	static uint8_t j = 0;
	static uint8_t data = 0x00;

	TIMED_BEGIN();

	LOCK_TIMER();

	// Skip ROM: Write 0xCC (Only 1 device allowed on the bus!)
	i = 0b00000001;										// LSB first
	data = 0xCC;
	while(i) {
		GPIOPinWrite(dsPort, dsPin, 0);					// Data low
		if(data & i) {
			TIMER_WAIT(dsReadTimed, DS_WRITE_1);
			GPIOPinWrite(dsPort, dsPin, dsPin);			// Back up
			TIMER_WAIT(dsReadTimed, DS_WRITE_1_WAIT);
		} else {
			TIMER_WAIT(dsReadTimed, DS_WRITE_0);
			GPIOPinWrite(dsPort, dsPin, dsPin);			// Back up
			TIMER_WAIT(dsReadTimed, DS_WRITE_0_WAIT);
		}
		i <<= 1;
	}

	// Read scratchpad command, 0xBE
	i = 0b00000001;										// LSB first
	data = 0xBE;
	while(i) {
		GPIOPinWrite(dsPort, dsPin, 0);					// Data low
		if(data & i) {
			TIMER_WAIT(dsReadTimed, DS_WRITE_1);
			GPIOPinWrite(dsPort, dsPin, dsPin);			// Back up
			TIMER_WAIT(dsReadTimed, DS_WRITE_1_WAIT);
		} else {
			TIMER_WAIT(dsReadTimed, DS_WRITE_0);
			GPIOPinWrite(dsPort, dsPin, dsPin);			// Back up
			TIMER_WAIT(dsReadTimed, DS_WRITE_0_WAIT);
		}
		i <<= 1;
	}

	// Then read 9 bytes
	for(j=0; j<9; j++) {
		data = 0x00;

		i = 0b00000001;
		while(i) {
			GPIOPinWrite(dsPort, dsPin, 0);				// Data low
			TIMER_WAIT(dsReadTimed, DS_READ_PULSE);

			//GPIOPinWrite(dsPort, dsPin, dsPin);		// Back up
			GPIOPinTypeGPIOInput(dsPort, dsPin);		// Change to input and after wait read status
			TIMER_WAIT(dsReadTimed, DS_READ_DELAY);


			if(GPIOPinRead(dsPort, dsPin))
				data |= i;

			TIMER_WAIT(dsReadTimed, DS_READ_WAIT);
			GPIOPinTypeGPIOOutputOD(dsPort, dsPin);		// Re-configure as output for next cycle
			i <<= 1;
		}
		dsData.raw[j] = data;
	}

	// TODO: Add CRC check?

	dsFlags |= DS_DATA_VALID;							// Mark data as valid from now on

	RELEASE_TIMER();

	TIMED_END();
}

static uint8_t dsDataReady = 0;

/**
 * Main driver loop for DS18B20
 *
 * Resets the bus, starts temperature conversion, waits until it
 * is complete and then reads the data
 */
PT_THREAD(dsLoop(struct pt *pt))
{
	static int retries = 0;
	PT_BEGIN(pt);

	while(1) {
		// Convert temperature
		dsFlags &= ~DS_DATA_VALID;

		// Reset and check device on the bus
		PT_WAIT_UNTIL(pt, dsResetTimed(0, CALLER_THREAD) == RETURN_DONE);

		if(dsFlags & DS_DEVICE_FOUND) {	 // Device present on the bus

			// Start conversion
			PT_WAIT_UNTIL(pt, dsConvertTTimed(0, CALLER_THREAD) == RETURN_DONE);

			// Wait until conversion is done (TODO: Add a timeout?)
			retries = 100;  // Check bus at least 100 times, NOTE this method does not work with parasite powered device
			while(retries && !(dsFlags & DS_CONVERSION_DONE)) {
				PT_WAIT_UNTIL(pt, dsConversionDoneTimed(0, CALLER_THREAD) == RETURN_DONE);
				retries--;
			}

			// TODO: Vaihtoehto: Kiinteä odotusaika
			//commonTimer[DS_TIMER] = 1100;  // Wait 1100 ms
			//PT_WAIT_WHILE(pt, commonTimer[DS_TIMER]);

			if(dsFlags & DS_CONVERSION_DONE) {  // Conversion was done before timeout
				// Read cycle: Reset, then read scratchpad
				PT_WAIT_UNTIL(pt, dsResetTimed(0, CALLER_THREAD) == RETURN_DONE);

				PT_WAIT_UNTIL(pt, dsReadTimed(0, CALLER_THREAD) == RETURN_DONE);

				// Reset once again, we're done!
				PT_WAIT_UNTIL(pt, dsResetTimed(0, CALLER_THREAD) == RETURN_DONE);

				// TODO: Add CRC check?

				dsFlags |= DS_NEW_DATA;
			}
		}

		if(dsTimer) {
			*dsTimer = DS_TIME_INTERVAL;
			PT_WAIT_WHILE(pt, *dsTimer);
		}
	}

	PT_END(pt);
}

/**
 * Returns true if conversion is done and data is valid
 */
uint8_t dsDataValid()
{
	return (dsFlags & DS_DATA_VALID) ? 1 : 0;
}

/**
 * Returns true if new data is available
 */
uint8_t dsNewData()
{
	return (dsFlags & DS_NEW_DATA) ? 1 : 0;
}

/**
 * Clear the new data flag
 */
void dsResetNewData()
{
	dsFlags &= ~DS_NEW_DATA;
}

/**
 * Get last conversion value (data)
 */
uint16_t dsGetLastValue()
{
	return (dsData.d.temperature[1] << 8) + dsData.d.temperature[0];
}

/**
 * Get a pointer to the scrathpad memory struct
 */
dsScratchpad *dsGetScratchpad()
{
	return &dsData;
}

