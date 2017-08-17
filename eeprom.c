/**
 * EEPROM Driver for TI Tiva TM4C123 microcontrollers
 *
 * Used with brewing monitor system
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
#include "driverlib/eeprom.h"

#include "common.h"
#include "eeprom.h"

static uint8_t eOK = 0;							// EEPROM ok? 1= ok, 0 = fail
static uint32_t eSize;								// EEPROM Size
static uint32_t eBlocks;							// EEPROM Block count;

static uint8_t nextBlock = EEPROM_DATA_LOC;		// Address of next data block
static uint8_t nextNum = 0;

//eConfig eepromConfig;

/**
 * Find next block in EEPROM to write to after system boot
 *
 * Goes through data stored in EEPROM and figures out which block is next
 * free one, or oldest one to be overwritten.
 *
 * Case 1: Find the largest index, free is the next one, EXCEPT when
 * Case 2: largest is 254 and next is 0, which means overflow, find the largest continuous one, free is then next one
 * Case 3: IF item is 255, that is empty block (eeprom reset value)
 */
void _eFindNextBlock(void)
{
	uint8_t currentNum = 0;
	uint8_t prevNum = currentNum;
	uint32_t addr = EEPROM_DATA_LOC;
	eData block;
	uint8_t i;

	for(i=0; i < EEPROM_DATA_BLOCKS; i++)
	{
		EEPROMRead(&block, addr, sizeof(eData));
		currentNum = block.n;
		if(i == 0) prevNum = currentNum;

/*		while(!UARTSendInt(i));
		while(!UARTSend("\r\n", 2));
		while(!UARTSendInt(currentNum));
		while(!UARTSend("\r\n", 2));
		while(!UARTSendInt(prevNum));
		while(!UARTSend("\r\n", 2));
		while(!UARTSend("\r\n", 2));
*/
		if(currentNum == 0xFF)								// Empty, case 3
		{
			nextBlock = i;
			if(i==0) nextNum = 0;							// Case where 1st is unused, starting from scratch
			else nextNum = (prevNum + 1) % EEPROM_MAX_N;
			break;
		}
		else if((prevNum > currentNum) && currentNum != 0)	// Largest number, case 1
		{
			nextBlock = i;
			nextNum = (prevNum + 1) % EEPROM_MAX_N;
			break;
		}
		else if((prevNum + 1) < currentNum)					// Discontinuous, case 2
		{
			nextBlock = i;
			nextNum = prevNum + 1;
			break;
		}

		prevNum = currentNum;
		addr += EEPROM_EDATA_SIZE;							// Next block
	}

	// All is fine, start from zero again
	if(i == EEPROM_DATA_BLOCKS) {
		nextBlock = 0;
		nextNum = (currentNum + 1) % EEPROM_MAX_N;
	}
/*	  while(!UARTSend("\r\n", 2));
	while(!UARTSendInt(nextBlock));
	while(!UARTSend("\r\n", 2));
	while(!UARTSendInt(nextNum));
	while(!UARTSend("---\r\n", 5));
	while(!UARTSend("\r\n", 2));
*/
}

/**
 * Initialize EEPROM peripheral
 * Also searches for next block number to write to, if initialization was OK.
 */
uint8_t eInit(void)
{
	uint8_t i;
	uint32_t ret;

	if(!SysCtlPeripheralReady(SYSCTL_PERIPH_EEPROM0))
	{
		SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
		while(!SysCtlPeripheralReady(SYSCTL_PERIPH_EEPROM0));
	}

	// Try to init max 200 times
	for(i=0; i<200; i++) {
		ret = EEPROMInit();
		if(ret == EEPROM_INIT_OK)
			break;
		else
			SysCtlDelay(SysCtlClockGet()/100);			// Wait a while...
	}

	// If eeprom init failed every time...
	if(ret != EEPROM_INIT_OK) {
		eOK = 0;
		return 1;  // Fail
	}

	eOK = 1;
	eSize = EEPROMSizeGet();
	eBlocks = EEPROMBlockCountGet();

	// Find address of next free block, try not to overwrite old ones if possible after reset
	_eFindNextBlock();

	return 0;
}

/**
 * Read system config from EEPROM
 */
uint8_t eReadConfig(eConfig *conf)
{
	if(!eOK) return 1;									// Eeprom not initialized
	EEPROMRead(conf, EEPROM_CONF_LOC, sizeof(eConfig));
	return 0;
}

/**
 * Write system config to EEPROM
 */
uint8_t eWriteConfig(eConfig *conf)
{
	if(!eOK) return 1;									// Eeprom not initialized
	return EEPROMProgram(conf, EEPROM_CONF_LOC, sizeof(eConfig));
}

/**
 * Write data to next address
 * 
 * Increments data block number and address automatically
 */
uint8_t eWriteData(eData *data)
{
	uint32_t ret;
	if(!eOK) return 1;									// Eeprom not initialized

	data->n = nextNum;
	ret = EEPROMProgram(data, EEPROM_DATA_LOC + nextBlock * EEPROM_EDATA_SIZE, sizeof(eData));

	nextNum = (nextNum + 1) % EEPROM_MAX_N;
	nextBlock++;
	if(nextBlock >= EEPROM_DATA_BLOCKS) nextBlock = 0;		// Roll over

	return ret;
}

/**
 * Read data block from EEPROM
 *
 * Address is data block number, the real EEPROM address is calculated
 */
uint8_t eReadData(eData *data, uint8_t addr)
{
	if(!eOK) return 1;  // Eeprom not initialized
	if(addr >= EEPROM_DATA_BLOCKS) return 2; // Read out of bounds
	EEPROMRead(data, EEPROM_DATA_LOC + addr * EEPROM_EDATA_SIZE, sizeof(eData));
	return 0;
}

/**
 * Dump byte from EEPROM at real address
 *
 * Data must be a pointer to at least 4 bytes
 */
uint8_t eDumpData(uint8_t *data, uint16_t addr)
{
	if(!eOK) return 1;
	if(addr >= eSize) return 2;
	if(addr & 0x03) return 3;
	EEPROMRead(data, addr, 4);
	return 0;
}

/**
 * Reset (clear) the EEPROM to factory default
 */
void eReset(void)
{
	if(!eOK) return;
	EEPROMMassErase();
}

/**
 * Get number of data blocks that fit to EEPROM
 */
uint8_t eGetNumBlocks(void)
{
	return EEPROM_DATA_BLOCKS;
}

/**
 * Get the number of next data block to be stored
 */
uint8_t eGetNextNum(void)
{
	return nextNum;
}

/**
 * Return EEPROM initialization status
 */
uint8_t eIsOK(void)
{
	return eOK;
}

/**
 * Get EEPROM size in bytes
 */
uint32_t eGetSize(void)
{
	return eSize;
}

/**
 * Get EEPROM hardware block count, should be size/4
 */
uint32_t eGetBlocks(void)
{
	return eBlocks;
}
