/**
 * Driver for Nordic NRF24L01 radio module.
 * To be used with TI Tiva or Stellaris launchpads at least
 * i.e. TM4C123 or LM4F120 microprocessors.
 *
 * Driver is based on RF24 driver for Arduino, http://www.tinkerer.eu/AVRLib/nRF24L01/
 *
 * Copyright (C) 2016 Lauri Peltonen
 * Copyright (c) 2007 Stefan Engelke <mbox@stefanengelke.de>
 */

/**
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <stdint.h>
typedef uint8_t bool;

// TODO: This is a dirty hack...
#define PART_TM4C123GH6PM

#include "inc/tm4c123gh6pm.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ssi.h"

#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/ssi.h"

#include "common.h"
#include "nrf24l01.h"

// PA2	  = NRF24L01 SPI CLK
// PA3	  = NRF24L01 SPI FSS
// PA4	  = NRF24L01 SPI RX
// PA5	  = NRF24L01 SPI TX
// PB3	  = NRF24L01 CE
// PB4	  = NRF24L01 CSN

// #define RF24DEBUG
#undef RF24DEBUG

const uint32_t RF24_CE_PORT = GPIO_PORTB_BASE;
const uint32_t RF24_CSN_PORT = GPIO_PORTB_BASE;
const uint32_t RF24_CE_PIN = GPIO_PIN_3;
const uint32_t RF24_CSN_PIN = GPIO_PIN_4;

static const uint8_t rf24_max_payload_size = 32;		// Maximum payload length of the chip
uint8_t payload_size = 32;						// Default
static uint64_t pipe0_read_addr = 0;
static uint8_t rf24Flags = 0;

/**
 * SPI settings for NRF24L01 module
 *
 * Send most significant bit first!
 * Send least significant byte first!
 * Clock idles low
 * Data is sampled on the rising edge of the clock
 * -> SSI SPO=0 and SPH=0 -> clock idles low and phase = sample on rising edge -> Mode 0!
 */
 
/**
 * Setup the NRF24L01 driver.
 *
 * Initializes all necessary peripherals and sets the pins
 */
void rf24Setup(void)
{
	// Initialize peripherals
	if(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA))
	{
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
		while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));
	}
	if(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB))
	{
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
		while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));
	}
	if(!SysCtlPeripheralReady(SYSCTL_PERIPH_SSI0))
	{
		SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
		while(!SysCtlPeripheralReady(SYSCTL_PERIPH_SSI0));
	}

	// Pin mappings
	GPIOPinConfigure(GPIO_PA2_SSI0CLK);										// Enable CLK pin
	GPIOPinConfigure(GPIO_PA4_SSI0RX);										// Enable RX pin
	GPIOPinConfigure(GPIO_PA5_SSI0TX);										// Enable TX pin

	// TODO: tähän väliin SSI0 alustus?
	GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_2);							// Enable comm. pins for SSI module
	SSIConfigSetExpClk(SSI0_BASE, 80000000, SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 10000, 8);		// 1 MHz clock speed, 8 bits interface

	// TODO: Change to 2 lines to have both ports
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, RF24_CSN_PIN | RF24_CE_PIN);		// CE & CSN
	GPIOPinWrite(RF24_CE_PORT, RF24_CE_PIN, 0);								// CE low
	GPIOPinWrite(RF24_CSN_PORT, RF24_CSN_PIN, RF24_CSN_PIN);				// CSN Hi (not active)

	SSIEnable(SSI0_BASE);													// Enable SSI0
}

/**
 * Set CE (chip enable) high or low to activate/deactive radio communication
 *
 * Set CE to high state to activate radio of the module, i.e. to send or receive data
 * 1 = CE high, radio enabled
 * 0 = CE low, radio disabled
 */
inline void rf24ce(uint8_t level)
{
	GPIOPinWrite(RF24_CE_PORT, RF24_CE_PIN, level ? RF24_CE_PIN : 0);
}

/**
 * Set CSN (SPI Chip select) high or low to select the module for SPI communication
 *
 * Set CSN to low to activate the SPI communication on the radio module
 * 1 = CSN high, SPI disabled
 * 0 = CSN low, SPI enabled
 */
inline void rf24csn(uint8_t level)
{
	GPIOPinWrite(RF24_CSN_PORT, RF24_CSN_PIN, level ? RF24_CSN_PIN : 0);
}

/**
 * Read a register from the NRF24L01 memory
 *
 * See NRF24L01 datasheet for list of registers and their addresses.
 * Returns the register value
 */
uint8_t rf24ReadRegister(uint8_t reg)
{
	uint32_t data;

	rf24csn(0);
	SSIDataPut(SSI0_BASE, R_REGISTER | (REGISTER_MASK & reg));
	SSIDataPut(SSI0_BASE, 0xFF);				// Dummy send to get the value
	while(SSIBusy(SSI0_BASE));					// Keep CSN down until everything is written!
	rf24csn(1);

	// Buffered values read below
	SSIDataGet(SSI0_BASE, &data);				// Dummy read (status) from first transmission
	SSIDataGet(SSI0_BASE, &data);				// Real register data

	return data & 0xFF;  // Only lowest 8 bits
}

/**
 * Write value into NRF24L01 register
 *
 * See NRF24L01 datasheet for list of registers
 * Returns module status (STATUS register 0x07)
 */
uint8_t rf24WriteRegister(uint8_t reg, uint8_t value)
{
	uint32_t stat, dummy;

	rf24csn(0);
	SSIDataPut(SSI0_BASE, W_REGISTER | (REGISTER_MASK & reg));
	SSIDataPut(SSI0_BASE, value);
	while(SSIBusy(SSI0_BASE));							// Keep CSN down until everything is written!
	rf24csn(1);
	SSIDataGet(SSI0_BASE, &stat);
	SSIDataGet(SSI0_BASE, &dummy);

	return stat & 0xFF;
}

/**
 * Write more than one byte of data into register
 *
 * See NRF24L01 datasheet for list of registers
 * Returns module status (STATUS register 0x07)
 */
uint8_t rf24WriteLongRegister(uint8_t reg, uint8_t *buf, uint8_t len)
{
	uint32_t stat, dummy;
	rf24csn(0);
	SSIDataPut(SSI0_BASE, W_REGISTER | (REGISTER_MASK & reg));
	SSIDataGet(SSI0_BASE, &stat);

	while(len--) {
		SSIDataPut(SSI0_BASE, *buf++);
		SSIDataGet(SSI0_BASE, &dummy);						// Flush RX buffer
	}

	while(SSIBusy(SSI0_BASE));								// Keep CSN down until everything is written!
	rf24csn(1);

	return stat & 0xFF;
}

/**
 * Set the radio channel (frequency) to be used for communication
 *
 * Radio frequency is 2400 MHz + channel [MHz]
 */
void rf24SetChannel(uint8_t channel)
{
	if(channel > 127) channel = 127;						// TODO: Max channel number hardcoded
	rf24WriteRegister(RF_CH, channel);
}

/**
 * Set the transmitter power level
 *
 * 0 = Minimum level -18 dBm
 * 1 = -12 dBm
 * 2 = -6 dBm
 * 3 = Maximum 0 dBm
 */
void rf24SetPALevel(uint8_t level)
{
	uint8_t conf;

	conf = rf24ReadRegister(RF_SETUP);
	conf &= ~((1 << RF_PWR_LOW) | (1 << RF_PWR_HIGH));		// Clear RF power bits

	if(level == 0) {	 // Minimum level
		// No bits set
	} else if(level == 1) {
		conf |= (1 << RF_PWR_LOW);
	} else if(level == 2) {
		conf |= (1 << RF_PWR_HIGH);
	} else {			 // Maximum level
		conf |= (1 << RF_PWR_LOW) || (1 << RF_PWR_HIGH);
	}
	rf24WriteRegister(RF_SETUP, conf);
}

/**
 * Set communication data rate
 * 
 * Data rate given in 100 kBps
 * 0...9   = 250 kBps
 * 10...19 = 1 MBps
 * 20...   = 2 MBps if supported
 */ 
void rf24SetDataRate(uint8_t rate)
{
	uint8_t conf;

	conf = rf24ReadRegister(RF_SETUP);
	conf &= ~((1 << RF_DR_LOW) | (1 << RF_DR_HIGH));		// Clear data rate bits

	if(rate < 10) {  // 250 kbps (< 1 MBps)
		conf |= (1 << RF_DR_LOW);
	} else if( rate >= 20) { // 2 MBps
		conf |= (1 << RF_DR_HIGH);
	} else {	// Default to 1 Mbps
		// No bits set
	}
	rf24WriteRegister(RF_SETUP, conf);

  // TODO: Verify?
}

/**
 * Set CRC length or disable CRC checking
 *
 * 0 = CRC check disabled
 * 1 = 1 byte CRC
 * 2 = 2 bytes CRC
 */
void rf24SetCRCLength(uint8_t length)
{
	uint8_t conf;

	conf = rf24ReadRegister(CONFIG);
	conf &= ~((1 << CRCO) | (1 << EN_CRC));				// Clear CRC length bits

	if(length == 0) { // disabled
		// No bits set
	} else if(length == 1) {	// 1 byte
		conf |= (1 << EN_CRC);
	} else { // Default 2 bytes
		conf |= (1 << EN_CRC) | (1 << CRCO);
	}

	rf24WriteRegister(CONFIG, conf);
}

/**
 * Flush the RX (receive) FIFO
 *
 * Returns the STATUS register value
 */
uint8_t rf24FlushRx(void)
{
	uint32_t data;
	rf24csn(0);
	SSIDataPut(SSI0_BASE, FLUSH_RX);
	SSIDataGet(SSI0_BASE, &data);
	while(SSIBusy(SSI0_BASE));							// Keep CSN down until everything is written!
	rf24csn(1);
	return data & 0xFF;
}

/**
 * Flust the TX (transmit) FIFO
 *
 * Returns the STATUS register value
 */
uint8_t rf24FlushTx(void)
{
	uint32_t data;
	rf24csn(0);
	SSIDataPut(SSI0_BASE, FLUSH_TX);
	SSIDataGet(SSI0_BASE, &data);
	while(SSIBusy(SSI0_BASE));							// Keep CSN down until everything is written!
	rf24csn(1);
	return data & 0xFF;
}

/**
 * Initialize the NRF24L01
 *
 * Sets the default parameter values so that radio module is ready for operation
 */
void rf24Init(void)
{
	// Set 1500 us timeout
	rf24WriteRegister(SETUP_RETR, (0b0100 << ARD) | (0b1111 < ARC));

	// Set output power to high level (not max)
	rf24SetPALevel(2);

	rf24SetDataRate(10);								// Set 1 Mbps (10 x 100 kbps)
	rf24SetCRCLength(2);								// 2 byte crc

	rf24SetDynamicPayload(0, 0);							// Dynamic payload off by default

	rf24WriteRegister(STATUS, (1 << RX_DR) | (1 << TX_DS) | (1 << MAX_RT));			// Reset status

	rf24SetChannel(76);									// Default channel

	// Flush
	rf24FlushRx();
	rf24FlushTx();
}

/**
 * Configure a pipe to be used for data receiving (RTX)
 *
 * NRF24L01 configured in receive (RTX) mode can receive from 6 transmitters (PTX)
 * on one frequency. The receivers are separated by their logical address.
 *
 * Pipe is the receiving pipe number (0...5)
 * Address is the logical address of the pipe
 * Note that pipes 1 to 5 share the 5 most significant bytes of the address
 */
void rf24OpenReadingPipe(uint8_t pipe, uint64_t addr)
{
	// If this is pipe 0, cache the address.	This is needed because
	// openWritingPipe() will overwrite the pipe 0 address, so
	// startListening() will have to restore it.
	if(pipe == 0)
		pipe0_read_addr = addr;

	if(pipe > 5) return;	// Not valid pipe address

	// Pipe 0 has unique 5 byte address, while 1...5 share the 4 MSB:s. LSB must be unique for all.
	// The shared bytes are written for pipe 2 only.
	if (pipe < 2)
		rf24WriteLongRegister(RX_ADDR_P0 + pipe, (uint8_t*)(&addr), 5);
	else
		rf24WriteRegister(RX_ADDR_P0 + pipe, addr & 0xFF);	// Only write low byte

	rf24WriteRegister(RX_PW_P0 + pipe, payload_size);

	// Note it would be more efficient to set all of the bits for all open
	// pipes at once.	 However, I thought it would make the calling code
	// more simple to do it this way.
	rf24WriteRegister(EN_RXADDR, rf24ReadRegister(EN_RXADDR) | (1 << (ERX_P0 + pipe)));
}

/**
 * Configure pipe for transmitting data (PTX)
 *
 * NRF24L01 can transmit to only one address at the time
 * This function also configures receive pipe 0 to the same address
 * to allow automatic ACK packet handling from the receiver.
 */
void rf24OpenWritingPipe(uint64_t value)
{
	// Note that AVR 8-bit uC's store this LSB first, and the NRF24L01(+)
	// expects it LSB first too, so we're good.

	rf24WriteLongRegister(RX_ADDR_P0, (uint8_t*)(&value), 5);
	rf24WriteLongRegister(TX_ADDR, (uint8_t*)(&value), 5);

	rf24WriteRegister(RX_PW_P0, payload_size);
}

/**
 * Set the module to receive mode (RTX)
 *
 */
void rf24StartListening(void)
{
	// Turn on the radio and set receive mode
	rf24WriteRegister(CONFIG, rf24ReadRegister(CONFIG) | (1 << PWR_UP) | (1 << PRIM_RX));
	rf24WriteRegister(STATUS, (1 << RX_DR) | (1 << TX_DS) | (1 << MAX_RT));		// Clear received and error flags

	// Restore the pipe0 adddress, if exists
	if (pipe0_read_addr)
		rf24WriteLongRegister(RX_ADDR_P0, (uint8_t*)(&pipe0_read_addr), 5);

	// Flush buffers
	rf24FlushRx();
	rf24FlushTx();

	// Go!
	rf24ce(1);

	// wait for the radio to come up (130us actually only needed)
	delayMicrosec(130);	 // TODO: Change to non blocking?
}

/**
 * Stop receiving data and allow the module to sleep
 *
 */
void rf24StopListening(void)
{
	rf24ce(0);
	rf24FlushTx();
	rf24FlushRx();
}

/**
 * Power down (sleep) the radio part of the module
 *
 */
void rf24PowerDown(void)
{
	rf24WriteRegister(CONFIG, rf24ReadRegister(CONFIG) & ~(1 << PWR_UP));
}

/**
 * Power up the radio part of the module
 *
 */
void rf24PowerUp(void)
{
	// Transmitter power-up
	rf24WriteRegister(CONFIG, (rf24ReadRegister(CONFIG) | (1 << PWR_UP)));
	delayMicrosec(150);
}

/**
 * Write transmit payload to the FIFO
 *
 * Returns the STATUS register value
 */
uint8_t rf24WritePayload(const void *buf, uint8_t len)
{
	uint32_t status, dummy;

	const uint8_t *current = (uint8_t *)buf;
	uint8_t data_len, blank_len;
	
	if(rf24Flags & RF24_FLAG_DYNAMIC) {
		data_len = len < rf24_max_payload_size ? len : rf24_max_payload_size;
		blank_len = 0;
	} else {
		data_len = len < payload_size ? len : payload_size;
		blank_len = payload_size - data_len;
	}

	rf24csn(0);
	SSIDataPut(SSI0_BASE, W_TX_PAYLOAD);
	SSIDataGet(SSI0_BASE, &status);
	while(data_len--) {
		SSIDataPut(SSI0_BASE, *current++);				// Send payload
		SSIDataGet(SSI0_BASE, &dummy);
	}
	while(blank_len--) {
		SSIDataPut(SSI0_BASE, 0x00);					// Send padding to match payload size
		SSIDataGet(SSI0_BASE, &dummy);
	}
	while(SSIBusy(SSI0_BASE));							// Keep CSN down until everything is actually written!
	rf24csn(1);

	// TODO: Is this unnecessary???
	while(SSIDataGetNonBlocking(SSI0_BASE, &dummy));	// Flush, i.e. read until there is no more data

	return status & 0xFF;
}

/**
 * Transmit buffer over air
 *
 * Sends a buffer with defined length over air.
 * If length is longer than defined packet size, or maximum
 * dynamic packet size, it will get truncated.
 */
void rf24Write(const void *buf, uint8_t len)
{
	// Clear the max retries flag (if set, no communication can happen)
	// and set the transimitting mode
	rf24WriteRegister(STATUS, rf24ReadRegister(STATUS) | (1 << MAX_RT) | (1 << TX_DS)/* | (1 << RX_DR)*/);
	rf24WriteRegister(CONFIG, rf24ReadRegister(CONFIG) & ~(1 << PRIM_RX));

	// Send the payload
	rf24WritePayload(buf, len);

	// Start transmitting the data over air (by keeping CE up for > 10 us)
	rf24ce(1);
	delayMicrosec(15);
	rf24ce(0);
}

/**
 * Read the STATUS register value
 *
 */
uint8_t rf24ReadStatus(void)
{
	uint32_t stat;

	rf24csn(0);
	SSIDataPut(SSI0_BASE, NOP);
	while(SSIBusy(SSI0_BASE));	// Keep CSN down until everything is written!
	rf24csn(1);
	SSIDataGet(SSI0_BASE, &stat);

	return stat & 0xFF;
}

/**
 * Check the packet transmission status
 *
 * Check the status of currently ongoing transmission.
 * Returns RF24_TX_OK if transmission was succesfull and ACK packet was received.
 * RF24_TX_FAIL if maximum transmit retries was reached and ACK packet was not
 * received, i.e. the transmission may be lost.
 * RF24_TX_BUSY if the module is still transmitting or re-transmitting the packet.
 *
 * The status flags are cleared when this function returns.
 *
 * NOTE: This does not handle any data received in the ACK packet
 * nor does this report if there was data in the ACK packet.
 * After succsefull transmission (RF24_TX_OK), use rf24Received() or rf24Available()
 * to check whether data was received in the ACK packet.
 */
uint8_t rf24TransmitStatus(void)
{
	uint8_t status;

	status = rf24ReadStatus();

	// Test if data sent properly (and ACK received)
	if(status & (1 << TX_DS)) {
		
		// Check if auto-ack packet was also received
		if(status & (1 << RX_DR))
			rf24Flags |= RF24_FLAG_AUTOACKAVAIL;

		// Clear transmit and receive ready flags
		rf24WriteRegister(STATUS, rf24ReadRegister(STATUS) | (1 << TX_DS) | (1 << RX_DR));
		//rf24FlushTx();
		return RF24_TX_OK;
	}
	
	// Maximum retries reached
	if(status & (1 << MAX_RT)) {
		rf24WriteRegister(STATUS, rf24ReadRegister(STATUS) | (1 << MAX_RT));
		rf24FlushTx();
		return RF24_TX_FAIL;
	}

	return RF24_TX_BUSY;
}


/**
 * Query whether receive interrupt has occurred.
 *
 * If the interrupt has occurred, there should be new data in the receive (RX) FIFO.
 *
 * To query any pipe, set pipe to NULL (0), otherwise a pointer to uint8, which
 * will be filled with the pipe number from which the data was received.
 */
uint8_t rf24Received(uint8_t *pipe)
{
	uint8_t status, result;

	status = rf24ReadStatus();
	result = status & (1 << RX_DR);

	if(result)
	{
		// If the caller wants the pipe number, include that
		if(pipe)
			*pipe = (status >> RX_P_NO) & 0b111;

		// Clear the RX status bit. Done here so that new data is not lost...
		rf24WriteRegister(STATUS, (1 << RX_DR));

		// Clear TX flag if ack payload receipt was sent
		if (status & (1 << TX_DS))
			rf24WriteRegister(STATUS, (1 << TX_DS));
	}

	// Also return true if auto-ack packet was received when transmitting
	if(rf24Flags & RF24_FLAG_AUTOACKAVAIL) {
		result = 1;
		rf24Flags &= ~RF24_FLAG_AUTOACKAVAIL;
	}

	return result;
}

/**
 * Check whether there is data available in the receive (RX) FIFO
 *
 * Returns 1 if the FIFO is not empty, otherwise 0.
 */
uint8_t rf24Available(void)
{
	return (rf24ReadRegister(FIFO_STATUS) & (1 << RX_EMPTY)) ? 0 : 1;
}


/**
 * Read the payload in the latest packet
 *
 * Returns the value of the STATUS register.
 */
uint8_t rf24ReadPayload(void *buf, uint8_t len)
{
	uint32_t status, dummy;
	uint8_t *current = (uint8_t *)(buf);

	uint8_t data_len = (len < payload_size) ? len : payload_size;
	uint8_t blank_len = (rf24Flags & RF24_FLAG_DYNAMIC) ? 0 : (payload_size - data_len);	// TODO: Dynamics

	// TODO: Test if this changes anything...
	while(SSIDataGetNonBlocking(SSI0_BASE, &dummy));	 // Read until FIFO is empty

	rf24csn(0);
	SSIDataPut(SSI0_BASE, R_RX_PAYLOAD);
	SSIDataGet(SSI0_BASE, &status);

	while(data_len--) {
		SSIDataPut(SSI0_BASE, 0xFF);
		SSIDataGet(SSI0_BASE, &dummy);
		*current++ = dummy & 0xFF;
	}
	while(blank_len--) {
		SSIDataPut(SSI0_BASE, 0xFF);
		SSIDataGet(SSI0_BASE, &dummy);
		//while(SSIDataGetNonBlocking(SSI0_BASE, &dummy));			// Read until FIFO is empty
	}
	while(SSIBusy(SSI0_BASE));  // Wait until everything's done
	rf24csn(1);

	return status;
}


/**
 * Read data from the receive (RX) FIFO
 *
 * Length is the maximum length to read.
 *
 * Returns 1 if there is still data on the FIFO after this read,
 * otherwise 0.
 */
uint8_t rf24Read(const void *buf, uint8_t len)
{
	// Check if buffer is empty?
	if(rf24ReadRegister(FIFO_STATUS) & (1 << RX_EMPTY)) return 0;

	// Fetch the payload
	rf24ReadPayload(buf, len);

	// was this the last of the data available?
	return (rf24ReadRegister(FIFO_STATUS) & (1 << RX_EMPTY)) ? 0 : 1;
}


/**
 * Set the payload packet size
 *
 * Returns the actual payload size, in case it gets limited by module capabilities.
 */
uint8_t rf24SetPayloadSize(uint8_t size)
{
	payload_size = (size < rf24_max_payload_size) ? size : rf24_max_payload_size;
	return payload_size;
}

/**
 * Toggle additional features (auto ACK, dynamic package length etc) on or off
 *
 */
uint8_t rf24ToggleFeatures()
{
	uint32_t dummy;

	rf24csn(0);
	SSIDataPut(SSI0_BASE, ACTIVATE);
	SSIDataPut(SSI0_BASE, 0x73);
	while(SSIBusy(SSI0_BASE));							// Keep CSN down until everything is written!
	rf24csn(1);
	SSIDataGet(SSI0_BASE, &dummy);
	SSIDataGet(SSI0_BASE, &dummy);

	// Toggle flag
	rf24Flags ^= RF24_FLAG_FEATURES;
	return (rf24Flags & RF24_FLAG_FEATURES) ? 1 : 0;
}

/**
 * Enable or disable dynamic payload size on selected pipes
 *
 * Allows package length to vary from packet to packet.
 * Pipes is a bitmask of the affected pipes, value should be from 0x01 to 0x3F (6 pipes).
 */
void rf24SetDynamicPayload(uint8_t enable, uint8_t pipes)
{
	if(!enable) {
		rf24WriteRegister(DYNPD, 0);						// Disable dynamic payload
		rf24Flags &= ~RF24_FLAG_DYNAMIC;
		return;
	}

	// Check that features are enabled; if not enable them
	if(!(rf24Flags & RF24_FLAG_FEATURES))
		rf24ToggleFeatures();

	// Enable dynamic payload
	rf24WriteRegister(FEATURE, rf24ReadRegister(FEATURE) | (1 << EN_DPL));

	// Enable dynamic payload length on all pipes
	// DPL_P0 .. DPL_5 are bits 0...5 and thus we can just use the pipes variable
	rf24WriteRegister(DYNPD, pipes & 0b111111);		// Bits 0...5 = 6 bits

	rf24Flags |= RF24_FLAG_DYNAMIC;
}


/**
 * Enable or disable automatic payload sending in ACK packets
 * 
 * This enables or disables receiver (PRX) module to automatically send reply
 * packet when it receives data on certain pipe.
 * Pipes is a bitmask of the affected pipes, value should be from 0x01 to 0x3F (6 pipes).
 *
 * Also enables dynamic payload size, which is required for this feature.
 */
void rf24UseAckPayload(uint8_t enable, uint8_t pipes)
{
	if(!enable) {
		// Disable only ACK payload
		rf24WriteRegister(FEATURE, rf24ReadRegister(FEATURE) & ~(1 << EN_ACK_PAY));
		rf24Flags &= ~RF24_FLAG_ACKPAYLOAD;
		return;
	}

	// Check that features are enabled; if not enable them
	if(!(rf24Flags & RF24_FLAG_FEATURES))
		rf24ToggleFeatures();

	// Enable ACK payload and dynamic payload (required)
	rf24WriteRegister(FEATURE, rf24ReadRegister(FEATURE) | (1 << EN_ACK_PAY) | (1 << EN_DPL));

	// Verify
	if(!(rf24ReadRegister(FEATURE) & (1 << EN_ACK_PAY)))
		UARTSend("+\r\n", 3);

	// Enable dynamic payload on "pipes" in addition to existing; if any
	rf24WriteRegister(DYNPD, rf24ReadRegister(DYNPD) | (pipes & 0b111111));		// Bits 0...5 = 6 bits

	rf24Flags |= RF24_FLAG_ACKPAYLOAD | RF24_FLAG_DYNAMIC;
}


/**
 * Write data to the FIFO to be sent automatically with ACK packet.
 *
 * Automatic payload sending with ACK packet must be enabled.
 */
void rf24WriteAckPayload(uint8_t pipe, const void *buf, uint8_t len)
{
	uint8_t data_len = (len < rf24_max_payload_size) ? len : rf24_max_payload_size;
	uint32_t dummy;
	uint8_t *current = (uint8_t*)buf;

	rf24csn(0);
	SSIDataPut(SSI0_BASE, W_ACK_PAYLOAD | (pipe & 0b111));
	SSIDataGet(SSI0_BASE, &dummy);
	while(data_len--) {
		SSIDataPut(SSI0_BASE, *current++);		// Send payload
		SSIDataGet(SSI0_BASE, &dummy);
	}
	while(SSIBusy(SSI0_BASE));							// Keep CSN down until everything is written!
	rf24csn(1);
}


/**
 * Query the payload size of the next packet in receive (RX) FIFO
 *
 */
uint8_t rf24GetPayloadSize(void)
{
	uint32_t result;

	// If dynamic payloads are not enabled
	if(!(rf24Flags & RF24_FLAG_DYNAMIC)) return payload_size;

	// Dynamics enabled, query the size of top FIFO
	rf24csn(0);
	SSIDataPut(SSI0_BASE, R_RX_PL_WID);
	SSIDataGet(SSI0_BASE, &result);		// Dummy read
	SSIDataPut(SSI0_BASE, 0xFF);		// Dummy write
	SSIDataGet(SSI0_BASE, &result);		// Read the payload size
	while(SSIBusy(SSI0_BASE));
	rf24csn(1);

	return result & 0xFF;
}

/**
 * Set the automatic retransmit count and delay
 *
 * Enables or disables automatic retransmit of packages for which
 * ACK was not received in time.
 * Count is how many times the sending is retries, delay is 
 * the delay between retries in 250 us increments (0 = 250 us, 1 = 500 us).
 * Maximum delay is 15 = 4000 us.
 *
 * Note: for long automatic ACK packets, you need to increase the retransmit delay.
 * The ACK payload lengths for different delays are:
 * 250 us (0)  = 15 bytes in 2 Mbsp mode, 5 bytes in 1 Mbps mode, no payload in 250 kbps mode
 * 500 us (1)  = Any size in 2 Mbps or 1 Mbsp mode, no payload in 250 kbps mode
 * 750 us (2)  = 8 bytes in 250 kbps mode
 * 1000 us (3) = 16 bytes in 250 kbps mode
 * 1250 us (4) = 24 bytes in 250 kbps mode
 * 1500 us (5) = All payload sizes in 250 kbps mode
 * 
 * See the datasheet for more details.
 */
void rf24SetAutoRetransmit(uint8_t count, uint16_t delay)
{
	if(count > 15) count = 15;
	delay &= 0x0F;
	
	rf24WriteRegister(SETUP_RETR, (delay << ARD) | (count << ARC));
}


/**
 * Get the status flags of the driver
 *
 * Flags are a bitmask of following:
 * RF24_FLAG_DYNAMIC		= Dynamic payload size is enabled
 * RF24_FLAG_FEATURES		= Additional features are enabled
 * RF24_FLAG_ACKPAYLOAD		= Automatic ACK payload is enabled
 * RF24_FLAG_AUTOACKAVAIL	= Auto-ACK payload is available
 */
uint8_t rf24GetFlags()
{
	return rf24Flags;
}

