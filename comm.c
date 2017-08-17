#include <stdint.h>
typedef uint8_t bool;

#include "inc/tm4c123gh6pm.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"

#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"

#include "pt.h"
#include "common.h"
#include "eeprom.h"
#include "nrf24l01.h"
#include "comm.h"


// From main.c
extern volatile uint8_t ledStatus;
extern eConfig systemConfig;
extern eData latestData;
extern eData previousData;
extern uint16_t bubbleRawValue;
extern uint8_t newDataFlags;

extern uint32_t *storeTimer;


// Serial port communication buffers
static volatile uint8_t rxBuffer[RXBUFFERSIZE+1] = {0};
static uint8_t readPos = 0;									// Buffer read position (only changed in serial thread)
static volatile uint8_t writePos = 0;						// Buffer write position (only changed in interrupt)

// RF communication varaibles
static const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };


// Timers
static uint32_t *uartTimer;
static uint32_t *rfTimer;
static uint16_t rfDataTimer = 0;
static uint16_t rfConfigTimer = 0;


// Handler for UART receive interrupt
void __attribute__ ((interrupt)) UARTIntHandler(void)
{
	unsigned long ulInts;
	long lChar;
	uint8_t ucChar;

	// Get and clear the current interrupt source(s)
	ulInts = UARTIntStatus(UART0_BASE, ~0);
	UARTIntClear(UART0_BASE, ulInts);

	// TX FIFO has space available
	if(ulInts & UART_INT_TX) { }

	// Receive interrupts
	if(ulInts & (UART_INT_RX | UART_INT_RT))
	{
		// Read the UART's characters into the buffer.
		while(UARTCharsAvail(UART0_BASE))
		{
  // TODO: Enable this!!
//			if(((writePos+1) & RXBUFFERSIZE) == readPos) break;	// Buffer full

			lChar = UARTCharGetNonBlocking(UART0_BASE);

			// If the character did not contain any error notifications, copy it to the output buffer.
			if(!(lChar & ~0xFF)) {
				ucChar = lChar & 0xFF;
				ledStatus ^= LED_GREEN;

				UARTCharPutNonBlocking(UART0_BASE, ucChar);		// Echo back

				rxBuffer[writePos] = ucChar;
				writePos = (writePos + 1) & RXBUFFERSIZE;

			} else {
				// TODO: Handle uart errors here
			}
		}
	}
}


// Send buffer over UART0
// Returns 0 if uart is busy (FIFO not empty), otherwise 1
// Note: Buffer may need to be < 16 chars (FIFO size), otherwise some characters may get lost...
uint8_t UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
	if(UARTBusy(UART0_BASE)) return 0;
	while(ui32Count--) {
		//UARTCharPut(UART0_BASE, *pui8Buffer++);
		UARTCharPutNonBlocking(UART0_BASE, *pui8Buffer++);
	}
	return 1;
}

const unsigned char hexmap[] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
			'A', 'B', 'C', 'D', 'E', 'F' };
const unsigned char newline[] = { '\r', '\n' };

// Send 32 bit unsigned int as hex values
// Returns 0 if uart is busy (FIFO not empty), otherwise 1
uint8_t UARTSendHex(uint32_t value)
{
	unsigned char str[8] = {0};

	if(UARTBusy(UART0_BASE)) return 0;

	str[0] = hexmap[(value >> 28) & 0xF];
	str[1] = hexmap[(value >> 24) & 0xF];
	str[2] = hexmap[(value >> 20) & 0xF];
	str[3] = hexmap[(value >> 16) & 0xF];
	str[4] = hexmap[(value >> 12) & 0xF];
	str[5] = hexmap[(value >> 8) & 0xF];
	str[6] = hexmap[(value >> 4) & 0xF];
	str[7] = hexmap[value & 0xF];

	UARTSend(str, 8);
	return 1;
}

// Send 32 bit unsigned int as decimal
// Returns 0 if uart is busy (FIFO not empty), otherwise 1
uint8_t UARTSendInt(uint32_t value)
{
	unsigned char str[10] = {0};	// Max 10 chars in uint32
	uint8_t i;

	if(UARTBusy(UART0_BASE)) return 0;

	for(i=1;i<11;i++) {
		str[10-i] = '0' + (value % 10);
		value /= 10;
		if(!value) break;		// Value left is 0
	}
	UARTSend(&str[10-i], i);
	return 1;
}

// Get integer value from receive buffer, with maxN maximum numbers and from offset from current read position
uint8_t rxGetInt(uint8_t offset, uint8_t maxN)
{
	uint8_t tPos = (readPos + offset) & RXBUFFERSIZE;
	uint8_t result = 0;
	while(maxN && tPos != writePos && (rxBuffer[tPos] >= '0' || rxBuffer[tPos] <= '9'))
	{
		result = result * 10 + (rxBuffer[tPos] - '0');
		tPos = (tPos + 1) & RXBUFFERSIZE;
		maxN--;
	}
	return result;
}


void serialCommSetup(void)
{
	// Initialize peripherals
	if(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA))
	{
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
		while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));
	}

	// Initialize peripherals
	if(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0))
	{
		SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
		while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0));
	}

	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
							(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
							 UART_CONFIG_PAR_NONE));
	UARTIntRegister(UART0_BASE, UARTIntHandler);
	UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);	// Interrupt on receive full and receive byte
	
	// Set the timer
	uartTimer = getFreeTimer();
	if(uartTimer)
		*uartTimer = COMM_INTERVAL;
}

PT_THREAD(commLoop(struct pt *pt))
{
	uint8_t command;
	uint8_t bytes;
	uint8_t handled = 0;

	static uint32_t dumpData = 0;
	static uint16_t dumpAddr = 0;

	// Trim newlines
	while(readPos != writePos && (rxBuffer[readPos] == '\r' || rxBuffer[readPos] == '\n')) readPos = (readPos + 1) & RXBUFFERSIZE;

	// Thread continues here
	PT_BEGIN(pt);

	while(1)
	{
		// Handle commands here
		if(readPos != writePos) {
			if(writePos > readPos) bytes = writePos - readPos;
			else {
				bytes = RXBUFFERSIZE - readPos + writePos + 1;
			}
			command = rxBuffer[readPos];

			do {  // Use do...while(0) se break out is easy
				// TODO: Check that everything is numbers...?
				if(command == 'b') {	  // Bubble threshold, bXXX, where XXX is threshold in decimal
					if(bytes < 4) break;
					systemConfig.bubbleLevel = rxGetInt(1, 3);
					bubbleSetThreshold(systemConfig.bubbleLevel);
					handled = 4;
				} else if(command == 's') {  // Eeprom store interval, sXX, where XX is interval in minutes (decimal)'
					if(bytes < 3) break;
					systemConfig.storeInterval = rxGetInt(1, 2);
					if(systemConfig.storeInterval == 0) systemConfig.storeInterval = 1;
					handled = 3;
				} else if(command == 'c') { // Print out config word
					PT_WAIT_UNTIL(pt, UARTSendHex(systemConfig.flags));
					PT_WAIT_UNTIL(pt, UARTSend("\r\n", 2));
					PT_WAIT_UNTIL(pt, UARTSendInt(systemConfig.bubbleLevel));
					PT_WAIT_UNTIL(pt, UARTSend("\r\n", 2));
					PT_WAIT_UNTIL(pt, UARTSendInt(systemConfig.storeInterval));
					PT_WAIT_UNTIL(pt, UARTSend("\r\n", 2));
					handled = 1;
				} else if(command == 'w') { // Write current config to EEPROM
					if(!eWriteConfig(&systemConfig))
						PT_WAIT_UNTIL(pt, UARTSend("K\r\n", 3));
					else
						PT_WAIT_UNTIL(pt, UARTSend("F\r\n", 3));

					handled = 1;
				} if(command == 'd') { // Dump eeprom contents in hex
					for(dumpAddr = 0; dumpAddr < EEPROM_SIZE; dumpAddr += 4) {
						if(eDumpData(&dumpData, dumpAddr) > 0) break;
						// Change byte order (uint32 seems to be LSByte first in mem, while we stored MSByte first to eeprom)
						dumpData = (dumpData >> 24) + ((dumpData >> 8) & 0xFF00) +
								((dumpData << 8) & 0xFF0000) + ((dumpData << 24) & 0xFF000000);

						PT_WAIT_UNTIL(pt, UARTSend(">", 1));
						PT_WAIT_UNTIL(pt, UARTSendHex(dumpData));
						PT_WAIT_UNTIL(pt, UARTSend("\r\n", 2));
					}
					PT_WAIT_UNTIL(pt, UARTSend("K\r\n", 3));
					handled = 1;
				} if(command == 'x') {
					if(bytes < 4) break;
					if(rxGetInt(1,3) == 170) {
						eReset();
						PT_WAIT_UNTIL(pt, UARTSend("K\r\n", 3));
					}
					handled = 4;
				} if(command == 'f') {	// Set configuration flags
					if(bytes < 4) break;
					systemConfig.flags = rxGetInt(1, 3);
					
					if(systemConfig.flags & CONF_BUBBLE_AUTOLEVEL) bubbleSetThreshold(0);
					else bubbleSetThreshold(systemConfig.bubbleLevel);
						
					handled = 4;
				} else {
					// Get rid of unknown characters...
					while(readPos != writePos) readPos = (readPos + 1) & RXBUFFERSIZE;
				}
			} while(0);

			if(handled) {
				readPos = (readPos + handled) & RXBUFFERSIZE;
			}
		}

		// Communications running with timer
		// Doing this in while loop so that we can break out easily
		while(uartTimer && !(*uartTimer)) {
			*uartTimer = COMM_INTERVAL;

			if(!(systemConfig.flags & CONF_SEND_UART)) break;

			if(newDataFlags & NEW_DS) {
				if(latestData.temperature != previousData.temperature) {
					PT_WAIT_UNTIL(pt, UARTSend("T", 1));
					PT_WAIT_UNTIL(pt, UARTSendInt(latestData.temperature));
					PT_WAIT_UNTIL(pt, UARTSend("\r\n", 2));
					previousData.temperature = latestData.temperature;
				}
				newDataFlags &= ~NEW_DS;
			}
			if((newDataFlags & NEW_BUBBLE)) {
				if(systemConfig.flags & CONF_ECHO_BUBBLE) {
					PT_WAIT_UNTIL(pt, UARTSend("R", 1));
					PT_WAIT_UNTIL(pt, UARTSendInt(bubbleRawValue));
					PT_WAIT_UNTIL(pt, UARTSend("\r\n", 2));
				}
				if(latestData.bubble != previousData.bubble && (systemConfig.flags & CONF_ECHO_BINTEGRAL)) {
					PT_WAIT_UNTIL(pt, UARTSend("B", 1));
					PT_WAIT_UNTIL(pt, UARTSendInt(latestData.bubble));
					PT_WAIT_UNTIL(pt, UARTSend("\r\n", 2));
					previousData.bubble = latestData.bubble;
				}
				if(systemConfig.flags & CONF_ECHO_BUBBLE_LIMITS) {
					PT_WAIT_UNTIL(pt, UARTSend("L", 1));
					PT_WAIT_UNTIL(pt, UARTSendInt(bubbleGetSensorMaximum()));
					PT_WAIT_UNTIL(pt, UARTSend(",", 1));
					PT_WAIT_UNTIL(pt, UARTSendInt(bubbleGetThreshold()));
					PT_WAIT_UNTIL(pt, UARTSend(",", 1));
					PT_WAIT_UNTIL(pt, UARTSendInt(bubbleGetSensorMinimum()));
					PT_WAIT_UNTIL(pt, UARTSend(",", 1));
					PT_WAIT_UNTIL(pt, UARTSendInt(bubbleGetCo2Sensor()));
					PT_WAIT_UNTIL(pt, UARTSend("\r\n", 2));
				}
				if(latestData.co2 != previousData.co2) {
					PT_WAIT_UNTIL(pt, UARTSend("C", 1));
					PT_WAIT_UNTIL(pt, UARTSendInt(latestData.co2));
					PT_WAIT_UNTIL(pt, UARTSend("\r\n", 2));
					previousData.co2 = latestData.co2;
				}
				newDataFlags &= ~NEW_BUBBLE;
			}
			if(newDataFlags & NEW_HX711) {
				if(latestData.weight != previousData.weight) {
					PT_WAIT_UNTIL(pt, UARTSend("W", 1));
					PT_WAIT_UNTIL(pt, UARTSendInt(latestData.weight));
					PT_WAIT_UNTIL(pt, UARTSend("\r\n", 2));
					previousData.weight = latestData.weight;
				}
				newDataFlags &= ~NEW_HX711;
			}
			if(newDataFlags & NEW_MQ3) {
				if(latestData.ethanol != previousData.ethanol) {
					PT_WAIT_UNTIL(pt, UARTSend("E", 1));
					PT_WAIT_UNTIL(pt, UARTSendInt(latestData.ethanol));
					PT_WAIT_UNTIL(pt, UARTSend("\r\n", 2));
					previousData.ethanol = latestData.ethanol;
				}
				newDataFlags &= ~NEW_MQ3;
			}
		}

		PT_YIELD(pt);
	}

	PT_END(pt);
}


void rfCommSetup(void)
{
	// Initialize the timer
	rfTimer = getFreeTimer();
	if(rfTimer)
		*rfTimer = RF_PING_INTERVAL;
	
	// Initialize the radio module
	rf24Setup();

	//for(i=0;i<0x18;i++)
	//	rf24ReadRegister(i);

	// Initialize radio comm
	rf24Init();

	rf24SetDynamicPayload(1, 0x3F);			// All pipes
	rf24UseAckPayload(1, 0x3F);				// All pipes send auto-ack

	rf24OpenWritingPipe(pipes[1]);

	//rf24OpenReadingPipe(1, pipes[0]);
	//rf24StartListening();

	rf24PowerUp();
}

/**
 * Write a 1 byte value to hex to buf and buf+1
 */
void dec2hex(uint8_t dec, uint8_t *buf)
{
	uint8_t high = (dec >> 4) & 0x0F;
	dec = dec & 0x0F;

	if(high <= 9) buf[0] = high + '0';
	else buf[0] = high - 10 + 'A';

	if(dec <= 9) buf[1] = dec + '0';
	else buf[1] = dec - 10 + 'A';
}

/**
 * Write lower 4 bits of a byte to buf
 */
void nibble2hex(uint8_t dec, uint8_t *buf)
{
	dec = dec & 0x0F;

	if(dec <= 9) *buf = dec + '0';
	else *buf = dec - 10 + 'A';
}

/**
 * Create a data packet from all sensor data to send over air
 */
uint8_t *serializeData(eData *data, uint8_t *buf)
{
	dec2hex((data->weight >> 24) & 0xFF, buf++); buf++;
	dec2hex((data->weight >> 16) & 0xFF, buf++); buf++;
	dec2hex((data->weight >> 8) & 0xFF, buf++); buf++;
	dec2hex(data->weight & 0xFF, buf++); buf++;
	dec2hex((data->temperature >> 8) & 0xFF, buf++); buf++;
	dec2hex(data->temperature & 0xFF, buf++); buf++;
	dec2hex((data->ethanol >> 8) & 0xFF, buf++); buf++;
	dec2hex(data->ethanol & 0xFF, buf++); buf++;
	dec2hex((data->bubble >> 24) & 0xFF, buf++); buf++;
	dec2hex((data->bubble >> 16) & 0xFF, buf++); buf++;
	dec2hex((data->bubble >> 8) & 0xFF, buf++); buf++;
	dec2hex(data->bubble & 0xFF, buf++); buf++;
	dec2hex((data->co2 >> 8) & 0xFF, buf++); buf++;
	dec2hex(data->co2 & 0xFF, buf++); buf++;
	dec2hex(data->n & 0xFF, buf++); buf++;
	nibble2hex(newDataFlags, buf++);
	return buf;
}


PT_THREAD(rfCommLoop(struct pt *pt))
{
	uint16_t temp;
	static uint8_t mode = RF_MODE_PING;			// Start in PING mode
	static uint8_t status = 0;
	static uint8_t errorCount = 0;
	static uint8_t sendPayload[32] = {0};
	static uint8_t receivePayload[32] = {0};
	static uint8_t len, more;
	static uint8_t blockNum = 0;
	static eData data;
	static uint8_t flags;
	uint8_t i;
	
	PT_BEGIN(pt);
	
	while(1)
	{
		i = 0;
		if(mode == RF_MODE_DATA)
		{
			if(rfDataTimer) {
				rfDataTimer--;

				// Send bubble sensor raw value and sensor limits
				sendPayload[i++] = 'B';
				dec2hex((bubbleRawValue >> 8) & 0xFF, &sendPayload[i++]); i++;
				dec2hex(bubbleRawValue & 0xFF, &sendPayload[i++]); i++;
				// Threshold
				temp = bubbleGetThreshold();
				dec2hex((temp >> 8) & 0xFF, &sendPayload[i++]); i++;
				dec2hex(temp & 0xFF, &sendPayload[i++]); i++;
				temp = bubbleGetSensorMaximum();
				dec2hex((temp >> 8) & 0xFF, &sendPayload[i++]); i++;
				dec2hex(temp & 0xFF, &sendPayload[i++]); i++;
				temp = bubbleGetSensorMinimum();
				dec2hex((temp >> 8) & 0xFF, &sendPayload[i++]); i++;
				dec2hex(temp & 0xFF, &sendPayload[i++]); i++;
			} else {
				rfDataTimer = RF_DATA_INTERVAL;

				// Build a string from eData buffer
				sendPayload[i++] = 'D';
				i = serializeData(&latestData, &sendPayload[i]) - sendPayload;
				newDataFlags &= 0xF0;			// Clear lower 4 bits that were sent
				mode = RF_MODE_CONFIG;			// Send config right after data
			}

		} else if(mode == RF_MODE_CONFIG) {
			// Config data
			sendPayload[i++] = 'C';
			dec2hex((systemConfig.bubbleLevel >> 8) & 0xFF, &sendPayload[i++]); i++;
			dec2hex(systemConfig.bubbleLevel & 0xFF, &sendPayload[i++]); i++;
			dec2hex((systemConfig.storeInterval >> 8) & 0xFF, &sendPayload[i++]); i++;
			dec2hex(systemConfig.storeInterval & 0xFF, &sendPayload[i++]); i++;
			dec2hex((systemConfig.flags >> 8) & 0xFF, &sendPayload[i++]); i++;
			dec2hex(systemConfig.flags & 0xFF, &sendPayload[i++]); i++;
			dec2hex(eGetNextNum() & 0xFF, &sendPayload[i++]); i++;
			
			if(storeTimer)
			{
				dec2hex(((*storeTimer) >> 24) & 0xFF, &sendPayload[i++]); i++;
				dec2hex(((*storeTimer) >> 16) & 0xFF, &sendPayload[i++]); i++;
				dec2hex(((*storeTimer) >> 8) & 0xFF, &sendPayload[i++]); i++;
				dec2hex((*storeTimer) & 0xFF, &sendPayload[i++]); i++;
			}
			
			mode = RF_MODE_DONE;						// Back to data mode after one config send
		} else if(mode == RF_MODE_DUMP) {
			// Dump contents of EEPROM
			if(blockNum < eGetNumBlocks() && !eReadData(&data, blockNum++)) {
				// Successfull read from eeprom
				sendPayload[i++] = 'E';
				i = serializeData(&data, &sendPayload[i]) - sendPayload;
			} else {
				// Read was unsuccesfull -> last block maybe?
				blockNum = 0;
				mode = RF_MODE_DONE;
			}
		} else if(mode == RF_MODE_WRITECONF) {
			if(!eWriteConfig(&systemConfig))
				sendPayload[i++] = 'W';
			else
				sendPayload[i++] = 'F';
			mode = RF_MODE_DONE;
		} else if(mode == RF_MODE_ACK) {
			sendPayload[i++] = 'A';
			mode = RF_MODE_DATA;
		} else if(mode == RF_MODE_DONE) {
			sendPayload[i++] = 'K';
			mode = RF_MODE_DATA;
		} else {
			// Ping, also indicates end of transmission (for multiline data)
			sendPayload[i++] = 'P';
		}
		
		// Send the packet if there is some payload
		if(i) {
			rf24Write(sendPayload, i);
		
			// Check transmit status
			do {
				status = rf24TransmitStatus();
				PT_YIELD(pt);
			} while(status == RF24_TX_BUSY);
		
			// If transmission fails, increase error counter
			if(status == RF24_TX_FAIL && errorCount < 0xFF) {	// Counter saturates
				errorCount++;
			} else if(status == RF24_TX_OK) {
				errorCount = 0;									// Clear when ACK received
			}
		}

		// Check for any messages received back
		if(rf24Received(0) || rf24Available()) {				// All pipes and pending messages
			do {
				len = rf24GetPayloadSize();
				more = rf24Read(receivePayload, len);

				// TODO: Handle received message
				if(receivePayload[0] == 'c') mode = RF_MODE_CONFIG;
				else if(receivePayload[0] == 'd') mode = RF_MODE_DUMP;
				else if(receivePayload[0] == 'w') mode = RF_MODE_WRITECONF;
				else if(receivePayload[0] == 'f') {
					i = (receivePayload[1] - '0') * 100;
					i += (receivePayload[2] - '0') * 10;
					i += (receivePayload[3] - '0');
					systemConfig.flags = i;
				}
				PT_YIELD(pt);
			} while(more);										// Read until RX_EMPTY
		}
		

		// Wait
		while(rfTimer && (*rfTimer))
			PT_YIELD(pt);

		if(errorCount > RF_ERROR_LEVEL) {
			if(rfTimer) *rfTimer = RF_PING_INTERVAL;
			mode = RF_MODE_PING;		// Ping mode
		} else {
			if(rfTimer) *rfTimer = RF_COMM_INTERVAL;
			if(mode == RF_MODE_PING) mode = RF_MODE_DATA;
		}
	}
	PT_END(pt);
}