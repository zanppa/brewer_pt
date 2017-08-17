#ifndef __MQ3_H__
#define __MQ3_H__

// Library to interface with MQ-3 ethanol sensor breakout board

#define MQ3_TIME_INTERVAL		10000			// Read every 10 seconds

#define mq3ADCFifoDepth			8				// Sequence 1 has fifo of 8 samples

#define MQ3_DATA_VALID			0x01
#define MQ3_NEW_DATA			0x02

// Initialize all pins and ports
void mq3setup(void);

// Read one sample from MQ-3
// Blocks until conversion is done
PT_THREAD(mq3Loop(struct pt *pt));


// Returns 1 if data is valid (i.e. conversion not running)
uint8_t mq3DataValid();
// Returns 1 if new data since last reset (value reset)
uint8_t mq3NewData();
// Reset new data flag
void mq3ResetNewData();
// Return the value of ethanol sensor value
uint16_t mq3GetValue();


#endif
