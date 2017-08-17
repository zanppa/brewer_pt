#ifndef __BUBBLE_H__
#define __BUBBLE_H__

// Library to read LDR and parse it based on ADC value

#define BUBBLE_TIME_INTERVAL		10		// [ms] interval to run
#define bubbleADCFifoDepth			8		// TODO: Sequence 1 has fifo of 8 samples

#define BUBBLE_AUTOLEVEL_CYCLES		200		// Every n cycles decrease/increase threshold in auto level mode

// Initialize all pins and ports
void bubbleSetup(void);

// Returns 1 if data is valid (i.e. conversion not running)
uint8_t bubbleDataValid();
// Returns 1 if new data since last reset (value reset)
uint8_t bubbleNewData();
// Reset new data flag
void bubbleResetNewData();
// Return the value of bubble sensor value compared to threshold
uint8_t bubbleGetBubble();
// Get the raw ADC value
uint16_t bubbleGetLastValue();
// Get the value of the integrated bubbling value
uint32_t bubbleGetIntegral();
// Set the threshold level
void bubbleSetThreshold(uint8_t threshold);
// Get the co2 production value from the volume sensor
uint16_t bubbleGetCo2Value(void);
// Get the latest co2 sensor value
uint8_t bubbleGetCo2Sensor(void);
// Get if automatic mode is on
uint8_t bubbleGetAutoLevelMode(void);
// Get bubble level threshold
uint16_t bubbleGetThreshold(void);
// Get the maximum signal level
uint16_t bubbleGetSensorMaximum(void);
// Get the minimum signal level
uint16_t bubbleGetSensorMinimum(void);

// Loop to read ADC sensor and calculate bubbling index
PT_THREAD(bubbleLoop(struct pt *pt));


#endif
