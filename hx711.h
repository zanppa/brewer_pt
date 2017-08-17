#ifndef __HX711_H__
#define __HX711_H__

// Library for communicating with HX711 weigh scale sensor IC

#define HX711_TIME_INTERVAL			60000		// [ms] Timer interval, 1 minute
#define HX711_SLEEP_THRESHOLD		10000		// Sleep if time between read is more than 10 seconds

#define HX711_SETTLING_TIME			400			// ms after reset etc, in 10 Hz mode 400 ms, in 80 Hz mode 50 ms
#define HX711_CLOCK_TIME			10			// us, clock high and low times
#define HX711_SLEEP_TIME			60			// 60 us pulse sets hx711 to sleep mode

#define HX711_WAIT_TIME				1000		 // Wait 1 ms if data is not ready when trying to read
#define HX711_RETRIES				100			// Try 100 times (*wait time) = 100 ms more

// Flags
#define HX711_DATA_VALID			0x01
#define HX711_NEW_DATA				0x02
#define HX711_SLEEPING				0x04

// Oversampling
#define HX711_SAMPLES				16

// Initialize weigh scale communication, i.e. set pins
// Puts hx711 in sleep (reset) mode, so call hx711SleepOff when about to start
void hx711Setup(void);

// Set channel and gain; 0 = A gain 128, 1 = B gain 32, 2 or others = A gain 64
// Returns previous mode
// Change is sent to board when next conversion is read, after which
// certain delay is needed before HX711 output is stable
uint8_t hx711SetChannel(uint8_t ch);

// Returns 1 if data is ready (data pin low), 0 otherwise
uint8_t hx711DataReady(void);

// Read conversion value from selected channel
// waits until data set is ready and blocks interrupts during read
// Note that reading data while in sleep mode ends sleep mode and waits until data is ready
int32_t hx711ReadData(void);

// Turn sleep mode on, does not block interrupts
void hx711SleepOn(void);

// Turns off sleep mode, does not block interrupts
// If block = 0, returns immediately, otherwise waits until outputs are stable
// (400 ms or so)
void hx711SleepOff(uint8_t block);


// Main loop to read hx711 sensor data
PT_THREAD(hx711Loop(struct pt *pt));

// Returns 1 if data is valid (i.e. conversion not running)
uint8_t hx711DataValid();
// Returns 1 if new data since last reset (value reset)
uint8_t hx711NewData();
// Reset new data flag
void hx711ResetNewData();
// Return the latest conversion result in fixed point 12.4 (i.e. divide by 16.0 to get temp in Celsius)
int32_t hx711GetLastValue();


#endif
