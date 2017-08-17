/**
 * Library for HX711 weight scale amplifier/AD converter
 *
 *
 * Uses protothreads (by Adam Dunkels, http://dunkels.com/adam/pt/)
 * Some functions also are timed (non-blocking), library by me
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

#include "common.h"
#include "hx711.h"

// Port & pin mappings
const uint32_t hx711Peripheral = SYSCTL_PERIPH_GPIOB;
const uint32_t hx711Port = GPIO_PORTB_BASE;
const uint32_t hx711ClockPin = GPIO_PIN_0;					// PB0 = clock
const uint32_t hx711DataPin = GPIO_PIN_1;					// PB1 = data

static uint8_t hx711Channel = 25;							// 25 = channel A gain 128, 26 = B gain 32, 27 = A gain 64, others = error/undefined
//uint8_t hx711Sleeping = 0;								// 0 = not sleeping, 1 = sleeping

static int32_t hx711Value = 0;								// Measured weight after oversampling
static int32_t hx711LastData = 0;							// Latest conversion result
static volatile uint32_t hx711ConversionData = 0;

static uint8_t hx711Flags = 0;

// Timer
uint32_t *hx711Timer;

/**
 * Setup peripherals and pins required for HX711 communications
 * Communication is done by software
 */
void hx711Setup(void)
{
	// Enable output GPIO peripheral if not yet enabled
	if(!SysCtlPeripheralReady(hx711Peripheral))
	{
		SysCtlPeripheralEnable(hx711Peripheral);
		while(!SysCtlPeripheralReady(hx711Peripheral));
	}

	// Initialize pin types and set values
	GPIOPinTypeGPIOOutput(hx711Port, hx711ClockPin);
	GPIOPinWrite(hx711Port, hx711ClockPin, hx711ClockPin);  // Clock 1 = reset

	GPIOPinTypeGPIOInput(hx711Port, hx711DataPin);

	//hx711Sleeping = 1;
	hx711Flags |= HX711_SLEEPING;

	delayMicrosec(60);										// > 60 us clock pulse high sets sleep mode and resets hx711
	
	hx711Timer = getFreeTimer();
}

/**
 * Set the channel to be used (0, 1 or 2)
 * Returns the previous channel used (0, 1 or 2)
 */
uint8_t hx711SetChannel(uint8_t ch)
{
	uint8_t oldChannel = hx711Channel;

	if(ch == 0) hx711Channel = 25;
	else if(ch == 1) hx711Channel = 26;
	else hx711Channel = 27;

	if(oldChannel == 25) return 0;
	else if(oldChannel == 26) return 1;
	else return 2;
}

/**
 * Check if hx711 has valid data to be read
 * Returns 1 if data is ready, otherwise 0
 */
uint8_t hx711DataReady(void)
{
	// Data pin high means data is not ready
	return GPIOPinRead(hx711Port, hx711DataPin) ? 0 : 1;
}

/**
 * Read last conversion result  from hx711
 * Blocking function
 */
int32_t hx711ReadData(void)
{
	uint8_t clocks;
	uint32_t data = 0;
	bool bInt;

	if(hx711Flags & HX711_SLEEPING)
		hx711SleepOff(1);

  // Disable interrupts to make sure clocking is correct
  //bInt = IntMasterDisable();

	for(clocks = hx711Channel; clocks > 0; clocks--) {
		// Clock high
		GPIOPinWrite(hx711Port, hx711ClockPin, hx711ClockPin);
		delayMicrosec(HX711_CLOCK_TIME);

		// Read bit
		data = (data << 1) + (GPIOPinRead(hx711Port, hx711DataPin) ? 1 : 0);

		// Clock low
		GPIOPinWrite(hx711Port, hx711ClockPin, 0);
		delayMicrosec(HX711_CLOCK_TIME);
	}

	//if(bInt) IntMasterEnable();

	// Scale output to 32 bits and MSB is sign bit -> cast to signed and return
	return (int32_t)(data << 8);
}

/**
 * Put hx711 to sleep mode
 */
void hx711SleepOn(void)
{
	GPIOPinWrite(hx711Port, hx711ClockPin, hx711ClockPin);  // Clock high > 60 us
	delayMicrosec(HX711_SLEEP_TIME);
	hx711Flags |= HX711_SLEEPING;
}

/**
 * Wake hx711 up from sleep mode
 */
void hx711SleepOff(uint8_t block)
{
	GPIOPinWrite(hx711Port, hx711ClockPin, 0);  // Clock low
	if(block) delayMillisec(HX711_SETTLING_TIME);	   // Maximum time before conversion is stable (10 Hz mode; 80 Hz mode only 50 ms)
	hx711Flags &= ~HX711_SLEEPING;
}


// TODO: Vaihda sleep omaan timed functioniin tämän lopusta
// TODO: -> Sallii sitten useamman luvun peräkkäin helposti
/**
 * Read conversion data from hx711 using a non-blocking function 
 * i.e. a timed function
 */
TIMED_FUNCTION(hx711ReadTimed)
{
	static uint8_t clocks = 0;
	TIMED_BEGIN();

	// Wait until timer is free, then lock it
	LOCK_TIMER();

	// Wake up from sleep mode if sleeping
	/*
	if(hx711Flags & HX711_SLEEPING) {
		GPIOPinWrite(hx711Port, hx711ClockPin, 0);	// Clock low
		TIMER_WAIT(hx711ReadTimed, HX711_SETTLING_TIME*1000);	 // Time in ms, wait in us
		hx711Flags &= ~HX711_SLEEPING;
	}*/

	// Check and wait until data is ready
	clocks = HX711_RETRIES;
	while(clocks & !hx711DataReady()) {
		TIMER_WAIT(hx711ReadTimed, HX711_WAIT_TIME);
		clocks--;
	}

	// Did not time out, i.e. retries were left over
	if(clocks) {
		for(clocks = hx711Channel; clocks > 0; clocks--) {
			// Clock high
			GPIOPinWrite(hx711Port, hx711ClockPin, hx711ClockPin);
			TIMER_WAIT(hx711ReadTimed, HX711_CLOCK_TIME);

			// Read bit
			hx711ConversionData = (hx711ConversionData << 1) + (GPIOPinRead(hx711Port, hx711DataPin) ? 1 : 0);

			// Clock low
			GPIOPinWrite(hx711Port, hx711ClockPin, 0);
			TIMER_WAIT(hx711ReadTimed, HX711_CLOCK_TIME);
		}

		// Scale output to 32 bits and MSB is sign bit -> cast to signed and return
		// TODO: Muuta skaalaus niin että on signed + 15 bittiä eikä s+31.
		// Sallii sitten oversamplayksen myöhemmin.
		//hx711LastData = (int32_t)(hx711ConversionData << 8);
		// Convert 24 bit signed integer to a 32 bit signed integer (i.e. two's complement)
		//hx711LastData = (int32_t)((hx711ConversionData & 0x800000) ? (0xFF000000 | hx711ConversionData) : hx711ConversionData);

		// NOTE! The behaviour is undefined by C standard, so most likely not portable :)
		hx711LastData = ((int32_t)(hx711ConversionData << 8)) >> 8;		// TODO: Testaa että menee aritmeettisesti oikein!

		hx711Flags |= HX711_DATA_VALID;
	}

	// If read time interval is long enough, put sensor to sleep mode
	/*
	if(HX711_TIME_INTERVAL > HX711_SLEEP_THRESHOLD) {
		GPIOPinWrite(hx711Port, hx711ClockPin, hx711ClockPin);	// Clock high > 60 us
		TIMER_WAIT(hx711ReadTimed, HX711_SLEEP_TIME);
		hx711Flags |= HX711_SLEEPING;
	}*/

	RELEASE_TIMER();

	TIMED_END();
}

/**
 * Put hx711 to sleep mode
 * Non-blocking, timed function
 */
TIMED_FUNCTION(hx711SleepOnTimed)
{
	TIMED_BEGIN();
	LOCK_TIMER();
	GPIOPinWrite(hx711Port, hx711ClockPin, hx711ClockPin);	// Clock high > 60 us
	TIMER_WAIT(hx711SleepOnTimed, HX711_SLEEP_TIME);
	hx711Flags |= HX711_SLEEPING;
	RELEASE_TIMER();
	TIMED_END();
}

/**
 * Wake hx711 from sleep mode
 * non-blocking, timed function
 */
 TIMED_FUNCTION(hx711SleepOffTimed)
 {
	TIMED_BEGIN();
	if(hx711Flags & HX711_SLEEPING) {
		LOCK_TIMER();
		GPIOPinWrite(hx711Port, hx711ClockPin, 0);	// Clock low
		TIMER_WAIT(hx711SleepOffTimed, HX711_SETTLING_TIME*1000);	 // Time in ms, wait in us
		hx711Flags &= ~HX711_SLEEPING;
		RELEASE_TIMER();
	}
	TIMED_END();
 }


/**
 * Main loop to read data from hx711 weigh scale sensor
 */
PT_THREAD(hx711Loop(struct pt *pt))
{
	static uint8_t samples;
	PT_BEGIN(pt);

	while(1)
	{
		if(hx711Timer) {
			*hx711Timer = HX711_TIME_INTERVAL;
			PT_WAIT_WHILE(pt, *hx711Timer);
		}

		hx711Flags &= ~HX711_DATA_VALID;

		// Wake up from sleep mode if sleeping
		PT_WAIT_UNTIL(pt, hx711SleepOffTimed(0, CALLER_THREAD) == RETURN_DONE);		// Timed call

		samples = HX711_SAMPLES;
		hx711Value = 0;
		do {
			// Wait until conversion data is ready
			PT_WAIT_UNTIL(pt, hx711DataReady());
			// Start conversion and wait for result
			PT_WAIT_UNTIL(pt, hx711ReadTimed(0, CALLER_THREAD) == RETURN_DONE);			// Timed call
			// hx711LastData = hx711ReadData();	// Blocking call
			
			hx711Value += hx711LastData;

			PT_YIELD(pt);						// Yield in between reads
			samples--;
		} while(samples);
		
		if(hx711Flags & HX711_DATA_VALID)		// Conversion was succesfull
		hx711Flags |= HX711_NEW_DATA;
		
		// Put hx711 to sleep mode if delay between reads is long enough
		if(HX711_TIME_INTERVAL > HX711_SLEEP_THRESHOLD) {
			PT_WAIT_UNTIL(pt, hx711SleepOnTimed(0, CALLER_THREAD) == RETURN_DONE);	// Timed call
		}

	}

	PT_END(pt);
}

/**
 * Returns 1 if data is valid (i.e. conversion not running)
 */
uint8_t hx711DataValid()
{
	return (hx711Flags & HX711_DATA_VALID) ? 1 : 0;
}

/**
 * Returns 1 if there is new data since last reset (value reset)
 */
uint8_t hx711NewData()
{
	return (hx711Flags & HX711_NEW_DATA) ? 1 : 0;
}

/**
 * Reset new data flag
 */
void hx711ResetNewData()
{
	hx711Flags &= ~HX711_NEW_DATA;
}

/**
 * Get the latest conversion value
 */
int32_t hx711GetLastValue()
{
	return hx711Value;
}
