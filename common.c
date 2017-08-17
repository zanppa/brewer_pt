#include <stdint.h>
typedef uint8_t bool;

#include "inc/tm4c123gh6pm.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"

#include "driverlib/sysctl.h"
#include "driverlib/rom_map.h"
#include "driverlib/rom.h"
#include "driverlib/timer.h"

#include "common.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"


// Thread timer variables
static uint8_t nextTimer = 0;
static uint32_t commonTimer[TIMERS] = {0};
static volatile timerCallback timerCb[TIMER_CALLBACKS] = {0};

// Timer helper variables
static volatile uint8_t timerTriggered = 0;
static volatile uint8_t timerLatencyError = 255;			// Timer triggered before it was handled (will decrease on error)


// These are left as globals for now
volatile uint8_t waitMutex;		// Lock for the exact wait timer
volatile timerCallback waitCb;	// Callback for exact waitCb

void delayMicrosec(uint32_t time)
{
	// ROM_SysCtlDelay(ROM_SysCtlClockGet()/3000000 * time);
	SysCtlDelay(SysCtlClockGet()/3000000 * time);
}

void delayMillisec(uint32_t time)
{
	// ROM_SysCtlDelay(ROM_SysCtlClockGet()/3000 * time);
	SysCtlDelay(SysCtlClockGet()/3000 * time);
}


void __attribute__ ((interrupt)) timedFunctionsIntHandler(void)
{
	uint8_t i;
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

	waitMutex = TIMER_LOCK;	// Timer stopped
	if(waitCb.callback)
		(*waitCb.callback)(timerCb[i].data, CALLER_TIMER);	// Handle callback
}

void InitTimedFunctions(void)
{
	// General timer, meant for one-shot accurate delays
	if(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0))
	{
		SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
		while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0));
	}
	TimerConfigure(TIMER0_BASE, TIMER_CFG_ONE_SHOT);
	TimerLoadSet64(TIMER0_BASE, (uint64_t)SysCtlClockGet());	// Default
	TimerIntRegister(TIMER0_BASE, TIMER_A, timedFunctionsIntHandler);
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	// TimerEnable(TIMER0_BASE, TIMER_A);
	IntEnable(INT_TIMER0A);
}

// Interrupt for system timer
// This takes about 528 ns to run (not counting the main loop portion)
void __attribute__ ((interrupt)) timerIntHandler(void)
{
	// This uses System Tick so the interrupt does not need to be reset
	if(timerTriggered && timerLatencyError) timerLatencyError--;
	timerTriggered = 1;
	//	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	//	slowTimerTriggered = 1;
}

void setupTimer(uint32_t timerInterval)
{
	// Configure timers for reading sensors
	// Generic timer for thread scheduling
	// Using systick
	SysTickPeriodSet((uint64_t)(SysCtlClockGet() / 1000000) * timerInterval);
	SysTickIntRegister(timerIntHandler);
	SysTickEnable();

	// Second timer with multiple callbacks available
#if 0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
	TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
	TimerLoadSet64(TIMER1_BASE, (uint64_t)(SysCtlClockGet() / 1000000) * commonTimerStep);
	TimerIntRegister(TIMER1_BASE, TIMER_A, commonTimerIntHandler);
	TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	TimerEnable(TIMER1_BASE, TIMER_A);
	IntEnable(INT_TIMER1A);
#endif
}

uint32_t *getFreeTimer(void)
{
	if(nextTimer >= TIMERS) return 0;
	return &commonTimer[nextTimer++];
}

void handleTimers(void)
{
	uint8_t i;
	if(timerTriggered)
	{
		timerTriggered = 0;
		// Decrease timers
		for(i=0; i<TIMERS; i++)
			if(commonTimer[i]) commonTimer[i]--;
	}
}


// Fast timer interrupt
// Intended to do short time critical sections
// Handler takes about 560 ... 600 ns / timer + time of any handler @ 80 MHz
// 580 ns / 1
// 940 ns / 2
// 1180 ns / 3
// 1420 ns / 4
// 1680 ns / 5
// 2940 ns / 10
// When callback is handled, to turn on/off a I/O and re-init the timer, it takes 2110 ns / 4 timers (1 callback)
void __attribute__ ((interrupt)) commonTimerIntHandler(void)
{
	uint8_t i;
	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

	// Handle callbacks
	for(i=0; i < TIMER_CALLBACKS; i++)
	{
		if(timerCb[i].timer) {
			timerCb[i].timer--;
			if(!timerCb[i].timer && timerCb[i].callback) {
				// Timer reached zero -> triggered
				(*timerCb[i].callback)(timerCb[i].data, CALLER_TIMER);
			}
		}
	}
}

uint32_t getTimerTime(uint8_t timer)
{
	if(timer >= TIMERS) return 0;
	return commonTimer[timer];
}