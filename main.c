/**
 * Brewing monitor system prototype
 *
 * For TI Tiva or Stellaris launchpads, TM4C123 or LM4F120 microcontrollers.
 *
 * Tested to work with Energia IDE, after some modifications to
 * the default main C/Cpp file, init and main functions need to be weakened:
 * __attribute__((weak))
 *
 * Copyright (C) 2016 Lauri Peltonen
 */

#include <stdint.h>
typedef uint8_t bool;

#include "inc/tm4c123gh6pm.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"

#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/systick.h"

#include "pt.h"

#include "common.h"
#include "hx711.h"
#include "mq3.h"
#include "bubble.h"
#include "ds18b20.h"
#include "eeprom.h"
#include "comm.h"


// Default configuration
eConfig systemConfig;

// Latest data to be stored to EEPROM
// Note: this only stores latest values, does not care if some sensor has
// failed and does not provide new values...
// That must be checked online with communication interface
eData latestData = {0};
eData previousData = {0};
uint16_t bubbleRawValue = 0;

// Timer for storing to eeprom
uint32_t *storeTimer = 0;

// Bitmask for indicating when new data was read from sensor
#define NEW_DS						0x01
#define NEW_BUBBLE					0x02
#define NEW_HX711					0x04
#define NEW_MQ3						0x08
uint8_t newDataFlags = 0;

volatile uint8_t ledStatus = 0;

// Needed to get rid of Energia's default initializations
void _init(void) {;}

uint8_t mess[4] = { 0xDE, 0xAD, 0xBE, 0xEF };

void setDefaultConfig(void)
{
	systemConfig.bubbleLevel = 34;			// Bubble threshold 34 * 32 = 1100 ADC units
	systemConfig.storeInterval = 60;		// Store every 60 minutes
	systemConfig.flags = CONF_ECHO_BUBBLE | CONF_SEND_UART | CONF_BUBBLE_AUTOLEVEL;
}

int main(void)
{
	int i;
	int temp;
	struct pt bubblePt, hx711Pt, mq3Pt, dsPt, commPt, rfCommPt;

	// Set clock speed to 80 MHz
	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL| SYSCTL_OSC_INT);

	// Led peripheral and pins
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

	setupTimer(THREAD_TIMER_INTERVAL);

	// Initialize timer
	InitTimedFunctions();
	
	// Initialize communications
	serialCommSetup();
	rfCommSetup();


	// Initialize eeprom
	UARTSend("Init\r\n", 6);
	eInit();
	if(eIsOK()) {
		// Try and read the configuration word...
		eReadConfig(&systemConfig);
		if(systemConfig.bubbleLevel == 255) {
			UARTSend("Default conf\r\n", 14);
			while(UARTBusy(UART0_BASE));
			setDefaultConfig();
		} else {
			UARTSendInt(systemConfig.bubbleLevel);
			while(UARTBusy(UART0_BASE));
			UARTSend("\r\n", 2);
			while(UARTBusy(UART0_BASE));
			UARTSendInt(systemConfig.storeInterval);
			while(UARTBusy(UART0_BASE));
			UARTSend("\r\n", 2);
			while(UARTBusy(UART0_BASE));

			// Set the values to subsystems
			bubbleSetThreshold(systemConfig.bubbleLevel);
			if(systemConfig.flags & CONF_BUBBLE_AUTOLEVEL) bubbleSetThreshold(0);	// Auto level on
		}
	} else {
		UARTSend("Eeprom fail!\r\n", 14);
		while(UARTBusy(UART0_BASE));
		setDefaultConfig();
	}

	// Verify struct sizes
	if(sizeof(eData) != EEPROM_EDATA_SIZE) while(!UARTSend("eData size!\r\n", 13));
	if(sizeof(eConfig) != EEPROM_ECONFIG_SIZE) while(!UARTSend("eConf size!\r\n", 13));

	
	// Initialize all sensors
	hx711Setup();
	mq3setup();
	bubbleSetup();
	dsSetup();

	// Initialize timer for storing data to EEPROM
	storeTimer = getFreeTimer();
	if(storeTimer)
		*storeTimer = systemConfig.storeInterval * 60000;		// Given in minutes

	// Slow timer for all other stuff
	//SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
	//TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
	//TimerLoadSet64(TIMER1_BASE, (uint64_t)(SysCtlClockGet()/1000) * slowTimerStep);
	//TimerIntRegister(TIMER1_BASE, TIMER_A, slowIntHandler);
	//TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	//TimerEnable(TIMER1_BASE, TIMER_A);
	//IntEnable(INT_TIMER1A);

	IntMasterEnable();

	// Initializer protothreads
	PT_INIT(&bubblePt);
	PT_INIT(&hx711Pt);
	PT_INIT(&mq3Pt);
	PT_INIT(&dsPt);
	PT_INIT(&commPt);
	PT_INIT(&rfCommPt);
	
	// Update block number
	latestData.n = eGetNextNum();
	
	// Main loop
	while(1)
	{
		// Handle common (thread) timers here so they don't need to be volatile
		handleTimers();

		// Schedule sensors
		bubbleLoop(&bubblePt);
		hx711Loop(&hx711Pt);
		mq3Loop(&mq3Pt);
		dsLoop(&dsPt);
		commLoop(&commPt);
		rfCommLoop(&rfCommPt);

		// Blink LEDs if they are set
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, ledStatus);

		// Check if EEPROM store timer was changed in communications loop
		// If interval is shorter than current wait time, update it to the new interval
		if(storeTimer && (*storeTimer) > (systemConfig.storeInterval * 60000)) *storeTimer = systemConfig.storeInterval * 60000;

		// Check all sensors for new data
		if(bubbleNewData()) {
			bubbleRawValue = bubbleGetLastValue();
			latestData.bubble = bubbleGetIntegral();
			latestData.co2 = bubbleGetCo2Value();
			newDataFlags |= NEW_BUBBLE;
			bubbleResetNewData();
		}
		if(hx711NewData()) {
			latestData.weight = hx711GetLastValue();
			newDataFlags |= NEW_HX711;
			hx711ResetNewData();
		}
		if(mq3NewData()) {
			latestData.ethanol = mq3GetValue();
			newDataFlags |= NEW_MQ3;
			mq3ResetNewData();
		}
		if(dsNewData()) {
			latestData.temperature = dsGetLastValue();
			newDataFlags |= NEW_DS;
			dsResetNewData();
		}
		
		// Read the latest bubble sensor level to config
		/*
		if(bubbleGetAutoLevelMode())
			systemConfig.flags |= CONF_BUBBLE_AUTOLEVEL;
		else
			systemConfig.flags &= ~CONF_BUBBLE_AUTOLEVEL;*/
		systemConfig.bubbleLevel = bubbleGetThreshold() >> 5;

		// Store to eeprom if everything is fine!
		if(storeTimer && !(*storeTimer)) {
			*storeTimer = systemConfig.storeInterval * 60000;

			if(!eWriteData(&latestData))
				while(!UARTSend("S\r\n", 3));			// Storing data succesfull
			else
				while(!UARTSend("F\r\n", 3));

			ledStatus ^= LED_RED;						// Toggle red led
			latestData.n = eGetNextNum();
		}
	}

	return 0;
}