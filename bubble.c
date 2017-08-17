/**
 * Simple library to read bubbling sensor values
 *
 * Supports airlock bubbling sensor and a fill-and-dump type
 * volumetric sensor
 *
 * Uses protothreads (by Adam Dunkels, http://dunkels.com/adam/pt/)
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
#include "driverlib/adc.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"

#include "pt.h"

#include "common.h"
#include "eeprom.h"
#include "bubble.h"


// From main.c
extern eConfig systemConfig;

// Port & pin mappings
const uint32_t bubblePeripheral = SYSCTL_PERIPH_GPIOE;
const uint32_t bubblePort = GPIO_PORTE_BASE;
const uint32_t bubblePin = GPIO_PIN_1;						// PE1 (AIN2)		// Oli PIN_1???

const uint32_t bubbleADCPeripheral = SYSCTL_PERIPH_ADC0;
const uint32_t bubbleADC = ADC0_BASE;
const uint32_t bubbleADCSeq = 0;							// ADC sequence number

const uint32_t bubbleCo2Peripheral = SYSCTL_PERIPH_GPIOD;
const uint32_t bubbleCo2Port = GPIO_PORTD_BASE;
const uint32_t bubbleCo2Pin = GPIO_PIN_0;					// PD0


// Other variables
static const uint8_t bubbleLevelMargin = 200;				// Margin when using auto level
static uint16_t bubbleLevelTimer = BUBBLE_AUTOLEVEL_CYCLES;

// This counter increases every time a bubble is detected
// uint32 can store about 49 days with 1 ms timer
static uint8_t bubbleFlags = 0;								// bit 0 = data is valid, bit 1 = new data
static uint32_t bubbleIntegral = 0;
static uint8_t bubbleDetected = 0;							// 1 if last sample had a bubble

static uint8_t bubbleCo2LastState = 0;
static uint16_t bubbleCo2 = 0;


static uint32_t ulData[bubbleADCFifoDepth];
static uint16_t bubbleSensorValue = 0;

static uint8_t bubbleAutoLevel = 1;							// Set to 1 (set level to 0 to automatically tune the threshold
static uint16_t bubbleSensorMax = 0x0FFF;						// Auto leveling top and bottom limits
static uint16_t bubbleSensorMin = 0;
static uint16_t bubbleLevel = 820;							// Bubble detection level in ADC values


// Timer
uint32_t *bubbleTimer;

void bubbleSetup(void)
{
	// Configure input pins
	if(!SysCtlPeripheralReady(bubblePeripheral))
	{
		SysCtlPeripheralEnable(bubblePeripheral);
		while(!SysCtlPeripheralReady(bubblePeripheral));
	}

	if(!SysCtlPeripheralReady(bubbleCo2Peripheral))
	{
		SysCtlPeripheralEnable(bubbleCo2Peripheral);
		while(!SysCtlPeripheralReady(bubbleCo2Peripheral));
	}


	// Make the pin ADC
	GPIOPinTypeADC(bubblePort, bubblePin);

	// Make the Hall sensor pin input, with weak pull-up
	GPIOPinTypeGPIOInput(bubbleCo2Port, bubbleCo2Pin);
	GPIOPadConfigSet(bubbleCo2Port, bubbleCo2Pin, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

	// Configure ADC peripheral
	if(!SysCtlPeripheralReady(bubbleADCPeripheral))
	{
		SysCtlPeripheralEnable(bubbleADCPeripheral);
		while(!SysCtlPeripheralReady(bubbleADCPeripheral));
	}

	ADCSequenceDisable(bubbleADC, bubbleADCSeq);
	ADCSequenceConfigure(bubbleADC, bubbleADCSeq, ADC_TRIGGER_PROCESSOR, 0);
	ADCSequenceStepConfigure(bubbleADC, bubbleADCSeq, 0, ADC_CTL_CH2 | ADC_CTL_IE | ADC_CTL_END);
	ADCSequenceEnable(bubbleADC, bubbleADCSeq);
	
	bubbleTimer = getFreeTimer();
	if(bubbleTimer) *bubbleTimer = BUBBLE_TIME_INTERVAL;
}



PT_THREAD(bubbleLoop(struct pt *pt))
{
	uint8_t temp;
	PT_BEGIN(pt);

	while(1) {
		// Reset data valid before conversion
		bubbleFlags &= 0xFE;

		ADCIntClear(bubbleADC, bubbleADCSeq);
		ADCProcessorTrigger(bubbleADC, bubbleADCSeq);

		// Wait until conversion is complete
		// Releases the thread here
		PT_WAIT_UNTIL(pt, ADCIntStatus(bubbleADC, bubbleADCSeq, 0));

		ADCSequenceDataGet(bubbleADC, bubbleADCSeq, ulData);
		bubbleSensorValue = (uint16_t)ulData[0];

		if((!(systemConfig.flags & CONF_BUBBLE_INVERT) && bubbleSensorValue <= bubbleLevel) ||
			((systemConfig.flags & CONF_BUBBLE_INVERT) && bubbleSensorValue >= bubbleLevel)) {
			bubbleDetected = 1;
			bubbleIntegral++;
		} else {
			bubbleDetected = 0;
		}
		
		// Tune the threshold in auto leveling mode
		// Keeps also track of the maximum and minimum signal levels
		if(bubbleSensorValue > bubbleSensorMax) bubbleSensorMax = bubbleSensorValue;
		if(bubbleSensorValue < bubbleSensorMin) bubbleSensorMin = bubbleSensorValue;
		if(bubbleAutoLevel) {
			bubbleLevel = (bubbleSensorMax / 2) + (bubbleSensorMin / 2);			// Use middle as threshold
		}
		if(!bubbleLevelTimer) {
			bubbleLevelTimer = BUBBLE_AUTOLEVEL_CYCLES;
			
			// Slowly pull limits together, keeping margin between them
			// Pull top level faster than bottom level, or other way if inverter
			if(bubbleSensorMax > bubbleSensorMin + bubbleLevelMargin) bubbleSensorMax--;
			if(!(systemConfig.flags & CONF_BUBBLE_INVERT)) {
				if(bubbleSensorMax > bubbleSensorMin + bubbleLevelMargin) bubbleSensorMax--;
				if(bubbleSensorMax > bubbleSensorMin + bubbleLevelMargin) bubbleSensorMax--;
			}

			if(bubbleSensorMin < bubbleSensorMax - bubbleLevelMargin) bubbleSensorMin++;
			if(systemConfig.flags & CONF_BUBBLE_INVERT) {
				if(bubbleSensorMin < bubbleSensorMax - bubbleLevelMargin) bubbleSensorMin++;
				if(bubbleSensorMin < bubbleSensorMax - bubbleLevelMargin) bubbleSensorMin++;
			}
		} else {
			bubbleLevelTimer--;
		}

		// Read the Co2 sensor status
		temp = GPIOPinRead(bubbleCo2Port, bubbleCo2Pin);
		if(temp != bubbleCo2LastState) {
			bubbleCo2++;									// Increase integral on every state change (i.e. emptying or filling)
			bubbleCo2LastState = temp;
		}

		// Data is valid, and new data is available
		bubbleFlags |= 0x03;								// Bits 0 and 1

		// Wait for next running time, i.e. timer to trig
		if(bubbleTimer) {
			*bubbleTimer = BUBBLE_TIME_INTERVAL;	// TODO: Should be moved to top of the function for timing accuracy
			PT_WAIT_WHILE(pt, *bubbleTimer);
		}
	}

	PT_END(pt);
}


uint8_t bubbleDataValid()
{
	return (bubbleFlags & 0x01) ? 1 : 0;					// Bit 0 = data valid
}

uint8_t bubbleNewData()
{
	return (bubbleFlags & 0x02) ? 1 : 0;					// Bit 1 = new data
}

void bubbleResetNewData()
{
	bubbleFlags &= 0xFD;									// Clear bit 1
}

uint8_t bubbleGetBubble()
{
	return bubbleDetected;
}

uint16_t bubbleGetLastValue()
{
	return bubbleSensorValue;
}

uint32_t bubbleGetIntegral()
{
	return bubbleIntegral;
}

void bubbleSetThreshold(uint8_t threshold)
{
	if(threshold == 0) {
		bubbleAutoLevel = 1;

		// Continue tuning from about the current level
		if(bubbleLevel < 0x0FFF-bubbleLevelMargin) bubbleSensorMax = bubbleLevel + bubbleLevelMargin;
		else bubbleSensorMax = 0x0FFF;

		if(bubbleLevel > bubbleLevelMargin) bubbleSensorMin = bubbleLevel - bubbleLevelMargin;
		else bubbleSensorMin = 0;
	} else {
		bubbleAutoLevel = 0;
		bubbleLevel = threshold << 5;
	}
}

uint16_t bubbleGetCo2Value(void)
{
	return bubbleCo2;
}

uint8_t bubbleGetCo2Sensor(void)
{
	return bubbleCo2LastState;
}

uint8_t bubbleGetAutoLevelMode(void)
{
	return bubbleAutoLevel;
}

uint16_t bubbleGetThreshold(void)
{
	return bubbleLevel;
}

uint16_t bubbleGetSensorMaximum(void)
{
	return bubbleSensorMax;
}

uint16_t bubbleGetSensorMinimum(void)
{
	return bubbleSensorMin;
}
