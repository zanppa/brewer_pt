#include <stdint.h>
typedef uint8_t bool;

#include "inc/tm4c123gh6pm.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"

#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/adc.h"

#include "pt.h"

#include "common.h"
#include "mq3.h"

// Port & pin mappings
const uint32_t mq3Peripheral = SYSCTL_PERIPH_GPIOE;
const uint32_t mq3Port = GPIO_PORTE_BASE;
const uint32_t mq3Pin = GPIO_PIN_3;    // PE3 (AIN0) 

const uint32_t mq3ADCPeripheral = SYSCTL_PERIPH_ADC0;
const uint32_t mq3ADC = ADC0_BASE;
const uint32_t mq3ADCSeq = 1;      // ADC sequence number

uint8_t mq3Flags = 0;
static unsigned long ulData[mq3ADCFifoDepth];
uint16_t mq3Value = 0;

// Timer
uint32_t *mq3Timer;

void mq3setup(void)
{
  // Configure input pin
  if(!SysCtlPeripheralReady(mq3Peripheral))
  {
    SysCtlPeripheralEnable(mq3Peripheral);
    while(!SysCtlPeripheralReady(mq3Peripheral));
  }

  // Make the pin ADC
  GPIOPinTypeADC(mq3Port, mq3Pin);

  // Configure ADC peripheral
  if(!SysCtlPeripheralReady(mq3ADCPeripheral))
  {
    SysCtlPeripheralEnable(mq3ADCPeripheral);
    while(!SysCtlPeripheralReady(mq3ADCPeripheral));
  }
    
  ADCSequenceDisable(mq3ADC, mq3ADCSeq);
  ADCSequenceConfigure(mq3ADC, mq3ADCSeq, ADC_TRIGGER_PROCESSOR, 0);
  ADCSequenceStepConfigure(mq3ADC, mq3ADCSeq, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
  ADCSequenceEnable(mq3ADC, mq3ADCSeq);
  
  mq3Timer = getFreeTimer();
}

PT_THREAD(mq3Loop(struct pt *pt))
{
  PT_BEGIN(pt);

  while(1)
  {
	if(mq3Timer) {
		*mq3Timer = MQ3_TIME_INTERVAL;
		PT_WAIT_WHILE(pt, *mq3Timer);
	}
    
    // Do ADC conversion
    ADCIntClear(mq3ADC, mq3ADCSeq);
    ADCProcessorTrigger(mq3ADC, mq3ADCSeq);

    // Wait until conversion is complete (yeild from thread)
    PT_WAIT_UNTIL(pt, ADCIntStatus(mq3ADC, mq3ADCSeq, 0));
  
    ADCSequenceDataGet(mq3ADC, mq3ADCSeq, ulData);
    mq3Value = (uint16_t)ulData[0];
    mq3Flags |= (MQ3_DATA_VALID | MQ3_NEW_DATA);
  }  

  PT_END(pt);
}

uint8_t mq3DataValid()
{
  return (mq3Flags & MQ3_DATA_VALID) ? 1 : 0;
}

uint8_t mq3NewData()
{
  return (mq3Flags & MQ3_NEW_DATA) ? 1 : 0;
}

void mq3ResetNewData()
{
  mq3Flags &= ~MQ3_NEW_DATA;
}


uint16_t mq3GetValue()
{
  return mq3Value;
}

