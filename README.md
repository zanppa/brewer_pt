# brewer_pt
Fermentation monitoring gadget using protothreads for virtual multitasking.

This is firmware for a fermentation monitoring gadged, using TI Tiva launchapd (TM4C123GH6PMI) as the core.
Firmware supports several sensors:
- Weight sensor with HX711 ADC for weigh scales and suitable load cells
- Temperature sensor DS18B20
- Bubbling sensor using a LED and photoresistor
- CO2 volume sensor with a HALL effect switch

Firmware also supports communication through USB-UART (built in the launchpad debug interface) and wireless 
communication with Nordic NRF24L01 radio modules.

Results are also stored periodically to EEPROM to prevent data loss on blackouts.

Protothreads are utilised to simplify the scheduling of the sensor readings. Also I use my own derivate of 
the Protothread, Timed function, which allows precise timing/waiting in functions. This simplifies multithreading 
even when needing precise timing e.g. during serial communication. Single function can be stepper forward with 
timer interrupts while everything else runs on background.
