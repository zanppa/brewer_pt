// HX711   - 4 pin
// MQ-3    - 3 pin
// DS18B20 - 3 pin
// Bubble  - 4 ... 6 pin
// 



// Port mapping

// PA0    = UART (debug)
// PA1    = UART (debug)
// PA2    = NRF24L01 SPI CLK
// PA3    = NRF24L01 SPI FSS
// PA4    = NRF24L01 SPI RX
// PA5    = NRF24L01 SPI TX
// PA6
// PA7

// PB0    = HX711 Clock pin
// PB1    = HX711 Data pin
// PB2    = DS18B20 1-wire data pin  ( ext. pull-up)
// PB3    = NRF24L01 CE
// PB4    = NRF24L01 CSN
// PB5    = CO2 volume sensor (HALL switch)
// PB6
// PB7

// PE0
// PE1    = Bubble sensor LDR in (AIN2)
// PE2
// PE3    = MQ3 sensor analog value (AIN0)
// PE4
// PE5
// PE6

// PF0
// PF1    = LED
// PF2    = LED
// PF3    = LED
// PF4
// PF5
// PF6
// PF7


// Other
// ADC0 sequence 0 channel 1  = Bubble sensor LDR
// ADC0 sequence 1 channel 0  = MQ3 sensor


// RF commands
// c		= Request config dump
// d		= Request EEPROM dump
// f000		= Set config flags, 000 is uint8 in decimal for the flags
// w		= Write config to eeprom, returns W if ok, F if failed, then K (ACK)

// RF messages
// A		= Acknowledge last commands
// BAAAABBBBCCCCDDDD					// Bubble sensor A=raw value, sensor latest B=threshold, C=maximum and D=minimum values
// DAAAAAAAABBBBCCCCDDDDDDDDEEEEFFG		// Data packet, values in hex, A=weight, B=temperature, C=ethanol, D=bubble integral, E=co2 integral, F=package number, G=new data flags
// CAAAABBBBCCCCDDEEEE					// Config word, values in hex, A=bubble sensor threshold, B=eeprom write interval, C=config flags, D=next write block number, E=eeprom write timer value
// EXXX		= Eemprom data packet (data that was stored to eeprom), same contents as with D data packet
// K		= Done with (end of) multi-packet messages
// P		= Ping


// UART Commands (all in small letters)
// c    = Display current configuration word
// w    = Write current configuration to eeprom
// d    = Dump current eeprom contents in HEX
// rXXX = Read eeprom data bank XX (1...254) TODO
// sXX  = Set eeprom write interval to XX minutes (1...99)
// bXXX = Set bubbling sensor threshold level to XXX (1...254)
// pXXX = Set output printing interval in 10 ms intervals (1...999) TODO
// fXXX = Set config flags (0...255) 
// x170 = Reset whole eeprom (0xAA, 0b10101010)

// Values printed out all the time on UART
// BXXX  = Bubbling sensor integral value, XXX is uint32_t
// RXXX  = Bubbling sensor raw ADC value
// CXXX  = Co2 volume sensor integral, XXX is uint32_t
// EXXX  = Ethanol sensor raw value
// WXXX  = Weight sensor raw value
// TXXX  = Temperature measurement raw value
