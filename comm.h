#ifndef __COMM_H__
#define __COMM_H__

#define COMM_TIMER					4				// Timer for serial communications
#define COMM_INTERVAL				50				// Runs at 50 ms intervals

#define RF_TIMER					5				// Timer for RF communication
#define RF_COMM_INTERVAL			100				// Run at 100 ms
#define RF_PING_INTERVAL			200				// Ping every 2 seconds if no ACK received
#define RF_ERROR_LEVEL				200				// About 5 seconds trying to send, before going to ping mode

#define RF_DATA_INTERVAL			100				// Send full data packet every now and then

#define RF_MODE_DATA				1
#define RF_MODE_CONFIG				2
#define RF_MODE_DUMP				10
#define RF_MODE_WRITECONF			11
#define RF_MODE_ACK					80				// Confirm command received
#define RF_MODE_DONE				90				// Confirm end of multiline message
#define RF_MODE_PING				100


#define RXBUFFERSIZE			0x0F					// 16 bytes

// Serial port communications
void serialCommSetup(void);
PT_THREAD(commLoop(struct pt *pt));


// RF communications
void rfCommSetup(void);
PT_THREAD(rfCommLoop(struct pt *pt));


#endif