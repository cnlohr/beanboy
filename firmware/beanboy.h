#ifndef _BEANBOY_H
#define _BEANBOY_H

/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
// iSLER support
//
// BLE-like interface.

#define ISLER_CALLBACK ISLER_BEANBOY_INTERNAL_CALLBACK
void ISLER_BEANBOY_INTERNAL_CALLBACK();
#include "iSLER.h"
#define PHY_MODE       PHY_1M
#define ACCESS_ADDRESS 0x8E89BED6 // the "BED6" address for BLE advertisements

void ISLERCallback( uint8_t * txmac, uint8_t * message, int messageLength, int rssi );

uint8_t iSLERChannel;

void ISLER_BEANBOY_INTERNAL_CALLBACK()
{
	// The chip stores the incoming frame in LLE_BUF, defined in extralibs/iSLER.h
	uint8_t *frame = (uint8_t*)LLE_BUF;
	if( !CRCOK() ) return;

	int rssi = ReadRSSI();
	int len = frame[0];
	ISLERCallback( frame+2, frame+10, len+4, rssi );
}

void ISLERSetup( int channel )
{
	iSLERChannel = channel;
	RFCoreInit(LL_TX_POWER_6_DBM);
	Frame_RX(ACCESS_ADDRESS, channel, PHY_MODE);
}

void ISLERSend( const void * message, int messageLength )
{
	__attribute__((aligned(4)))  uint8_t pkt_tx[messageLength+10];
	pkt_tx[0] = 0x02;
	pkt_tx[1] = 10+messageLength;
	memcpy( pkt_tx+2, (uint32_t*)ESIG1_ADDRESS, 6 );
	//memset( pkt_tx+2, 0xff, 6 );
	pkt_tx[8] = 0xff; // Normally should be 0x03, 0x19, 0x00, 0x00 for BLE
	pkt_tx[9] = 0xff; 
	memcpy( pkt_tx+10, message, messageLength );

	Frame_TX(ACCESS_ADDRESS, pkt_tx, messageLength+10, iSLERChannel, PHY_MODE);
	Frame_RX(ACCESS_ADDRESS, iSLERChannel, PHY_MODE);
}

const uint8_t * GetSelfMAC() { return (const uint8_t*)ESIG1_ADDRESS; }



/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
// FakeDC
//
// Fake digital-to-analog, by looking at the time constant on a capcaitor
// and waiting for the scmitt trigger flip.
//

#define CAPACITANCE 0.00000001
#define RESISTANCE  33000.0
#define VREF 1.75
#define FIXEDPOINT_SCALE 100

volatile uint32_t lastfifo = 0;

void TMR1_IRQHandler(void) __attribute__((interrupt))  __attribute__((section(".srodata")));
void TMR1_IRQHandler(void)
{
	R8_TMR_INT_FLAG = 2;
	lastfifo = R32_TMR_FIFO;
	funPinMode( PA2, GPIO_ModeOut_PP_20mA );
}


// The timing on the setup has to be tight.
void EventRelease(void) __attribute__((section(".srodata"))) __attribute__((noinline));
void EventRelease(void)
{
	R8_TMR_CTRL_MOD = 0b00000010; // Reset Timer
	R8_TMR_CTRL_MOD = 0b11000101; // Capture mode rising edge
	funPinMode( PA2, GPIO_ModeIN_Floating );
}

void SetupADC(void)
{
	R8_TMR_CTRL_MOD = 0b00000010; // All clear
	R32_TMR_CNT_END = 0x03FFFFFF; // Maximum possible counter size.
	R8_TMR_CTRL_MOD = 0b11000101; // Capture mode rising edge
	R8_TMR_INTER_EN = 0b10; // Capture event.

	R16_PIN_ALTERNATE_H |= 1<<6; // Map PA2 to CAP1 (could be PA7, PA4, or PA9) (see RB_TMR_PIN)

	NVIC_EnableIRQ(TMR1_IRQn);
	__enable_irq();

	funPinMode( PA2, GPIO_ModeOut_PP_20mA );
}

/////////////////////////////////////////////////////////////////////////////////////////////////




#endif

