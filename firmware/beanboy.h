#ifndef _BEANBOY_H
#define _BEANBOY_H

/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
// iSLER support
//
// BLE-like interface.

#define ISLER_CALLBACK ISLER_BEANBOY_INTERNAL_CALLBACK
static void ISLER_BEANBOY_INTERNAL_CALLBACK();
#include "iSLER.h"
#define PHY_MODE       PHY_1M
#define ACCESS_ADDRESS 0x8E89BED6 // the "BED6" address for BLE advertisements

void ISLERCallback( uint8_t * txmac, uint8_t * message, int messageLength, int rssi );

uint8_t iSLERChannel;

static void ISLER_BEANBOY_INTERNAL_CALLBACK()
{
	// The chip stores the incoming frame in LLE_BUF, defined in extralibs/iSLER.h
	uint8_t *frame = (uint8_t*)LLE_BUF;
	if( !CRCOK() ) return;

	int rssi = ReadRSSI();
	int len = frame[0];
	ISLERCallback( frame+2, frame+10, len+4, rssi );
}

static void ISLERSetup( int channel )
{
	iSLERChannel = channel;
	RFCoreInit(LL_TX_POWER_6_DBM);
	Frame_RX(ACCESS_ADDRESS, channel, PHY_MODE);
}

static void ISLERSend( const void * message, int messageLength )
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
// Auxiliary rendering sources bsprite
//
// This drawing function is very slow.  Consider optimizing.
//
static void RenderBSprite( const bsprite * spr, int outx, int outy ) __attribute__ ((noinline));
static void RenderBSprite( const bsprite * spr, int outx, int outy )
{
	int sw = spr->w;
	int sh = spr->h;
	int x, y;
	const uint32_t * bin = &spr->data[0];
	for( y = 0; y < sh; y++, outy++ )
	{
		uint8_t * bout = &ssd1306_buffer[(outy>>3)*SSD1306_W + outx];
		int boutshift = outy&7;
		int boutshiftmaskinv = ~(1<<boutshift);
		int tx = outx;
		int group;
		if( outy < 0 )
		{
			bin += sw;
			continue;
		}
		else if( outy >= SSD1306_H )
		{
			break;
		}

		for( group = 0; group < sw; group++ )
		{
			uint32_t input = *(bin++);
			for( x = 0; x < 16; x++, tx++, input >>= 1 )
			{
				if( tx < 0 || tx >= SSD1306_W ) continue;
				if( input & 1 )
				{
					bout[x] = ( bout[x] & boutshiftmaskinv ) | (((input>>16)&1)<< ( boutshift));
				}
			}
			bout += 16;
		}
	}
}



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

static void SetupADC(void)
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

static void BeanBoyReadPressures( uint32_t * pressures )
{
	int btn = 0;
	for( btn = 0; btn < 4; btn++ )
	{
		// try GPIO_CFGLR_IN_PUPD, GPIO_ModeIN_Floating, GPIO_CFGLR_OUT_10Mhz_PP as well
		funPinMode( PA3, GPIO_ModeIN_Floating );
		funPinMode( PA8, GPIO_ModeIN_Floating );
		funPinMode( PA9, GPIO_ModeIN_Floating );
		switch( btn )
		{
		case 0:
			funPinMode( PA8, GPIO_CFGLR_OUT_10Mhz_PP );
			funDigitalWrite( PA3, 1 );
			break;
		case 1:
			funPinMode( PA9, GPIO_CFGLR_OUT_10Mhz_PP );
			funDigitalWrite( PA8, 1 );
			break;
		case 2:
			funPinMode( PA3, GPIO_CFGLR_OUT_10Mhz_PP );
			funDigitalWrite( PA9, 1 );
			break;
		}

		if( *((uint32_t*)0x4fff0000) == 0xaaaaaaaa )
		{
			pressures[btn] = *((uint32_t*)(0x4fff0004+4*btn));
		}
		else
		{
			lastfifo = 0;
			EventRelease();
			int to = 4000;
			while( !lastfifo && --to );

			#define COEFFICIENT (const uint32_t)(FUNCONF_SYSTEM_CORE_CLOCK*(RESISTANCE*CAPACITANCE)*VREF*FIXEDPOINT_SCALE+0.5)
			int r = lastfifo - 2; // 2 cycles back.
			int vtot = COEFFICIENT/r + ((const uint32_t)(VREF*FIXEDPOINT_SCALE));
			pressures[btn] = vtot - 70;
		}
	}
}
/////////////////////////////////////////////////////////////////////////////////////////////////

static void BeanboySetup()
{
	SystemInit();

	funGpioInitAll(); // no-op on ch5xx

	funPinMode( PA11, GPIO_CFGLR_OUT_10Mhz_PP );
//	funDigitalWrite( PA11, 1 ); // BS1 = 1 for I2C
	funDigitalWrite( PA11, 0 ); // BS1 = 0 for SPI

	funPinMode( PA6, GPIO_CFGLR_OUT_10Mhz_PP );
	funDigitalWrite( PA6, 1 ); // RES
	funPinMode( PA4, GPIO_CFGLR_OUT_10Mhz_PP );
	funDigitalWrite( PA4, 0 ); // SCS
	funPinMode( PA10, GPIO_CFGLR_OUT_10Mhz_PP );
	funDigitalWrite( PA10, 0 ); // D/C

	ssd1306_rst();
	ssd1306_spi_init();
	ssd1306_init();
	ssd1306_setbuf(0x00);

	// Turbo-time
	ssd1306_cmd(0xD5);
	ssd1306_cmd(0xe0);


	ssd1306_cmd(0xc8);
	ssd1306_cmd(0xa1);

	SetupADC();
}

#endif

