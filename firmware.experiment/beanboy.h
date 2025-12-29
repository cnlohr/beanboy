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
	if( !iSLERCRCOK() ) return;

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
	memcpy( pkt_tx+2, (uint32_t*)ROM_CFG_MAC_ADDR, 6 );
	//memset( pkt_tx+2, 0xff, 6 );
	pkt_tx[8] = 0xff; // Normally should be 0x03, 0x19, 0x00, 0x00 for BLE
	pkt_tx[9] = 0xff; 
	memcpy( pkt_tx+10, message, messageLength );

	Frame_TX(ACCESS_ADDRESS, pkt_tx, messageLength+10, iSLERChannel, PHY_MODE);
	Frame_RX(ACCESS_ADDRESS, iSLERChannel, PHY_MODE);
}

const uint8_t * GetSelfMAC() { return (const uint8_t*)ROM_CFG_MAC_ADDR; }


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


#define PIN_SDA PA10
#define PIN_SCL PA11

#define DELAY1 ADD_N_NOPS(20) // Delay_Us(1);
#define DELAY2 ADD_N_NOPS(30) // Delay_Us(1);
#define DSCL_IHIGH  { funPinMode( PIN_SCL, GPIO_CFGLR_IN_PUPD ); funDigitalWrite( PIN_SCL, 1 ); } 
#define DSDA_IHIGH  { funDigitalWrite( PIN_SDA, 1 ); funPinMode( PIN_SDA, GPIO_CFGLR_OUT_2Mhz_PP ); } 
#define DSDA_INPUT  { funPinMode( PIN_SDA, GPIO_CFGLR_IN_PUPD ); funDigitalWrite( PIN_SDA, 1 ); } 
#define DSCL_OUTPUT { funDigitalWrite( PIN_SCL, 0 ); funPinMode( PIN_SCL, GPIO_CFGLR_OUT_2Mhz_PP );  } 
#define DSDA_OUTPUT { funDigitalWrite( PIN_SDA, 0 ); funPinMode( PIN_SDA, GPIO_CFGLR_OUT_2Mhz_PP );  } 
#define READ_DSDA    funDigitalRead( PIN_SDA )
#define I2CNEEDGETBYTE 1
//#define I2CNEEDSCAN    1

#define I2CNOSTATIC 1
#define I2CSTATICODE static __attribute__((section(".highcode")))

#include "custom_static_i2c.h"

#define LSM6DS3_ADDRESS 0x6a
#define QMC6309_ADDRESS 0x7c

int SetupRegisterMap( int address, const uint8_t * regptr, int regs, const char * name )
{
	int i;
	int fail = 0;
	for( i = 0; i < regs; i++ )
	{
		SendStart();
		int b = SendByte( address<<1 );
		if( b )
		{
			printf( "%s Cannot Configure @ %d\n", name, i );
			fail = 1;
			continue;
		}
		int a = *(regptr++);
		SendByte( a );
		int v = *(regptr++);
		b = SendByte( v );
		SendStop();
		printf( "%s %02xh = %02x [%s]\n", name, a, v, b?"FAIL":"OK" );
		fail |= b;
	}
	return fail;
}



void ProcessQMC6309()
{
	int tsamp = 0;

	SendStart();
	int r = SendByte( QMC6309_ADDRESS<<1 );
	SendByte( 0x09 );
	SendStart();
	SendByte( (QMC6309_ADDRESS<<1)|1 );
	uint32_t magstat = GetByte( 1 );
	SendStop();

	if( r ) 
	{
		printf( "Failed to read from QMC6309\n" );
		return;
	}

	if( (magstat & 1) )
	{
		SendStart();
		SendByte( QMC6309_ADDRESS<<1 );
		SendByte( 0x01 );
		SendStart();
		SendByte( (QMC6309_ADDRESS<<1)|1 );
		uint16_t x = GetByte(0);
		x |= GetByte(0) << 8;
		uint16_t y = GetByte(0);
		y |= GetByte(0) << 8;
		uint16_t z = GetByte(0);
		z |= GetByte(1) << 8;

		printf ("%04x %04x %04x\n", x, y, z);
		SendStop();
	}

}

void ProcessLSM6DS3() __attribute__((section(".highcode")));

void ProcessLSM6DS3()
{
	SendStart();
	int ra = SendByte( LSM6DS3_ADDRESS<<1 );
	int rb = SendByte( 0x3a );
	SendStart();
	int rc = SendByte( (LSM6DS3_ADDRESS<<1)|1 );

	uint32_t sw = 0;
	sw = GetByte( 0 );
	sw |= GetByte( 0 ) << 8ULL;
	sw |= GetByte( 1 ) << 16ULL;

	SendStop();

	if( sw & 0x4000 )
	{
		printf( "Full SW: %06x [%d %d %d]\n", (int)sw, ra, rb, rc );
		// Need to reset fifo.

		SetupRegisterMap( LSM6DS3_ADDRESS, 
			(const uint8_t[]){ 0x0a, 0x28, 0x0a, 0x2e },
			2, "LSM6DS3 Overflow" );
		return;
	}

	printf( "%06x ", (unsigned int)sw );
	//pattern |= GetByte( 0 ) << 8;

	int samp = 0;
	int samps = (sw & 0x7ff); // If LSMDS3 probably divide by 6?

	// practical maximum
	if( samps > 63 ) samps = 63;

	if( samps == 0 )
	{
		return;
	}

	SendStart();
	SendByte( LSM6DS3_ADDRESS<<1 );
	SendByte( 0x20 );
	SendStart();
	SendByte( (LSM6DS3_ADDRESS<<1)|1 );

	uint32_t ta = GetByte( 0 );
	ta |= GetByte( 0 )<<8; // Ignore FIFO status 4

	for( samp = 0; samp < samps; samp++ )
	{
		int i;
		for( i = 0; i < 6; i++ )
		{
			uint16_t val = GetByte( 0 );
			int end = (i==5 && samp == samps-1);
		//	if( end ) printf( "." );
			val |= GetByte( end ) << 8;
		//	if( samp == 0 )
		//		printf( "%04x ", val );
		}
		//printf( "\n" );
	}
	printf( "\n" );

	SendStop();
}

void SetupI2C()
{

#if 1
	// Actuall LSM6DSR
	const static uint8_t LSM6DS3Regmap[] = {
		0x12, 0x44, // CTRL3_C - unset reboot. + BDU
		0x10, 0x7a, // CTRL1_XL - 833Hz, +/-8g
		0x11, 0x74, // CTRL2_G - 833Hz, 1000dps

		0x0a, 0x28, // FIFO_CTRL5 - Disable FIFO (will re-enable)
		0x06, 0x60, // FIFO_CTRL1 - FIFO size.
		0x07, 0x00, // FIFO_CTRL2 - No temperature in FIFO.
		0x08, 0x09, // FIFO_CTRL3 - Put accel+gyro in FIFO.
		0x09, 0xaa, // FIFO_CTRL4 - TODO: Investigate.
		0x0a, 0x03, // FIFO_CTRL5 -  Continuous-to-FIFO mode: Continuous mode until trigger is deasserted, then FIFO mode;
		0x13, 0x00, // CTRL4_C - No extra stuff, don't stop on fth.
		0x15, 0x10, // CTRL6_C - High Performance disabled
		0x16, 0x00, // CTRL7_C - Just default settings.
		0x13, 0x01, // CTRL4_C - Stop on fth
	};
#else
	const static uint8_t LSM6DS3Regmap[] = {
		0x12, 0x44, // CTRL3_C - unset reboot. + BDU
		0x10, 0x5a, // CTRL1_XL - 208Hz, +/-8g
		0x11, 0x54, // CTRL2_G - 208Hz, 1000dps

		0x0a, 0x28, // FIFO_CTRL5 - Disable FIFO (will re-enable)
		0x06, 0x60, // FIFO_CTRL1 - FIFO size.
		0x07, 0x00, // FIFO_CTRL2 - No temperature in FIFO.
		0x08, 0x09, // FIFO_CTRL3 - Put accel+gyro in FIFO.
		0x09, 0x00, // FIFO_CTRL4 - No decimation TODO: I think this is wrong.
		0x0a, 0x2b, // FIFO_CTRL5 - 208Hz, Continuous mode FIFO. (was 0x2e) - if 12.5Hz (set to 0x0e)
		0x13, 0x00, // CTRL4_C - No extra stuff, don't stop on fth.
		0x15, 0x10, // CTRL6_C - High Performance.
		0x16, 0x00, // CTRL7_C - Just default settings.
		0x13, 0x01, // CTRL4_C - Stop on fth
	};
#endif

	ConfigI2C();
//	SetupRegisterMap( QMC6309_ADDRESS, (const uint8_t[]){ 0x0b, 0x80, 0x0b, 0x00 }, 2, "QMC6309 RESET" );


	SetupRegisterMap( LSM6DS3_ADDRESS, LSM6DS3Regmap, sizeof(LSM6DS3Regmap)/2, "LSM6DS3" );

	const static uint8_t QMC6309Regmap[] = {
		0x0b, 0x48, // CTRL2 = ODR = 200Hz
		0x0a, 0x19, // CTRL3 = OSR = 8, OSR2=16
	};
//	SetupRegisterMap( QMC6309_ADDRESS, QMC6309Regmap, sizeof(QMC6309Regmap)/2, "QMC6309" );


	funPinMode( PIN_SCL, GPIO_CFGLR_OUT_2Mhz_PP );
	funPinMode( PIN_SDA, GPIO_CFGLR_OUT_2Mhz_PP );
	funDigitalWrite( PA11, 0 ); // BS1 = 0 for SPI
	funDigitalWrite( PA10, 1 ); // D/C
}

void DoI2C()
{
	ConfigI2C();
	//Scan(); // 0x6A, 0x7C

	SendStop();	
	ProcessLSM6DS3();
//	ProcessQMC6309();

	funPinMode( PIN_SCL, GPIO_CFGLR_OUT_2Mhz_PP );
	funPinMode( PIN_SDA, GPIO_CFGLR_OUT_2Mhz_PP );
	funDigitalWrite( PA11, 0 ); // BS1 = 0 for SPI
	funDigitalWrite( PA10, 1 ); // D/C
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

	SetupI2C();

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

