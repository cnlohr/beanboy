/* Small example showing how to use the SWIO programming pin to 
   do printf through the debug interface */

#include "ch32fun.h"
#include <stdio.h>

#define SH1107_128x128

#define SSD1306_RST_PIN  PA6
#define SSD1306_CS_PIN   PA4
#define SSD1306_DC_PIN   PA10
#define SSD1306_MOSI_PIN PA7
#define SSD1306_SCK_PIN  PA5

//#define SSD1306_SOFT_SPI 1

#include "ssd1306_spi.h"
#include "ssd1306.h"


#include "beanboy.h"



void ISLERCallback( uint8_t * txmac, uint8_t * message, int messageLength, int rssi )
{
	printf( "%02x:%02x:%02x:%02x:%02x:%02x:%3d %d:", txmac[0], txmac[1], txmac[2], txmac[3], txmac[4], txmac[5], rssi, messageLength );
	int i;
	for( i = 0; i < messageLength; i++ )
	{
		printf( "%02x ", message[i] );
	}
	printf( "\n" );
}


int main()
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

	int frameno = 0;

	int debug = 0;

	unsigned start;

	ISLERSetup( 14 );

	while(1)
	{
		start = SysTick->CNT;

		uint32_t pressures[4];

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

		debug = SysTick->CNT - start;
		frameno++;
		
		ssd1306_setbuf(0x00); // Clear screen


#if 1
		// Draw stuff to screen
		char st[128];
		sprintf( st, "%08x", (int)SysTick->CNT );
		ssd1306_drawstr_sz(0, 0, st, 1, 2 );

		sprintf( st, "%3d %d", debug>>8, (int)pressures[3] );
		ssd1306_drawstr_sz(0, 24, st, 1, 2 );


		for( btn = 0; btn < 3; btn++ )
		{
			int x = 32 + btn * 32;
			int y = 90;
			int p = pressures[btn]>>9;
			ssd1306_drawCircle( x, y, p, 1 );
			//ssd1306_fillCircle( x, y, p, 1 );
		}
#endif

		//char st[128];
		//sprintf( st, "%08x\n", AES->some_reg4 );
		//ssd1306_drawstr_sz(0, 0, st, 1, 2 );
//		printf( "%08x %08x\n", ESIG1_ADDRESS[0], ESIG1_ADDRESS[1] );
		if( ( frameno & 0xff ) == 0 )
		{
			ISLERSend( "\xaa\xbb\xcc\xdd\xee\xff", 6 );
		}

		// Output screen contents to OLED display.
		ssd1306_refresh();
	}
}
