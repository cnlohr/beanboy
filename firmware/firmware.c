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

#include "assets.h"

#include "beanboy.h"

#define RANDOM_STRENGTH 2

#include "lib_rand.h"

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
	BeanboySetup();

	int frameno = 0;

	int debug = 0;

	unsigned start;

	ISLERSetup( 14 );

	while(1)
	{
		start = SysTick->CNT;

		uint32_t pressures[4];

		BeanBoyReadPressures( pressures );

		debug = SysTick->CNT - start;
		frameno++;
		
		ssd1306_setbuf(0x00); // Clear screen

		// Draw stuff to screen
		char st[128];
		sprintf( st, "%08x", (int)SysTick->CNT );
		ssd1306_drawstr_sz(0, 0, st, 1, 2 );

		sprintf( st, "%3d %d", debug>>8, (int)pressures[3] );
		ssd1306_drawstr_sz(0, 24, st, 1, 2 );

		int btn;
		for( btn = 0; btn < 3; btn++ )
		{
			int x = 32 + btn * 32;
			int y = 30;
			int p = pressures[btn]>>9;
			ssd1306_drawCircle( x, y, p, 1 );
			//ssd1306_fillCircle( x, y, p, 1 );
		}

		int sprites = 0;
		for( sprites = 0; sprites < 1; sprites++ )
		{
			int x = (rand() % (128+80))-40;
			int y = (rand() % (128+80))-40;
			RenderBSprite( &bubble, x, y );
		}

		if( ( frameno & 0xff ) == 0 )
		{
			ISLERSend( "\xaa\xbb\xcc\xdd\xee\xff", 6 );
		}

		// Output screen contents to OLED display.
		ssd1306_refresh();


		//DoI2C();
		//Delay_Ms(2000);
	}
}

