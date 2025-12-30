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

// Include mode here.
#include "lib_rand.h"
#include "mode_test.h"
#include "mode_example.h"

// Include name of mode here.
const char * gameModes[] = { "Menu", "Example", "Test" };

#include "mode_menu.h"

// Include its struct entry here.
union
{
	ModeTemplate template;
	ModeMenu menu;
	ModeExample example;
	ModeTest test;
} game;

// Add it to this list.
void SelectMode( int modeNumber )
{
	switch(modeNumber)
	{
		case 0:
		default:
			EnterMenuMode( &game.menu );
			break;
		case 1:
			EnterExampleMode( &game.example );
			break;
		case 2:
			EnterTestMode( &game.test );
			break;
	}
}






void ISLERCallback( uint8_t * txmac, uint8_t * message, int messageLength, int rssi )
{
	if( game.template.WirelessRX )
		game.template.WirelessRX( txmac, message, messageLength, rssi );
}


int main()
{
	BeanboySetup();

	int frameno = 0;

	unsigned start;

	ISLERSetup( 14 );

	SelectMode( 0 ); // Menu mode.

	uint32_t lastClickedMask = 0;
	while(1)
	{
		start = SysTick->CNT;

		uint32_t pressures[4];

		BeanBoyReadPressures( pressures );

		// TODO: Make this debounce.
		int i;
		uint32_t clickedMask = 0;
		for( i = 0; i < 3; i++ )
		{
			if( pressures[i] > 5000 ) clickedMask |= 1<<i;
		}

		unsigned deltaTimeTicks = SysTick->CNT - start;
		if( game.template.Update )
		{
			game.template.Update( &game, deltaTimeTicks, pressures, clickedMask, lastClickedMask );
		}
		frameno++;

		lastClickedMask = clickedMask;
	}
}

