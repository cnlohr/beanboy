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

void * game;

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
} gameUnion;

// Add it to this list.
void SelectMode( int modeNumber )
{
	switch(modeNumber)
	{
		case 0:
		default:
			EnterMenuMode( &gameUnion.menu );
			break;
		case 1:
			EnterExampleMode( &gameUnion.example );
			break;
		case 2:
			EnterTestMode( &gameUnion.test );
			break;
	}
}






void ISLERCallback( uint8_t * txmac, uint8_t * message, int messageLength, int rssi )
{
	if( gameUnion.template.WirelessRX )
		gameUnion.template.WirelessRX( txmac, message, messageLength, rssi );
}


int main()
{
	BeanboySetup();

	int frameno = 0;

	unsigned lastStart = 0;

	ISLERSetup( 14 );

	game = &gameUnion.template;

	SelectMode( 2 ); // Menu mode NOCHECKIN

	uint32_t lastClickedMask = 0;
	while(1)
	{
		uint32_t pressures[4];

		BeanBoyReadPressures( pressures );

		// TODO: Make this debounce.
		int i;
		uint32_t clickedMask = 0;
		for( i = 0; i < 3; i++ )
		{
			if( pressures[i] > 5000 ) clickedMask |= 1<<i;
		}

		unsigned now = SysTick->CNT;
		unsigned deltaUS = (now - lastStart)/48; // Wacky math to make sure we don't lose microseconds.
		if( gameUnion.template.Update )
		{
			gameUnion.template.Update( &gameUnion, deltaUS, pressures, clickedMask, lastClickedMask );
		}
		lastStart += deltaUS * 48;
		frameno++;

		// EMULATOR ONLY! BB19 contains a pointer to an incoming packet.
		while( BB->BB19 )
		{
			ISLER_BEANBOY_INTERNAL_CALLBACK();
			BB->BB19 = 0;
		}
/*

static void ISLER_BEANBOY_INTERNAL_CALLBACK()
{
	// The chip stores the incoming frame in LLE_BUF, defined in extralibs/iSLER.h
	uint8_t *frame = (uint8_t*)LLE_BUF;
	if( !iSLERCRCOK() ) return;

	int rssi = ReadRSSI();
	int len = frame[0];
	ISLERCallback( frame+2, frame+10, len+4, rssi );
}*/
		lastClickedMask = clickedMask;
	}
}

