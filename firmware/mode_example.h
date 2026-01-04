#ifndef _MODE_EXAMPLE_H
#define _MODE_EXAMPLE_H

typedef struct ModeExample_t
{
	UpdateFunction Update;
	WirelessRXFunction WirelessRX;

	uint32_t frameNumber;
	int usTilNextSend;
} ModeExample;


void ModeExampleWirelessRX( uint8_t * txmac, uint8_t * message, int messageLength, int rssi )
{
	printf( "%02x:%02x:%02x:%02x:%02x:%02x:%3d %d:", txmac[0], txmac[1], txmac[2], txmac[3], txmac[4], txmac[5], rssi, messageLength );
	int i;
	for( i = 0; i < messageLength; i++ )
	{
		printf( "%02x ", message[i] );
	}
	printf( "\n" );
}

void ModeExampleLoop( void * mode, uint32_t deltaTime, uint32_t * pressures, uint32_t clickedMask, uint32_t lastClickMask )
{
	DoI2C();

	int i;
	ModeExample * m = (ModeExample *)mode;
 
	ssd1306_setbuf(0x00); // Clear screen
	funDigitalWrite( PIN_SCL, 1 );

	// Draw stuff to screen
	char st[128];
	sprintf( st, "%08x", (int)SysTick->CNT );
	ssd1306_drawstr_sz(0, 0, st, 1, 2 );

	sprintf( st, "%3d %d", (int)m->frameNumber, (int)pressures[3] );
	ssd1306_drawstr_sz(0, 24, st, 1, 2 );

	funDigitalWrite( PIN_SCL, 0 );
	int btn;
	for( btn = 0; btn < 3; btn++ )
	{
		int x = 32 + btn * 32;
		int y = 90;
		int p = pressures[btn]>>9;
		ssd1306_drawCircle( x, y, p, 1 );
		//ssd1306_fillCircle( x, y, p, 1 );
	}
	funDigitalWrite( PIN_SCL, 1 );

	for( i = 0; i < 3; i++ )
	{
		int x = (rand() % (128+80))-40;
		int y = (rand() % (128+80))-40;
		RenderBSprite( (i&1)?&bubble:&bubblemirror, x, y );
	}

	funDigitalWrite( PIN_SCL, 0 );
	// Output screen contents to OLED display.
	ssd1306_refresh();
	int32_t us = m->usTilNextSend -= deltaTime;
	if( us < 0 )
	{
		m->usTilNextSend = 400000;
		ISLERSend( "\xaa\xbb\xcc\xdd\xee\xff", 6 );
	}
}


void EnterExampleMode( ModeExample * m )
{
	memset( m, 0, sizeof(*m) );
	m->Update = ModeExampleLoop;
	m->WirelessRX = ModeExampleWirelessRX;
	m->frameNumber = 0;
	SetupI2C();
}


#endif

