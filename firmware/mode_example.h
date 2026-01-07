#ifndef _MODE_EXAMPLE_H
#define _MODE_EXAMPLE_H

typedef struct ModeExample_t
{
	UpdateFunction Update;
	WirelessRXFunction WirelessRX;

	uint32_t frameNumber;
	int usTilNextSend;

	#define BUBBLES 64
	int bubbleindex;
	int16_t bubblex[BUBBLES];
	int16_t bubbley[BUBBLES];

	int fishx;
	int fishy;
	int fishtargetx;
	int fishtargety;
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

	// Draw stuff to screen

//	srand( frameentropy );


	ssd1306_drawstr_sz(12, 0, "beanboy", 1, 2 );

	// Half the bubbles in front
	for( i = 0; i < BUBBLES; i+=2 )
	{
		int by = m->bubbley[i];
		if( m->bubbley[i] > -32 )
		{
			int xofs = (rand()&3) - 1;
			if( xofs == 2 ) xofs = 0;
			int x = m->bubblex[i] += xofs;
			int y = m->bubbley[i] = by - 1;
			RenderBSprite( (i&1)?&bubble:&bubblemirror, x, y );
		}
	}

	{
		// Tricky filter thingy.
		int fishx = m->fishx;
		int fishy = m->fishy;
		fishx = fishx - (fishx>>7) + m->fishtargetx;
		fishy = fishy - (fishy>>7) + m->fishtargety;

		if( m->fishtargetx<<7 < fishx )
			RenderBSprite( &fish0, fishx>>12, fishy>>12 );
		else
			RenderBSprite( &fish0flip, fishx>>12, fishy>>12 );

		m->fishx = fishx;
		m->fishy = fishy;

		if( (frameentropy & 0x7f) == 0 )
		{
			m->fishtargetx = ((rand()&0x3f)+16)<<5;
			m->fishtargety = ((rand()&0x3f)+16)<<5;
		}
	}

	// Half the bubbles behind.
	for( i = 1; i < BUBBLES; i+=2 )
	{
		int by = m->bubbley[i];
		if( m->bubbley[i] > -32 )
		{
			int xofs = (rand()&3) - 1;
			if( xofs == 2 ) xofs = 0;
			int x = m->bubblex[i] += xofs;
			int y = m->bubbley[i] = by - 1;
			RenderBSprite( (i&1)?&bubble:&bubblemirror, x, y );
		}
	}

//	char st[128];
//	sprintf( st, "%08x", (int)SysTick->CNT );
//	ssd1306_drawstr_sz(0, 0, st, 1, 2 );

//	sprintf( st, "%3d %d", (int)m->frameNumber, (int)pressures[3] );
//	ssd1306_drawstr_sz(0, 24, st, 1, 2 );

	int btn;
	for( btn = 0; btn < 3; btn++ )
	{
		int x = 32 + btn * 32;
		int y = 90;
		int p = pressures[btn]>>9;
		if( p > 1 )
			ssd1306_fillCircle( x, y, p-1, 0 );
		ssd1306_drawCircle( x, y, p, 1 );

		p-=3;
		if( p < 0 ) p = 0;
		if( p > (frameentropy&0x7f) )
		{
			// spawn bubble at button
			int idx = m->bubbleindex;
			m->bubbleindex = (idx+1)&(BUBBLES-1);
			m->bubbley[idx] = 128;
			m->bubblex[idx] = (rand()&0xf) + (btn * 32) + 8;
		}
	}




	// Output screen contents to OLED display.
	ssd1306_refresh();
	int32_t us = m->usTilNextSend -= deltaTime;
	if( us < 0 )
	{
		m->usTilNextSend = 400000;
		//ISLERSend( "\xaa\xbb\xcc\xdd\xee\xff", 6 );
	}
}


void EnterExampleMode( ModeExample * m )
{
	memset( m, 0, sizeof(*m) );
	m->Update = ModeExampleLoop;
	m->WirelessRX = ModeExampleWirelessRX;
	m->frameNumber = 0;

	m->fishtargetx = ((rand()&0x3f)+16)<<5;
	m->fishtargety = ((rand()&0x3f)+16)<<5;
	m->fishx = ((rand()&0x3f)+16)<<12;
	m->fishy = ((rand()&0x3f)+16)<<12;

	int i;
	for( i = 0; i < BUBBLES; i++ )
	{
		m->bubbley[i] = -100;
	}
	SetupI2C();
}


#endif

