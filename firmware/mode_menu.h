#ifndef _MODE_MENU_H
#define _MODE_MENU_H


typedef struct ModeMenu_t
{
	UpdateFunction Update;
	WirelessRXFunction WirelessRX;

	int selectedEntry;

	int currentRelative; // in 1/(2^20ths) of a pixel.
	int animWidth; // Of box in fractions of a pixel.
	int zeroEntryLaunch;
	int frameNumber;
} ModeMenu;

void ModeMenuLoop( void * mode, uint32_t deltaTime, uint32_t * pressures, uint32_t clickedMask, uint32_t lastClickMask )
{
	ModeMenu *m  = (ModeMenu*)mode;
	ssd1306_setbuf(0x00); // Clear screen

	uint32_t newClickedMask = (clickedMask & ( clickedMask ^ lastClickMask ) );
	const int centerline_minus_half = 64 - 8;
	const int nrgameModesM1 = sizeof(gameModes)/sizeof(gameModes[0])-1;

	int ent = m->selectedEntry;

	int segm = ent-1;
	int i;
	for( i = -nrgameModesM1; i <= nrgameModesM1; i++ )
	{
		while( segm >= nrgameModesM1) segm -= nrgameModesM1-1; 
		int offset = m->currentRelative>>20;

		int yofs = offset + i * 18;
		int xofs = ((yofs*yofs)>>6)+2;

		if( i == 0 ) xofs += m->zeroEntryLaunch>>2;

		ssd1306_drawstr_sz( xofs, yofs + centerline_minus_half, gameModes[segm], 1, 2 );
		//printf( "%d\n", offset + i * 16 + centerline_minus_half );
		segm++;
	}

	ssd1306_drawRect( 0+m->zeroEntryLaunch>>2, centerline_minus_half-2, m->animWidth>>3, 18, 1 );

	m->currentRelative -= m->currentRelative>>3;

	if( m->zeroEntryLaunch )
	{
		m->zeroEntryLaunch += (m->zeroEntryLaunch>>3)+1;
	}

	int targetWidth = strlen( gameModes[m->selectedEntry] ) * 16 + 4;

	// Neat trick to do IIRs in fixed point.  animWidth is now 1<<4 too big.
	m->animWidth += targetWidth - (m->animWidth>>3);

/*
	int i;
	for( i = 1; i < sizeof(gameModes)/sizeof(gameModes[0]); i++ )
	{
		// Draw stuff to screen
		char st[128];
		sprintf( st, "%c%s", m->selectedEntry == i ? '>':' ', gameModes[i] );
		ssd1306_drawstr_sz(0, 20*(i-1), st, 1, 2 );
	}
*/

	if( !m->zeroEntryLaunch )
	{
		if( newClickedMask & 1 )
		{
			ent--;
			m->currentRelative-=16*(1<<20);
			if( ent < 1 ) ent = nrgameModesM1+1;
		}
		if( newClickedMask & 4 )
		{
			m->currentRelative+=16*(1<<20);
			ent++;
			if( ent >= nrgameModesM1 ) ent = 1;
		}

		m->selectedEntry = ent;

		if( (newClickedMask & 2) && (m->frameNumber > 2 ) )
		{
			m->zeroEntryLaunch = 1;
		}
	}
	else if( m->zeroEntryLaunch > 512 )
	{
		SelectMode( m->selectedEntry );
	}

	// Output screen contents to OLED display.
	ssd1306_refresh();
	m->frameNumber++;
}

void EnterMenuMode( ModeMenu * m )
{
	memset( m, 0, sizeof( *m ) );
	m->Update = ModeMenuLoop;
	m->selectedEntry = 1;
	m->frameNumber = 0;
	m->animWidth = 0;
	m->currentRelative = 0;
}

#endif

