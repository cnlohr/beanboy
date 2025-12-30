#ifndef _MODE_MENU_H
#define _MODE_MENU_H


typedef struct ModeMenu_t
{
	UpdateFunction Update;
	WirelessRXFunction WirelessRX;

	int selectedEntry;
} ModeMenu;

void ModeMenuLoop( void * mode, uint32_t deltaTime, uint32_t * pressures, uint32_t clickedMask, uint32_t lastClickMask )
{
	ModeMenu *m  = (ModeMenu*)mode;
	ssd1306_setbuf(0x00); // Clear screen

	uint32_t newClickedMask = (clickedMask & ( clickedMask ^ lastClickMask ) );

	int i;
	for( i = 1; i < sizeof(gameModes)/sizeof(gameModes[0]); i++ )
	{
		// Draw stuff to screen
		char st[128];
		sprintf( st, "%c%s", m->selectedEntry == i ? '>':' ', gameModes[i] );
		ssd1306_drawstr_sz(0, 20*(i-1), st, 1, 2 );
	}

	int ent = m->selectedEntry;
	if( newClickedMask & 1 )
	{
		ent--;
		if( ent < 1 ) ent = 1;
	}
	if( newClickedMask & 4 )
	{
		ent++;
		if( ent >= sizeof(gameModes)/sizeof(gameModes[0]) ) ent = sizeof(gameModes)/sizeof(gameModes[0])-1;
	}
	m->selectedEntry = ent;

	if( newClickedMask & 2 )
	{
		SelectMode( m->selectedEntry );
	}

	// Output screen contents to OLED display.
	ssd1306_refresh();
}

void EnterMenuMode( ModeMenu * m )
{
	memset( m, 0, sizeof( *m ) );
	m->Update = ModeMenuLoop;
	m->selectedEntry = 1;
}

#endif

