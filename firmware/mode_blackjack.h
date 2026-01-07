#ifndef _MODE_BlackJack_H
#define _MODE_BlackJack_H

typedef struct ModeBlackJack_t
{
	UpdateFunction Update;
	WirelessRXFunction WirelessRX;
} ModeBlackJack;

void ModeBlackJackWirelessRX( uint8_t * txmac, uint8_t * message, int messageLength, int rssi )
{
	//Note to self, this is bad but it works
	ModeBlackJack *m = (ModeBlackJack*) game;
	//Include this in modes to handle incoming traffic from the internet
}

void ModeBlackJackLoop( void * mode, uint32_t deltaTime, uint32_t * pressures, uint32_t clickedMask, uint32_t lastClickMask )
{
	ModeBlackJack * m = (ModeBlackJack *)mode;
	uint32_t newClickedMask = (clickedMask & ( clickedMask ^ lastClickMask ) );
    ssd1306_setbuf(0x00); // Clear screen
	char t[128];//Allows us to dynamically draw text
	RenderBSprite(&card_player, 0,0);
	ssd1306_drawstr(5,5,"Q",0);

	ssd1306_refresh();
}


void EnterBlackJackMode( ModeBlackJack * m )
{
	memset( m, 0, sizeof(*m) );
	m->Update = ModeBlackJackLoop;
	m->WirelessRX = ModeBlackJackWirelessRX;
}

#endif