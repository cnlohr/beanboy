#ifndef _MODE_PUFFER_H
#define _MODE_PUFFER_H

typedef struct ModePuffer_t
{
	UpdateFunction Update;
	WirelessRXFunction WirelessRX;

    int16_t frameNum;
    int16_t popNum;
    int16_t curPoke;
} ModePuffer;
static const int64_t TimePerTick = 10000000; 

void ModePufferWirelessRX( uint8_t * txmac, uint8_t * message, int messageLength, int rssi )
{
	//Note to self, this is bad but it works
	ModePuffer *m = (ModePuffer*) game;
	//Include this in modes to handle incoming traffic from the internet
}

void ModePufferLoop( void * mode, uint32_t deltaTime, uint32_t * pressures, uint32_t clickedMask, uint32_t lastClickMask )
{
	ModePuffer * m = (ModePuffer *)mode;
    ssd1306_setbuf(0x00); // Clear screen

    m->frameNum++;

    for(int i=0; i<16; i++){
		RenderBSprite( &bubble, (i%4)*32,  (int)(i/4)*32 );
    }
    RenderBSprite(&fish1, m->frameNum-32, 0);


	ssd1306_refresh();
}


void EnterPufferMode( ModePuffer * m )
{
	memset( m, 0, sizeof(*m) );
	m->Update = ModePufferLoop;
	m->WirelessRX = ModePufferWirelessRX;
	m->frameNum = 0;

    m->popNum = (rand()%50) + 10;
    m->curPoke = 0;
}

#endif