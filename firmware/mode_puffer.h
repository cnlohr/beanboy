#ifndef _MODE_PUFFER_H
#define _MODE_PUFFER_H

typedef struct ModePuffer_t
{
	UpdateFunction Update;
	WirelessRXFunction WirelessRX;

    int16_t frameNum;
	int64_t curTime;
    int16_t popNum;
    int16_t curPoke;
} ModePuffer;

static const int64_t TimePerTick = 20000; 
static const int64_t TicksPerLoop = 32;
static const char* NewGameText1 = "Press R for a";
static const char* NewGameText2 = "New Game";
static const char* GameOverText = "Game Over!";

void ModePufferWirelessRX( uint8_t * txmac, uint8_t * message, int messageLength, int rssi )
{
	//Note to self, this is bad but it works
	ModePuffer *m = (ModePuffer*) game;
	//Include this in modes to handle incoming traffic from the internet
}

int getNewPufferPokes(){
	return (rand() % 15 + 4);
}
int getRandExtraPokes(){
	return (rand() % 7);
}

void ModePufferLoop( void * mode, uint32_t deltaTime, uint32_t * pressures, uint32_t clickedMask, uint32_t lastClickMask )
{
	ModePuffer * m = (ModePuffer *)mode;
	uint32_t newClickedMask = (clickedMask & ( clickedMask ^ lastClickMask ) );
    ssd1306_setbuf(0x00); // Clear screen

	if(m->curPoke < m->popNum){
		//Only update frame information if the game is still playing
		m->curTime += deltaTime;
		m->frameNum += (int)(m->curTime / TimePerTick);
		m->curTime = m->curTime % TimePerTick;
		m->curPoke += (int)(m->frameNum / TicksPerLoop);
		m->frameNum = m->frameNum % TicksPerLoop;
	}
    

	if(newClickedMask & 2){
		//Pressed the button
		m->curPoke++;
	}
	if(newClickedMask & 4 && m->curPoke >= m->popNum){
		m->curPoke = 0;
		m->frameNum = 0;
		m->curTime = 0;
		m->popNum = getNewPufferPokes();
	}

    if(m->popNum == 0){
		ssd1306_drawstr_sz(0,0,NewGameText1,1,1);
		ssd1306_drawstr_sz(0,20,NewGameText2,1,1);
	}else if(m->curPoke >= m->popNum){
		ssd1306_drawstr_sz(0,0,GameOverText,1,1);
		ssd1306_drawstr_sz(0,20,NewGameText1,1,1);
		ssd1306_drawstr_sz(0,40,NewGameText2,1,1);
	}else{
		//render the fish during the game
		if(  (float)((m->curPoke + getRandExtraPokes()) / m->popNum)  >  0.9f  ){
			RenderBSprite(&bubble, 32,32);
		}else{
			RenderBSprite(&fish1, 32, 32);
		}
		
	}
    
	char st[128];
	//sprintf( st, "%d", m->frameNum );
	//ssd1306_drawstr_sz(0, 60, st, 1, 2 );
	sprintf( st, "%d", m->curPoke );
	ssd1306_drawstr_sz(0, 80, st, 1, 2 );
	sprintf( st, "%d", m->popNum );
	ssd1306_drawstr_sz(0, 100, st, 1, 2 );
	
	ssd1306_refresh();
}


void EnterPufferMode( ModePuffer * m )
{
	memset( m, 0, sizeof(*m) );
	m->Update = ModePufferLoop;
	m->WirelessRX = ModePufferWirelessRX;
	m->frameNum = 0;

    m->popNum = 0;
    m->curPoke = 0;
	m->curTime = 0;
	m->frameNum = 0;
}

#endif