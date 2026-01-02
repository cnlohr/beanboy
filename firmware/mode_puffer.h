#ifndef _MODE_PUFFER_H
#define _MODE_PUFFER_H

typedef struct ModePuffer_t
{
	UpdateFunction Update;
	WirelessRXFunction WirelessRX;

	
    int16_t frameNum;
	uint32_t spriteTime;
    int16_t popNum;
    int16_t curPoke;
	int16_t displayAdd;
} ModePuffer;

static const uint32_t TimePerTick = 85000; 
static const uint32_t TicksPerLoop = 21;
static const int16_t SinStepsPerFrame = 5; //This number should be coprime with ticksperloop if it's not itself prime
static const int16_t imageOffset = 12; //Fish Images are 90 pixels square, so the coordinate we want it drawn on is 45 pixels right and down from where we actually need to draw it
static const char* NewGameText1 = "-Press R to";
static const char* NewGameText2 = " play again!";
static const char* GameOverText = "Game Over!";
static const char* TitleText = "Pass the Puffer!";
static const char* Instructions1 = "-Press C to Poke";
static const char* Instructions2 = "-Get rid of him";
static const char* Instructions3 = " before he pops!";


void ModePufferWirelessRX( uint8_t * txmac, uint8_t * message, int messageLength, int rssi )
{
	//Note to self, this is bad but it works
	ModePuffer *m = (ModePuffer*) game;
	//Include this in modes to handle incoming traffic from the internet
}

int getNewPufferPokes(){
	return ((rand() % 35) + 20);
}
int getRandExtraPokes(){
	return (rand() % 5) + 10;
}

int badSin(int degrees){
	//0 returns 1
	//90 returns 0
	//180 returns -1
	//270 returns 0
	//We interpolate these values linearly because who has time for real lookup tables
	if(degrees >= 180){
		return -1 + ((degrees-180)/90);
	}else{
		return -1 + ((180-degrees)/90);
	}
}

void ModePufferLoop( void * mode, uint32_t deltaTime, uint32_t * pressures, uint32_t clickedMask, uint32_t lastClickMask )
{
	
	ModePuffer * m = (ModePuffer *)mode;
	uint32_t newClickedMask = (clickedMask & ( clickedMask ^ lastClickMask ) );
    ssd1306_setbuf(0x00); // Clear screen
	
	if(m->curPoke < m->popNum){
		//Only update frame information if the game is still playing
		m->spriteTime += deltaTime;
		while(m->spriteTime > TimePerTick){
			m->spriteTime -= TimePerTick;
			m->frameNum++;
		}
		while(m->frameNum > TicksPerLoop){
			m->frameNum -= TicksPerLoop;
			m->curPoke++;
		}
		//printf("%d,%d\n",m->spriteTime,deltaTime);
	}
	//printf("%d\n",deltaTime);
	
	if(newClickedMask & 2){
		//Pressed the button
		m->curPoke++;
	}
	if(newClickedMask & 4 && m->curPoke >= m->popNum){
		//Reset the game
		m->curPoke = 0;
		m->frameNum = 0;
		m->spriteTime = 0;
		m->popNum = getNewPufferPokes();
		m->displayAdd = getRandExtraPokes();
	}
	
    if(m->popNum == 0){
		ssd1306_drawstr_sz(5,0,TitleText,1,1);
		ssd1306_drawstr_sz(0,20,Instructions1,1,1);
		ssd1306_drawstr_sz(0,32,Instructions2,1,1);
		ssd1306_drawstr_sz(0,44,Instructions3,1,1);
		ssd1306_drawstr_sz(0,73,NewGameText1,1,1);
		ssd1306_drawstr_sz(0,85,NewGameText2,1,1);
	}else if(m->curPoke >= m->popNum){
		ssd1306_drawstr_sz(25,40,GameOverText,1,1);
		ssd1306_drawstr_sz(0,60,NewGameText1,1,1);
		ssd1306_drawstr_sz(0,72,NewGameText2,1,1);
	}else{
		int radialSteps = (m->frameNum * SinStepsPerFrame) % TicksPerLoop;
		int sinDegrees = radialSteps * 360  / TicksPerLoop;
		int cosDegrees = (sinDegrees + 90) % 360;
		int displayedPokes = (m->curPoke + m->displayAdd)*100/m->popNum;
		//printf("%d\n",displayedPokes);
		//render the fish during the game
		if(  displayedPokes  >  90  ){
			RenderBSprite(&fish2, 64+24*badSin(sinDegrees)-imageOffset,64+28*badSin(cosDegrees)-imageOffset);
		}else if(  displayedPokes  >  65  ){
			RenderBSprite(&fish1, 64+17*badSin(sinDegrees)-imageOffset,64+17*badSin(cosDegrees)-imageOffset);
		}else if(  displayedPokes  >  30  ){
			RenderBSprite(&fish0, 64+5*badSin(sinDegrees)-imageOffset,64+5*badSin(cosDegrees)-imageOffset);
		}else{
			RenderBSprite(&fish0, 64-imageOffset,64-imageOffset);
		}

	}
	
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
	m->spriteTime = 0;
	m->frameNum = 0;
	m->displayAdd = 0;

	seed(frameentropy);
}

#endif