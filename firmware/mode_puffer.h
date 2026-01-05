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
	int animTime;
} ModePuffer;

static const uint32_t TimePerTick = 85000; 
static const uint32_t TicksPerLoop = 21;
static const int16_t SinStepsPerFrame = 5; //This number should be coprime with ticksperloop if it's not itself prime
static const int16_t imageOffset = 24; //
static const int deathAnimationTime = 2000000;
static const char* NewGameText1 = "R to Play";
static const char* NewGameText1_1 = "R to Play Again";
static const char* NewGameText2 = "L to Leave";
static const char* DeathText = "You lose :(";
static const char* GameOverText = "Game Over!";
static const char* TitleText = "Pass the Puffer";
static const char* Instructions1 = "Press C to Poke";
static const char* Instructions2 = "then pass to the";
static const char* Instructions3 = "next player";


void ModePufferWirelessRX( uint8_t * txmac, uint8_t * message, int messageLength, int rssi )
{
	//Note to self, this is bad but it works
	ModePuffer *m = (ModePuffer*) game;
	//Include this in modes to handle incoming traffic from the internet
}

int getNewPufferPokes(){
	return ((rand() % 2) + 20);
}
int getRandExtraPokes(){
	return (rand() % 5) + 10;
}

int badSin(int degrees, int radius){
	//0 returns 1
	//90 returns 0
	//180 returns -1
	//270 returns 0
	//We interpolate these values linearly because who has time for real lookup tables
	if(degrees >= 180){
		return -radius + ((degrees-180)*radius/90);
	}else{
		return -radius + ((180-degrees)*radius/90);
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
		ssd1306_drawRect(1,1,126,126,1);
	}
	if(newClickedMask & 4 && m->curPoke >= m->popNum){
		//Reset the game
		m->curPoke = 0;
		m->frameNum = 0;
		m->spriteTime = 0;
		m->animTime = 0;
		m->popNum = getNewPufferPokes();
		m->displayAdd = getRandExtraPokes();
	}
	
    if(m->popNum == 0){
		ssd1306_drawstr_sz(5,2,TitleText,1,1);
		ssd1306_drawstr_sz(4,20,Instructions1,1,1);
		ssd1306_drawstr_sz(0,32,Instructions2,1,1);
		ssd1306_drawstr_sz(20,44,Instructions3,1,1);
		ssd1306_drawstr_sz(30,73,NewGameText1,1,1);
		ssd1306_drawstr_sz(26,85,NewGameText2,1,1);
	}else if(m->curPoke >= m->popNum){
		if(m->animTime < deathAnimationTime){
			m->animTime += deltaTime;
			ssd1306_drawstr_sz(25,40,DeathText,1,1);
			RenderBSprite(&fishDead, 32,60);
		}else{
			ssd1306_drawstr_sz(25,40,GameOverText,1,1);
			ssd1306_drawstr_sz(6,60,NewGameText1_1,1,1);
			ssd1306_drawstr_sz(26,72,NewGameText2,1,1);
			RenderBSprite(&fishDead,32,90);
		}
		
	}else{
		int radialSteps = (m->frameNum * SinStepsPerFrame) % TicksPerLoop;
		int sinDegrees = radialSteps * 360  / TicksPerLoop;
		int cosDegrees = (sinDegrees + 90) % 360;
		int displayedPokes = (m->curPoke + m->displayAdd)*100/m->popNum;
		//printf("%d\n",displayedPokes);
		//render the fish during the game
		if(  displayedPokes  >  90  ){
			RenderBSprite(&fish2, 64-imageOffset+badSin(sinDegrees,24),64-imageOffset+badSin(cosDegrees,24));
		}else if(  displayedPokes  >  65  ){
			RenderBSprite(&fish1, 64-imageOffset+badSin(sinDegrees,17),64-imageOffset+badSin(cosDegrees,17));
		}else if(  displayedPokes  >  30  ){
			RenderBSprite(&fish0, 64-imageOffset+badSin(sinDegrees,5),64-imageOffset+badSin(cosDegrees,5));
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
	m->animTime = 0;

	seed(frameentropy);
}

#endif