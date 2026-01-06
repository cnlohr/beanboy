#ifndef _MODE_SHUT_H
#define _MODE_SHUT_H

typedef struct ModeShut_t
{
	UpdateFunction Update;
	WirelessRXFunction WirelessRX;

	int score;
	int remainingRoll;

	int shutters[10];
	int selected;

	int gameState;

	uint64_t animTimer;
} ModeShut;

static const int shutterWidth = 22;
static const int shutterOffset = 3;
static const int openShutterHeight = 45;
static const int closedShutterHeight = 22;

static const char* deathText1 = "Game Over!";
static const char* deathText2 = "Your Score:";
static const char* deathText3 = "R to Restart";
static const char* deathText4 = "L to Leave";

static const char* winText1 = "You Shut";
static const char* winText1_2 = "the Box!";
static const char* winText2 = "C to Continue";
static const int   boxStaticTime = 60000;
static const int   boxMovingTime = 1250000;
static const int   boxMovePixels = 27;


int rollDice(){
	//more instructions but also more realistic!
	return (rand() % 6) + (rand() % 6) + 2; 
}
int legalMoveLeft(ModeShut* m){
	for(int i=0; i<m->remainingRoll; i++){
		if(m->shutters[i] == 0){
			return 1;
		}
	}
	return 0;
}
int victoryAchieved(ModeShut* m){
	for(int i=0; i<10; i++){
		if(m->shutters[i] == 0){
			return 0;
		}
	}
	return 1;
}

void ModeShutWirelessRX( uint8_t * txmac, uint8_t * message, int messageLength, int rssi )
{
	//Note to self, this is bad but it works
	ModeShut *m = (ModeShut*) game;
	//Include this in modes to handle incoming traffic from the internet
}

void ModeShutLoop( void * mode, uint32_t deltaTime, uint32_t * pressures, uint32_t clickedMask, uint32_t lastClickMask )
{
	ModeShut * m = (ModeShut *)mode;
	uint32_t newClickedMask = (clickedMask & ( clickedMask ^ lastClickMask ) );
    ssd1306_setbuf(0x00); // Clear screen
	char t[128];//Allows us to dynamically draw text

	switch (m->gameState)
	{
		case 0: //playing the game
			if(newClickedMask & 1){//Left
				do{
					m->selected = (m->selected + 9) % 10;
				}while(m->shutters[m->selected]!=0);
			}
			if(newClickedMask & 4){//Right
				do{
					m->selected = (m->selected + 1) % 10;
				}while(m->shutters[m->selected]!=0);
			}
			if(newClickedMask & 2){//Center, aka shutting
				if(m->remainingRoll >= (m->selected+1)){//if selected is 0 we're actually shutting the 1 spot
					m->shutters[m->selected] = 1;
					m->score += m->selected + 1;
					m->remainingRoll -= m->selected + 1;
				}

				//Check if that won the game
				if(victoryAchieved(m)){
					memset(m->shutters, 0, sizeof(int)*10);
					m->remainingRoll = rollDice();
					m->selected = 0;
					m->gameState = 2;
				}
				//Check if we need to reroll
				if(m->remainingRoll == 0){
					m->remainingRoll = rollDice();
				}

				//Check if we just lost the game
				if(legalMoveLeft(m)){
					while(m->shutters[m->selected]==1){
						m->selected = (m->selected + 1)%10;
					}
				}else{
					m->gameState = 1;
				}
			}

			//draw screen
			if(m->gameState != 0){break;}//Cancels out a flash frame of drawing the empty board before the box on hardware
			for(int x=0; x<5; x++){
				for(int y=0; y<2; y++){
					if(m->shutters[x+y*5]==0){
						ssd1306_drawRect(x*(shutterWidth+shutterOffset)+3, 30+y*(openShutterHeight+shutterOffset), shutterWidth, openShutterHeight, 1);

						sprintf(t, "%d", x + (y*5) + 1);
						ssd1306_drawstr_sz(
							(x==4&&y==1)?x*(shutterWidth+shutterOffset)+6:x*(shutterWidth+shutterOffset)+11,
							y*(openShutterHeight+shutterOffset)+35,
							t,1,1);
					}else{
						if(m->shutters[x+y*5]){//This box is shut
							ssd1306_drawRect(x*(shutterWidth+shutterOffset)+3, 30+(openShutterHeight-closedShutterHeight)+y*(closedShutterHeight+shutterOffset) , shutterWidth, closedShutterHeight, 1);
						}
					}
					
			
					

					if(m->selected == (x + y*5)){
						RenderBSprite(&finger, x*(shutterWidth+shutterOffset) + 6,
							y*(openShutterHeight+shutterOffset)+41);
					}
				}
			}
			sprintf(t, "%d", m->remainingRoll);
			ssd1306_drawstr_sz(40,5,t,1,2);
			break;

		case 1:
			ssd1306_drawstr_sz(24,10,deathText1,1,1);
			ssd1306_drawstr_sz(20,22,deathText2,1,1);

			sprintf(t,"%3d",m->score);
			ssd1306_drawstr_sz(30,48,t,1,3);

			ssd1306_drawstr_sz(17,90,deathText3,1,1);
			ssd1306_drawstr_sz(24,102,deathText4,1,1);

			if(newClickedMask & 4){//Right
				m->gameState = 0;
				memset(m->shutters, 0, sizeof(int)*10);
				m->remainingRoll = rollDice();
				m->selected = 0;
				m->score=0;
			}if(newClickedMask & 1){
				SelectMode(0);
			}
			break;
		case 2:
			
			if(m->animTimer <= (boxMovingTime+boxStaticTime)){
				m->animTimer += deltaTime;
			}

			if(m->animTimer < boxStaticTime){
				ssd1306_drawstr_sz(24,3,winText1,1,1);
				ssd1306_drawstr_sz(44,15,winText1_2,1,1);
				RenderBSprite(&closed_box, 0, 16+openShutterHeight-closedShutterHeight);
			}else if(m->animTimer < (boxStaticTime + boxMovingTime)){
				int movePix = boxMovePixels * (m->animTimer-boxStaticTime) / boxMovingTime;
				ssd1306_drawstr_sz(24,3,winText1,1,1);
				ssd1306_drawstr_sz(44,15,winText1_2,1,1);
				RenderBSprite(&closed_box, 0, 16+openShutterHeight-closedShutterHeight+movePix);
			}else{
				//The box is shut, display the happy box
				ssd1306_drawstr_sz(24,3,winText1,1,1);
				ssd1306_drawstr_sz(44,15,winText1_2,1,1);
				ssd1306_drawstr_sz(3,28,deathText3,1,1);
				ssd1306_drawstr_sz(3,40,deathText4,1,1);
				ssd1306_drawstr_sz(3,52,winText2,1,1);
				RenderBSprite(&closed_box, 0, 16+openShutterHeight-closedShutterHeight+boxMovePixels);
			}
			
			//ssd1306_drawstr_sz(3,3,winText1,1,1);

			if(newClickedMask&2){//Any button to continue
				m->gameState = 0;
				m->animTimer = 0;
			}else if(newClickedMask&4){
				m->score = 0;
				m->gameState = 0;
				m->animTimer = 0;
			}else if(newClickedMask & 1){
				SelectMode(0);
			}
			
			break;
	}
	
	ssd1306_refresh();
}


void EnterShutMode( ModeShut * m )
{
	memset( m, 0, sizeof(*m) );
	m->Update = ModeShutLoop;
	m->WirelessRX = ModeShutWirelessRX;
	seed(frameentropy);

	memset(m->shutters, 0, sizeof(int)*10);
	m->gameState = 0;
	m->score = 0;
	m->selected = 0;
	m->remainingRoll = rollDice();
	m->animTimer = 0;
	/*
	for(int i=2; i<10; i++){
		m->shutters[i] = 1;
	}*/
}

#endif