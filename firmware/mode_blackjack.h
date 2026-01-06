#ifndef _MODE_BlackJack_H
#define _MODE_BlackJack_H

typedef struct ModeBlackJack_t
{
	UpdateFunction Update;
	WirelessRXFunction WirelessRX;

	int money;
	int deck[52];
	int cardsDealt;
	int GameState;

	int PlayerHand[40];
	int PlayerTotals[4];
	int bets[4];
	int currentHand;
	int DealerHand[10];
	int DealerTotal;
} ModeBlackJack;

static const int cardsBeforeShuffle = 10;
static const int enteringMoney = 100;
static const char* Ranks[] = {"A","2","3","4","5","6","7","8","9","T","J","Q","K"};
static const int Points[] = {1,2,3,4,5,6,7,8,9,10,10,10,10};
static const char* Suits[] = {"S","H","D","C"};

void ShuffleDeck(ModeBlackJack* m){
	for (int i = 51; i>0; i--){
		int spot = rand()%i;
		int hold = m->deck[i];
		m->deck[i] = m->deck[spot];
		m->deck[spot] = hold;
	}
	m->cardsDealt = 0;
}

void DealPlayer(ModeBlackJack* m){
	for(int i=10 * m->currentHand; i<40; i++){
		if(m->PlayerHand[i] == -1){
			m->PlayerHand[i] = m->deck[m->cardsDealt];
			break;
		}
	}
	m->cardsDealt++;
}
void DealDealer(ModeBlackJack* m){
	for(int i=0; i<10; i++){
		if(m->DealerHand[i] == -1){
			m->DealerHand[i] = m->deck[m->cardsDealt];
			break;
		}
	}
	m->cardsDealt++;
}

int CalculateScores(ModeBlackJack* m){
	/*
		Assigns the score of each hand to the proper place in the m
	*/
	int total = 0;int softs = 0;
	for(i=0; i<10; i++){
		if(m->DealerHand[i] != -1){
			total += Points[m->DealerHand[i]%13];
			if(m->DealerHand[i]%13 == 0){
				softs++;
			}
		}else{//If we find -1, then the dealer is out of cards
			break;
		}
	}
	if(softs > 0 && total <= 11){
		total += 10;
		softs--;
	}else if(total > 21){
		total = -1;
	}
	m->DealerTotal = total;

	for(int hand = 0; hand < 4; hand++){
		total = 0;
		for(int i=10*hand; i<40; i++){
			if(m->PlayerHand[i] != -1){
				total+=Points[m->PlayerHand[i]%13];
			}else{
				break;
			}
		}
		if(total > 21){
			return hand;
		}
	}
	return 0;
}

void NextHand(ModeBlackJack* m){
	if(m->currentHand == 0){
		m->GameState = 2;
	}else{
		m->currentHand--;
	}
}


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
	
	switch (m->GameState)
	{
		case 0://Dealing cards
			//Add animation logic here
			DealPlayer(m);
			DealDealer(m);
			DealPlayer(m);
			DealDealer(m);

			m->bets[0] = 1;
			m->money -= 1;
			m->GameState = 1;
			break;


		case 1: //Player Agency
			if(newClickedMask & 1){ //Stand
				NextHand(m);
			}else if(newClickedMask & 2 && m->PlayerHand[10*m->currentHand + 2]==-1){ //Double
				//We check that it's only a 2-card hand first, you cannot double after you hit once
				m->bets[m->currentHand]++;
				m->money--;
				DealPlayer(m);
				NextHand(m);
			}else if(newClickedMask & 4){ //Hit
				DealPlayer(m);
				if(CheckForBust(m) == m->currentHand){
					NextHand(m);
				}//Else, same choices, hit or stand
			}


			break;
		case 2: //Resolve Dealer
			//This happens if the dealer does not have 21 on the table
			int hardTotal = Points[m->DealerHand[0]%13]+Points[m->DealerHand[1]%13];
			int softs=0;
			if(hardTotal >= 17 && hardTotal <= 21){
				m->GameState = 3;
			}
			if(m->DealerHand[0]%13==0){
				softs++;
			}
			if(m->DealerHand[1]%13==0){
				softs++;
			}
			for(int i=0; i<softs; i++){
				if(hardTotal + 10*i >= 17 && hardTotal + 10*i <= 21){
					m->GameState = 3;
				}
			}
			if(m->GameState == 2){
				//Dealer hits until a resolution is found
				for(int hit=3;hit<10;hit++){
					DealDealer(m);
					hardTotal += Points[m->DealerHand[hit]];
					if(hardTotal>21){
						m->GameState=4;
						break;
					}else{
						if(m->DealerHand[hit]%13 == 0){softs++;}
						for(int i=0; i<softs; i++){
							if(hardTotal + 10*i >= 17 && hardTotal + 10*i <= 21){
								m->GameState = 3;
								break;
							}
						}
					}
					if(m->GameState!=2){break;}
				}
			}
			break;

		case 3://Dealer stands
			int DealerTotal = 0;
			for(int i=0; i<10; i++){
				DealerTotal += Points[m->DealerHand[i]%13];
			}

			break;

		case 4://Dealer busts
			break;
	
		default:
			break;
	}

	ssd1306_refresh();
}


void EnterBlackJackMode( ModeBlackJack * m )
{
	memset( m, 0, sizeof(*m) );
	memset( m->PlayerHand, -1, sizeof(int)*40);
	memset( m->DealerHand, -1, sizeof(int)*10);
	memset( m->PlayerTotals, 0, sizeof(int)*4);
	memset( m->bets, 0, sizeof(int)*4);
	m->Update = ModeBlackJackLoop;
	m->WirelessRX = ModeBlackJackWirelessRX;
	m->currentHand = 0;
	m->money = enteringMoney;
	m->currentHand = 0;
	m->GameState = 0;
	m->DealerTotal=0;

	seed(frameentropy);
	
	for(int i=0; i<52; i++){
		m->deck[i] = i;
	}
	ShuffleDeck(m);

	
}

#endif