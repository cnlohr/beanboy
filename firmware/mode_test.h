#ifndef _MODE_TEST_H
#define _MODE_TEST_H

typedef struct ModeTest_t
{
	UpdateFunction Update;
	WirelessRXFunction WirelessRX;
} ModeTest;

// Super duper X plot fast
void sh1107_setup_for_scope()
{
	ssd1306_setbuf(0x00); // Clear screen
	int i;
	for( i = 0; i < 128; i++ )
	{
		ssd1306_drawPixel(i, i, 1);
	}
	ssd1306_refresh();

	// Set mux ratio.
	ssd1306_cmd(0xa8);
	ssd1306_cmd(0x0 );

	// As fast as possible
	ssd1306_cmd(0xD5);
	ssd1306_cmd(0xf0);

	ssd1306_cmd(0xc8);
	ssd1306_cmd(0xa1);

	ssd1306_cmd(0xD9);
	ssd1306_cmd(0x11);

	// It seems that when operating after boot, and fast osc, you can go up to about 44MHz (/2)
	// But we slow down a tad.
	R8_SPI0_CLOCK_DIV = 3; 

	// Setup FIFO mode.
	R8_SPI0_CTRL_MOD &= ~RB_SPI_FIFO_DIR;
	R8_SPI0_CTRL_CFG |= RB_SPI_AUTO_IF | RB_SPI_DMA_ENABLE;
}

void spi_send_command_fast(uint32_t x, uint32_t y)
{
	funDigitalWrite( SSD1306_DC_PIN, FUN_LOW );
	funDigitalWrite( SSD1306_CS_PIN, FUN_LOW );
	R16_SPI0_TOTAL_CNT = 4;
	R8_SPI0_FIFO = 0xd3;
	R8_SPI0_FIFO = x;
	R8_SPI0_FIFO = 0xdc;
	R8_SPI0_FIFO = y;
	while( R16_SPI0_TOTAL_CNT );
	//while( !(R8_SPI0_INT_FLAG & RB_SPI_FREE)) { }
	funDigitalWrite( SSD1306_CS_PIN, FUN_HIGH );
}

void ModeTestWirelessRX( uint8_t * txmac, uint8_t * message, int messageLength, int rssi )
{
	printf( "%02x:%02x:%02x:%02x:%02x:%02x:%3d %d:", txmac[0], txmac[1], txmac[2], txmac[3], txmac[4], txmac[5], rssi, messageLength );
	int i;
	for( i = 0; i < messageLength; i++ )
	{
		printf( "%02x ", message[i] );
	}
	printf( "\n" );
}


void CoreLoop() __HIGH_CODE;

void CoreLoop()
{
	uint32_t tmp;
	uint32_t fn = 0;

//	ssd1306_cmd(0xd3);
//	ssd1306_cmd((fn&0x7f));
//	ssd1306_cmd(0xdc);
//	ssd1306_cmd((fn>>7));

	uint32_t cimutag;
	uint32_t cimudat;
	int16_t cimu[3] = { 0 };

	unsigned nextdeadline = SysTick->CNT;
	void * nextjump = &&lsm6_getcimu;

	cont:
		// Target 16.697 kHz
		while( ((int32_t)( SysTick->CNT - nextdeadline )) < 0  );
		nextdeadline += 2500;
		spi_send_command_fast( fn&0x7f, fn>>7 );
		fn++;
		goto *nextjump;

	lsm6_getcimu:
		SendStart();
		SendByteNoAck( LSM6DS3_ADDRESS<<1 );
		SendByteNoAck( 0x3a );
		SendStart();
		SendByteNoAck( (LSM6DS3_ADDRESS<<1)|1 );
		cimudat = GetByte( 0 );
		cimudat |= GetByte( 0 ) << 8ULL;
		cimudat |= GetByte( 1 ) << 16ULL;
		SendStop();
		if( cimudat & 0x4000 )
		{
			nextjump = &&lsm6_force_fifo_reset;
			goto cont;
		}
		else if( ( cimudat & 0x7ff ) > 0 )
		{
			nextjump = &&lsm6_pull0;
			goto cont;
		}
		nextjump = &&lsm6_1;
		goto cont;

	lsm6_force_fifo_reset:
		SendStart();
		SendByteNoAck( LSM6DS3_ADDRESS<<1 );
		SendByteNoAck( 0x0a );
		SendByteNoAck( 0x28 );
		SendStop();
		SendStart();
		SendByteNoAck( LSM6DS3_ADDRESS<<1 );
		SendByteNoAck( 0x0a );
		SendByteNoAck( 0x2e );
		SendStop();
		Delay_Ms(20);
		nextjump = &&lsm6_getcimu;
		goto cont;

	lsm6_pull0:
		SendStart();
		SendByteNoAck( LSM6DS3_ADDRESS<<1 );
		SendByteNoAck( 0x78 );
		SendStart();
		SendByteNoAck( (LSM6DS3_ADDRESS<<1)|1 );
		cimutag = GetByte( 0 ); // FIFO Tag
	//	GetByte( 0 )<<8; // Ignore FIFO status 4
		nextjump = &&lsm6_pull1;
		goto cont;

	lsm6_pull1:
		tmp = GetByte( 0 );
		cimu[0] = tmp | GetByte( 0 ) << 8;
		tmp = GetByte( 0 );
		cimu[1] = tmp | GetByte( 0 ) << 8;
		tmp = GetByte( 0 );
		cimu[2] = tmp | GetByte( 1 ) << 8;
		SendStop();
		nextjump = &&lsm6_pull2;
		goto cont;

	lsm6_pull2:
		cimutag >>= 3;
		if( cimutag == 1 )
		{
			//static int32_t ctot = 0;
			//static uint32_t last_time;
			//ctot += cimu[2];
			//uint32_t now = SysTick->CNT;
			//printf( "%d\n", now );
			//printf( "%d\n", now - last_time );
			//last_time = now;
			// Actual gyro = 

			//printf( "%d %d %d\n", cimu[0], cimu[1], cimu[2] );
			//Gyro
		}
		else if( cimutag == 2 )
		{
			// Accel
		}
		else if( cimutag == 3 )
		{
			// Temperature
		}
		else
		{
			printf( "Confusing tag: %d\n", (int)cimutag );
		}
		nextjump = &&lsm6_process;
		goto cont;

	lsm6_process:
		nextjump = &&lsm6_1;
		goto cont;

	lsm6_1:
		nextjump = &&lsm6_2;
		goto cont;

	lsm6_2:
		nextjump = &&lsm6_getcimu;
		goto cont;


}

void ModeTestLoop( void * mode, uint32_t deltaTime, uint32_t * pressures, uint32_t clickedMask, uint32_t lastClickMask )
{
	int i;
	ModeTest * m = (ModeTest *)mode;

	CoreLoop();
}


void EnterTestMode( ModeTest * m )
{
	memset( m, 0, sizeof(*m) );
	m->Update = ModeTestLoop;
	m->WirelessRX = ModeTestWirelessRX;


	sh1107_setup_for_scope();

}


#endif

