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

void ModeTestLoop( void * mode, uint32_t deltaTime, uint32_t * pressures, uint32_t clickedMask, uint32_t lastClickMask )
{
	int i;
	ModeTest * m = (ModeTest *)mode;

	uint32_t fn = 0;

//	ssd1306_cmd(0xd3);
//	ssd1306_cmd((fn&0x7f));
//	ssd1306_cmd(0xdc);
//	ssd1306_cmd((fn>>7));

	while( 1 )
	{
		// Target 16.697 kHz
		spi_send_command_fast( fn&0x7f, fn>>7 );
		fn++;
		Delay_Us(30);

#if 1 
		if( (fn & 0xff) == 0 )
		{
			//spi_send_command_fast( 128, 128 );

			ConfigI2C();

			ProcessLSM6DS3();
			//ProcessQMC6309();

			funPinMode( PIN_SCL, GPIO_CFGLR_OUT_2Mhz_PP );
			funPinMode( PIN_SDA, GPIO_CFGLR_OUT_2Mhz_PP );
			funDigitalWrite( PA11, 0 ); // BS1 = 0 for SPI
			funDigitalWrite( PA10, 1 ); // D/C
		}
#endif
	}

	//Delay_Ms(5);
}


void EnterTestMode( ModeTest * m )
{
	memset( m, 0, sizeof(*m) );
	m->Update = ModeTestLoop;
	m->WirelessRX = ModeTestWirelessRX;


	sh1107_setup_for_scope();

}


#endif

