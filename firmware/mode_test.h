#ifndef _MODE_TEST_H
#define _MODE_TEST_H

typedef struct ModeTest_t
{
	UpdateFunction Update;
	WirelessRXFunction WirelessRX;
} ModeTest;

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

static inline int32_t _mulhs( int32_t a, int32_t b )
{
	int32_t ret;
	asm volatile( "mulh %[out], %[a], %[b]" : [out]"=r"(ret) : [a]"r"(a), [b]"r"(b) : );
	return ret;
}

//https://www.coranac.com/2009/07/sines/

/// A sine approximation via a fourth-order cosine approx.
/// @param x   angle (with 2^26 units/circle)
/// @return     Sine value (Q12)
int32_t isin_S4(int32_t x) __HIGH_CODE;
//https://www.coranac.com/2009/07/sines/
/// A sine approximation via a fourth-order cosine approx.
/// @param x   angle (with 2^24 units/circle)
/// @return     Sine value (Q12)
int32_t isin_S4(int32_t x)
{
	int c, x2, y;
	// qN = 13 -> 25
	// qA = 12 -> 24
/// = 26-bit unit circle, 24-bit output
///	static const int qN= 25, qA= 24, B=19900<<12, C=3516<<12; // TODO :Make better C/B
/// static const int qM= 20;

/// = 24-bit unit circle, 24-bit output
// c = 1-pi/4 => 0.214601837 * 1<<(qA+2)
// c = 14401685.493383168
// b = 2-pi/4 => 1.214601837 * (1<<26)
	const int tune = 120000; // Tuning manual, by cnlohr
	const int qN= 22, qA= 24, B=40755275+tune, C=7202343+tune;
	const int qM= 17;

	c= x<<(30-qN);				// Semi-circle info into carry.
	x -= 1<<qN;					// sine -> cosine calc
	x= x<<(31-qN);				// Mask with PI
	x= x>>(31-qN);				// Note: SIGNED shift! (to qN)
	x= (x*(int64_t)x)>>(2*qN-qM);			// x=x^2 To Q14

	y= B - ((x*(int64_t)C)>>qM);			// B - x^2*C
	y= (1<<qA)-((x*(int64_t)y)>>18);		// A - x^2*(B-x^2*C)

	// Added by cnlohr
	int retune = (x*(int64_t)y)>>24;
	y -= retune;
 
	return c>=0 ? y : -y;
}
static inline int32_t icos_S4(int32_t x) { return isin_S4( x+(1<<22)); }

int32_t mul3x24( int32_t a, int32_t b, int32_t c ) __HIGH_CODE;
int32_t mul3x24( int32_t a, int32_t b, int32_t c )
{
	// XXX TODO: Can this be improved?
	int32_t intermediate = (a * (int64_t)b) >> 24;
	return (intermediate * (int64_t)c) >> 24;
}

int32_t mul2x24( int32_t a, int32_t b ) __HIGH_CODE;
int32_t mul2x24( int32_t a, int32_t b )
{
	return (a * (int64_t)b) >> 24;
}

// Only really suitable for noramalization.
int32_t rsqrtx24_rough( int32_t a )
{
	// Computes 1/sqrt(n)
	int32_t x2 = a >> 1;

	// Would be great if we had a better first guess.
	int32_t y = 1<<24;

	int iter = 0;
	for( iter = 0; iter < 4; iter++ )
	{
		y = mul2x24(y, ( (3<<23) - mul3x24( x2, y, y ) ));
	}

	return y;
}

void QuatApplyQuat_Fix24( int32_t qout[4], int32_t q1[4], int32_t q2[4] ) __HIGH_CODE;
void QuatApplyQuat_Fix24( int32_t qout[4], int32_t q1[4], int32_t q2[4] )
{
	int32_t tmpw, tmpx, tmpy;
    tmpw    = mul2x24(q1[0], q2[0]) - mul2x24(q1[1], q2[1]) - mul2x24(q1[2], q2[2]) - mul2x24(q1[3], q2[3]);
    tmpx    = mul2x24(q1[0], q2[1]) + mul2x24(q1[1], q2[0]) + mul2x24(q1[2], q2[3]) - mul2x24(q1[3], q2[2]);
    tmpy    = mul2x24(q1[0], q2[2]) - mul2x24(q1[1], q2[3]) + mul2x24(q1[2], q2[0]) + mul2x24(q1[3], q2[1]);
    qout[3] = mul2x24(q1[0], q2[3]) + mul2x24(q1[1], q2[2]) - mul2x24(q1[2], q2[1]) + mul2x24(q1[3], q2[0]);
    qout[2] = tmpy;
    qout[1] = tmpx;
    qout[0] = tmpw;
}

void QuatNormalize_Fix24(int32_t* qout, const int32_t* qin)
{
    int32_t qmag = mul2x24( qin[0], qin[0] ) + mul2x24( qin[1], qin[1] ) +
		mul2x24( qin[2], qin[2] ) + mul2x24( qin[3], qin[3] );
    qmag       = rsqrtx24_rough(qmag);

    qout[0]    = mul2x24( qin[0], qmag );
    qout[1]    = mul2x24( qin[1], qmag );
    qout[2]    = mul2x24( qin[2], qmag );
    qout[3]    = mul2x24( qin[3], qmag );
}



void QuatFromEuler_Fix24( int32_t q[4], const int32_t e[3] ) __HIGH_CODE;
void QuatFromEuler_Fix24( int32_t q[4], const int32_t e[3] )
{
	int32_t pitchhalf = e[0]/2;
	int32_t yawhalf   = e[1]/2;
	int32_t rollhalf  = e[2]/2;
	int32_t cr    = icos_S4(pitchhalf);
	int32_t sr    = isin_S4(pitchhalf); // Pitch: About X
	int32_t cp    = icos_S4(yawhalf);
	int32_t sp    = isin_S4(yawhalf); // Yaw:   About Y
	int32_t cy    = icos_S4(rollhalf);
	int32_t sy    = isin_S4(rollhalf); // Roll:  About Z

	q[0]        = mul3x24( cr, cp, cy ) + mul3x24( sr, sp, sy );
	q[1]        = mul3x24( sr, cp, cy ) - mul3x24( cr, sp, sy );
	q[2]        = mul3x24( cr, sp, cy ) + mul3x24( sr, cp, sy );
	q[3]        = mul3x24( cr, cp, sy ) - mul3x24( sr, sp, cy );
}


void CoreLoop() __HIGH_CODE;


void CoreLoop()
{
	uint32_t tmp;
	uint32_t pixelNumber = 0;
	int x_coord;
	int y_coord;

	// IMU
	uint32_t cimutag;
	uint32_t cimudat;
	int16_t cimu[3] = { 0 };

	// IMU (updates)
	#define MATHFIXED 16
	int32_t debugGyroAccum[3] = { 0 };
	int32_t currentQuat[4] = { 1<<24, 0, 0, 0 };

	unsigned nextdeadline = SysTick->CNT;
	void * nextjump = &&lsm6_getcimu;

	cont:
		// Target 16.697 kHz
		while( ((int32_t)( SysTick->CNT - nextdeadline )) < 0  );
		nextdeadline += 2500;
		{

			funDigitalWrite( SSD1306_DC_PIN, FUN_LOW );
			funDigitalWrite( SSD1306_CS_PIN, FUN_LOW );

			R16_SPI0_TOTAL_CNT = 4;
			R8_SPI0_FIFO = 0xd3;
			if( (unsigned)(x_coord) > 127 || ((unsigned)(y_coord) > 127 ) ) y_coord = 0; // OOB = blackout
			R8_SPI0_FIFO = x_coord;
			R8_SPI0_FIFO = 0xdc;
			R8_SPI0_FIFO = y_coord;

			// Here, we have about 2us time to calculate the next point we want to jump to.
			x_coord = pixelNumber&0x7f;
			y_coord = (pixelNumber>>7) & 0x7f;

			while( R16_SPI0_TOTAL_CNT );
			//while( !(R8_SPI0_INT_FLAG & RB_SPI_FREE)) { }
			funDigitalWrite( SSD1306_CS_PIN, FUN_HIGH );
		}
		pixelNumber++;
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
			static int gyronum = 0; gyronum++;	
			// Based on the euler angles, apply a change to the rotation matrix.

			int32_t eulerAngles[3];
			const int32_t eulerScale = 2000000;
			eulerAngles[0] = _mulhs( cimu[0]<<16, eulerScale );
			eulerAngles[1] = _mulhs( cimu[1]<<16, eulerScale );
			eulerAngles[2] = _mulhs( cimu[2]<<16, eulerScale );

			// Perform calibration here

			// Convert euler angles to quaternion.
			//fixQuatFromEuler( quaternion, eulerAngles );
funDigitalWrite( PIN_SCL, 1 );

			int32_t thisQ[4];
			QuatFromEuler_Fix24( thisQ, eulerAngles );
funDigitalWrite( PIN_SCL, 0 );
			QuatApplyQuat_Fix24( currentQuat, currentQuat, thisQ );
funDigitalWrite( PIN_SCL, 1 );
QuatNormalize_Fix24( currentQuat, currentQuat );
funDigitalWrite( PIN_SCL, 0 );
			printf( "%10d%10d%10d%10d\n", (int)currentQuat[0], (int)currentQuat[1] , (int)currentQuat[2], (int)currentQuat[3] );

/*
			fixQuatFromEuler( quaternion, eulerAngles );

void mathEulerToQuat(float* q, const float* euler)
{
    float pitch = euler[0];
    float yaw   = euler[1];
    float roll  = euler[2];
    float cr    = cosf(pitch * 0.5);
    float sr    = sinf(pitch * 0.5); // Pitch: About X
    float cp    = cosf(yaw * 0.5);
    float sp    = sinf(yaw * 0.5); // Yaw:   About Y
    float cy    = cosf(roll * 0.5);
    float sy    = sinf(roll * 0.5); // Roll:  About Z
    q[0]        = cr * cp * cy + sr * sp * sy;
    q[1]        = sr * cp * cy - cr * sp * sy;
    q[2]        = cr * sp * cy + sr * cp * sy;
    q[3]        = cr * cp * sy - sr * sp * cy;
}*/

			debugGyroAccum[0] += eulerAngles[0];
			debugGyroAccum[1] += eulerAngles[1];
			debugGyroAccum[2] += eulerAngles[2];
			if( ( gyronum & 0xff ) == 0 )
				printf( "%d %d %d\n", (int) debugGyroAccum[0], (int)debugGyroAccum[1], (int)debugGyroAccum[2] );

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


	//sh1107_setup_for_scope();
	ssd1306_setbuf(0x00); // Clear screen
	int i;

	// Line 0 is clear.
	for( i = 1; i < 128; i++ )
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


#endif



#if 0

The attic

#if 0
//Cordic in 32 bit signed fixed point math
//Function is valid for arguments in range -pi/2 -- pi/2
//for values pi/2--pi: value = half_pi-(theta-half_pi) and similarly for values -pi---pi/2
//
// 1.0 = 21361414
// 1/k = 0.4769353200
// pi = 3.1415926536
//Constants
#define cordic_1K 0x009B74EC
#define half_pi 0x01FFFFFE
#define MUL 21361414.000000
#define CORDIC_NTAB 32
const int cordic_ctab [] = {0x00FFFFFF, 0x00972028, 0x004FD9C2, 0x0028888E, 0x0014586A, 0x000A2EBF, 0x000517B0, 0x00028BE2, 0x000145F2, 0x0000A2F9, 0x0000517C, 0x000028BE, 0x0000145F, 0x00000A2F, 0x00000517, 0x0000028B, 0x00000145, 0x000000A2, 0x00000051, 0x00000028, 0x00000014, 0x0000000A, 0x00000005, 0x00000002, 0x00000001, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, };

void cordic(int theta, int *s, int *c, int n) __HIGH_CODE;

void cordic(int theta, int *s, int *c, int n)
{
  int k, d, tx, ty, tz;
  int x=cordic_1K,y=0,z=theta;
  n = (n>CORDIC_NTAB) ? CORDIC_NTAB : n;
  for (k=0; k<n; ++k)
  {
    d = z>>31;
    //get sign. for other architectures, you might want to use the more portable version
    //d = z>=0 ? 0 : -1;
    tx = x - (((y>>k) ^ d) - d);
    ty = y + (((x>>k) ^ d) - d);
    tz = z - ((cordic_ctab[k] ^ d) - d);
    x = tx; y = ty; z = tz;
  }  
 *c = x; *s = y;
}
#endif
#endif

