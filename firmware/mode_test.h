// TODO: Further improve icos_S4_BAM32!!

#ifndef _MODE_TEST_H
#define _MODE_TEST_H

#include "test/bunny.c"

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


//https://www.coranac.com/2009/07/sines/
// Based on isin_S4, but fiddled to get the right range.
// 0...2^32 for a full circle in,
//  Out = -2^30 to 2^30

int32_t isin_S4_BAM32(int32_t x) __HIGH_CODE;
int32_t isin_S4_BAM32(int32_t x)
{
	int c, x2, y;

	// I fiddled til I got what I wanted.

	const int zerotune = 1500;
	const int tune = 5000;// 360000;
	const int qN= 30, qA= 30, B=40755275+tune, C=7202343+tune-zerotune;
	const int qM= 16;
	const int qMO = (qM+(25-qA));

	c = x<<(30-qN);				// Semi-circle info into carry.
	x -= 1<<qN;					// sine -> cosine calc
	x= x<<(31-qN);				// Mask with PI
	x= x>>(31-qN);				// Note: SIGNED shift! (to qN)
	x= (x*(int64_t)x)>>(2*qN-qM);			// x=x^2 To Q14

	y = B - ((x*(int64_t)C)>>qM);				// B - x^2*C
	y = (1<<qA)-((x*(int64_t)y)>>(qMO));		// A - x^2*(B-x^2*C)

	y -= ((x*(int64_t)y*800)>>(32));

	//y = (1<<qA)-((x*(int64_t)y)>>(qMO));		// A - x^2*(B-x^2*C)

	return c>=0 ? y : -y;
}





static inline int32_t icos_S4(int32_t x) { return isin_S4( x+(1<<22)); }
static inline int32_t icos_S4_BAM32(int32_t x) { return isin_S4_BAM32( x+(1<<30)); }

int32_t mul3x24( int32_t a, int32_t b, int32_t c ) __HIGH_CODE;
int32_t mul3x24( int32_t a, int32_t b, int32_t c )
{
	// XXX TODO: Can this be improved?
	int32_t intermediate = (a * (int64_t)b) >> 24;
	return (intermediate * (int64_t)c) >> 24;
}

int32_t mul3x30( int32_t a, int32_t b, int32_t c ) __HIGH_CODE;
int32_t mul3x30( int32_t a, int32_t b, int32_t c )
{
	int32_t intermediate = (a * (int64_t)b) >> 30;
	return (intermediate * (int64_t)c) >> 30;
}

int32_t mul2x24( int32_t a, int32_t b ) __HIGH_CODE;
int32_t mul2x24( int32_t a, int32_t b )
{
	return (a * (int64_t)b) >> 24;
}

int32_t mul2x30( int32_t a, int32_t b ) __HIGH_CODE;
int32_t mul2x30( int32_t a, int32_t b )
{
	return (a * (int64_t)b) >> 30;
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

// Only really suitable for noramalization.
int32_t rsqrtx30_rough( int32_t a )
{
	// Computes 1/sqrt(n)
	int32_t x2 = a >> 1;

	// Would be great if we had a better first guess.
	int32_t y = 1<<30;

	int iter = 0;
	for( iter = 0; iter < 4; iter++ )
	{
		y = mul2x30(y, ( (3<<30) - mul3x30( x2, y, y ) ));
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


void QuatApplyQuat_Fix30( int32_t qout[4], int32_t q1[4], int32_t q2[4] ) __HIGH_CODE;
void QuatApplyQuat_Fix30( int32_t qout[4], int32_t q1[4], int32_t q2[4] )
{
	int32_t tmpw, tmpx, tmpy;
    tmpw    = mul2x30(q1[0], q2[0]) - mul2x30(q1[1], q2[1]) - mul2x30(q1[2], q2[2]) - mul2x30(q1[3], q2[3]);
    tmpx    = mul2x30(q1[0], q2[1]) + mul2x30(q1[1], q2[0]) + mul2x30(q1[2], q2[3]) - mul2x30(q1[3], q2[2]);
    tmpy    = mul2x30(q1[0], q2[2]) - mul2x30(q1[1], q2[3]) + mul2x30(q1[2], q2[0]) + mul2x30(q1[3], q2[1]);
    qout[3] = mul2x30(q1[0], q2[3]) + mul2x30(q1[1], q2[2]) - mul2x30(q1[2], q2[1]) + mul2x30(q1[3], q2[0]);
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


void QuatNormalize_Fix30(int32_t* qout, const int32_t* qin)
{
    int32_t qmag = mul2x30( qin[0], qin[0] ) + mul2x30( qin[1], qin[1] ) +
		mul2x30( qin[2], qin[2] ) + mul2x30( qin[3], qin[3] );
    qmag       = rsqrtx30_rough(qmag);

    qout[0]    = mul2x30( qin[0], qmag );
    qout[1]    = mul2x30( qin[1], qmag );
    qout[2]    = mul2x30( qin[2], qmag );
    qout[3]    = mul2x30( qin[3], qmag );
}


// Tested, working, before we devolve down into a world of madness.
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

void QuatFromEuler_Fix30( int32_t q[4], const int32_t e[3] ) __HIGH_CODE;
void QuatFromEuler_Fix30( int32_t q[4], const int32_t e[3] )
{
	int32_t pitchhalf = e[0];
	int32_t yawhalf   = e[1];
	int32_t rollhalf  = e[2];
	int32_t cr    = icos_S4_BAM32(pitchhalf);
	int32_t sr    = isin_S4_BAM32(pitchhalf); // Pitch: About X
	int32_t cp    = icos_S4_BAM32(yawhalf);
	int32_t sp    = isin_S4_BAM32(yawhalf); // Yaw:   About Y
	int32_t cy    = icos_S4_BAM32(rollhalf);
	int32_t sy    = isin_S4_BAM32(rollhalf); // Roll:  About Z

	q[0]        = mul3x30( cr, cp, cy ) + mul3x30( sr, sp, sy );
	q[1]        = mul3x30( sr, cp, cy ) - mul3x30( cr, sp, sy );
	q[2]        = mul3x30( cr, sp, cy ) + mul3x30( sr, cp, sy );
	q[3]        = mul3x30( cr, cp, sy ) - mul3x30( sr, sp, cy );
}

// TODO:
// 
//  Can we do things like cross products faster by adding and shifting in pairs or doing so less accurately?
//
//    i.e. _mulhs() to >> by 32 instead of 24?

void CrossProduct_Fix24(int32_t* p, const int32_t* a, const int32_t* b) __HIGH_CODE;
void CrossProduct_Fix24(int32_t* p, const int32_t* a, const int32_t* b)
{
    int32_t tx = mul2x24( a[1], b[2] ) - mul2x24( a[2], b[1] );
    int32_t ty = mul2x24( a[2], b[0] ) - mul2x24( a[0], b[2] );
    p[2]       = mul2x24( a[0], b[1] ) - mul2x24( a[1], b[0] );
    p[1]       = ty;
    p[0]       = tx;
}

void CrossProduct_Fix24_to_16_rough(int32_t* p, const int32_t* a, const int32_t* b) __HIGH_CODE;
void CrossProduct_Fix24_to_16_rough(int32_t* p, const int32_t* a, const int32_t* b)
{
    int32_t tx = _mulhs( a[1], b[2] ) - _mulhs( a[2], b[1] );
    int32_t ty = _mulhs( a[2], b[0] ) - _mulhs( a[0], b[2] );
    p[2]       = _mulhs( a[0], b[1] ) - _mulhs( a[1], b[0] );
    p[1]       = ty;
    p[0]       = tx;
}

// this takes about 5.48us.
void RotateVectorByQuaternion_Fix24(int32_t* pout, const int32_t* q, const int32_t* p) __HIGH_CODE;
void RotateVectorByQuaternion_Fix24(int32_t* pout, const int32_t* q, const int32_t* p)
{
    // return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
    int32_t iqo[3];
    CrossProduct_Fix24(iqo, q + 1 /*.xyz*/, p);
    iqo[0] += mul2x24(q[0], p[0]);
    iqo[1] += mul2x24(q[0], p[1]);
    iqo[2] += mul2x24(q[0], p[2]);
    int32_t ret[3];
    CrossProduct_Fix24(ret, q + 1 /*.xyz*/, iqo);
    pout[0] = ret[0] * 2 + p[0];
    pout[1] = ret[1] * 2 + p[1];
    pout[2] = ret[2] * 2 + p[2];
}


// Inaccurate version of rotate vector by quaternion, throws away about 8 bits of precision.
// This took about 4.5us before inlining the cross product.
// This takes about 3.5us @75MHz  (Before manually registerifying the inputs)
// After, it takes about 2.1us
void RotateVectorByQuaternion_Fix24_rough(int32_t* pout, const int32_t* q, const int32_t* p) __HIGH_CODE;
void RotateVectorByQuaternion_Fix24_rough(int32_t* pout, const int32_t* q, const int32_t* p)
{
    // return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
    int32_t iqoX, iqoY, iqoZ;
	int32_t q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
	int32_t p0 = p[0], p1 = p[1], p2 = p[2];

    iqoX = _mulhs( q2, p2 ) - _mulhs( q3, p1 );
    iqoY = _mulhs( q3, p0 ) - _mulhs( q1, p2 );
    iqoZ = _mulhs( q1, p1 ) - _mulhs( q2, p0 );

    iqoX += _mulhs(q0, p0);
    iqoY += _mulhs(q0, p1);
    iqoZ += _mulhs(q0, p2);

	// Fixup 16-bit to 24-bit fixed.
	iqoX <<= 8;
	iqoY <<= 8;
	iqoZ <<= 8;

	// We cannot combine the shifts, because q is still limited to a 24-bit number.
	pout[0] = ( (_mulhs( q2, iqoZ ) - _mulhs( q3, iqoY ))<<9 ) + p0;
    pout[1] = ( (_mulhs( q3, iqoX ) - _mulhs( q1, iqoZ ))<<9 ) + p1;
    pout[2] = ( (_mulhs( q1, iqoY ) - _mulhs( q2, iqoX ))<<9 ) + p2;
}

// slow for some reason
void RotateVectorByInverseOfQuaternion_Fix24(int32_t* pout, const int32_t* q, const int32_t* p) __HIGH_CODE;
void RotateVectorByInverseOfQuaternion_Fix24(int32_t* pout, const int32_t* q, const int32_t* p)
{
    // General note: Performing a transform this way can be about 20-30% slower than a well formed 3x3 matrix.
    // return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
    int32_t iqo[3];
    CrossProduct_Fix24(iqo, p, q + 1 /*.xyz*/);
    iqo[0] += mul2x24( q[0], p[0] );
    iqo[1] += mul2x24( q[0], p[1] );
    iqo[2] += mul2x24( q[0], p[2] );
    int32_t ret[3];
    CrossProduct_Fix24(ret, iqo, q + 1 /*.xyz*/);
    pout[0] = ret[0] * 2.0 + p[0];
    pout[1] = ret[1] * 2.0 + p[1];
    pout[2] = ret[2] * 2.0 + p[2];
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
	int32_t currentQuat[4] = { 1<<30, 0, 0, 0 };
	int32_t objectToWorldQuat[4] = { 1<<30, 0, 0, 0 };

	unsigned nextdeadline = SysTick->CNT;
	void * nextjump = &&lsm6_getcimu;

	cont:
		// Target 16.697 kHz
		while( ((int32_t)( SysTick->CNT - nextdeadline )) < 0  );
		nextdeadline += 2910;
		{

			funDigitalWrite( SSD1306_DC_PIN, FUN_LOW );
			funDigitalWrite( SSD1306_CS_PIN, FUN_LOW );

			R16_SPI0_TOTAL_CNT = 8;

			// TODO: Try other methods to turn off while loading.
			// Otherwise the display captures the current X and last frame's Y
			// The D9 method doesn't work super well.
			R8_SPI0_FIFO = 0xd9;
			R8_SPI0_FIFO = 0x0f;
			R8_SPI0_FIFO = 0xd3;
			if( (unsigned)(x_coord) > 127 || ((unsigned)(y_coord) > 127 ) ) y_coord = 0; // OOB = blackout
			R8_SPI0_FIFO = x_coord;
			R8_SPI0_FIFO = 0xdc;
			R8_SPI0_FIFO = y_coord;
			R8_SPI0_FIFO = 0xd9;
			R8_SPI0_FIFO = 0x11;

			// We've loaded up the point, now, figure out the next one.
			if( 0 )
			{
				x_coord = pixelNumber&0x7f;
				y_coord = (pixelNumber>>7)&0x7f;
			}
			else
			{
				static int percent_on_line;
				static int lineid;

				const int bunnylines = (sizeof(bunny_lines)/sizeof(bunny_lines[0])/2);
				if( 0 )
				{
					_rand_lfsr_update();
					lineid = _rand_lfsr%bunnylines;
					_rand_lfsr_update();
					percent_on_line = _rand_lfsr & 0xffff;
				}
				else
				{
					percent_on_line+=4829;
					if( percent_on_line >= 65536 )
					{
						percent_on_line -= 65536;
						if( 0 )
						{
							_rand_lfsr_update();
							lineid = _rand_lfsr%bunnylines;
						}
						else
						{
							lineid++;
							if( lineid == 22 ) lineid = 0;
						}
					}
				}

				int vid0 = bunny_lines[lineid*2+0]*3; // Todo: Optimize!
				int vid1 = bunny_lines[lineid*2+1]*3;
				int32_t laX = bunny_verts[vid0+0];
				int32_t laY = bunny_verts[vid0+1];
				int32_t laZ = bunny_verts[vid0+2];
				int32_t lbX = bunny_verts[vid1+0];
				int32_t lbY = bunny_verts[vid1+1];
				int32_t lbZ = bunny_verts[vid1+2];

				int invpercent = 65535-percent_on_line;

				int32_t vIn[3] = {
					(( laX * percent_on_line) + ( lbX * invpercent ))>>8,
					(( laY * percent_on_line) + ( lbY * invpercent ))>>8,
					(( laZ * percent_on_line) + ( lbZ * invpercent ))>>8 };

				int32_t vo[3];
				RotateVectorByQuaternion_Fix24_rough( vo, objectToWorldQuat, vIn );

				// Bunny is rotated in bunny-local coordinates, where
				// X, Y and Z are all rotated in 3D space, so z = -1..1


				x_coord = (vo[0]>>17) + 64;
				y_coord = (vo[1]>>17) + 64;
			}

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
		cimudat |= GetByte( 1 ) << 8ULL;
//		cimudat |= GetByte( 1 ) << 16ULL;
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
		nextjump = &&lsm6_getcimu;
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
funDigitalWrite( PIN_SCL, 1 );

			int32_t eulerAngles[3];
			const int32_t eulerScale = 200000000/16;// /8 if running at 1.66kHz instead of 208 Hz

			// XXX We actually counterrotate here so the quaternion presents object-to-world space.
			eulerAngles[0] =  _mulhs( cimu[0]<<16, eulerScale );
			eulerAngles[1] =  _mulhs( cimu[1]<<16, eulerScale );
			eulerAngles[2] =  _mulhs( cimu[2]<<16, eulerScale );
//printf( "%d %d\n", eulerAngles[0], cimu[0] );
			// Perform calibration here

			// Convert euler angles to quaternion.
			//fixQuatFromEuler( quaternion, eulerAngles );
			int32_t thisQ[4];
#if 0
			QuatFromEuler_Fix24( thisQ, eulerAngles );
			QuatApplyQuat_Fix24( currentQuat, currentQuat, thisQ );
			QuatNormalize_Fix24( currentQuat, currentQuat );
#endif

			QuatFromEuler_Fix30( thisQ, eulerAngles );
//printf( "%d %d %d -> %d %d %d %d\n", eulerAngles[0], eulerAngles[1], eulerAngles[2], thisQ[0], thisQ[1], thisQ[2], thisQ[3] );

			QuatApplyQuat_Fix30( currentQuat, currentQuat, thisQ );
		//	QuatNormalize_Fix30( currentQuat, currentQuat );

//printf( "%d %d %d %d -> %d %d %d %d\n", thisQ[0], thisQ[1], thisQ[2], thisQ[3], currentQuat[0], currentQuat[1], currentQuat[2], currentQuat[3] );
			objectToWorldQuat[0] = currentQuat[0]/64;
			objectToWorldQuat[1] =-currentQuat[1]/64;
			objectToWorldQuat[2] =-currentQuat[2]/64;
			objectToWorldQuat[3] =-currentQuat[3]/64;

funDigitalWrite( PIN_SCL, 0 );

			if( 0 )
			{
				static int32_t debugGyroAccum[3] = { 0 };
				debugGyroAccum[0] += eulerAngles[0];
				debugGyroAccum[1] += eulerAngles[1];
				debugGyroAccum[2] += eulerAngles[2];
				printf( "%d %d %d\n", (int) debugGyroAccum[0], (int)debugGyroAccum[1], (int)debugGyroAccum[2] );
			}
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
		nextjump = &&lsm6_getcimu;
		goto cont;
/*
	lsm6_process:
		nextjump = &&lsm6_1;
		goto cont;

	lsm6_1:
		nextjump = &&lsm6_2;
		goto cont;

	lsm6_2:
		nextjump = &&lsm6_getcimu;
		goto cont;
*/

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

	funDigitalWrite( SSD1306_DC_PIN, FUN_LOW );
	funDigitalWrite( SSD1306_CS_PIN, FUN_LOW );

	SendStart();
	SendByteNoAck( LSM6DS3_ADDRESS<<1 );
	SendByteNoAck( 0x11 );
	SendByteNoAck( 0x9d ); // 1.66kHz = 8d (does not work at all at 6.66, and works worse at 3.33 = 9d)
	SendStop();

	funDigitalWrite( SSD1306_CS_PIN, FUN_HIGH );

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

