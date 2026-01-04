#ifndef _FIXEDMATH_H
#define _FIXEDMATH_H

#include <stdint.h>

#define MAX( x, y ) (( (x) < (y) )? (y) : (x) )
#define ABS( x ) (( (x) < (0) ) ? (-(x)) : (x) )

int32_t ApplyHysteresis( int32_t v, int32_t hysteresis ) __HIGH_CODE;
int32_t ApplyHysteresis( int32_t v, int32_t hysteresis )
{
	if( v < 0 )
	{
		if( v < -hysteresis )
			v += hysteresis;
		else
			v = 0;
	}
	else
	{
		if( v > hysteresis )
			v-= hysteresis; 
		else
			v = 0;
	}

	return v;
}

//https://www.coranac.com/2009/07/sines/
// Based on isin_S4, but fiddled to get the right range.
// 0...2^32 for a full circle in,
//  Out = -2^30 to 2^30
static int32_t isin_S4_BAM32(int32_t x) __HIGH_CODE;
static int32_t isin_S4_BAM32(int32_t x)
{
	int c, x2, y;

	// I fiddled til I got what I wanted

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

	// This was added by cnlohr, as a sort-of-5th order term
	y -= ((x*(int64_t)y*800)>>(32));

	//y = (1<<qA)-((x*(int64_t)y)>>(qMO));		// A - x^2*(B-x^2*C)

	return c>=0 ? y : -y;
}

int32_t isin_S4(int32_t x) __HIGH_CODE;

//https://www.coranac.com/2009/07/sines/
/// A sine approximation via a fourth-order cosine approx.
/// @param x   angle (with 2^24 units/circle)
/// @return	 Sine value (Q12)
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


static int32_t icos_S4_BAM32(int32_t x) { return isin_S4_BAM32( x+(1<<30)); }
static inline int32_t icos_S4(int32_t x) { return isin_S4( x+(1<<22)); }



static inline int32_t _mulhs( int32_t a, int32_t b )
{
	int32_t ret;
	asm volatile( "mulh %[out], %[a], %[b]" : [out]"=r"(ret) : [a]"r"(a), [b]"r"(b) : );
	return ret;
}

//https://www.coranac.com/2009/07/sines/

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

int32_t mul2x28( int32_t a, int32_t b ) __HIGH_CODE;
int32_t mul2x28( int32_t a, int32_t b )
{
	return (a * (int64_t)b) >> 28;
}

int32_t mul2x29( int32_t a, int32_t b ) __HIGH_CODE;
int32_t mul2x29( int32_t a, int32_t b )
{
	return (a * (int64_t)b) >> 29;
}

int32_t mul2x30( int32_t a, int32_t b ) __HIGH_CODE;
int32_t mul2x30( int32_t a, int32_t b )
{
	return (a * (int64_t)b) >> 30;
}


static inline int32_t dotm_mulhs3( const int32_t * a, const int32_t *b )
{
	return _mulhs( a[0], b[0] ) + _mulhs( a[1], b[1] ) + _mulhs( a[2], b[2] );
}

// Not super accurate, but pretty good.  Can go way faster if you have
// div (this assumes no div)

int32_t fixedsqrt_x30( int32_t i ) __HIGH_CODE;
int32_t fixedsqrt_x30( int32_t i )
{
	int sign = i < 0;

	if( sign ) i = -i;

	if( i == 0 ) return 0;
	int bclz = 31-__builtin_clz(i);

	// Make a best guess based on log(2)
	int x = 1 << (15 + ( bclz >> 1));

	// Add small bias.
	x += x>>1;

	int adj = x >> 1;
	int ir4 = i >> 2;
	int l;

	// Increase the iterations for more accuracy.
	// NOTE: There is always inaccuracy near zero.
	for( l = 0; l < 24; l++ )
	{
		int32_t comp = _mulhs( x, x );
		//if( comp == ir4 ) break;
		if( comp < ir4 )
			x += adj;
		else
			x -= adj;
		adj>>=1;
	}

	if( sign ) x = -x;

	return x;
}

// You can't do more than 28 bits here without the math falling apart.
// Only works within a = 2^28 +/- 2^26  
int32_t rsqrtx28_rough( int32_t a ) __HIGH_CODE;
int32_t rsqrtx28_rough( int32_t a )
{
	// Computes 1/sqrt(n)
	int32_t x2 = a >> 1;

	// Would be great if we had a better first guess.
	int32_t y = 1<<28;

	int iter = 0;
	for( iter = 0; iter < 4; iter++ )
	{
		y = mul2x28(y, ( (3<<27) - mul2x28(mul2x28( x2, y), y ) ));
	}

	return y;
}

// Only really suitable for noramalization.
// Only works within a = 2^24 +/- 2^22  
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

// More computationally expensive but provides 2-lsb-accurate results.
int32_t rsqrtx24_good( int32_t a )
{
	// Computes 1/sqrt(n)
	int32_t x2 = a >> 1;

	// Would be great if we had a better first guess.
	int32_t y = 1<<24;

	int iter = 0;
	for( iter = 0; iter < 12; iter++ )
	{
		int32_t ndiff = (((3<<23) - (int64_t)( ( ((x2*(int64_t)y) >> 24 ) * (int64_t)y ) >> 24 )));
		if( ndiff < 0 ) ndiff = 1<<23; // Can happen if initial value is too large.
		y = (   (int64_t)y *  ndiff ) >> 24;
	}

	return y;
}


void QuatApplyQuat_Fix24( int32_t qout[4], int32_t q1[4], int32_t q2[4] ) __HIGH_CODE;
void QuatApplyQuat_Fix24( int32_t qout[4], int32_t q1[4], int32_t q2[4] )
{
	int32_t tmpw, tmpx, tmpy;
	tmpw	= mul2x24(q1[0], q2[0]) - mul2x24(q1[1], q2[1]) - mul2x24(q1[2], q2[2]) - mul2x24(q1[3], q2[3]);
	tmpx	= mul2x24(q1[0], q2[1]) + mul2x24(q1[1], q2[0]) + mul2x24(q1[2], q2[3]) - mul2x24(q1[3], q2[2]);
	tmpy	= mul2x24(q1[0], q2[2]) - mul2x24(q1[1], q2[3]) + mul2x24(q1[2], q2[0]) + mul2x24(q1[3], q2[1]);
	qout[3] = mul2x24(q1[0], q2[3]) + mul2x24(q1[1], q2[2]) - mul2x24(q1[2], q2[1]) + mul2x24(q1[3], q2[0]);
	qout[2] = tmpy;
	qout[1] = tmpx;
	qout[0] = tmpw;
}


void QuatApplyQuat_Fix30( int32_t qout[4], int32_t q1[4], int32_t q2[4] ) __HIGH_CODE;
void QuatApplyQuat_Fix30( int32_t qout[4], int32_t q1[4], int32_t q2[4] )
{
	int32_t tmpw, tmpx, tmpy;
	tmpw	= mul2x30(q1[0], q2[0]) - mul2x30(q1[1], q2[1]) - mul2x30(q1[2], q2[2]) - mul2x30(q1[3], q2[3]);
	tmpx	= mul2x30(q1[0], q2[1]) + mul2x30(q1[1], q2[0]) + mul2x30(q1[2], q2[3]) - mul2x30(q1[3], q2[2]);
	tmpy	= mul2x30(q1[0], q2[2]) - mul2x30(q1[1], q2[3]) + mul2x30(q1[2], q2[0]) + mul2x30(q1[3], q2[1]);
	qout[3] = mul2x30(q1[0], q2[3]) + mul2x30(q1[1], q2[2]) - mul2x30(q1[2], q2[1]) + mul2x30(q1[3], q2[0]);
	qout[2] = tmpy;
	qout[1] = tmpx;
	qout[0] = tmpw;
}

void QuatNormalize_Fix24(int32_t* qout, const int32_t* qin)
{
	int32_t qmag = mul2x24( qin[0], qin[0] ) + mul2x24( qin[1], qin[1] ) +
		mul2x24( qin[2], qin[2] ) + mul2x24( qin[3], qin[3] );
	qmag	   = rsqrtx24_rough(qmag);

	qout[0]	= mul2x24( qin[0], qmag );
	qout[1]	= mul2x24( qin[1], qmag );
	qout[2]	= mul2x24( qin[2], qmag );
	qout[3]	= mul2x24( qin[3], qmag );
}


#if 0
// Only really suitable for noramalization.
int32_t rsqrtx30_rough_32( int32_t a )
{
	// Computes 1/sqrt(n)
	int32_t x2 = a >> 1;

	// Would be great if we had a better first guess.
	int32_t y = 1<<30;

	int iter = 0;
	for( iter = 0; iter < 4; iter++ )
	{
		y = mul2x30(y, ( (3<<28) - mul3x30( x2, y, y ) ));
	}

	return y;
}
#endif

void QuatNormalize_Fix30(int32_t* qout, const int32_t* qin)
{
	int32_t qmag = 
		_mulhs( qin[0], qin[0] ) +
		_mulhs( qin[1], qin[1] ) +
		_mulhs( qin[2], qin[2] ) +
		_mulhs( qin[3], qin[3] );
	int32_t qmagsqrtinverse = rsqrtx28_rough(qmag);
	qout[0]	= (qin[0] * (int64_t)qmagsqrtinverse) >> 28;
	qout[1]	= (qin[1] * (int64_t)qmagsqrtinverse) >> 28;
	qout[2]	= (qin[2] * (int64_t)qmagsqrtinverse) >> 28;
	qout[3]	= (qin[3] * (int64_t)qmagsqrtinverse) >> 28;
}


// Tested, working, before we devolve down into a world of madness.
void QuatFromEuler_Fix24( int32_t q[4], const int32_t e[3] ) __HIGH_CODE;
void QuatFromEuler_Fix24( int32_t q[4], const int32_t e[3] )
{
	int32_t pitchhalf = e[0]/2;
	int32_t yawhalf   = e[1]/2;
	int32_t rollhalf  = e[2]/2;
	int32_t cr	= icos_S4(pitchhalf);
	int32_t sr	= isin_S4(pitchhalf); // Pitch: About X
	int32_t cp	= icos_S4(yawhalf);
	int32_t sp	= isin_S4(yawhalf); // Yaw:   About Y
	int32_t cy	= icos_S4(rollhalf);
	int32_t sy	= isin_S4(rollhalf); // Roll:  About Z

	q[0]		= mul3x24( cr, cp, cy ) + mul3x24( sr, sp, sy );
	q[1]		= mul3x24( sr, cp, cy ) - mul3x24( cr, sp, sy );
	q[2]		= mul3x24( cr, sp, cy ) + mul3x24( sr, cp, sy );
	q[3]		= mul3x24( cr, cp, sy ) - mul3x24( sr, sp, cy );
}

void QuatFromEuler_Fix30( int32_t q[4], const int32_t e[3] ) __HIGH_CODE;
void QuatFromEuler_Fix30( int32_t q[4], const int32_t e[3] )
{
	int32_t pitchhalf = e[0];
	int32_t yawhalf   = e[1];
	int32_t rollhalf  = e[2];
	int32_t cr	= icos_S4_BAM32(pitchhalf);
	int32_t sr	= isin_S4_BAM32(pitchhalf); // Pitch: About X
	int32_t cp	= icos_S4_BAM32(yawhalf);
	int32_t sp	= isin_S4_BAM32(yawhalf); // Yaw:   About Y
	int32_t cy	= icos_S4_BAM32(rollhalf);
	int32_t sy	= isin_S4_BAM32(rollhalf); // Roll:  About Z

	q[0]		= mul3x30( cr, cp, cy ) + mul3x30( sr, sp, sy );
	q[1]		= mul3x30( sr, cp, cy ) - mul3x30( cr, sp, sy );
	q[2]		= mul3x30( cr, sp, cy ) + mul3x30( sr, cp, sy );
	q[3]		= mul3x30( cr, cp, sy ) - mul3x30( sr, sp, cy );
}

// TODO:
// 
//  Can we do things like cross products faster by adding and shifting in pairs or doing so less accurately?
//
//	i.e. _mulhs() to >> by 32 instead of 24?

void CrossProduct_Fix24(int32_t* p, const int32_t* a, const int32_t* b) __HIGH_CODE;
void CrossProduct_Fix24(int32_t* p, const int32_t* a, const int32_t* b)
{
	int32_t tx = mul2x24( a[1], b[2] ) - mul2x24( a[2], b[1] );
	int32_t ty = mul2x24( a[2], b[0] ) - mul2x24( a[0], b[2] );
	p[2]	   = mul2x24( a[0], b[1] ) - mul2x24( a[1], b[0] );
	p[1]	   = ty;
	p[0]	   = tx;
}

void CrossProduct_Fix24_to_16_rough(int32_t* p, const int32_t* a, const int32_t* b) __HIGH_CODE;
void CrossProduct_Fix24_to_16_rough(int32_t* p, const int32_t* a, const int32_t* b)
{
	int32_t tx = _mulhs( a[1], b[2] ) - _mulhs( a[2], b[1] );
	int32_t ty = _mulhs( a[2], b[0] ) - _mulhs( a[0], b[2] );
	p[2]	   = _mulhs( a[0], b[1] ) - _mulhs( a[1], b[0] );
	p[1]	   = ty;
	p[0]	   = tx;
}

void CrossProduct_Fix30OutFix29In(int32_t* p, const int32_t* a, const int32_t* b) __HIGH_CODE;
void CrossProduct_Fix30OutFix29In(int32_t* p, const int32_t* a, const int32_t* b)
{
	int32_t tx = mul2x28( a[1], b[2] ) - mul2x28( a[2], b[1] );
	int32_t ty = mul2x28( a[2], b[0] ) - mul2x28( a[0], b[2] );
	p[2]	   = mul2x28( a[0], b[1] ) - mul2x28( a[1], b[0] );
	p[1]	   = ty;
	p[0]	   = tx;
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
	pout[0] = ret[0] * 2 + p[0];
	pout[1] = ret[1] * 2 + p[1];
	pout[2] = ret[2] * 2 + p[2];
}


// Fix29 in is odd, but it's because there's ever so slight negative margin on the math if you have
// vectors of exactly 1<<30.  IT REQUIRES the incoming vectors to be normalized.
void CreateQuatFromTwoVectorRotation_Fix30OutFix29In(int32_t* qOut, const int32_t* v1, const int32_t* v2)
{
	int32_t ideal_up[3]  = {v1[0], v1[1], v1[2]};
	int32_t target_up[3] = {v2[0], v2[1], v2[2]};
	int32_t half[3]	  = {target_up[0] + ideal_up[0], target_up[1] + ideal_up[1], target_up[2] + ideal_up[2]};
	int32_t qmag = 
		_mulhs( half[0], half[0] ) +
		_mulhs( half[1], half[1] ) +
		_mulhs( half[2], half[2] );
	int32_t halfnormreq = rsqrtx24_good(qmag);
	half[0] = (half[0] * (int64_t)halfnormreq) >> (23); 
	half[1] = (half[1] * (int64_t)halfnormreq) >> (23);
	half[2] = (half[2] * (int64_t)halfnormreq) >> (23);

	CrossProduct_Fix30OutFix29In( qOut + 1, target_up, half );

	// This is super suss.  Should there be an isqrt?
	int32_t dotdiff = 
		(( target_up[0] * (int64_t)half[0] ) >> 28) +
		(( target_up[1] * (int64_t)half[1] ) >> 28) +
		(( target_up[2] * (int64_t)half[2] ) >> 28);
	qOut[0]	   = dotdiff;
}


#endif
