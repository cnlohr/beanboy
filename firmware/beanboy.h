#ifndef _BEANBOY_H
#define _BEANBOY_H

// Forward declaration to push code into __HIGH_CODE which is really fast.
void ssd1306_drawPixel(uint32_t x, uint32_t y, int color) __HIGH_CODE;


#include "fixedmath.h"

/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
// misc functions.

static inline void Reboot() __attribute__((noreturn));
static inline void Reboot()
{
	asm volatile( ".option push\n.option norelax\njr x0\n.option pop" );
	__builtin_unreachable();
}


/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
// iSLER support
//
// BLE-like interface.

#define ISLER_CALLBACK ISLER_BEANBOY_INTERNAL_CALLBACK
static void ISLER_BEANBOY_INTERNAL_CALLBACK();
#include "iSLER.h"
#define PHY_MODE       PHY_1M
#define ACCESS_ADDRESS 0x8E89BED6 // the "BED6" address for BLE advertisements

void ISLERCallback( uint8_t * txmac, uint8_t * message, int messageLength, int rssi );

uint8_t iSLERChannel;

static void ISLER_BEANBOY_INTERNAL_CALLBACK()
{
	// The chip stores the incoming frame in LLE_BUF, defined in extralibs/iSLER.h
	uint8_t *frame = (uint8_t*)LLE_BUF;
	if( !iSLERCRCOK() ) return;

	int rssi = ReadRSSI();
	int len = frame[0];
	ISLERCallback( frame+2, frame+10, len+4, rssi );
}

static void ISLERSetup( int channel )
{
	iSLERChannel = channel;
	RFCoreInit(LL_TX_POWER_6_DBM);
	Frame_RX(ACCESS_ADDRESS, channel, PHY_MODE);
}

static void ISLERSend( const void * message, int messageLength )
{
	__attribute__((aligned(4)))  uint8_t pkt_tx[messageLength+10];
	pkt_tx[0] = 0x02;
	pkt_tx[1] = 10+messageLength;
	memcpy( pkt_tx+2, (uint32_t*)ROM_CFG_MAC_ADDR, 6 );
	//memset( pkt_tx+2, 0xff, 6 );
	pkt_tx[8] = 0xff; // Normally should be 0x03, 0x19, 0x00, 0x00 for BLE
	pkt_tx[9] = 0xff; 
	memcpy( pkt_tx+10, message, messageLength );

	Frame_TX(ACCESS_ADDRESS, pkt_tx, messageLength+10, iSLERChannel, PHY_MODE);
	Frame_RX(ACCESS_ADDRESS, iSLERChannel, PHY_MODE);
}

const uint8_t * GetSelfMAC() { return (const uint8_t*)ROM_CFG_MAC_ADDR; }


/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
// Auxiliary rendering sources bsprite
//
// This drawing function is very slow.  Consider optimizing.
//
static void RenderBSprite( const bsprite * spr, int outx, int outy ) __attribute__ ((noinline));
static void RenderBSprite( const bsprite * spr, int outx, int outy )
{
	int sw = spr->w;
	int sh = spr->h;
	int x, y;
	const uint32_t * bin = &spr->data[0];
	for( y = 0; y < sh; y++, outy++ )
	{
		uint8_t * bout = &ssd1306_buffer[(outy>>3)*SSD1306_W + outx];
		int boutshift = outy&7;
		int boutshiftmaskinv = ~(1<<boutshift);
		int tx = outx;
		int group;
		if( outy < 0 )
		{
			bin += sw;
			continue;
		}
		else if( outy >= SSD1306_H )
		{
			break;
		}

		for( group = 0; group < sw; group++ )
		{
			uint32_t input = *(bin++);
			for( x = 0; x < 16; x++, tx++, input >>= 1 )
			{
				if( tx < 0 || tx >= SSD1306_W ) continue;
				if( input & 1 )
				{
					bout[x] = ( bout[x] & boutshiftmaskinv ) | (((input>>16)&1)<< ( boutshift));
				}
			}
			bout += 16;
		}
	}
}



/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
// FakeDC
//
// Fake digital-to-analog, by looking at the time constant on a capcaitor
// and waiting for the scmitt trigger flip.
//

#define CAPACITANCE 0.00000001
#define RESISTANCE  33000.0
#define VREF 1.75
#define FIXEDPOINT_SCALE 100

volatile uint32_t lastfifo = 0;
volatile uint32_t frameentropy = 0;

void TMR1_IRQHandler(void) __attribute__((interrupt))  __attribute__((section(".highcode")));
void TMR1_IRQHandler(void)
{
	R8_TMR_INT_FLAG = 2;
	lastfifo = R32_TMR_FIFO;
	funPinMode( PA2, GPIO_ModeOut_PP_20mA );
}


// The timing on the setup has to be tight.
void EventRelease(void) __attribute__((section(".highcode"))) __attribute__((noinline));
void EventRelease(void)
{
	R8_TMR_CTRL_MOD = 0b00000010; // Reset Timer
	R8_TMR_CTRL_MOD = 0b11000101; // Capture mode rising edge
	funPinMode( PA2, GPIO_ModeIN_Floating );
}

static void SetupADC(void)
{
	R8_TMR_CTRL_MOD = 0b00000010; // All clear
	R32_TMR_CNT_END = 0x03FFFFFF; // Maximum possible counter size.
	R8_TMR_CTRL_MOD = 0b11000101; // Capture mode rising edge
	R8_TMR_INTER_EN = 0b10; // Capture event.

	R16_PIN_ALTERNATE_H |= 1<<6; // Map PA2 to CAP1 (could be PA7, PA4, or PA9) (see RB_TMR_PIN)

	NVIC_EnableIRQ(TMR1_IRQn);
	__enable_irq();

	funPinMode( PA2, GPIO_ModeOut_PP_20mA );
}

static void BeanBoyReadPressures( uint32_t * pressures )
{
	int btn = 0;
	for( btn = 0; btn < 4; btn++ )
	{
		// try GPIO_CFGLR_IN_PUPD, GPIO_ModeIN_Floating, GPIO_CFGLR_OUT_10Mhz_PP as well
		funPinMode( PA3, GPIO_ModeIN_Floating );
		funPinMode( PA8, GPIO_ModeIN_Floating );
		funPinMode( PA9, GPIO_ModeIN_Floating );
		switch( btn )
		{
		case 0:
			funPinMode( PA8, GPIO_CFGLR_OUT_10Mhz_PP );
			funDigitalWrite( PA3, 1 );
			break;
		case 1:
			funPinMode( PA9, GPIO_CFGLR_OUT_10Mhz_PP );
			funDigitalWrite( PA8, 1 );
			break;
		case 2:
			funPinMode( PA3, GPIO_CFGLR_OUT_10Mhz_PP );
			funDigitalWrite( PA9, 1 );
			break;
		}

		uint32_t p;

		if( *((uint32_t*)0x4fff0000) == 0xaaaaaaaa )
		{
			p = *((uint32_t*)(0x4fff0004+4*btn));
		}
		else
		{
			lastfifo = 0;
			EventRelease();
			int to = 4000;
			while( !lastfifo && --to );

			#define COEFFICIENT (const uint32_t)(FUNCONF_SYSTEM_CORE_CLOCK*(RESISTANCE*CAPACITANCE)*VREF*FIXEDPOINT_SCALE+0.5)
			int r = lastfifo - 2; // 2 cycles back.
			int vtot = COEFFICIENT/r + ((const uint32_t)(VREF*FIXEDPOINT_SCALE));
			p = vtot - 70;
		}

		pressures[btn] = p;

		// Junky entropy.
		uint32_t fe = frameentropy;
		fe += p;
		fe ^= fe << 10;
		fe ^= fe << 21;
		frameentropy = fe;
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////


#define PIN_SDA PA10
#define PIN_SCL PA11

/*
		*(&R32_PA_PD_DRV + OFFSET_FOR_GPIOB(pin)) &= ~(pin & ~PB);
		*(&R32_PA_PU + OFFSET_FOR_GPIOB(pin))     |= (pin & ~PB);
		*(&R32_PA_DIR + OFFSET_FOR_GPIOB(pin))    &= ~(pin & ~PB);
		break;
	case GPIO_ModeIN_PD:
		*(&R32_PA_PD_DRV + OFFSET_FOR_GPIOB(pin)) |= (pin & ~PB);
		*(&R32_PA_PU + OFFSET_FOR_GPIOB(pin))     &= ~(pin & ~PB);
		*(&R32_PA_DIR + OFFSET_FOR_GPIOB(pin))    &= ~(pin & ~PB);
		break;
	case GPIO_ModeOut_PP_5mA:
		*(&R32_PA_PD_DRV + OFFSET_FOR_GPIOB(pin)) &= ~(pin & ~PB);
		*(&R32_PA_DIR + OFFSET_FOR_GPIOB(pin))    |= (pin & ~PB);
		break;
	case GPIO_ModeOut_PP_20mA:
		*(&R32_PA_PD_DRV + OFFSET_FOR_GPIOB(pin)) |= (pin & ~PB);
		*(&R32_PA_DIR + OFFSET_FOR_GPIOB(pin))    |= (pin & ~PB);
		break;
*/

#define DELAY1 ADD_N_NOPS(9); //Delay_Us(1);
#define DELAY2 ADD_N_NOPS(9); //Delay_Us(1);
#define DSCL_IHIGH  { funDigitalWrite( PIN_SCL, 1 ); } 
#define DSDA_IHIGH  { funDigitalWrite( PIN_SDA, 1 ); } 
#define DSDA_INPUT  { \
	/*funPinMode( PIN_SDA, GPIO_CFGLR_IN_PUPD );*/ \
	\
	\
	 funDigitalWrite( PIN_SDA, 1 ); } 

#define DSDA_DONE_INPUT {  /* funPinMode( PIN_SDA, GPIO_CFGLR_OUT_2Mhz_PP );  */ \
	\
	\
	}
#define DSCL_OUTPUT { funDigitalWrite( PIN_SCL, 0 ); } 
#define DSDA_OUTPUT { funDigitalWrite( PIN_SDA, 0 ); } 
#define READ_DSDA    funDigitalRead( PIN_SDA )
#define I2CNEEDGETBYTE 1
//#define I2CNEEDSCAN    1

#define I2CCODE __HIGH_CODE

#include "custom_i2c.h"

#define I2CSTATICODE static __attribute__((section(".highcode")))

#define LSM6DS3_ADDRESS 0x6a
#define QMC6309_ADDRESS 0x7c

int SetupRegisterMap( int address, const uint8_t * regptr, int regs, const char * name )
{
	int i;
	int fail = 0;
	for( i = 0; i < regs; i++ )
	{
		SendStart();
		int b = SendByte( address<<1 );
		if( b )
		{
			printf( "%s Cannot Configure @ %d\n", name, i );
			fail = 1;
			continue;
		}
		int a = *(regptr++);
		SendByte( a );
		int v = *(regptr++);
		b = SendByte( v );
		SendStop();
		//printf( "%s %02xh = %02x [%s]\n", name, a, v, b?"FAIL":"OK" );
		fail |= b;
	}
	return fail;
}



void ProcessQMC6309()
{
	SendStart();
	int r = SendByte( QMC6309_ADDRESS<<1 );
	SendByte( 0x09 );
	SendStart();
	SendByte( (QMC6309_ADDRESS<<1)|1 );
	uint32_t magstat = GetByte( 1 );
	SendStop();

	if( r ) 
	{
		printf( "Failed to read from QMC6309\n" );
		return;
	}

	if( (magstat & 1) )
	{
		SendStart();
		SendByte( QMC6309_ADDRESS<<1 );
		SendByte( 0x01 );
		SendStart();
		SendByte( (QMC6309_ADDRESS<<1)|1 );
		uint16_t x = GetByte(0);
		x |= GetByte(0) << 8;
		uint16_t y = GetByte(0);
		y |= GetByte(0) << 8;
		uint16_t z = GetByte(0);
		z |= GetByte(1) << 8;

		printf ("%04x %04x %04x\n", x, y, z);
		SendStop();
	}

}

// Should expose.
int32_t imuViewQuatFix24[4] = { 1<<24, 0, 0, 0 };
int32_t imuAccelUpFix29Norm[3] = { 0 };
int32_t imuAccelUpFix24[3] = { 0 };
int32_t imuViewQuatFix30[4] = { 1<<30, 0, 0, 0 };

void ProcessLSM6DS3();
void ProcessLSM6DS3()
{
	int32_t corrective_quaternion[4]; // Internal, for gravity updates.
	int32_t what_we_think_is_up[3];
	int32_t gyroBias[3] = { 0, 0, 0 };
	int gyronum = 0;
	static int accelnum = 0;

resample:

	SendStart();
	SendByteNoAck( LSM6DS3_ADDRESS<<1 );
	SendByteNoAck( 0x3a );
	SendStart();
	SendByteNoAck( (LSM6DS3_ADDRESS<<1)|1 );

	uint32_t sw = 0;
	sw = GetByte( 0 );
	sw |= GetByte( 1 ) << 8ULL;

	SendStop();

	if( sw & 0x4000 )
	{
		printf( "Full SW: %06x\n", (int)sw );
		// Need to reset fifo.

		SetupRegisterMap( LSM6DS3_ADDRESS, 
			(const uint8_t[]){ 0x0a, 0x28, 0x0a, 0x2e },
			2, "LSM6DS3 Overflow" );
		return;
	}
	else if( ( sw & 0x7ff ) <= 0 )
	{
		return;
	}

	SendStart();
	SendByteNoAck( LSM6DS3_ADDRESS<<1 );
	SendByteNoAck( 0x78 );
	SendStart();
	SendByteNoAck( (LSM6DS3_ADDRESS<<1)|1 );
	uint32_t cimutag = GetByte( 0 ); // FIFO Tag
	uint32_t cimu[3];
	uint32_t tmp = GetByte( 0 );
	cimu[0] = tmp | GetByte( 0 ) << 8;
	tmp = GetByte( 0 );
	cimu[1] = tmp | GetByte( 0 ) << 8;
	tmp = GetByte( 0 );
	cimu[2] = tmp | GetByte( 1 ) << 8;
	SendStop();

	cimutag >>= 3;
	if( cimutag == 1 )
	{
		gyronum++;	
		// Based on the euler angles, apply a change to the rotation matrix.

		funDigitalWrite( PIN_SCL, 1 );

		int32_t eulerAngles[3];
		const int32_t eulerScale = 15500000*64;// XXX Change me when you change IMU update rate.

		// Step 9: validation.  Make sure your gyroBias = eulerScale * 2^16 * 200,
		// and in the correct directions and doesn't change as you rotate the system.
		//cimu[0] += 200; cimu[1] += 200; cimu[2] += 200;

		// STEP 1:  Visually inspect the gyro values.
		// STEP 2:  Integrate the gyro values, verify they are correct.

		// We bring the IMU into our world coordinate frame here. 
		// YOU MAY NEED TO add negatives to specific terms here.
		eulerAngles[0] = -_mulhs( cimu[0]<<16, eulerScale ) + gyroBias[0];
		eulerAngles[1] =  _mulhs( cimu[1]<<16, eulerScale ) + gyroBias[1];
		eulerAngles[2] = -_mulhs( cimu[2]<<16, eulerScale ) + gyroBias[2];

		int32_t thisQ[4];

		// STEP 3:  Integrate gyro values into a quaternion.
		// This step is validated by working with just one axis at a time
		// then apply a coordinate frame to ld->fqQuat and validate that it is
		// correct.

		QuatFromEuler_Fix30( thisQ, eulerAngles );
		QuatApplyQuat_Fix30( imuViewQuatFix30, imuViewQuatFix30, thisQ );
		QuatNormalize_Fix30( imuViewQuatFix30, imuViewQuatFix30 );

		imuViewQuatFix24[0] = imuViewQuatFix30[0]>>6;
		imuViewQuatFix24[1] =-imuViewQuatFix30[1]>>6;
		imuViewQuatFix24[2] =-imuViewQuatFix30[2]>>6;
		imuViewQuatFix24[3] =-imuViewQuatFix30[3]>>6;

		funDigitalWrite( PIN_SCL, 0 );

#if 0
		{
			static int32_t debugGyroAccum[3] = { 0 };
			debugGyroAccum[0] += eulerAngles[0];
			debugGyroAccum[1] += eulerAngles[1];
			debugGyroAccum[2] += eulerAngles[2];
			//printf( "%d %d %d\n", (int) debugGyroAccum[0], (int)debugGyroAccum[1], (int)debugGyroAccum[2] );
		}
#endif
		// STEP 4: Validate yor values by doing 4 90 degree turns
		//  across multiple axes.
		// i.e. rotate controller down, clockwise from top, up, counter-clockwise.
		// while investigating quaternion.  It should return to identity.
	}
	else if( cimutag == 2 )
	{
		funDigitalWrite( PIN_SCL, 1 );

		const int32_t accelScale = 1<<27;
		// Get into correct coordinate frame (you may have to mess with these)
		// I.e. making some negative.
		imuAccelUpFix24[0] = -_mulhs( cimu[0]<<16, accelScale );
		imuAccelUpFix24[1] =  _mulhs( cimu[1]<<16, accelScale );
		imuAccelUpFix24[2] = -_mulhs( cimu[2]<<16, accelScale );

		int32_t maga24 = mul2x24(imuAccelUpFix24[0],imuAccelUpFix24[0]) +
			mul2x24(imuAccelUpFix24[1],imuAccelUpFix24[1]) +
			mul2x24(imuAccelUpFix24[2],imuAccelUpFix24[2]);
		int32_t maga24recip = rsqrtx24_good( maga24 );

		imuAccelUpFix29Norm[0] = (imuAccelUpFix24[0] * (int64_t)maga24recip)>>19;
		imuAccelUpFix29Norm[1] = (imuAccelUpFix24[1] * (int64_t)maga24recip)>>19;
		imuAccelUpFix29Norm[2] = (imuAccelUpFix24[2] * (int64_t)maga24recip)>>19;
		int32_t newscale = mul2x24(imuAccelUpFix29Norm[0],imuAccelUpFix29Norm[0]) +
			mul2x24(imuAccelUpFix29Norm[1],imuAccelUpFix29Norm[1]) +
			mul2x24(imuAccelUpFix29Norm[2],imuAccelUpFix29Norm[2]);

		funDigitalWrite( PIN_SCL, 0 );

		// Get an initial guess
		if( accelnum++ == 0 )
		{
			int32_t ideal_up[3] = {0, 1<<29, 0};

			CreateQuatFromTwoVectorRotation_Fix30OutFix29In(imuViewQuatFix30, ideal_up, imuAccelUpFix29Norm);
		}
		else
		{
			// STEP 6: Determine our "error" based on accelerometer.
			// NOTE: This step could be done on the inner loop if you want, and done over
			// every accelerometer cycle, or it can be done on the outside, every few cycles.
			// all that realy matters is that it is done periodically.

			// STEP 6A: Examine vectors.  Generally speaking, we want an "up" vector, not a gravity vector.
			// this is "up" in the controller's point of view.
			int32_t cquat_x24[4] = {
				imuViewQuatFix30[0] >> 6, 
				imuViewQuatFix30[1] >> 6, 
				imuViewQuatFix30[2] >> 6, 
				imuViewQuatFix30[3] >> 6 };

			// Step 6A: Next, compute what we think "up" should be from our point of view.  We will use +Y Up.
			RotateVectorByInverseOfQuaternion_Fix24(what_we_think_is_up, cquat_x24, (const int32_t[3]){0, -1<<29, 0});

			// Step 6C: Next, we determine how far off we are.  This will tell us our error.

			// TRICKY: The ouput of this is actually the axis of rotation, which is ironically
			// in vector-form the same as a quaternion.  So we can write directly into the quat.
			CrossProduct_Fix30OutFix29In(corrective_quaternion + 1, what_we_think_is_up, imuAccelUpFix29Norm);


			// Now, we apply this in step 7.

			// First, we can compute what the drift values of our axes are, to anti-drift them.
			// If you do only this, you will always end up in an unstable oscillation.
			//memcpy(correctiveLast, corrective_quaternion + 1, 12);

			//int32_t gyroBiasTug = 1<<8;
			int32_t correctiveForceTug = 1<<21;
			
			const int gyroBiasForce = 1<<9;

			int32_t confidences[3] = {
				(gyroBiasForce>>3) - ABS( dotm_mulhs3( (const int32_t[3]){ gyroBiasForce, 0, 0 }, what_we_think_is_up ) ),
				(gyroBiasForce>>3) - ABS( dotm_mulhs3( (const int32_t[3]){ 0, gyroBiasForce, 0 }, what_we_think_is_up ) ),
				(gyroBiasForce>>3) - ABS( dotm_mulhs3( (const int32_t[3]){ 0, 0, gyroBiasForce }, what_we_think_is_up ) ) };

			int32_t gbadg[3] = {
				_mulhs( fixedsqrt_x30(corrective_quaternion[1]), confidences[0] ),
				_mulhs( fixedsqrt_x30(corrective_quaternion[2]), confidences[1] ),
				_mulhs( fixedsqrt_x30(corrective_quaternion[3]), confidences[2] ) };

			// Tricky: when updating the gyro bias, if you take a negative number and >> it, then it will be sticky at -1.
			// You're dealing with very small numbers.  When integrating many times they add up.  This nerfs that.
			const int hysteresis = 2;

			gyroBias[0] += ApplyHysteresis( gbadg[0], hysteresis );
			gyroBias[1] += ApplyHysteresis( gbadg[1], hysteresis );
			gyroBias[2] += ApplyHysteresis( gbadg[2], hysteresis );

			//printf( "%d %d %d\n",confidences[0], _mulhs( fixedsqrt_x30(corrective_quaternion[2]), confidences[1] ), confidences[2] );

			// Second, we can apply a very small corrective tug.  This helps prevent oscillation
			// about the correct answer.  This acts sort of like a P term to a PID loop.
			// This is actually the **primary**, or fastest responding thing.
			corrective_quaternion[1] = _mulhs( corrective_quaternion[1], correctiveForceTug );
			corrective_quaternion[2] = _mulhs( corrective_quaternion[2], correctiveForceTug );
			corrective_quaternion[3] = _mulhs( corrective_quaternion[3], correctiveForceTug );


			// x^2+y^2+z^2+q^2 -> ALGEBRA! -> sqrt( 1-x^2-y^2-z^2 ) = w
			corrective_quaternion[0] = fixedsqrt_x30((1<<30) - mul2x30(corrective_quaternion[1], corrective_quaternion[1] )
												 - mul2x30( corrective_quaternion[2], corrective_quaternion[2] )
												 - mul2x30( corrective_quaternion[3], corrective_quaternion[3] ) );


			QuatApplyQuat_Fix30( imuViewQuatFix30, imuViewQuatFix30, corrective_quaternion );
			funDigitalWrite( PIN_SCL, 0 );	

			// Validate:
			//   Among other tests:
			//   Apply a strong bias to the IMU falsely, then make sure in all orientations the gyroBias term doesn't spin around.


		}
	}
	else if( cimutag == 3 )
	{
		// Temperature
	}
	else
	{
		//printf( "Confusing tag: %d\n", (int)cimutag );
	}

	goto resample;
}

void SetupI2C()
{

#if 1
	// Actuall LSM6DSR
	const static uint8_t LSM6DS3Regmap[]  __attribute__((section(".rodata"))) = {
		0x12, 0x44, // CTRL3_C - unset reboot. + BDU
		0x10, 0x4a, // CTRL1_XL - 103Hz, +/-8g
		0x11, 0x4d, // CTRL2_G - 103Hz, 4000dps

		0x0a, 0x28, // FIFO_CTRL5 - Disable FIFO (will re-enable)
		0x06, 0x60, // FIFO_CTRL1 - FIFO size.
		0x07, 0x00, // FIFO_CTRL2 - No temperature in FIFO.
		0x08, 0x09, // FIFO_CTRL3 - Put accel+gyro in FIFO.
		0x09, 0xaa, // FIFO_CTRL4 - TODO: Investigate.
		0x0a, 0x01, // FIFO_CTRL5 -  FIFO mode.
		0x0b, 0x60, // Trigger on gyro event.
		0x0d, 0x00, // Disable weird triggers.
		0x13, 0x00, // CTRL4_C - No extra stuff, don't stop on fth.
		0x15, 0x10, // CTRL6_C - High Performance disabled
		0x16, 0x00, // CTRL7_C - Just default settings.  No gyro filter.
		0x13, 0x01, // CTRL4_C - Stop on fth
	};
#else
	const static uint8_t LSM6DS3Regmap[]  __attribute__((section(".rodata"))) = {
		0x12, 0x44, // CTRL3_C - unset reboot. + BDU
		0x10, 0x5a, // CTRL1_XL - 208Hz, +/-8g
		0x11, 0x54, // CTRL2_G - 208Hz, 1000dps

		0x0a, 0x28, // FIFO_CTRL5 - Disable FIFO (will re-enable)
		0x06, 0x60, // FIFO_CTRL1 - FIFO size.
		0x07, 0x00, // FIFO_CTRL2 - No temperature in FIFO.
		0x08, 0x09, // FIFO_CTRL3 - Put accel+gyro in FIFO.
		0x09, 0x00, // FIFO_CTRL4 - No decimation TODO: I think this is wrong.
		0x0a, 0x2b, // FIFO_CTRL5 - 208Hz, Continuous mode FIFO. (was 0x2e) - if 12.5Hz (set to 0x0e)
		0x13, 0x00, // CTRL4_C - No extra stuff, don't stop on fth.
		0x15, 0x10, // CTRL6_C - High Performance.
		0x16, 0x00, // CTRL7_C - Just default settings.
		0x13, 0x01, // CTRL4_C - Stop on fth
	};
#endif

	ConfigI2C();
	SetupRegisterMap( QMC6309_ADDRESS, (const uint8_t[]){ 0x12, 0x81 }, 1, "LSM6DS3 RESET" );
	SetupRegisterMap( QMC6309_ADDRESS, (const uint8_t[]){ 0x0b, 0x80, 0x0b, 0x00 }, 2, "QMC6309 RESET" );


	SetupRegisterMap( LSM6DS3_ADDRESS, LSM6DS3Regmap, sizeof(LSM6DS3Regmap)/2, "LSM6DS3" );

	const static uint8_t QMC6309Regmap[]  __attribute__((section(".rodata"))) = {
		0x0b, 0x48, // CTRL2 = ODR = 200Hz
		0x0a, 0x19, // CTRL3 = OSR = 8, OSR2=16
	};
	SetupRegisterMap( QMC6309_ADDRESS, QMC6309Regmap, sizeof(QMC6309Regmap)/2, "QMC6309" );

	DSCL_OUTPUT;
	DSDA_OUTPUT;
//	funDigitalWrite( PA11, 0 ); // BS1 = 0 for SPI
//	funDigitalWrite( PA10, 1 ); // D/C
}

void DoI2C()
{
	ConfigI2C();
	//Scan(); // 0x6A, 0x7C

	ProcessLSM6DS3();
	//ProcessQMC6309();

	DSCL_OUTPUT;
	DSDA_OUTPUT;

/*	funPinMode( PIN_SCL, GPIO_CFGLR_OUT_2Mhz_PP );
	funPinMode( PIN_SDA, GPIO_CFGLR_OUT_2Mhz_PP );
	funDigitalWrite( PA11, 0 ); // BS1 = 0 for SPI
	funDigitalWrite( PA10, 1 ); // D/C*/
}

/////////////////////////////////////////////////////////////////////////////////////////////////

static void BeanboySetup()
{
	SystemInit();

	funGpioInitAll(); // no-op on ch5xx

	funPinMode( PA11, GPIO_CFGLR_OUT_10Mhz_PP );
//	funDigitalWrite( PA11, 1 ); // BS1 = 1 for I2C
	funDigitalWrite( PA11, 0 ); // BS1 = 0 for SPI

	funPinMode( PA6, GPIO_CFGLR_OUT_10Mhz_PP );
	funDigitalWrite( PA6, 1 ); // RES
	funPinMode( PA4, GPIO_CFGLR_OUT_10Mhz_PP );
	funDigitalWrite( PA4, 0 ); // SCS
	funPinMode( PA10, GPIO_CFGLR_OUT_10Mhz_PP );
	funDigitalWrite( PA10, 0 ); // D/C

	ssd1306_rst();

	ssd1306_spi_init();
	ssd1306_init();
	ssd1306_setbuf(0x00);

	// Turbo-time
	ssd1306_cmd(0xD5);
	ssd1306_cmd(0xe0);

	ssd1306_cmd(0xc8);
	ssd1306_cmd(0xa1);

	SetupADC();
}

/////////////////////////////////////////////////////////////////////////////////////////////////

typedef void (*UpdateFunction)( void * mode, uint32_t deltaTime, uint32_t * pressures, uint32_t clickedMask, uint32_t lastClickMask );
typedef void (*WirelessRXFunction)( uint8_t * txmac, uint8_t * message, int messageLength, int rssi );


typedef struct ModeTemplate_t
{
	UpdateFunction Update;
	WirelessRXFunction WirelessRX;
} ModeTemplate;

void SelectMode( int modeNumber );


#endif

