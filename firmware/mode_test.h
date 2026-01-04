//
// TODO:  FIX self-fronting code.  It only works at 90 degree angle.
// TODO: Use lowpass on accel for bouncy.  Unrotate accelerometer so rotations don't rek us.
// Fix IMU for other usage.
//

#ifndef _MODE_TEST_H
#define _MODE_TEST_H

#include "test/bunny.h"
#include "test/bean.h"

typedef struct ModeTest_t
{
	UpdateFunction Update;
	WirelessRXFunction WirelessRX;

	int tbtn;
	int tppressures[4];

	int timeMenuDown;
	int model;
	int animationType;
} ModeTest;

ModeTest * testMode;

int SlowGameCheck()
{
	static int fastgamestate = 0;

	if( ( fastgamestate & 0xf ) == 0 )
	{
		// try GPIO_CFGLR_IN_PUPD, GPIO_ModeIN_Floating, GPIO_CFGLR_OUT_10Mhz_PP as well
		funPinMode( PA3, GPIO_ModeIN_Floating );
		funPinMode( PA8, GPIO_ModeIN_Floating );
		funPinMode( PA9, GPIO_ModeIN_Floating );
		testMode->tbtn = (fastgamestate>>4)&3;

		switch( testMode->tbtn )
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
		lastfifo = 0;
		EventRelease();
	}
	else if ( (fastgamestate & 0xf) == 4 )
	{
		int tbtn = testMode->tbtn;
		int lastp = testMode->tppressures[tbtn];
		testMode->tppressures[tbtn] = lastfifo;
		int wasdown = lastp < 1000;
		int nowdown = lastfifo < 1000;

		// Handle exiting.
		if( tbtn == 1 )
		{
			if( !nowdown )
			{
				if( testMode->timeMenuDown > 0 ) testMode->timeMenuDown--;
			}
			else
			{
				if( testMode->timeMenuDown++ > 80 )
				{
					Reboot();
				}
			}
		}

		if( tbtn == 2 && !wasdown && nowdown )
		{
			testMode->model = (testMode->model+1)&1;
		}

		if( tbtn == 0 && !wasdown && nowdown )
		{
			testMode->animationType = (testMode->animationType+1)&3;
		}
	}
	fastgamestate = ( fastgamestate + 1 ) & 0x3f;
	return 0;
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
	int32_t dispPixel[3] = { 0, 0, 0 };

	uint32_t logicTick;

	// In world space
	int32_t worldTranslate[3] = { 0 };
	int32_t dongleVelocity[3] = { 0 };

	// In object space.
	int32_t accel_up_Fix29Norm[3] = { 0 };
	int32_t accel_up_Fix24[3] = { 0 };

	// IMU (updates)
	int32_t currentQuat[4] = { 1<<30, 0, 0, 0 };
	int32_t corrective_quaternion[4]; // Internal, for gravity updates.
	int32_t what_we_think_is_up[3];
	int32_t gyroBias[3] = { 0, 0, 0 };
	int gyronum = 0;
	int accelnum = 0;



	int32_t viewQuatx24[4] = { 1<<24, 0, 0, 0 };
	int32_t objectQuatx24[4] = { 1<<24, 0, 0, 0 };

	int32_t loccenter[3] = { 0 };
//	int32_t loccenter_filt[3] = { 0 };

	int skip = 0;	
	uint32_t nextdeadline = SysTick->CNT;
	uint32_t logicTickNext = SysTick->CNT;
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

			const uint8_t * indices3d;
			const int16_t * vertices3d;

			// We've loaded up the point, now, figure out the next one.
			if( 0 )
			{
				x_coord = pixelNumber&0x7f;
				y_coord = (pixelNumber>>7)&0x7f;
			}
			else if( skip )
			{
				// Skip (For sparkle)
				skip--;
			}
			else
			{
				static int percent_on_line;
				static int lineid;
				int totalLines = 0;
				if( testMode->model )
				{
					totalLines = (sizeof(bunny_lines)/sizeof(bunny_lines[0])/2);
					indices3d = bunny_lines;
					vertices3d = bunny_verts;
				} else {
					totalLines = (sizeof(bean_lines)/sizeof(bean_lines[0])/2);
					indices3d = bean_lines;
					vertices3d = bean_verts;
				}

				int at = testMode->animationType;
				switch( at )
				{
				case 0:
					_rand_lfsr_update();
					lineid = _rand_lfsr%totalLines;
					_rand_lfsr_update();
					percent_on_line = _rand_lfsr & 0xffff;
					break;
				case 1:
				case 2:
				case 3:
				{
					int zspeed;
					if( at == 1 )
					{
						int32_t relz = -dispPixel[2]+1000000;
						zspeed = 60000000/((relz>>10));
						if( relz < 0 ) zspeed = 65536;
						if( zspeed < 3000 ) zspeed = 3000;
						if( zspeed > 65534 ) zspeed = 65534;
					}
					else if( at == 2 )
					{
						zspeed = 100;
					}
					else if( at == 3 )
					{
						zspeed += SysTick->CNT & 0xff;
					}
					percent_on_line+= zspeed;
					if( percent_on_line >= 65536  )
					{
						percent_on_line -= 65536;
						if( percent_on_line < 0 ) percent_on_line = 0;
						if( percent_on_line > 65535 ) percent_on_line = 65535;

						if( 0 )
						{
							_rand_lfsr_update();
							lineid = _rand_lfsr%totalLines;
						}
						else
						{
							lineid++;
							if( lineid == totalLines ) lineid = 0;
						}
					}
				}
					break;
				}

				int vid0 = indices3d[lineid*2+0]*3; // Todo: Optimize!
				int vid1 = indices3d[lineid*2+1]*3;
				int32_t laX = vertices3d[vid0+0];
				int32_t laY = vertices3d[vid0+1];
				int32_t laZ = vertices3d[vid0+2];
				int32_t lbX = vertices3d[vid1+0];
				int32_t lbY = vertices3d[vid1+1];
				int32_t lbZ = vertices3d[vid1+2];

				int invpercent = 65535-percent_on_line;

				int32_t vIn[3] = {
					(( laX * percent_on_line) + ( lbX * invpercent ))>>8,
					(( laY * percent_on_line) + ( lbY * invpercent ))>>8,
					(( laZ * percent_on_line) + ( lbZ * invpercent ))>>8 };


				RotateVectorByQuaternion_Fix24_rough( vIn, objectQuatx24, vIn );
				RotateVectorByQuaternion_Fix24_rough( dispPixel, viewQuatx24, vIn );

				dispPixel[0] += worldTranslate[0];
				dispPixel[1] += worldTranslate[1];
				dispPixel[2] += worldTranslate[2];

				// Bunny is rotated in bunny-local coordinates, where
				// X, Y and Z are all rotated in 3D space, so z = -1..1

			//	dispPixel[0] -= loccenter_filt[0];
			//	dispPixel[1] -= loccenter_filt[1];
			//	dispPixel[2] -= loccenter_filt[2];

				// Animate going to main menu
				dispPixel[2] += testMode->timeMenuDown * 6000000;

				//int32_t wxpos = _mulhs( dispPixel[0]/((dispPixel[2]+2000000)), 20000 );
				//int32_t wypos = _mulhs( dispPixel[1]/((dispPixel[2]+2000000)), 20000 );
				int32_t wxpos = dispPixel[0]/((dispPixel[2]+54000000)>>8);
				int32_t wypos = dispPixel[1]/((dispPixel[2]+54000000)>>8);

				// convert from RHS (World) to NDC
				x_coord =  wxpos + 64;
				y_coord = -wypos + 64;

				// Make sparkley effect.
				if( ((SysTick->CNT>>8) & 0x3f) < 3 ) skip = (SysTick->CNT & 0xf) ;
			}

			while( R16_SPI0_TOTAL_CNT );
			//while( !(R8_SPI0_INT_FLAG & RB_SPI_FREE)) { }
			funDigitalWrite( SSD1306_CS_PIN, FUN_HIGH );
		}
		pixelNumber++;
		goto *nextjump;

	base_loop_logic:
	{
		// This can take a while, it's all fine.   This happens when we have free time.
		// TODO: Make this only happen every so often.

		//////////////////////////////////////////////////////////////////////////
		// Push the square wave towards the user.

		if( (int32_t)(SysTick->CNT - logicTickNext) > 0 )
		{

			logicTickNext += 400000;

			int32_t unrotated[3] = { 0, 0, 1<<24 };
			int32_t rotated[3];
			int32_t rotatedFinal[3];
			RotateVectorByQuaternion_Fix24_rough( rotated, objectQuatx24, unrotated );
			RotateVectorByQuaternion_Fix24_rough( rotated, viewQuatx24,   rotated );

			// Ideal rotation = rotated[x] = -2^16.
			// nudge objectQuat left and right based on the value of [z] being > or < 0.

			// Cross between ideal and what we have.
			int32_t dragcenter[4] = { 0, 0, 0, 0 };

			//CrossProduct_Fix24( dragcenter + 1, ideal, rotated );
			// Because we are only rotating around Y all we need is the Y component of the corrective amount.
			dragcenter[2] = rotated[2];

			// Don't yank.
			dragcenter[2] >>= 10;

			// Because we only have one term, normalizing is ez.
			dragcenter[0] = rsqrtx24_rough((1<<24) - mul2x30(dragcenter[2], dragcenter[2] ));

			QuatApplyQuat_Fix24( objectQuatx24, objectQuatx24, dragcenter );
			QuatNormalize_Fix24( objectQuatx24, objectQuatx24 );
		}
		else
		{
			funDigitalWrite( PIN_SCL, 1 );
			if( SlowGameCheck() ) return;
			funDigitalWrite( PIN_SCL, 0 );
		}
		nextjump = &&lsm6_getcimu;
		goto cont;
	}

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
		nextjump = &&base_loop_logic;
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
		nextjump = &&base_loop_logic;
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
			gyronum++;	
			// Based on the euler angles, apply a change to the rotation matrix.

			funDigitalWrite( PIN_SCL, 1 );

			int32_t eulerAngles[3];
			const int32_t eulerScale = 15500000*2;// XXX Change me when you change IMU update rate.

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
			QuatApplyQuat_Fix30( currentQuat, currentQuat, thisQ );
			QuatNormalize_Fix30( currentQuat, currentQuat );

			viewQuatx24[0] = currentQuat[0]>>6;
			viewQuatx24[1] =-currentQuat[1]>>6;
			viewQuatx24[2] =-currentQuat[2]>>6;
			viewQuatx24[3] =-currentQuat[3]>>6;

			funDigitalWrite( PIN_SCL, 0 );
#if 0
			if( 0 )
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
			accel_up_Fix24[0] = -_mulhs( cimu[0]<<16, accelScale );
			accel_up_Fix24[1] =  _mulhs( cimu[1]<<16, accelScale );
			accel_up_Fix24[2] = -_mulhs( cimu[2]<<16, accelScale );

			int32_t maga24 = mul2x24(accel_up_Fix24[0],accel_up_Fix24[0]) +
				mul2x24(accel_up_Fix24[1],accel_up_Fix24[1]) +
				mul2x24(accel_up_Fix24[2],accel_up_Fix24[2]);
			int32_t maga24recip = rsqrtx24_good( maga24 );

			accel_up_Fix29Norm[0] = (accel_up_Fix24[0] * (int64_t)maga24recip)>>19;
			accel_up_Fix29Norm[1] = (accel_up_Fix24[1] * (int64_t)maga24recip)>>19;
			accel_up_Fix29Norm[2] = (accel_up_Fix24[2] * (int64_t)maga24recip)>>19;
			int32_t newscale = mul2x24(accel_up_Fix29Norm[0],accel_up_Fix29Norm[0]) +
				mul2x24(accel_up_Fix29Norm[1],accel_up_Fix29Norm[1]) +
				mul2x24(accel_up_Fix29Norm[2],accel_up_Fix29Norm[2]);

			funDigitalWrite( PIN_SCL, 0 );

			// Get an initial guess
			if( accelnum++ == 0 )
			{
				int32_t ideal_up[3] = {0, 1<<29, 0};

				CreateQuatFromTwoVectorRotation_Fix30OutFix29In(currentQuat, ideal_up, accel_up_Fix29Norm);
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
					currentQuat[0] >> 6, 
					currentQuat[1] >> 6, 
					currentQuat[2] >> 6, 
					currentQuat[3] >> 6 };

				// Step 6A: Next, compute what we think "up" should be from our point of view.  We will use +Y Up.
				RotateVectorByInverseOfQuaternion_Fix24(what_we_think_is_up, cquat_x24, (const int32_t[3]){0, -1<<29, 0});

				// Step 6C: Next, we determine how far off we are.  This will tell us our error.

				// TRICKY: The ouput of this is actually the axis of rotation, which is ironically
				// in vector-form the same as a quaternion.  So we can write directly into the quat.
				CrossProduct_Fix30OutFix29In(corrective_quaternion + 1, what_we_think_is_up, accel_up_Fix29Norm);

				nextjump = &&continue_accelerometer_logic;
				goto cont;
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
		nextjump = &&lsm6_getcimu;
		goto cont;




	continue_accelerometer_logic:

		funDigitalWrite( PIN_SCL, 1 );

		// Now, we apply this in step 7.

		// First, we can compute what the drift values of our axes are, to anti-drift them.
		// If you do only this, you will always end up in an unstable oscillation.
		//memcpy(correctiveLast, corrective_quaternion + 1, 12);

		//int32_t gyroBiasTug = 1<<8;
		int32_t correctiveForceTug = 1<<21;
		
		const int gyroBiasForce = 1<<9;

		int32_t confidences[3] = {
			(gyroBiasForce>>3) - dotm_mulhs3( (const int32_t[3]){ gyroBiasForce, 0, 0 }, what_we_think_is_up ),
			(gyroBiasForce>>3) - dotm_mulhs3( (const int32_t[3]){ 0, gyroBiasForce, 0 }, what_we_think_is_up ),
			(gyroBiasForce>>3) - dotm_mulhs3( (const int32_t[3]){ 0, 0, gyroBiasForce }, what_we_think_is_up ) };

		if( confidences[0] < 0 ) confidences[0] = -confidences[0];
		if( confidences[1] < 0 ) confidences[1] = -confidences[1];
		if( confidences[2] < 0 ) confidences[2] = -confidences[2];

		// XXX TODO: We need to multiply by amount the accelerometer gives us assurance.
		int32_t gbadg[3] = {
			_mulhs( fixedsqrt_x30(corrective_quaternion[1]), confidences[0] ),
			_mulhs( fixedsqrt_x30(corrective_quaternion[2]), confidences[1] ),
			_mulhs( fixedsqrt_x30(corrective_quaternion[3]), confidences[2] ) };

		// Tricky if you take a negative number and >> it, then it will be sticky at -1.
		// Surely there's a more elegant way of doing this!
		const int histeresis = 2;
		if( gbadg[0] < 0 ) { if( gbadg[0] < -histeresis ) gbadg[0] += histeresis; else gbadg[0] = 0; }
		else { if( gbadg[0] > histeresis ) gbadg[0]-= histeresis;  else gbadg[0] = 0; }
		if( gbadg[1] < 0 ) { if( gbadg[1] < -histeresis ) gbadg[1] += histeresis; else gbadg[1] = 0; }
		else { if( gbadg[1] > histeresis ) gbadg[1]-= histeresis;  else gbadg[1] = 0; }
		if( gbadg[2] < 0 ) { if( gbadg[2] < -histeresis ) gbadg[2] += histeresis; else gbadg[2] = 0; } 
		else { if( gbadg[2] > histeresis ) gbadg[2]-= histeresis;  else gbadg[2] = 0; }

		gyroBias[0] += gbadg[0];
		gyroBias[1] += gbadg[1];
		gyroBias[2] += gbadg[2];

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


		QuatApplyQuat_Fix30( currentQuat, currentQuat, corrective_quaternion );
		funDigitalWrite( PIN_SCL, 0 );	

		// Validate:
		//   Among other tests:
		//   Apply a strong bias to the IMU falsely, then make sure in all orientations the gyroBias term doesn't spin around.



		// We're actually done with IMU work.
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


		// But we still have time, so let's handle letting our object bounce around.

		funDigitalWrite( PIN_SCL, 1 );

		// CAREFUL: Perform rotations to acceleration reverse in object space.
		int32_t rotatedAccel[3] = { };
		//RotateVectorByQuaternion_Fix24_rough( rotatedAccel, viewQuatx24,   accel_up_Fix24 );
		int32_t invq[4] = { viewQuatx24[0],-viewQuatx24[1],-viewQuatx24[2],-viewQuatx24[3] };
		RotateVectorByQuaternion_Fix24_rough( rotatedAccel, invq,   accel_up_Fix24 );

		static int32_t averageAccel[3];
		const int32_t accelGamma = 100000;
		averageAccel[0] =  mul2x24( averageAccel[0], (1<<24)-accelGamma ) + mul2x24(  rotatedAccel[0], accelGamma );
		averageAccel[1] =  mul2x24( averageAccel[1], (1<<24)-accelGamma ) + mul2x24(  rotatedAccel[1], accelGamma );
		averageAccel[2] =  mul2x24( averageAccel[2], (1<<24)-accelGamma ) + mul2x24(  rotatedAccel[2], accelGamma );
//printf( "%d %d %d\n", rotatedAccel[0], rotatedAccel[1], rotatedAccel[2] );

		int32_t diffAccel[3] = {
			rotatedAccel[0] - averageAccel[0],
			rotatedAccel[1] - averageAccel[1],
			rotatedAccel[2] - averageAccel[2] };

		// Put it back into world space.
		RotateVectorByQuaternion_Fix24_rough( diffAccel, viewQuatx24,   diffAccel );


		// Gravity
		dongleVelocity[0] += _mulhs(diffAccel[0], -300000 );
		dongleVelocity[1] += _mulhs(diffAccel[1], -300000 );
		dongleVelocity[2] += _mulhs(diffAccel[2], -300000 );

		// Drag
		dongleVelocity[0] -= _mulhs(dongleVelocity[0], 12000000 );
		dongleVelocity[1] -= _mulhs(dongleVelocity[1], 12000000 );
		dongleVelocity[2] -= _mulhs(dongleVelocity[2], 12000000 );

		worldTranslate[0] += mul2x24(dongleVelocity[0],100000000);
		worldTranslate[1] += mul2x24(dongleVelocity[1],100000000);
		worldTranslate[2] += mul2x24(dongleVelocity[2],100000000);

		int32_t offset[3] = { 0, 0, 0 };

		//int32_t quse[4] = { currentQuat[0], currentQuat[1], currentQuat[2], currentQuat[3] };

		//RotateVectorByQuaternion_Fix24_rough( offset, quse, offset );


		int32_t tcenter[3] = {
			( worldTranslate[0] + offset[0] ) <<1,
			( worldTranslate[1] + offset[1] ) <<1,
			( worldTranslate[2] + offset[2] ) <<1,
		};

		int32_t dist = fixedsqrt_x30( _mulhs( tcenter[0], tcenter[0] ) +
			_mulhs( tcenter[1], tcenter[1] ) + _mulhs( tcenter[2], tcenter[2] )  );

		dist-= 5000000;
		if( dist > 0 )
		{
			dist>>=4;
			int32_t resistance[3] = {
				_mulhs( tcenter[0], -dist ),
				_mulhs( tcenter[1], -dist ),
				_mulhs( tcenter[2], -dist ),
			};
			dongleVelocity[0] += resistance[0]>>4;
			dongleVelocity[1] += resistance[1]>>4;
			dongleVelocity[2] += resistance[2]>>4;
		}

		funDigitalWrite( PIN_SCL, 0 );


		nextjump = &&lsm6_getcimu;
		goto cont;

	// Extra states look like this:
//	game_logic_after_accel:
//	{
//
//		nextjump = &&lsm6_getcimu;
//		goto cont;
//	}	

}

void ModeTestLoop( void * mode, uint32_t deltaTime, uint32_t * pressures, uint32_t clickedMask, uint32_t lastClickMask )
{
	int i;
	testMode = (ModeTest *)mode;

	CoreLoop();
}


void EnterTestMode( ModeTest * m )
{
	memset( m, 0, sizeof(*m) );
	m->Update = ModeTestLoop;
	m->WirelessRX = 0;

	testMode->animationType = 1;
	testMode->model = 0;
	testMode->timeMenuDown = 0;

	funDigitalWrite( SSD1306_DC_PIN, FUN_LOW );
	funDigitalWrite( SSD1306_CS_PIN, FUN_LOW );

	SetupI2C();

	SendStart();
	SendByteNoAck( LSM6DS3_ADDRESS<<1 );
	SendByteNoAck( 0x11 );
	SendByteNoAck( 0x8d ); // 1.66kHz gyro updates
	SendStop();

	SendStart();
	SendByteNoAck( LSM6DS3_ADDRESS<<1 );
	SendByteNoAck( 0x10 );
	SendByteNoAck( 0x8a ); // 833Hz Accel updates
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

