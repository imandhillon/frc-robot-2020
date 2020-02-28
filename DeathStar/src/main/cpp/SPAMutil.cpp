/*----------------------------------------------------------------------------*/
/* Copyright (c) S.P.A.M 2012. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include <sstream>
#include "SPAMutil.h"
#include <string.h>
#include <time.h>

frc::Timer* SPAMutil::matchTimer = NULL;
//std::shared_ptr<Timer> SPAMutil::matchTimer;

// level 
//       0 - ERROR
//       1 - INFO
//       2 - DEBUG
#define DBG_ENABLED

/**
 * Limits a value to the Victor output range of -1.0 to 1.0.
 * 
 * @param f The value to limit
 */
float SPAMutil::limitOutput(float f)
{

	//if (f > 1.0)
	if (f > 1.0)
	{
		//return 1.0;
		return 1.0;
	}
	//else if (f < -1.0)
	else if (f < -1.0)
	{
		//return -1.0;
		return -1.0;
	}
	else
	{
	}

	return f;
}

float SPAMutil::limitPositiveOutput(float f)
{

	//if (f > 1.0)
	if (f > 1.0)
	{
		//return 1.0;
		return 1.0;
	}
	//else if (f < -1.0)
	else if (f <= 0.0)
	{
		//return -1.0;
		return 0.0;
	}
	else
	{
	}

	return f;
}

/**
 * Squares a value while preserving sign.
 * 
 * @param f The value to square
 */
float SPAMutil::signSquare(float f)        
{	                                  
	if (f < 0)                          
	{	
		return -1.0 * f * f;
	}
	else
	{
		return f * f;
	}
}
/* 
 * 
 */
float SPAMutil::RateLimitPWM(float CurPWM, float DesiredPWM, float Rate)
{
  if (SPAMutil::Abs(CurPWM - DesiredPWM) <= Rate) 
    return DesiredPWM;
  else if (CurPWM < DesiredPWM)
    return CurPWM + Rate;
  else
    return CurPWM - Rate;
}
/* 
 * 
 */
float SPAMutil::Abs(float value)
{
  if (value >= 0.) 
    return value;
  else
    return -value;
}
/* 
 * 
 */
float SPAMutil::Sign(float value)
{
  if (value >= 0.)
    return 1.;
  else
    return -1.;
}

/*
*  an up/down counter function
*/
void SPAMutil::Acounter(UpDn* acounter )
{

  if( !acounter->UpLatch && acounter->UpSwitch)
  {  
     acounter->UpKnt++;
     if( acounter->UpKnt >= acounter->UpMax)
     {
        acounter->UpLatch = 1;
        acounter->UpKnt = acounter->UpMax ;
        acounter->DnKnt = 0 ;
        acounter->DnLatch = 0;
     }
  }
  else if( !acounter->DnLatch && acounter->DnSwitch)
  { 
     acounter->DnKnt++;
     if( acounter->DnKnt >= acounter->DnMax)
     {
        acounter->DnLatch = 1;
        acounter->DnKnt = acounter->DnMax ;
        acounter->UpKnt = 0 ;
        acounter->UpLatch = 0;
     }
  }
}
/* 
*  linear interpolater/extrapolator function (lerp)
*/
float SPAMutil::SPAMlerp(float *x, float *y, int nx, float x3)
{
   float y3;
   int i = 0;
   int j = 0;


   while ( (x3 > x[i]) && (i < (nx-1)) )  
   {                                       
      i++ ;
   }
   if( i > 0 && i < nx )
   {
     j = i;
     i -= 1 ;
   }
   else
   {
     j=i+1;
   }
   y3 = y[i] + (y[j] - y[i]) / (x[j] - x[i]) * (x3 - x[i]) ;  
   return y3 ;

}

/**
 * Analog / Digital Switch for autonomous
 * i = # of points
 * * AIDI
 * index:      0    1     2    3    4    5    6    7    8    9   
 * ~real value: 990, 880, 760, 650, 540, 430, 325, 220, 109, 0
 * ~threshold  1000,945, 820, 705, 595, 495, 375, 270, 150, 50  
 */
int SPAMutil::AutoAIDI(int x)
{	                    
	int Aray[]={1000, 945, 820, 705, 595, 495, 375, 270, 150, 50} ;
//	
//	
	int j = 0;
	int i = 10;
	    
	   while ( j < (i-1))  // j runs 0 to 9
	   {            
//		  
//		   
		   if(x > Aray[j])
			   return j;
	      j++ ; 
	   }
		return j;
}
void SPAMutil::DoubleBounce ( unsigned int anTrigger, unsigned int* apTriggerOld, unsigned int* apTriggerToggle)
{
   if (anTrigger == 1 && *apTriggerOld == 0)
   {
      *apTriggerToggle = 1 - *apTriggerToggle;
   }
   *apTriggerOld = anTrigger;
   return;
}

/*
 * 
 * a speed controller from a vi on C.D.
 * 
 */
float SPAMutil::Set_Motivator_Speed(float *asetPtRpm, float *aLPdelRpm, float *aCountRpm,
		float K_I, float kntsPerRev, float mxRPM )
{
	float setPtRpm; // set p[oint speed in rpm
	float countRpm; // counter in put converted to rpm
	float LPdelRpm; // last pass rpm
	float deltaRpm; // rpm error
	float rawDrive; // drive error
	float gainDrive; // new output - raw
	float drive;     // new output - normalized
	setPtRpm = *asetPtRpm;
	countRpm = *aCountRpm;
	LPdelRpm = *aLPdelRpm;
	deltaRpm = setPtRpm - countRpm;
	rawDrive = deltaRpm + LPdelRpm;
	gainDrive = rawDrive * K_I;
	drive = (setPtRpm + gainDrive)/mxRPM; //normalize by the motor speed
	*aLPdelRpm = deltaRpm;
	return drive;
}

float SPAMutil::Hysteresis(float *aCurrent, float *aLastPass, float *aMaxDelta, float *aMinDelta)
{
	float lCurrent;   // present value
	float lLastPass;  // last good value
	float lMaxDelta;  // hystersis hi band
	float lMinDelta;  // hysteresis lo band
	
	lCurrent  = *aCurrent ;
	lLastPass = *aLastPass;
	lMaxDelta = *aMaxDelta;
	lMinDelta = *aMinDelta;
	
	if ( (lCurrent < (lLastPass + lMaxDelta)) && 
		 (lCurrent > (lLastPass - lMinDelta)) )	
	{
		*aLastPass = lLastPass ;
		return lLastPass;
	}
	else
	{
		*aLastPass = lCurrent ;
		return lCurrent;
	}
}

/*
// Traction COntrol
// xBacon_ATC   based on a description by Doug Leopard on C.D.
//
float SPAMutil::xBacon_ATC(float u_in,   float deltaT, float *v_lp, float *u_lp,  
		                   float *pv_lp, int FolDelta, int PwrDelta, bool turning)
{
	float u_out ;
	float SetPoint;
	float Error ;
	float lv;
	float la;
	float Du;
// 
 float Kp = 0.04;  // 
 float Ki = 0.004;
 float Kd = 0.0004;
//
 float Convert2PwrDelta  =   22. / 45.;  // sprocket adjustment (inches/tic)
 float Convert2rad =   0.0251327;        // (6.28[rad/rev] / 250[tics/rev]) 
                                         // 0.0251327 
 // no zero time!
 if ( 0. == deltaT)
	 deltaT = 0.01;

 // x1 =Vv/Rw = omegaV = #tics*(6.28[rad/rev] / 250[tics/rev]) / elapsed time [second]
 float x1 = (float)FolDelta * Convert2PwrDelta * Convert2rad / deltaT; // omegaV (radians/sec)
 float x2 = (float)PwrDelta * Convert2rad / deltaT;                    // omegaW (radians/sec)

 // set default output an local parameters
 u_out  = u_in ;
 float lu_lp  = *u_lp;
 float lv_lp  = *v_lp;
 float lpv_lp = *pv_lp;

 // zero speed condition, or wheels of opposite sign
 if( ((x1 >= 0.) && (x2 <= 0.)) ||
     ((x1 <= 0.) && (x2 >= 0.))  )
 {
    // stopped, starting or in a pushing match
	//
    // Stick in the dead-band...requesting stop
    if( ( u_in < 0.078125 ) && 
        ( u_in > -0.078125 ) )
	{
        u_out    = 0. ;
	}
	else // requesting motion, set a pwr wheel speed
	{
       // pwr wheel stopped or forward
       if( x2 >= 0.)
	   {
	      SetPoint = 2.512 ; // rad/sec ~1.25ft/sec 
	   }
	   else // pwr wheel reversed
	   {
	      SetPoint = -2.512 ;
	   }
	   Error    = SetPoint - x2 ;
	}
 }
 else  // moving, set a slip limit between power and follower
 {
    // the slip ratio, Lamda = 1. - (x1 / x2), setpoint is 20%
	//
	// derivation of Slip Set point for x2
	// Slip ratio Lamda = 1. - (x1 / x2)
    // Lamda - 1. = - x1 / x2  or   
    // (Lamda - 1.) / -x1 = 1. / x2 re-arrrangeing:
	// x2 = -x1 / (Lamda - 1.) == x1 / (1. - Lamda)
    //
	// LamdaSetPoint is 20%
	  SetPoint    = x1 / ( 1. - 0.20) ;  
      Error       = SetPoint - x2  ;

 }
                                          
 // run a velocity PID type C controller 
 if( 0. != u_out)
 {
    lv =( x2 - lpv_lp ) / deltaT;
    la =( lv - lv_lp) / deltaT;
    Du =(Ki * Error - Kp * lv - Kd * la) * deltaT;
    u_out = lu_lp + Du;
	// limit the change so that it can't reverse direction
	if (lu_lp >= 0)
	{
	   u_out = (u_out > -0.078125) ? u_out : -0.078125 ;
	   u_out = (u_out < 0.999) ? u_out : 0.999 ;
	}
	else
	{
	   u_out = (u_out > 0.078125) ? u_out : 0.078125 ;
	   u_out = (u_out < -0.999) ? u_out : -0.999 ;
	}
 }
 else // track velocity pid type C
 {
    lv = ( x2 - lpv_lp ) / deltaT; 
 }

 // chuck it all if turning
 if(turning)
 {
    lv = ( x2 - lpv_lp ) / deltaT; 
    u_out = u_in;
 }
  *v_lp   = lv;
  *u_lp   = u_out;
  *pv_lp  = x2 ;
 
  return u_out;
} // end xBaconATC
*/

frc::Timer* SPAMutil::GetMatchTimer()
{
	if (matchTimer == NULL) {
		matchTimer = new frc::Timer;
		matchTimer->Reset();
		matchTimer->Start();
	}
	//matchTimer.reset(new Timer());
	return matchTimer;
}
void SPAMutil::RestartMatchTimer()
{
	GetMatchTimer()->Reset();
	GetMatchTimer()->Start();
}

void SPAMutil::Log(const char* subsys, const char* msg, int level)
{
#ifdef DBG_ENABLED
	const char* lvl = (level == LOG_ERR) ? " ERROR" : (level == LOG_INFO) ? "INFO" : "DEBUG";
	double time = GetMatchTimer()->Get();
	if (time == 0.0) {
		static double start = 0.0;
	    struct timespec tp;

	    clock_gettime(CLOCK_REALTIME,&tp);
	    time = (double)tp.tv_sec + (double)((double)tp.tv_nsec*1e-9);
	    if (start == 0.0) {
	    	start = time;
	    }
	    time = time - start;
	}
	printf("%s: %lf (%s) %s\n", lvl, time, subsys, msg);
#endif
}

void SPAMutil::Log(const char* subsys, const std::stringstream &msg, int level)
{
	Log(subsys, msg.str().c_str(), level); 
}
// moving averae filter
        // ptrArrNumbers = sample array
        // ptrSum        = the moving average
        // pos           = ith position in the sample array
        // len           = length of the sample array
        // nextNum       = incoming sample
        // USAGE:
		// init:
		// arrNumbers[5] = {0};
		// int pos = 0;
        // double newAvg = 0.;
        // double sum = 0.;
		// int len = 5;
		// double nextSample = aValue;
		//
        // newAvg = MovingAvg(arrNumbers, &sum, pos, len, nextSample);
        // pos++;
        // if (pos >= len)
        //   pos = 0;
        // 
double SPAMutil::MovingAvg(double *ptrArrNumbers, double *ptrSum, int pos, int len, double nextNum){
    //Subtract the oldest number from the prev sum, add the new number
    *ptrSum = *ptrSum - ptrArrNumbers[pos] + nextNum;
    //Assign the nextNum to the position in the array
    ptrArrNumbers[pos] = nextNum;
    //return the average
    return *ptrSum / (double)len;
}       

void SPAMutil::ExpMovingAvg(double newVal, double curAvg, double alpha){
	// Should store 'curAvg somewhere in some global data state and call getCurAvg() here
	double newAvg = 0.0;
	// Defaulted to 0.5. Need to test and tune to see what to set this to.
    //double alpha = 0.5;
	// This equation should work for a frame of up to about 30 values.
	// This should set a private variable somewhere.
	newAvg = (alpha * newVal) + (1.0 - alpha) * curAvg;

	// setCurAvg(newAvg);

}