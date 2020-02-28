/*----------------------------------------------------------------------------*/
/* Copyright (c) S.P.A.M 2012. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/
#ifndef SPAMUTIL_H_
#define SPAMUTIL_H_

#include "frc/Timer.h"


/**
 * Class of static methods for common operations.
 * From SimBotics Beta Test code:
 *   limitOutput
 *   signSquare
 * From the SPAM archives
 *   RateLimitPWM   
 *   
 */ 
class SPAMutil
{
public:
/* Simbotics */
	static float limitOutput(float f);
	static float signSquare(float f);
	static float limitPositiveOutput(float f);

/* SPAM */
typedef struct {
   bool UpSwitch;
   bool DnSwitch;
   bool UpLatch;
   bool DnLatch;
   unsigned int UpMax;
   unsigned int DnMax;
   unsigned int UpKnt;
   unsigned int DnKnt;
} UpDn;

        static float RateLimitPWM(float CurPWM, float DesiredPWM, float Rate);
        static float Abs(float value);
        static float Sign(float value);
        static void  Acounter(UpDn* acounter );
        static float SPAMlerp(float *x, float *y, int nx, float x3);
        static int AutoAIDI(int x);
        static void DoubleBounce ( unsigned int anTrigger, unsigned int* apTriggerOld, unsigned int* apTriggerToggle);

        static float Set_Motivator_Speed(float *asetPtRpm, float *aLPdelRpm, float *aCountRpm,
        		                         float K_I, float kntsPerRev, float mxRPM);
        static float Hysteresis(float *aCurrent, float *aLastPass, float *aMaxDelta, float *aMinDelta) ;
        /*
        static float xBacon_ATC(float u_in,   float deltaT, float *v_lp, float *u_lp,  
        		                float *pv_lp, int FolDelta, int PwrDelta, bool turning);
        */
        
        // level 0 = error, 1 = info, 2 = debug
        enum {LOG_ERR = 0, LOG_INFO = 1, LOG_DBG = 2};
        static void Log(const char* subsys, const char* msg, int level = 0);
        static void Log(const char* subsys, const std::stringstream& msg, int level = 0);

        static frc::Timer* GetMatchTimer();
        static void RestartMatchTimer();
        static double MovingAvg(double *ptrArrNumbers, double *ptrSum, int pos, int len, double nextNum);
        static void ExpMovingAvg(double newVal, double curAvg, double alpha = 0.5);
    
private:
	// Ensure this class can't be instantiated.
	SPAMutil() {}
    static frc::Timer* matchTimer;
    //static std::shared_ptr<Timer> matchTimer;
};



#endif // SPAMUTI_H_




