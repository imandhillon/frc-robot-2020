// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "Subsystems/TheForce.h"
#include "Commands/Gravitate.h"
#include "ntcore.h"
#include "frc/smartdashboard/SmartDashboard.h"
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

TheForce::TheForce() : frc::Subsystem("TheForce") {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
liftMotor.reset(new ctre::phoenix::motorcontrol::can::WPI_VictorSPX(6));
liftMotor1.reset(new ctre::phoenix::motorcontrol::can::WPI_VictorSPX(7));

liftEncoder.reset(new frc::Encoder(0, 1, false, frc::Encoder::k4X));
AddChild("LiftEncoder", liftEncoder);
liftEncoder->SetDistancePerPulse(1.0);
liftEncoder->SetPIDSourceType(frc::PIDSourceType::kRate);
liftReferenceSwitch.reset(new frc::DigitalInput(11));
AddChild("LiftReferenceSwitch", liftReferenceSwitch);

liftMotor1->Follow(*liftMotor);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
}

void TheForce::InitDefaultCommand() {
    // Set the default command for a subsystem here.
    // SetDefaultCommand(new MySpecialCommand());
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
     SetDefaultCommand(new Gravitate(0.));

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
}

void TheForce::Periodic() {
    // Put code here to be run every loop
    frc::SmartDashboard::PutNumber("TheForce", liftEncoder->GetDistance());
    // YodaLift();

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS


// Put methods for controlling this subsystem
// here. Call these from Commands.

void TheForce::UseTheForce(std::shared_ptr<frc::Joystick> j)
{
    //float x = j->GetX(4);            // raw 0
    float y = j->GetRawAxis(5);            // raw 1
    if(y > 0.5){
        y = 0.5;
    }

    if(y < -0.5){
        y = -0.5;
    }
    liftMotor->Set(y);
    //differentialDrive->TankDrive(-y, -y, true);
}
void TheForce::YodaLift()
{
	   double lSetPoint;
	   double lEncVal;
	   double lJoyX ;
	   double lError ;
	   
	   // setPtFore is set by SetAngleFore in MoveWheel
	   lSetPoint = m_LiftSetPoint;
	   lEncVal = liftEncoder->GetDistance();
	   // lError in encoder counts
	   lError = lSetPoint - lEncVal;
	   if(lError < BANGBANG_LO_SIDE)     // low side of deadband
	   {
		   lJoyX = -1. * m_BangBang_Speed;
	   }
	   else if(lError > BANGBANG_HI_SIDE) // hi side of deadband
	   {
		   lJoyX = m_BangBang_Speed ;
	   }
	   else
	   {
		   lJoyX = 0.0 ;     // in deadband
	   }
	   liftMotor->Set(lJoyX);
	   
}
    void TheForce::SetLiftWookie() // max
    {
       m_LiftSetPoint = WOOKIE_HEIGHT;
    }
    void TheForce::SetLiftEwok()   // home
    {
       m_LiftSetPoint = EWOK_HEIGHT;
    }
    void TheForce::SetLiftSolo()   // hang
    {
      // allow a vernier
      std::shared_ptr<frc::Joystick> joyy = Robot::oi->getOperatorJoystick();
      double vernyY = -(joyy->GetRawAxis(5));	
      vernyY *= 50.;
       
       m_LiftSetPoint = SOLO_HEIGHT + vernyY;
       
    }

     void TheForce::SetLiftHome()   // hang
    {
       m_LiftSetPoint = 5.;
    }