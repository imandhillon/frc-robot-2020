// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "Commands/Lock.h"

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

Lock::Lock(double speed): frc::Command() {
m_speed = speed;
    // Use Requires() here to declare subsystem dependencies
    // eg. Requires(Robot::chassis.get());
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
	Requires(Robot::plasmaTank.get());
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
}
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

// Called just before this Command runs the first time
void Lock::Initialize() {
    Robot::plasmaTank->RunConveyor(m_speed);
}

// Called repeatedly when this Command is scheduled to run
void Lock::Execute() {
   // double speed = Robot::oi->getOperatorJoystick()->GetY();
   // if (speed > -0.1 && speed < 0.1)
   //     speed = 0.0;
   // Robot::plasmaTank->RunConveyor(speed);

   //  speed = Robot::oi->getDriverJoystick()->GetRawAxis(5);
   // if (speed > -0.1 && speed < 0.1)
   //     speed = 0.0;
   // Robot::plasmaTank->RunLoader(speed);

}

// Make this return true when this Command no longer needs to run execute()
bool Lock::IsFinished() {
    return false;
}

// Called once after isFinished returns true
void Lock::End() {
    //Robot::plasmaTank->StopConveyor();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void Lock::Interrupted() {
    End();
}
