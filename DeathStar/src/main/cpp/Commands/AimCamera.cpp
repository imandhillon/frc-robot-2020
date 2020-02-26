
#include "Commands/AimCamera.h"


AimCamera::AimCamera(): frc::Command() {
	Requires(Robot::ionCannon.get());
}

// Called just before this Command runs the first time
void AimCamera::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void AimCamera::Execute() {
    //Robot::ionCannon->AimCam();
    Robot::ionCannon->AimCamPosition();
}

// Make this return true when this Command no longer needs to run execute()
bool AimCamera::IsFinished() {
    return false;
}

// Called once after isFinished returns true
void AimCamera::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void AimCamera::Interrupted() {
    End();
}
