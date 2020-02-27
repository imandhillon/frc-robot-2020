

#include "Commands/JoyDriveCommand.h"


JoyDriveCommand::JoyDriveCommand(): frc::Command() {
    // Use Requires() here to declare subsystem dependencies
    // eg. Requires(Robot::chassis.get());
	Requires(Robot::hyperdrive.get());
}

// Called just before this Command runs the first time
void JoyDriveCommand::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void JoyDriveCommand::Execute() {
 
   Robot::hyperdrive->DriveArcade(Robot::oi->getDriverJoystick());


/*  TEST ME TEST ME TEST ME TEST ME */
/*
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    const auto xSpeed =
        -Robot::oi->getDriverJoystick()->GetY(frc::GenericHID::kLeftHand) * Hyperdrive::kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    const auto rot = -Robot::oi->getDriverJoystick()->GetX(frc::GenericHID::kRightHand) *
                     Hyperdrive::kMaxAngularSpeed;

    Robot::hyperdrive->Drive(xSpeed, rot);
*/
}

// Make this return true when this Command no longer needs to run execute()
bool JoyDriveCommand::IsFinished() {
    return false;
}

// Called once after isFinished returns true
void JoyDriveCommand::End() {
    // send zeros
    //Robot::hyperdrive->Drive(0_mps, (units::angular_velocity::radians_per_second_t)0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void JoyDriveCommand::Interrupted() {
    End();
}
