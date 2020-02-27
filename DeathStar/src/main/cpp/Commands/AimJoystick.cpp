
#include "Commands/AimJoystick.h"

constexpr double kDeadband = 0.1;

AimJoystick::AimJoystick(): frc::Command() {
	Requires(Robot::ionCannon.get());
}

// Called just before this Command runs the first time
void AimJoystick::Initialize() {
}

// Called repeatedly when this Command is scheduled to run
void AimJoystick::Execute() {
    double x = Robot::oi->getOperatorJoystick()->GetRawAxis(4);
    double y = Robot::oi->getOperatorJoystick()->GetRawAxis(5);

    if (x > -kDeadband && x < kDeadband)
        x = 0.0;
    if (y > -kDeadband && y < kDeadband)
        y = 0.0;

    // Move the turret left/right
    if (x > 0) {
        Robot::ionCannon->AimRight();
    } else if (x < 0) {
        Robot::ionCannon->AimLeft();
    } else {
        Robot::ionCannon->StopTurret();
    }

    // Move the dome up/down
    //double angle = (y + 1.0) / 2.0 * 90.0;
    //Robot::ionCannon->SetServo(angle);
    if (y > 0) {
        Robot::ionCannon->AimUp();
    } else if (y < 0) {
        Robot::ionCannon->AimDown();
    } else {
        Robot::ionCannon->StopDome();
    }
}

// Make this return true when this Command no longer needs to run execute()
bool AimJoystick::IsFinished() {
    return false;
}

// Called once after isFinished returns true
void AimJoystick::End() {
    Robot::ionCannon->AimStop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void AimJoystick::Interrupted() {
    End();
}
