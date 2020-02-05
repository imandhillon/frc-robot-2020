
#include "Commands/Burn.h"


Burn::Burn(): frc::Command() {
    // Use Requires() here to declare subsystem dependencies
    // eg. Requires(Robot::chassis.get());

    // only the ones that have CANSparkMAX
    Requires(Robot::hyperdrive.get());
	Requires(Robot::ionCanon.get());
    Requires(Robot::plasmaTank.get());
    //Requires(Robot::theForce.get());
    Requires(Robot::tractorBeam.get());
    //Requires(Robot::warpDriveInverter.get());
    
}

// Called just before this Command runs the first time
void Burn::Initialize() {
    Robot::hyperdrive->Burn();
    Robot::ionCanon->Burn();
    Robot::plasmaTank->Burn();
    Robot::tractorBeam->Burn();

}

// Called repeatedly when this Command is scheduled to run
void Burn::Execute() {

}

// Make this return true when this Command no longer needs to run execute()
bool Burn::IsFinished() {
    return true;
}

// Called once after isFinished returns true
void Burn::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void Burn::Interrupted() {
    End();
}
