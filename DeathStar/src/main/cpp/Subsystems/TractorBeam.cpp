// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "Subsystems/TractorBeam.h"
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

TractorBeam::TractorBeam() : frc::Subsystem("TractorBeam") {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
intakeMotor.reset(new rev::CANSparkMax(8, rev::CANSparkMax::MotorType::kBrushless));
//lw->AddActuator("TractorBeam", "IntakeMotor", intakeMotor);

tractorBeamBay.reset(new frc::DoubleSolenoid(0, 2, 3));
AddChild("TractorBeamBay", tractorBeamBay);


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
}

void TractorBeam::InitDefaultCommand() {
    // Set the default command for a subsystem here.
    // SetDefaultCommand(new MySpecialCommand());
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
}

void TractorBeam::Periodic() {
    // Put code here to be run every loop

}

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS


// Put methods for controlling this subsystem
// here. Call these from Commands.

void TractorBeam::Deploy()
{
    tractorBeamBay->Set(frc::DoubleSolenoid::Value::kForward);
}

void TractorBeam::Retract()
{
    tractorBeamBay->Set(frc::DoubleSolenoid::Value::kReverse);
}

bool TractorBeam::IsDeployed()
{
    return tractorBeamBay->Get() == frc::DoubleSolenoid::Value::kForward;
}

// Run Motor(s) at specific speed
// negative speed will Spit Out, zero to stop
void TractorBeam::SpinIn(double speed)
{
    intakeMotor->Set(speed);
}

