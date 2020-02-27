
#include "Subsystems/TractorBeam.h"

constexpr double kIntakeMaxCurrent = 40.0;


TractorBeam::TractorBeam() : frc::Subsystem("TractorBeam") {

    intakeMotor.reset(new rev::CANSparkMax(8, rev::CANSparkMax::MotorType::kBrushless));
    intakeMotor->SetSmartCurrentLimit(kIntakeMaxCurrent);

    tractorBeamBay.reset(new frc::DoubleSolenoid(0, 2, 3));
    AddChild("TractorBeamBay", tractorBeamBay);

}

void TractorBeam::InitDefaultCommand() {
    // Set the default command for a subsystem here.
    // SetDefaultCommand(new MySpecialCommand());
}

void TractorBeam::Periodic() {
    // Put code here to be run every loop

}


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

// Burn CANSparkMAX settings on motors
void TractorBeam::Burn()
{
    intakeMotor->BurnFlash();
}
