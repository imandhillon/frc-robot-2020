
#include "Subsystems/PlasmaTank.h"
#include "Commands/Lock.h"

constexpr double kConveyorMaxCurrent = 20.0;
constexpr double kLoaderMaxCurrent = 60.0;

PlasmaTank::PlasmaTank() : frc::Subsystem("PlasmaTank") {

    conveyorMotor.reset(new rev::CANSparkMax(9, rev::CANSparkMax::MotorType::kBrushless));
    conveyorMotor1.reset(new rev::CANSparkMax(10, rev::CANSparkMax::MotorType::kBrushless));
    conveyorMotor->SetSmartCurrentLimit(kConveyorMaxCurrent);
    conveyorMotor1->SetSmartCurrentLimit(kConveyorMaxCurrent);

    // Set the shooter motor follower
    conveyorMotor1->Follow(*conveyorMotor);

    loaderMotor.reset(new rev::CANSparkMax(11, rev::CANSparkMax::MotorType::kBrushless));
    loaderMotor->SetSmartCurrentLimit(kLoaderMaxCurrent);
}

void PlasmaTank::InitDefaultCommand() {
    //SetDefaultCommand(new Lock(0.75));
}

void PlasmaTank::Periodic() {
    // Put code here to be run every loop


}

// run the conveyor, forward is positive, reverse is negative
void PlasmaTank::RunConveyor(double speed)
{
    conveyorMotor->Set(speed);
}

// Run the loader
void PlasmaTank::RunLoader(double speed)
{
    loaderMotor->Set(speed);
}

// Burn CANSparkMAX settings on motors
void PlasmaTank::Burn()
{
    conveyorMotor->BurnFlash();
    conveyorMotor1->BurnFlash();
    loaderMotor->BurnFlash();
}
