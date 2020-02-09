// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "Subsystems/PlasmaTank.h"
#include "Commands/Lock.h"

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

constexpr double kConveyorMaxCurrent = 20.0;
constexpr double kLoaderMaxCurrent = 60.0;

PlasmaTank::PlasmaTank() : frc::Subsystem("PlasmaTank") {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
conveyorMotor.reset(new rev::CANSparkMax(9, rev::CANSparkMax::MotorType::kBrushless));
conveyorMotor1.reset(new rev::CANSparkMax(10, rev::CANSparkMax::MotorType::kBrushless));
//lw->AddActuator("PlasmaTank", "LoaderMotor", loaderMotor);

loaderMotor.reset(new rev::CANSparkMax(11, rev::CANSparkMax::MotorType::kBrushless));
//conveyorMotor.reset(new ctre::phoenix::motorcontrol::can::WPI_VictorSPX(10));



loadForce.reset(new frc::AnalogInput(0));
AddChild("LoadForce", loadForce);

  // Set the shooter motor follower
    conveyorMotor1->Follow(*conveyorMotor);


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    conveyorMotor->SetSmartCurrentLimit(kConveyorMaxCurrent);
    conveyorMotor1->SetSmartCurrentLimit(kConveyorMaxCurrent);
    //loaderMotor->SetSmartCurrentLimit(kLoaderMaxCurrent);
}

void PlasmaTank::InitDefaultCommand() {
    // Set the default command for a subsystem here.
    // SetDefaultCommand(new MySpecialCommand());
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
    SetDefaultCommand(new Lock(0.75));

}

void PlasmaTank::Periodic() {
    // Put code here to be run every loop
         //SetDefaultCommand(new Gravitate(0.));


}

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS


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
    //loaderMotor->BurnFlash();
}
