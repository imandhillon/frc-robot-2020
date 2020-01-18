// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "Subsystems/Hyperdrive.h"
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
#include "Commands/JoyDriveCommand.h"

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

Hyperdrive::Hyperdrive() : frc::Subsystem("Hyperdrive") {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
leftMotor.reset(new frc::NidecBrushless(0, 0));
AddChild("LeftMotor", leftMotor);

rightMotor.reset(new frc::NidecBrushless(1, 1));
AddChild("RightMotor", rightMotor);

differentialDrive.reset(new frc::DifferentialDrive(*leftMotor, *rightMotor));
AddChild("DifferentialDrive", differentialDrive);
differentialDrive->SetSafetyEnabled(true);
differentialDrive->SetExpiration(0.1);
differentialDrive->SetMaxOutput(1.0);

pigeonGPS.reset(new CANCoder(2));



    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
}

void Hyperdrive::InitDefaultCommand() {
    // Set the default command for a subsystem here.
    // SetDefaultCommand(new MySpecialCommand());
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        SetDefaultCommand(new JoyDriveCommand());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
}

void Hyperdrive::Periodic() {
    // Put code here to be run every loop

}

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS


// Put methods for controlling this subsystem
// here. Call these from Commands.
void Hyperdrive::DriveArcade(std::shared_ptr<frc::Joystick> j)
{
    float x = j->GetX();            // raw 0
    float y = j->GetY();            // raw 1
    differentialDrive->ArcadeDrive(x, -y, true);
}

