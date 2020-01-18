// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#pragma once

#include "frc/Joystick.h"
#include "frc/buttons/JoystickButton.h"

class OI {
private:
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS




















std::shared_ptr<frc::Joystick> operatorJoystick;
std::shared_ptr<frc::JoystickButton> spinWheelFieldBtn;
std::shared_ptr<frc::JoystickButton> spinWheelRightBtn;
std::shared_ptr<frc::JoystickButton> spinWheelLeftBtn;
std::shared_ptr<frc::JoystickButton> spitFuelBtn;
std::shared_ptr<frc::JoystickButton> suckFuelBtn;
std::shared_ptr<frc::JoystickButton> toggleTractorBeamBtn;
std::shared_ptr<frc::JoystickButton> oPShootBtn;
std::shared_ptr<frc::JoystickButton> ejectFuelBtn;
std::shared_ptr<frc::JoystickButton> aimDownBtn;
std::shared_ptr<frc::JoystickButton> aimUpBtn;
std::shared_ptr<frc::JoystickButton> aimRightBtn;
std::shared_ptr<frc::JoystickButton> aimLeftBtn;
std::shared_ptr<frc::Joystick> driverJoystick;
std::shared_ptr<frc::JoystickButton> gravitateDownBtn;
std::shared_ptr<frc::JoystickButton> gravitateUpBtn;
std::shared_ptr<frc::JoystickButton> shootBtn;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
public:
	OI();

	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=PROTOTYPES

std::shared_ptr<frc::Joystick> getDriverJoystick();
std::shared_ptr<frc::Joystick> getOperatorJoystick();

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=PROTOTYPES
};

