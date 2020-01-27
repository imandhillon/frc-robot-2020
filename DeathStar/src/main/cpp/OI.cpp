// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "OI.h"

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
#include "frc/smartdashboard/SmartDashboard.h"
#include "Commands/AimCamera.h"
#include "Commands/AimDown.h"
#include "Commands/AimLeft.h"
#include "Commands/AimRight.h"
#include "Commands/AimUp.h"
#include "Commands/AutonomousCommand.h"
#include "Commands/DeployTractorBeam.h"
#include "Commands/DriveStraight.h"
#include "Commands/DriveTurn.h"
#include "Commands/Eject.h"
#include "Commands/Fire.h"
#include "Commands/FuelGrab.h"
#include "Commands/Gravitate.h"
#include "Commands/JoyDriveCommand.h"
#include "Commands/Load.h"
#include "Commands/Lock.h"
#include "Commands/RetractTractorBeam.h"
#include "Commands/SpinShooter.h"
#include "Commands/SpinWheel.h"
#include "Commands/SpinWheelField.h"


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

OI::OI() {
    // Process operator interface input here.
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
operatorJoystick.reset(new frc::Joystick(1));

spinWheelFieldBtn.reset(new frc::JoystickButton(operatorJoystick.get(), 17));
spinWheelFieldBtn->WhenPressed(new SpinWheelField());
spinWheelRightBtn.reset(new frc::JoystickButton(operatorJoystick.get(), 16));
spinWheelRightBtn->WhileHeld(new SpinWheel(0.5));
spinWheelLeftBtn.reset(new frc::JoystickButton(operatorJoystick.get(), 15));
spinWheelLeftBtn->WhileHeld(new SpinWheel(-.5));
spitFuelBtn.reset(new frc::JoystickButton(operatorJoystick.get(), 12));
spitFuelBtn->WhileHeld(new FuelGrab(-0.5));
suckFuelBtn.reset(new frc::JoystickButton(operatorJoystick.get(), 11));
suckFuelBtn->WhileHeld(new FuelGrab(0.5));
toggleTractorBeamBtn.reset(new frc::JoystickButton(operatorJoystick.get(), 10));
toggleTractorBeamBtn->WhenPressed(new DeployTractorBeam());
oPShootBtn.reset(new frc::JoystickButton(operatorJoystick.get(), 1));
oPShootBtn->WhenPressed(new Fire());
ejectFuelBtn.reset(new frc::JoystickButton(operatorJoystick.get(), 22));
ejectFuelBtn->WhenPressed(new Eject());
aimDownBtn.reset(new frc::JoystickButton(operatorJoystick.get(), 5));
aimDownBtn->WhileHeld(new AimDown());
aimUpBtn.reset(new frc::JoystickButton(operatorJoystick.get(), 4));
aimUpBtn->WhileHeld(new AimUp());
aimRightBtn.reset(new frc::JoystickButton(operatorJoystick.get(), 3));
aimRightBtn->WhileHeld(new AimRight());
aimLeftBtn.reset(new frc::JoystickButton(operatorJoystick.get(), 2));
aimLeftBtn->WhileHeld(new AimLeft());
driverJoystick.reset(new frc::Joystick(0));

gravitateDownBtn.reset(new frc::JoystickButton(driverJoystick.get(), 3));
gravitateDownBtn->WhileHeld(new Gravitate(-0.5));
gravitateUpBtn.reset(new frc::JoystickButton(driverJoystick.get(), 2));
gravitateUpBtn->WhileHeld(new Gravitate(0.5));
shootBtn.reset(new frc::JoystickButton(driverJoystick.get(), 1));
shootBtn->WhenPressed(new Fire());

    // SmartDashboard Buttons
    frc::SmartDashboard::PutData("Gravitate: stop", new Gravitate(0));
    frc::SmartDashboard::PutData("Gravitate: up", new Gravitate(0.5));
    frc::SmartDashboard::PutData("Gravitate: down", new Gravitate(-0.5));
    frc::SmartDashboard::PutData("RetractTractorBeam", new RetractTractorBeam());
    frc::SmartDashboard::PutData("DeployTractorBeam", new DeployTractorBeam());
    frc::SmartDashboard::PutData("FuelGrab: stop", new FuelGrab(0));
    frc::SmartDashboard::PutData("FuelGrab: grab", new FuelGrab(-0.5));
    frc::SmartDashboard::PutData("FuelGrab: spit", new FuelGrab(0.5));
    frc::SmartDashboard::PutData("SpinWheelField", new SpinWheelField());
    frc::SmartDashboard::PutData("SpinWheel: stop", new SpinWheel(0));
    frc::SmartDashboard::PutData("SpinWheel: left", new SpinWheel(-0.5));
    frc::SmartDashboard::PutData("SpinWheel: right", new SpinWheel(0.5));
    frc::SmartDashboard::PutData("Load: stop", new Load(0));
    frc::SmartDashboard::PutData("Load: eject", new Load(-0.5));
    frc::SmartDashboard::PutData("Load: fire", new Load(0.9));
    frc::SmartDashboard::PutData("Lock: stop", new Lock(0));
    frc::SmartDashboard::PutData("Lock: run", new Lock(0.4));
    frc::SmartDashboard::PutData("Eject", new Eject());
    frc::SmartDashboard::PutData("SpinShooter", new SpinShooter());
    frc::SmartDashboard::PutData("AimCamera", new AimCamera());
    frc::SmartDashboard::PutData("AimDown", new AimDown());
    frc::SmartDashboard::PutData("AimUp", new AimUp());
    frc::SmartDashboard::PutData("AimRight", new AimRight());
    frc::SmartDashboard::PutData("AimLeft", new AimLeft());
    frc::SmartDashboard::PutData("Fire", new Fire());
    frc::SmartDashboard::PutData("DriveTurn: stop", new DriveTurn(0));
    frc::SmartDashboard::PutData("DriveTurn: left", new DriveTurn(-0.5));
    frc::SmartDashboard::PutData("DriveTurn: right", new DriveTurn(0.5));
    frc::SmartDashboard::PutData("DriveStraight: stop", new DriveStraight(0));
    frc::SmartDashboard::PutData("DriveStraight: rev", new DriveStraight(-0.5));
    frc::SmartDashboard::PutData("DriveStraight: forward", new DriveStraight(0.5));

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

}

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS

std::shared_ptr<frc::Joystick> OI::getDriverJoystick() {
   return driverJoystick;
}

std::shared_ptr<frc::Joystick> OI::getOperatorJoystick() {
   return operatorJoystick;
}


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
