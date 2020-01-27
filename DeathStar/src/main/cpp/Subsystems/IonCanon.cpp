// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "Subsystems/IonCanon.h"
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
#include "Commands/AimCamera.h"

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

IonCanon::IonCanon() : frc::Subsystem("IonCanon") {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
shooterMotor1.reset(new rev::CANSparkMax(5, rev::CANSparkMax::MotorType::kBrushless));
//lw->AddActuator("IonCanon", "ShooterMotor1", shooterMotor1);

shooterMotor2.reset(new rev::CANSparkMax(6, rev::CANSparkMax::MotorType::kBrushless));
//lw->AddActuator("IonCanon", "ShooterMotor2", shooterMotor2);

turretMotor.reset(new rev::CANSparkMax(7, rev::CANSparkMax::MotorType::kBrushless));
//lw->AddActuator("IonCanon", "TurretMotor", turretMotor);

domeServo.reset(new frc::Servo(4));
AddChild("DomeServo", domeServo);

turretReferenceSwitch.reset(new frc::DigitalInput(5));
AddChild("TurretReferenceSwitch", turretReferenceSwitch);

turretQuadEncoder.reset(new frc::Encoder(6, 7, false, frc::Encoder::k4X));
AddChild("TurretQuadEncoder", turretQuadEncoder);
turretQuadEncoder->SetDistancePerPulse(1.0);
turretQuadEncoder->SetPIDSourceType(frc::PIDSourceType::kRate);
loadedSensor.reset(new frc::DigitalInput(12));
AddChild("LoadedSensor", loadedSensor);


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    // Set the shooter motor follower
    shooterMotor2->Follow(*shooterMotor1);
}

void IonCanon::InitDefaultCommand() {
    // Set the default command for a subsystem here.
    // SetDefaultCommand(new MySpecialCommand());
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        SetDefaultCommand(new AimCamera());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
}

void IonCanon::Periodic() {
    // Put code here to be run every loop

}

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS


// Put methods for controlling this subsystem
// here. Call these from Commands.

// Shooter 
void IonCanon::LoadFuel()
{
    
}

void IonCanon::EjectFuel()
{

}

void IonCanon::Shoot()
{

}

void IonCanon::SpinUpShooter()
{
    shooterMotor1->Set(1.0);
}

void IonCanon::SpinDownShooter()
{
    shooterMotor1->Set(0.0);
}

// Turn Turret
void IonCanon::AimCam()
{

}
void IonCanon::AimLeft()
{
    turretMotor->Set(-0.4);
}
void IonCanon::AimRight()
{
    turretMotor->Set(0.4);
}
void IonCanon::AimUp()
{
    domeServo->SetSpeed(0.5);
}
void IonCanon::AimDown()
{
    domeServo->SetSpeed(-0.5);
}
void IonCanon::AimStop()
{
    turretMotor->StopMotor();
    domeServo->StopMotor();
}
