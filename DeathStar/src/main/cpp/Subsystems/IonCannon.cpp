// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "Subsystems/IonCannon.h"

#include "Commands/AimCamera.h"
#include "Commands/AimJoystick.h"

#include "frc/smartdashboard/SmartDashboard.h"

constexpr double kShooterMaxCurrent = 40.0;
constexpr double kTurretMaxCurrent = 20.0;


IonCannon::IonCannon() : frc::Subsystem("IonCannon") {

shooterMotor1.reset(new rev::CANSparkMax(12, rev::CANSparkMax::MotorType::kBrushless));
shooterMotor2.reset(new rev::CANSparkMax(13, rev::CANSparkMax::MotorType::kBrushless));
turretMotor.reset(new rev::CANSparkMax(14, rev::CANSparkMax::MotorType::kBrushless));
domeServo.reset(new frc::Servo(5));
AddChild("DomeServo", domeServo);

turretReferenceSwitch.reset(new frc::DigitalInput(4));
AddChild("TurretReferenceSwitch", turretReferenceSwitch);

// Set up encoders
//turretQuadEncoder.reset(new frc::Encoder(6, 7, false, frc::Encoder::k4X));
//AddChild("TurretQuadEncoder", turretQuadEncoder);
//turretQuadEncoder->SetDistancePerPulse(1.0);
//turretQuadEncoder->SetPIDSourceType(frc::PIDSourceType::kRate);
//
m_shooter1Encoder.reset(new rev::CANEncoder(*shooterMotor1));
turretQuadEncoder.reset(new rev::CANEncoder(*turretMotor));

// m_encoder.reset(new rev::CANEncoder(*leftElevatorMotor));
    

loadedSensor.reset(new frc::DigitalInput(12));
AddChild("LoadedSensor", loadedSensor);

    // limit current
    shooterMotor1->SetSmartCurrentLimit(kShooterMaxCurrent);
    shooterMotor2->SetSmartCurrentLimit(kShooterMaxCurrent);
    turretMotor->SetSmartCurrentLimit(kTurretMaxCurrent);
    
    // Set the shooter motor follower
    shooterMotor2->Follow(*shooterMotor1);
 
    //uint32_t lcpr = m_shooter1Encoder->GetCountsPerRevolution();
    //m_shooter1Encoder->SetPositionConversionFactor(2.0 * wpi::math::pi * (double)kWheelRadius * kGearRatio / lcpr);
     m_shooter1Encoder->SetPositionConversionFactor(1.0); 
    //uint32_t tcpr = turretQuadEncoder->GetCountsPerRevolution();
    //turretQuadEncoder->SetPositionConversionFactor(2.0 * wpi::math::pi * (double)kTurretRadius * kTGearRatio / tcpr);
      turretQuadEncoder->SetPositionConversionFactor(1.0);
} 


void IonCannon::InitDefaultCommand() {

   // SetDefaultCommand(new AimCamera());
    SetDefaultCommand(new AimJoystick());
}

void IonCannon::Periodic() {
    // Put code here to be run every loop
    frc::SmartDashboard::PutNumber("shooterSpd",m_shooter1Encoder->GetVelocity() );
    frc::SmartDashboard::PutNumber("turretPos",turretQuadEncoder->GetPosition() );
    frc::SmartDashboard::PutNumber("domePos",m_domeServo);

    
}

void IonCannon::AimCam(){
    float x = 0.;            // raw 0
    bool mMoving = false;

    if (Robot::limeAide->getLimeRoxInView()) {   
		double error = Robot::limeAide->getLimeRoxX(); 
		if (error > kCamTolerance) {
			double speed = min(kCamPower * error + kCamFriction, kCamLimit);
			// if (speed > kCamLimit)
			// 	speed = kCamLimit;
			x = float(speed);
			//turretMotor->Set(x);
			mMoving = true;
		}
		else if (error < -kCamTolerance) {
			double speed = max(kCamPower * error - kCamFriction, -kCamLimit);
			// if (speed < -kCamLimit)
			// 	speed = -kCamLimit;
			x = float(speed);
			//turretMotor->Set(x);
			mMoving = true;
		}
		else {
			//turretMotor->Set(x);
			mMoving = false;
		    turretMotor->StopMotor();
		}
	}
	else {
			//SPAMutil::Log("CamDrive","No data",SPAMutil::LOG_INFO);
			turretMotor->StopMotor();
			mMoving = false;
	}	

    if (x > 0){
        if (turretQuadEncoder->GetPosition() <= kLowLimit)
            turretMotor->StopMotor();
            x = 0;
        //else
            //turretMotor ->Set(-x);
    if (x < 0){
        if (turretQuadEncoder->GetPosition() >= kHighLimit)
            turretMotor->StopMotor();
            x = 0;
        //else
            //turretMotor ->Set(-x);
    }
    turretMotor ->Set(-x);
        
    }
	
	
}

// Shooter 
void IonCannon::SpinShooter(double speed)
{
    shooterMotor1->Set(speed);
}

void IonCannon::StopShooter()
{
    shooterMotor1->StopMotor();
}

void IonCannon::AimLeft()
{
    turretMotor->Set(-kTurretSpeed);
}
void IonCannon::AimRight()
{
    turretMotor->Set(kTurretSpeed);
}
void IonCannon::AimUp()
{
    domeServo->SetSpeed(kDomeSpeed);
}
void IonCannon::AimDown()
{
    domeServo->SetSpeed(-kDomeSpeed);
}

void IonCannon::SetServo(float value){
    m_domeServo = value / 2 + 0.5;
    domeServo->Set(m_domeServo);
}

void IonCannon::AimStop()
{
    turretMotor->StopMotor();
    domeServo->StopMotor();
}
void IonCannon::StopTurret()
{
        turretMotor->StopMotor();
}

void IonCannon::StopDome()
{
        domeServo->StopMotor();
}

// Burn CANSparkMAX settings on motors
void IonCannon::Burn()
{
    shooterMotor1->BurnFlash();
    shooterMotor2->BurnFlash();
    turretMotor->BurnFlash();
}
