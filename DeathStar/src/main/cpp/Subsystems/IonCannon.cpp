

#include "Subsystems/IonCannon.h"

#include "Commands/AimCamera.h"
#include "Commands/AimJoystick.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include <sstream>

constexpr double kShooterMaxCurrent = 40.0;
constexpr double kTurretMaxCurrent = 20.0;


IonCannon::IonCannon() : frc::Subsystem("IonCannon") {

    shooterMotor1.reset(new rev::CANSparkMax(12, rev::CANSparkMax::MotorType::kBrushless));
    shooterMotor2.reset(new rev::CANSparkMax(13, rev::CANSparkMax::MotorType::kBrushless));
    turretMotor.reset(new rev::CANSparkMax(14, rev::CANSparkMax::MotorType::kBrushless));
    domeServo.reset(new frc::Servo(4));
    domeServo2.reset(new frc::Servo(5));
    AddChild("DomeServo", domeServo);
    AddChild("DomeServo2", domeServo2);

    // Set up encoders
    shooter1Encoder.reset(new rev::CANEncoder(*shooterMotor1));
    turretQuadEncoder.reset(new rev::CANEncoder(*turretMotor));
    
    // initialize to zero, range is +/- 90 degrees
    turretQuadEncoder->SetPosition(0);

    // limit current
    shooterMotor1->SetSmartCurrentLimit(kShooterMaxCurrent);
    shooterMotor2->SetSmartCurrentLimit(kShooterMaxCurrent);
    turretMotor->SetSmartCurrentLimit(kTurretMaxCurrent);
    
    // Set the shooter motor follower
    shooterMotor2->Follow(*shooterMotor1);
 
    //uint32_t lcpr = shooter1Encoder->GetCountsPerRevolution();
    //shooter1Encoder->SetPositionConversionFactor(2.0 * wpi::math::pi * (double)kWheelRadius * kGearRatio / lcpr);
    //shooter1Encoder->SetPositionConversionFactor(1.0);

    //uint32_t tcpr = turretQuadEncoder->GetCountsPerRevolution();
    //turretQuadEncoder->SetPositionConversionFactor(2.0 * wpi::math::pi * (double)kTurretRadius * kTGearRatio / tcpr);
    //turretQuadEncoder->SetPositionConversionFactor(1.0);

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    //m_motor.RestoreFactoryDefaults();
  // default velocity PID coefficients
    shooterPIDController.reset(new rev::CANPIDController(*shooterMotor1));
    shooterPIDController->SetP(kvP);
    shooterPIDController->SetI(kvI);
    shooterPIDController->SetD(kvD);
    shooterPIDController->SetIZone(kvIz);
    shooterPIDController->SetFF(kvFF);
    shooterPIDController->SetOutputRange(kvMinOutput, kvMaxOutput);

      // default Position PID coefficients
    turretPIDController.reset(new rev::CANPIDController(*turretMotor));
    turretPIDController->SetP(kpP);
    turretPIDController->SetI(kpI);
    turretPIDController->SetD(kpD);
    turretPIDController->SetIZone(kpIz);
    turretPIDController->SetFF(kpFF);
    turretPIDController->SetOutputRange(kpMinOutput, kpMaxOutput);
} 


void IonCannon::InitDefaultCommand() {
    SPAMutil::Log("IonCannon", "InitDefaultCommand (aim joystick)", SPAMutil::LOG_DBG);

    //SetDefaultCommand(new AimCamera());
    SetDefaultCommand(new AimJoystick());
}

void IonCannon::Periodic() {

    // Put code here to be run every loop
    frc::SmartDashboard::PutNumber("shooterSpd",shooter1Encoder->GetVelocity() );
    frc::SmartDashboard::PutNumber("turretPos",GetTurretPosition() );
    frc::SmartDashboard::PutNumber("domePos",GetDomePosition());

    frc::SmartDashboard::PutNumber("servo 1 angle", domeServo->GetAngle());
    frc::SmartDashboard::PutNumber("servo 2 angle", domeServo2->GetAngle());
    //frc::SmartDashboard::PutNumber("servo 1 pos", domeServo->GetPosition());
    //frc::SmartDashboard::PutNumber("servo 2 pos", domeServo2->GetPosition());

    //ShooterPidControl();
    TurretPidControl();
}

void IonCannon::AimCamPosition() {
    SPAMutil::Log("IonCannon", "AimCamPosition", SPAMutil::LOG_DBG);
    if (Robot::limeAide->getLimeRoxInView()) {
		double tx = Robot::limeAide->getLimeRoxX();
        double dx = GetTurretPosition() - (tx * kTurretXFactor);
        if (dx < kTurretLowLimit)
            dx = kTurretLowLimit;
        if (dx > kTurretHighLimit)
            dx = kTurretHighLimit;
        frc::SmartDashboard::PutNumber("target x",tx);
        frc::SmartDashboard::PutNumber("get turret position",GetTurretPosition());
        frc::SmartDashboard::PutNumber("set turret position",dx);

        SetTurretPosition(dx);
    }
    else {
        frc::SmartDashboard::PutNumber("stop turret ", 0);
        AimStop();
    }
  
}

void IonCannon::AimCam() {
    SPAMutil::Log("IonCannon", "AimCam", SPAMutil::LOG_DBG);
    float x = 0.;            // raw 0
    bool mMoving = false;

    if (Robot::limeAide->getLimeRoxInView()) {   
		double error = Robot::limeAide->getLimeRoxX(); 
		if (error > kCamTolerance) {
			double speed = kCamPower * error + kCamFriction;
			if (speed > kCamLimit)
				speed = kCamLimit;
			x = float(speed);
			//turretMotor->Set(x);
			mMoving = true;
		}
		else if (error < -kCamTolerance) {
			double speed = kCamPower * error - kCamFriction;
			if (speed < -kCamLimit)
				speed = -kCamLimit;
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
  /* why does turretQuadEncoder->GetPosition() return -90?
    if (x > 0) {
        if (turretQuadEncoder->GetPosition() <= kTurretLowLimit)
            turretMotor->StopMotor();
            x = 0;
        //else
            //turretMotor ->Set(-x);
    }
    if (x < 0){
        if (turretQuadEncoder->GetPosition() >= kTurretHighLimit)
            turretMotor->StopMotor();
            x = 0;
        //else
            //turretMotor ->Set(-x);
    }
  */
    turretMotor ->Set(-x);
    frc::SmartDashboard::PutNumber("turretSpd",x );
	
}

// Shooter 
void IonCannon::SpinShooter(double speed)
{
    std::stringstream ss;
    ss << "SpinShooter(" << speed << ")";
    SPAMutil::Log("IonCannon", ss.str().c_str(), SPAMutil::LOG_DBG);

    shooterMotor1->Set(speed);
}

void IonCannon::StopShooter()
{
    SPAMutil::Log("IonCannon", "StopShooter", SPAMutil::LOG_DBG);
    shooterMotor1->StopMotor();
}

void IonCannon::AimLeft()
{
    SPAMutil::Log("IonCannon", "AimLeft", SPAMutil::LOG_DBG);
    turretMotor->Set(kTurretSpeed);
}
void IonCannon::AimRight()
{
    SPAMutil::Log("IonCannon", "AimRight", SPAMutil::LOG_DBG);
    turretMotor->Set(-kTurretSpeed);
}
void IonCannon::AimUp()
{
    SPAMutil::Log("IonCannon", "AimUp", SPAMutil::LOG_DBG);
    SetDomePosition(GetDomePosition() - 1);
    //domeServo->SetSpeed(kDomeSpeed);
    //domeServo2->SetSpeed(-kDomeSpeed);
}
void IonCannon::AimDown()
{
    SPAMutil::Log("IonCannon", "AimDown", SPAMutil::LOG_DBG);
    SetDomePosition(GetDomePosition() + 1);
    //domeServo->SetSpeed(-kDomeSpeed);
    //domeServo2->SetSpeed(kDomeSpeed);
}

// this takes a value between -1 to 1, i.e. joystick
// now this takes a value between 0 and 90 degrees
void IonCannon::SetServo(float value)
{
    std::stringstream ss;
    ss << "SetServo(" << value << ")";
    SPAMutil::Log("IonCannon", ss.str().c_str(), SPAMutil::LOG_DBG);

// on left servo (from behind the robot), 0 is down, shoots higher, 1 is up, shoots lower
    if (value > 90.0)
        value = 90.0;
    if (value < 0.0)
        value = 0.0;

    domeServo->SetAngle(value);
    domeServo2->SetAngle(180.0 - value);
/*
    m_domeServo = value / 2 + 0.5;
    if (m_domeServo > 1.0)
        m_domeServo = 1.0;
    if (m_domeServo < 0.0)
        m_domeServo = 0.0;

    // send a value between 0 and 1
    domeServo->Set(m_domeServo);
    domeServo2->Set(m_domeServo);
    */
}

void IonCannon::AimStop()
{
    SPAMutil::Log("IonCannon", "AimStop", SPAMutil::LOG_DBG);
    turretMotor->StopMotor();
    //domeServo->StopMotor();
    //domeServo2->StopMotor();
}
void IonCannon::StopTurret()
{
    SPAMutil::Log("IonCannon", "StopTurret", SPAMutil::LOG_DBG);
    turretMotor->StopMotor();
}

void IonCannon::StopDome()
{
    SPAMutil::Log("IonCannon", "StopDome", SPAMutil::LOG_DBG);
    //domeServo->StopMotor();
    //domeServo2->StopMotor();
}

// Burn CANSparkMAX settings on motors
void IonCannon::Burn()
{
    SPAMutil::Log("IonCannon", "Burn", SPAMutil::LOG_DBG);
    shooterMotor1->BurnFlash();
    shooterMotor2->BurnFlash();
    turretMotor->BurnFlash();
}

double IonCannon::GetTurretPosition()
{
    double pos = turretQuadEncoder->GetPosition();
/*
    std::stringstream ss;
    ss << "GetTurretPosition() == " << pos;
    SPAMutil::Log("IonCannon", ss.str().c_str(), SPAMutil::LOG_DBG);
*/
    return pos;
}

void IonCannon::SetTurretPosition(double position)
{
    std::stringstream ss;
    ss << "SetTurretPosition(" << position << ")" ;
    SPAMutil::Log("IonCannon", ss.str().c_str(), SPAMutil::LOG_DBG);

    turretQuadEncoder->SetPosition(position);
}

double IonCannon::GetDomePosition()
{
    double pos = domeServo->GetAngle();

/*
    std::stringstream ss;
    ss << "GetDomePosition() == " << pos;
    SPAMutil::Log("IonCannon", ss.str().c_str(), SPAMutil::LOG_DBG);
*/
    return pos;
}

// this takes a value between 0 and 90 degrees
void IonCannon::SetDomePosition(double value)
{
    std::stringstream ss;
    ss << "SetDomePosition(" << value << ")";
    SPAMutil::Log("IonCannon", ss.str().c_str(), SPAMutil::LOG_DBG);

    // on left servo (from behind the robot), 0 is down, shoots higher, 1 is up, shoots lower
    if (value > 90.0)
        value = 90.0;
    if (value < 0.0)
        value = 0.0;

    domeServo->SetAngle(value);
    domeServo2->SetAngle(180.0 - value);
}

void IonCannon::ShooterPidControl()
{
    // display PID coefficients on SmartDashboard
    frc::SmartDashboard::PutNumber("P Gain", kvP);
    frc::SmartDashboard::PutNumber("I Gain", kvI);
    frc::SmartDashboard::PutNumber("D Gain", kvD);
    frc::SmartDashboard::PutNumber("I Zone", kvIz);
    frc::SmartDashboard::PutNumber("Feed Forward", kvFF);
    frc::SmartDashboard::PutNumber("Max Output", kvMaxOutput);
    frc::SmartDashboard::PutNumber("Min Output", kvMinOutput);
    frc::SmartDashboard::PutNumber("Set Rotations", 0);

    // read PID coefficients from SmartDashboard
    double p = frc::SmartDashboard::GetNumber("P Gain", 0);
    double i = frc::SmartDashboard::GetNumber("I Gain", 0);
    double d = frc::SmartDashboard::GetNumber("D Gain", 0);
    double iz = frc::SmartDashboard::GetNumber("I Zone", 0);
    double ff = frc::SmartDashboard::GetNumber("Feed Forward", 0);
    double max = frc::SmartDashboard::GetNumber("Max Output", 0);
    double min = frc::SmartDashboard::GetNumber("Min Output", 0);
    double rotations = frc::SmartDashboard::GetNumber("Set Rotations", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kvP)) { shooterPIDController->SetP(p); kvP = p; }
    if((i != kvI)) { shooterPIDController->SetI(i); kvI = i; }
    if((d != kvD)) { shooterPIDController->SetD(d); kvD = d; }
    if((iz != kvIz)) { shooterPIDController->SetIZone(iz); kvIz = iz; }
    if((ff != kvFF)) { shooterPIDController->SetFF(ff); kvFF = ff; }
    if((max != kvMaxOutput) || (min != kvMinOutput)) {
      shooterPIDController->SetOutputRange(min, max);
      kvMinOutput = min; kvMaxOutput = max;
    }
    /**
     * PIDController objects are commanded to a set point using the
     * SetReference() method.
     *
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     *
     * The second parameter is the control type can be set to one of four
     * parameters:
     *  rev::ControlType::kDutyCycle
     *  rev::ControlType::kPosition
     *  rev::ControlType::kVelocity
     *  rev::ControlType::kVoltage
     */
    shooterPIDController->SetReference(rotations, rev::ControlType::kPosition);

    frc::SmartDashboard::PutNumber("SetPoint", rotations);
    frc::SmartDashboard::PutNumber("ProcessVariable", shooter1Encoder->GetPosition());
}

void IonCannon::TurretPidControl()
{
    // display PID coefficients on SmartDashboard
    frc::SmartDashboard::PutNumber("P Gain", kpP);
    frc::SmartDashboard::PutNumber("I Gain", kpI);
    frc::SmartDashboard::PutNumber("D Gain", kpD);
    frc::SmartDashboard::PutNumber("I Zone", kpIz);
    frc::SmartDashboard::PutNumber("Feed Forward", kpFF);
    frc::SmartDashboard::PutNumber("Max Output", kpMaxOutput);
    frc::SmartDashboard::PutNumber("Min Output", kpMinOutput);
    frc::SmartDashboard::PutNumber("Set Rotations", 0);

    // read PID coefficients from SmartDashboard
    double p = frc::SmartDashboard::GetNumber("P Gain", 0);
    double i = frc::SmartDashboard::GetNumber("I Gain", 0);
    double d = frc::SmartDashboard::GetNumber("D Gain", 0);
    double iz = frc::SmartDashboard::GetNumber("I Zone", 0);
    double ff = frc::SmartDashboard::GetNumber("Feed Forward", 0);
    double max = frc::SmartDashboard::GetNumber("Max Output", 0);
    double min = frc::SmartDashboard::GetNumber("Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kpP)) { turretPIDController->SetP(p); kpP = p; }
    if((i != kpI)) { turretPIDController->SetI(i); kpI = i; }
    if((d != kpD)) { turretPIDController->SetD(d); kpD = d; }
    if((iz != kpIz)) { turretPIDController->SetIZone(iz); kpIz = iz; }
    if((ff != kpFF)) { turretPIDController->SetFF(ff); kpFF = ff; }
    if((max != kpMaxOutput) || (min != kpMinOutput)) {
      turretPIDController->SetOutputRange(min, max);
      kpMinOutput = min; kpMaxOutput = max;
    }

    // read setpoint from joystick and scale by max rpm
    //double SetPoint = 0.0;// = MaxRPM*m_stick.GetY();
    double SetPoint = frc::SmartDashboard::GetNumber("SetPoint", 0.0);


    turretPIDController->SetReference(SetPoint, rev::ControlType::kVelocity);

    frc::SmartDashboard::PutNumber("SetPoint", SetPoint);
    frc::SmartDashboard::PutNumber("ProcessVariable", turretQuadEncoder->GetVelocity());

}
