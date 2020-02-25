
#include "Subsystems/Hyperdrive.h"
#include "Commands/JoyDriveCommand.h"


const double    kDriveMotorLimitAmps = 60.0;


Hyperdrive::Hyperdrive() : frc::Subsystem("Hyperdrive") {

    leftMotor1.reset(new rev::CANSparkMax(1, rev::CANSparkMax::MotorType::kBrushless));
    rightMotor1.reset(new rev::CANSparkMax(3, rev::CANSparkMax::MotorType::kBrushless));

    differentialDrive.reset(new frc::DifferentialDrive(*leftMotor1, *rightMotor1));
    AddChild("DifferentialDrive", differentialDrive);
    differentialDrive->SetSafetyEnabled(true);
    differentialDrive->SetExpiration(0.1);
    differentialDrive->SetMaxOutput(1.0);

    leftMotor2.reset(new rev::CANSparkMax(2, rev::CANSparkMax::MotorType::kBrushless));
    rightMotor2.reset(new rev::CANSparkMax(4, rev::CANSparkMax::MotorType::kBrushless));

    rightMotor1->SetInverted(true);
    rightMotor2->SetInverted(true);
    leftMotor1->SetInverted(false);
    leftMotor2->SetInverted(false);

    // set current limits
    leftMotor1->SetSmartCurrentLimit(kDriveMotorLimitAmps);
    leftMotor2->SetSmartCurrentLimit(kDriveMotorLimitAmps);
    rightMotor1->SetSmartCurrentLimit(kDriveMotorLimitAmps);
    rightMotor2->SetSmartCurrentLimit(kDriveMotorLimitAmps);

    // Set the followers
    leftMotor2->Follow(*leftMotor1);
    rightMotor2->Follow(*rightMotor1);

    // get encoders
    m_leftEncoder.reset(new rev::CANEncoder(*leftMotor1));
    m_rightEncoder.reset(new rev::CANEncoder(*rightMotor1));

    // Set the distance per pulse for the drive encoders. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    //leftEncoder->SetDistancePerPulse(Z2 * wpi::math::pi * kWheelRadius / kEncoderResolution);
    //rightEncoder->SetDistancePerPulse(2 * wpi::math::pi * kWheelRadius / kEncoderResolution);

    // Set encoder conversion factors, either calculate as above or measure with robot
    //double factor = 2.0 * wpi::math::pi * (double)kWheelRadius * kGearRatio / kEncoderResolution;
    uint32_t lcpr = m_leftEncoder->GetCountsPerRevolution();
    uint32_t rcpr = m_rightEncoder->GetCountsPerRevolution();

    m_leftEncoder->SetPositionConversionFactor(2.0 * wpi::math::pi * (double)kWheelRadius * kGearRatio / lcpr);
    m_rightEncoder->SetPositionConversionFactor(2.0 * wpi::math::pi * (double)kWheelRadius * kGearRatio / rcpr);
    m_leftEncoder->SetVelocityConversionFactor(2.0 * wpi::math::pi * (double)kWheelRadius * kGearRatio / lcpr);
    m_rightEncoder->SetVelocityConversionFactor(2.0 * wpi::math::pi * (double)kWheelRadius * kGearRatio / rcpr);

    // Reset sensors
    //ResetPose();
}

void Hyperdrive::InitDefaultCommand()
{
    // Set the default command for a subsystem here.
    // SetDefaultCommand(new MySpecialCommand());

    SetDefaultCommand(new JoyDriveCommand());

}


void Hyperdrive::Periodic()
{
    //UpdateOdometry();
    
}

frc::Rotation2d Hyperdrive::GetAngle() const
{
    // Negating the angle because WPILib Gyros are CW positive.
    return frc::Rotation2d(units::radian_t(0));
    //return frc::Rotation2d(units::degree_t(-Robot::lifeSupport->gyroAngle()));
}

// Put methods for controlling this subsystem
// here. Call these from Commands.

void Hyperdrive::DriveArcade(std::shared_ptr<frc::Joystick> j)
{

    float x = j->GetX();            // raw 0
    float y = j->GetY();            // raw 1
    // rev going to fwd
    if(m_Yspeed < 0. && y > m_Yspeed) {
	   m_Yspeed = SPAMutil::RateLimitPWM(m_Yspeed, y, kRateLimit);
    } 
    else if(m_Yspeed > 0. && y < m_Yspeed) {
	   m_Yspeed = SPAMutil::RateLimitPWM(m_Yspeed, y, kRateLimit);  
    }
    else if(y > -0.078 && y < 0.078) {
	   m_Yspeed = 0; 
    }
    else {
       m_Yspeed = y; 
    }
    
    //differentialDrive->ArcadeDrive(x, -y, true);
    differentialDrive->ArcadeDrive(x, -m_Yspeed, true);

    frc::SmartDashboard::PutNumber("joyY", y);
    frc::SmartDashboard::PutNumber("joyLPY", m_Yspeed);
}


// Burn CANSparkMAX settings on motors
void Hyperdrive::Burn()
{
    leftMotor1->BurnFlash();
    leftMotor2->BurnFlash();
    rightMotor1->BurnFlash();
    rightMotor2->BurnFlash();
}


void Hyperdrive::ResetPose()
{
    // reset gyro and encoders
    Robot::lifeSupport->gyroReset();
    m_leftEncoder->SetPosition(0);
    m_rightEncoder->SetPosition(0);
}
void Hyperdrive::SetSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds)
{
    const auto leftOutput = m_leftPIDController.Calculate(m_leftEncoder->GetVelocity(), speeds.left.to<double>());
    const auto rightOutput = m_rightPIDController.Calculate(m_rightEncoder->GetVelocity(), speeds.right.to<double>());

    leftMotor1->Set(leftOutput);
    rightMotor1->Set(rightOutput);
}

void Hyperdrive::Drive(units::meters_per_second_t xSpeed,
                       units::radians_per_second_t rot)
{
    SetSpeeds(m_kinematics.ToWheelSpeeds({xSpeed, 0_mps, rot}));
}

void Hyperdrive::UpdateOdometry() {
    m_odometry.Update(GetAngle(), units::meter_t(m_leftEncoder->GetPosition()),
                                  units::meter_t(m_rightEncoder->GetPosition()));
}
