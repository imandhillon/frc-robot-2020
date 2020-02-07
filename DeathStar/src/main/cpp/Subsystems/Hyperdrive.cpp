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


//const double    kWheelWidth = 30.0;
//const double    kWheelBase = 17.5;
const double    kDriveMotorLimitAmps = 60.0;


Hyperdrive::Hyperdrive() : frc::Subsystem("Hyperdrive") {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
leftMotor1.reset(new rev::CANSparkMax(1, rev::CANSparkMax::MotorType::kBrushless));
//lw->AddActuator("Hyperdrive", "LeftMotor1", leftMotor1);

rightMotor1.reset(new rev::CANSparkMax(3, rev::CANSparkMax::MotorType::kBrushless));

//lw->AddActuator("Hyperdrive", "RightMotor1", rightMotor1);

differentialDrive.reset(new frc::DifferentialDrive(*leftMotor1, *rightMotor1));
AddChild("DifferentialDrive", differentialDrive);
differentialDrive->SetSafetyEnabled(true);
differentialDrive->SetExpiration(0.1);
differentialDrive->SetMaxOutput(1.0);

leftMotor2.reset(new rev::CANSparkMax(2, rev::CANSparkMax::MotorType::kBrushless));
//lw->AddActuator("Hyperdrive", "LeftMotor2", leftMotor2);

rightMotor2.reset(new rev::CANSparkMax(4, rev::CANSparkMax::MotorType::kBrushless));
//lw->AddActuator("Hyperdrive", "RightMotor2", rightMotor2);


rightMotor1->SetInverted(true);
rightMotor2->SetInverted(true);
leftMotor1->SetInverted(false);
leftMotor2->SetInverted(false);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

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
    //leftEncoder->SetDistancePerPulse(2 * wpi::math::pi * kWheelRadius / kEncoderResolution);
    //rightEncoder->SetDistancePerPulse(2 * wpi::math::pi * kWheelRadius / kEncoderResolution);

    // Set encoder conversion factors, either calculate as above or measure with robot
    /*
    m_leftEncoder->SetPositionConversionFactor(factor);
    m_rightEncoder->SetPositionConversionFactor(factor);
    m_leftEncoder->SetVelocityConversionFactor(factor);
    m_rightEncoder->SetVelocityConversionFactor(factor);
    */

    // Reset sensors
    ResetPose();
}

void Hyperdrive::InitDefaultCommand()
{
    // Set the default command for a subsystem here.
    // SetDefaultCommand(new MySpecialCommand());
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        //SetDefaultCommand(new JoyDriveCommand());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
}

void Hyperdrive::Periodic()
{
    UpdateOdometry();
}

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

frc::Rotation2d Hyperdrive::GetAngle() const
{
    // Negating the angle because WPILib Gyros are CW positive.
    return frc::Rotation2d(units::degree_t(-Robot::lifeSupport->gyroAngle()));
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
void Hyperdrive::DriveArcade(std::shared_ptr<frc::Joystick> j)
{
    float x = j->GetX();            // raw 0
    float y = j->GetY();            // raw 1
    differentialDrive->ArcadeDrive(x, -y, true);
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
