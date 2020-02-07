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

#include "frc/commands/Subsystem.h"
#include "frc/Joystick.h"

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
#include "frc/drive/DifferentialDrive.h"
#include "rev/CANSparkMax.h"

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
#include <frc/controller/PIDController.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <units/units.h>
#include <wpi/math>

/**
 *
 *
 * @author ExampleAuthor
 */
class Hyperdrive: public frc::Subsystem {
private:
  static constexpr units::meter_t kTrackWidth = 27.0_in;//0.381_m * 2;
  static constexpr units::meter_t kWheelRadius = 3.0_in;
  static constexpr int kEncoderResolution = 4096;

	// It's desirable that everything possible is private except
	// for methods that implement subsystem capabilities
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
std::shared_ptr<rev::CANSparkMax> leftMotor1;
std::shared_ptr<rev::CANSparkMax> rightMotor1;
std::shared_ptr<frc::DifferentialDrive> differentialDrive;
std::shared_ptr<rev::CANSparkMax> leftMotor2;
std::shared_ptr<rev::CANSparkMax> rightMotor2;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

	std::shared_ptr<rev::CANEncoder> m_leftEncoder;
	std::shared_ptr<rev::CANEncoder> m_rightEncoder;

	frc2::PIDController m_leftPIDController{1.0, 0.0, 0.0};
  	frc2::PIDController m_rightPIDController{1.0, 0.0, 0.0};
  	frc::DifferentialDriveKinematics m_kinematics{kTrackWidth};
  	frc::DifferentialDriveOdometry m_odometry{GetAngle()};

  	static constexpr units::meters_per_second_t kMaxSpeed =      3.0_mps;  // 3 meters per second
  	static constexpr units::radians_per_second_t kMaxAngularSpeed{wpi::math::pi};  // 1/2 rotation per second

  	void SetSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds);
  	void UpdateOdometry();

public:
Hyperdrive();
	void InitDefaultCommand() override;
	void Periodic() override;
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS
	// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

  	/**
   	 * Get the robot angle as a Rotation2d.
   	 */
  	frc::Rotation2d GetAngle() const;
	void ResetPose();

	void DriveArcade(std::shared_ptr<frc::Joystick> j);
	void DriveStraight(double speed);
	void DriveTurn(double speed);

	// drive using kinematics
  	void Drive(units::meters_per_second_t xSpeed, units::radians_per_second_t rot);

	// Burn CANSparkMAX settings on motors
	void Burn();
};

