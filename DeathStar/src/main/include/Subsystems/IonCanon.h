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

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
#include "frc/DigitalInput.h"
#include "frc/Encoder.h"
#include "frc/Servo.h"
#include "rev/CANSparkMax.h"
#include <units/units.h>
#include <wpi/math>

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

/**
 *
 *
 * @author ExampleAuthor
 */
class IonCanon: public frc::Subsystem {
private:
	// It's desirable that everything possible is private except
	// for methods that implement subsystem capabilities
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
std::shared_ptr<rev::CANSparkMax> shooterMotor1;
std::shared_ptr<rev::CANSparkMax> shooterMotor2;
std::shared_ptr<rev::CANSparkMax> turretMotor;
std::shared_ptr<frc::Servo> domeServo;
std::shared_ptr<frc::DigitalInput> turretReferenceSwitch;
//std::shared_ptr<frc::Encoder> turretQuadEncoder;
std::shared_ptr<rev::CANEncoder> m_shooter1Encoder;
std::shared_ptr<rev::CANEncoder> turretQuadEncoder;

std::shared_ptr<frc::DigitalInput> loadedSensor;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
public:
	static constexpr double kShooterSpeed = 0.45;
	static constexpr double kShooterIdle = 0.2;
	static constexpr double kTurretSpeed = 0.4;
	static constexpr double kDomeSpeed = 0.5;

	static constexpr units::meter_t kWheelRadius = 2.0_in;
	static constexpr double kGearRatio = 1.;
	static constexpr double kTGearRatio = 5.;
	static constexpr units::meter_t kTurretRadius = 6.0_in;

	IonCanon();
	void InitDefaultCommand() override;
	void Periodic() override;

	// Shooter 
	void SpinShooter(double speed = kShooterSpeed);
	void StopShooter();

	// Turn Turret
	void AimCam();
	void AimLeft();
	void AimRight();
	void AimUp();
	void AimDown();
	void AimStop();
	void StopTurret();
	void StopDome();

	void SetServo(float value);


	// Burn CANSparkMAX settings on motors
	void Burn();
};

