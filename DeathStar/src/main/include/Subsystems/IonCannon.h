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
#include "rev/CANPIDController.h"
#include <units/units.h>
#include <wpi/math>
#include "SPAMutil.h"

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

/**
 *
 *
 * @author ExampleAuthor
 */
class IonCannon: public frc::Subsystem {
private:
	// It's desirable that everything possible is private except
	// for methods that implement subsystem capabilities

	std::shared_ptr<rev::CANSparkMax> shooterMotor1;
	std::shared_ptr<rev::CANSparkMax> shooterMotor2;
	std::shared_ptr<rev::CANSparkMax> turretMotor;
	std::shared_ptr<frc::Servo> domeServo;
	std::shared_ptr<frc::Servo> domeServo2;
	std::shared_ptr<frc::DigitalInput> turretReferenceSwitch;
	std::shared_ptr<rev::CANEncoder> shooter1Encoder;
	std::shared_ptr<rev::CANEncoder> turretQuadEncoder;
	std::shared_ptr<rev::CANPIDController> turretPIDController;
	std::shared_ptr<rev::CANPIDController> shooterPIDController;

	// Used for PID control on dashboard
	double kvP = 6e-5, kvI = 1e-6, kvD = 0, kvIz = 0, kvFF = 0.000015, kvMaxOutput = 1.0, kvMinOutput = -1.0;
	void ShooterPidControl();

	double kpP = 0.1, kpI = 1e-4, kpD = 1, kpIz = 0, kpFF = 0, kpMaxOutput = 1, kpMinOutput = -1;
	void TurretPidControl();


	float m_domeServo = 0.0;
//
// CanSparkMax PID coefficients
//  double kP ;
//	double kI ; 
//	double kD ; 
//	double kIz ;
//	double kFF ; 
//	double kMaxOutput; 
//  double kMinOutput;


public:
	static constexpr double kShooterSpeed = 0.45;
	static constexpr double kShooterIdle = 0.2;
	static constexpr double kTurretSpeed = 0.4;
	static constexpr double kTurretXFactor = 1.0;  // what we multiply the targeting X by to move the position
	static constexpr double kDomeSpeed = 0.5;

	static constexpr double kCamTolerance = 3.;
	static constexpr double kCamPower = 0.05;
	static constexpr double kCamFriction = 0; //0.07
	static constexpr double kCamLimit = .76;//0.38;
 
	static constexpr double	kTurretLowLimit = -90;//-106.64;
	static constexpr double kTurretHighLimit = 90;//112.43;

	static constexpr units::meter_t kWheelRadius = 2.0_in;
	static constexpr double kGearRatio = 1.;
	static constexpr double kTGearRatio = 5.;
	static constexpr units::meter_t kTurretRadius = 6.0_in;

	IonCannon();
	void InitDefaultCommand() override;
	void Periodic() override;

	// Shooter 
	void SpinShooter(double speed = kShooterSpeed);
	void StopShooter();

	// Turn Turret
	void AimCamPosition();
	void AimCam();

	void AimLeft();
	void AimRight();
	void AimUp();
	void AimDown();

	void AimStop();

	double GetTurretPosition();
	void SetTurretPosition(double position);
	void StopTurret();

	double GetDomePosition();
	void SetDomePosition(double value);
	void StopDome();

	void SetServo(float value);


	// Burn CANSparkMAX settings on motors
	void Burn();
};

//dome fully extended setting 0.
//           retractd setting  0.5
// shooter pwr 0.25  speed 1360
//             0.45        2537
//             0.65        3508
//             0.90        4750
//             
// turret  ~90 deg    reading -106
//          0 deg               0
//         ~-90deg             112
//


