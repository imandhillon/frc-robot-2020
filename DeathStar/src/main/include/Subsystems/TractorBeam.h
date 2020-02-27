
#pragma once

#include "frc/commands/Subsystem.h"
#include "frc/DoubleSolenoid.h"
#include "rev/CANSparkMax.h"


/**
 *
 *
 * @author ExampleAuthor
 */
class TractorBeam: public frc::Subsystem {
private:
	// It's desirable that everything possible is private except
	// for methods that implement subsystem capabilities
	std::shared_ptr<rev::CANSparkMax> intakeMotor;
	std::shared_ptr<frc::DoubleSolenoid> tractorBeamBay;

public:
	static constexpr double kIntakeSpeed = 1.0;

	TractorBeam();
	void InitDefaultCommand() override;
	void Periodic() override;

	// Pneumatics
	void	Deploy();
	void	Retract();
	bool	IsDeployed();

	// Motors
	void	SpinIn(double speed);		// negative speed will Spit Out, zero to stop

	// Burn CANSparkMAX settings on motors
	void Burn();

};

