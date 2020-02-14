
#pragma once

#include "frc/commands/Command.h"
#include "frc/commands/Subsystem.h"
#include "Robot.h"

/**
 *
 *
 * @author ExampleAuthor
 */
class AimJoystick: public frc::Command {
public:
	AimJoystick();

	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;

private:

};
