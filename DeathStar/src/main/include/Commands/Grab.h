
#pragma once

#include "frc/commands/CommandGroup.h"
#include "frc/commands/Subsystem.h"
#include "Robot.h"

/**
 *
 *
 * @author ExampleAuthor
 */


// which direction, in or out
enum GrabDirection { kIn = 1, kOut = -1 };

class Grab: public frc::CommandGroup {
public:

	Grab(GrabDirection direction = kIn);
	void Interrupted() override;

private:

};
