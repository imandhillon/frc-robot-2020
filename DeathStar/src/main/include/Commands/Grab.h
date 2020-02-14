
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
enum GrabDirection { kGrabIn = 1, kSpitOut = -1 };

class Grab: public frc::CommandGroup {
public:

	Grab(GrabDirection direction = kGrabIn);
	void Interrupted() override;

private:

};
