

#include "Commands/Grab.h"
#include "Commands/FuelGrab.h"
#include "Commands/SpinWheel.h"

Grab::Grab(GrabDirection direction): frc::CommandGroup() {
    AddParallel(new FuelGrab(TractorBeam::kIntakeSpeed * direction));
    
    // Make sure the spinner is non-interruptable
    Command* spin = new SpinWheel(WarpDriveInverter::kIntakeSpeed * -direction);
    spin->SetInterruptible(false);
    AddSequential(spin);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void Grab::Interrupted() {
    End();
}
