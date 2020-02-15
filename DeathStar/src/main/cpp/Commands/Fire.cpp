

#include "Commands/Fire.h"
#include "Commands/SpinShooter.h"
#include "Commands/Load.h"
#include "Commands/Lock.h"
#include "Commands/LockNLoad.h"


Fire::Fire(): frc::CommandGroup() {
    AddSequential(new SpinShooter(IonCanon::kShooterSpeed));
    AddSequential(new LockNLoad(PlasmaTank::kLoaderSpeed));
   // AddParallel(new Lock(PlasmaTank::kConveyorSpeed));
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void Fire::Interrupted() {
    End();
}
