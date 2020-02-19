

#include "Commands/Fire.h"
#include "Commands/SpinShooter.h"
#include "Commands/Load.h"
#include "Commands/Lock.h"
#include "Commands/LockNLoad.h"
#include "Commands/StopLockNLoad.h"



Fire::Fire(): frc::CommandGroup() {
    AddSequential(new SpinShooter(IonCannon::kShooterSpeed));
    //AddSequential(new frc::WaitCommand(.25));
    //AddSequential(new LockNLoad(PlasmaTank::kLoaderSpeed));
    AddSequential(new Load(PlasmaTank::kLoaderSpeed));
    AddSequential(new frc::WaitCommand(.75));
    AddSequential(new Lock(PlasmaTank::kConveyorSpeed));
    
    
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void Fire::Interrupted() {
    End();
}
