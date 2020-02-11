

#include "Commands/Fire.h"
#include "Commands/SpinShooter.h"
#include "Commands/Load.h"


Fire::Fire(): frc::CommandGroup() {
    AddSequential(new SpinShooter(IonCanon::kShooterSpeed));
    AddSequential(new Load(PlasmaTank::kLoaderSpeed));
}

