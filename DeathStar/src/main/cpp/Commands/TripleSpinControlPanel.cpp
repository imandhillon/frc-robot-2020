/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/TripleSpinControlPanel.h"


TripleSpinControlPanel::TripleSpinControlPanel() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(Robot::warpDriveInverter.get());
}

// Called just before this Command runs the first time
void TripleSpinControlPanel::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void TripleSpinControlPanel::Execute() {
  Robot::warpDriveInverter -> MoveMotor(1.0);
}

// Make this return true when this Command no longer needs to run execute()
bool TripleSpinControlPanel::IsFinished() { return false; }

// Called once after isFinished returns true
void TripleSpinControlPanel::End() {
   Robot::warpDriveInverter -> Halt();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void TripleSpinControlPanel::Interrupted() {
  End();
}
