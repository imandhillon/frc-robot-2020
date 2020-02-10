/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/LoadItUp.h"

LoadItUp::LoadItUp() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(Robot::plasmaTank.get());
}

// Called just before this Command runs the first time
void LoadItUp::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void LoadItUp::Execute() {
  Robot::plasmaTank -> RunLoader(0.5);
}

// Make this return true when this Command no longer needs to run execute()
bool LoadItUp::IsFinished() { return false; }

// Called once after isFinished returns true
void LoadItUp::End() {
   Robot::plasmaTank -> RunLoader(0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void LoadItUp::Interrupted() {
  End();
}
