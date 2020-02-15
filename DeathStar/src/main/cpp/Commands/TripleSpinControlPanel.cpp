/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/TripleSpinControlPanel.h"
#include "SPAMutil.h"

TripleSpinControlPanel::TripleSpinControlPanel() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(Robot::warpDriveInverter.get());
}

// Called just before this Command runs the first time
void TripleSpinControlPanel::Initialize() {
  MotorRunning = false;
  lastColorChangePos = Robot::warpDriveInverter -> getPosition();
  initialPos = Robot::warpDriveInverter -> getPosition();
  seenColors[Robot::warpDriveInverter -> BLUE] = 0;
  seenColors[Robot::warpDriveInverter -> RED] = 0;
  seenColors[Robot::warpDriveInverter -> GREEN] = 0;
  seenColors[Robot::warpDriveInverter -> YELLOW] = 0;

  int detectedColor = Robot::warpDriveInverter -> getDetectedColor();
  seenColors[detectedColor]++;
}

// Called repeatedly when this Command is scheduled to run
void TripleSpinControlPanel::Execute() {
  if (!MotorRunning){
    if (Robot::warpDriveInverter -> inRange()) {
      Robot::warpDriveInverter -> MoveMotor(0.50);
      MotorRunning = true;
    }
    else
    {
      return;
    }
  }
  else
    Robot::warpDriveInverter -> MoveMotor(0.50);
  int detectedColor = Robot::warpDriveInverter -> getDetectedColor();
  if ( detectedColor != lastColor)
  {
    lastColor = detectedColor;
    lastColorChangePos = Robot::warpDriveInverter -> getPosition();
    seenColors[detectedColor]++;
  }


  frc::SmartDashboard::PutNumber("Times BLUE Seen", seenColors[Robot::warpDriveInverter -> BLUE]);
  frc::SmartDashboard::PutNumber("Times RED Seen", seenColors[Robot::warpDriveInverter -> RED]);
  frc::SmartDashboard::PutNumber("Times YELLOW Seen", seenColors[Robot::warpDriveInverter -> YELLOW]);
  frc::SmartDashboard::PutNumber("Times GREEN Seen", seenColors[Robot::warpDriveInverter -> GREEN]);

}

// Make this return true when this Command no longer needs to run execute()
bool TripleSpinControlPanel::IsFinished() { 
  double deltaEncoder = Robot::warpDriveInverter -> getPosition(); - lastColorChangePos;
  /*
  if (fabs(deltaEncoder) > MAX_DELTA_ENCODER){
    //SPAMutil::Log("TripleSpinControlPanel", "Colors not changing! Is the robot on the wheel?", SPAMutil::LOG_ERR);
    return true;
  }
  */
  
  if (
    //seenColors[Robot::warpDriveInverter -> BLUE] >= 7 &&
    //seenColors[Robot::warpDriveInverter -> RED] >= 7 &&
    seenColors[Robot::warpDriveInverter -> GREEN] >= 7 &&
    //seenColors[Robot::warpDriveInverter -> YELLOW] >= 7){
    SPAMutil::Log("TripleSpinControlPanel", "All Colors Seen", SPAMutil::LOG_INFO);
    if(Robot::warpDriveInverter -> getPosition() - initialPos > MAX_DISTANCE){
     SPAMutil::Log("TripleSpinControlPanel", "Done :)", SPAMutil::LOG_INFO);
     return true;
    }
  }
  
  return false; 
}

// Called once after isFinished returns true
void TripleSpinControlPanel::End() {
   Robot::warpDriveInverter -> Halt();
   SPAMutil::Log("TripleSpinControlPanel", "END", SPAMutil::LOG_INFO);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void TripleSpinControlPanel::Interrupted() {
  SPAMutil::Log("TripleSpinControlPanel", "INTERRUPTED", SPAMutil::LOG_ERR);
  End();
}
