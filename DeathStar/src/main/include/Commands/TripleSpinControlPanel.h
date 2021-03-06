/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Command.h>
#include "Robot.h"

class TripleSpinControlPanel : public frc::Command {
 public:
  TripleSpinControlPanel();
  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End() override;
  void Interrupted() override;
private:
  bool MotorRunning;
  bool tripleSpinDone;
  double lastColorChangePos;
  double initialPos;
  int lastColor; 
  double MAX_DELTA_ENCODER = 0;
  double MAX_DISTANCE = 0;
  int seenColors[4];
};
