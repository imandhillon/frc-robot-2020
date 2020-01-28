/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Drivetrain.h"
#include "frc/smartdashboard/SmartDashboard.h"

void Drivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rot, bool fieldRelative) {

  if ((double)xSpeed > -0.08 && (double)xSpeed < 0.08)
    xSpeed = 0.0_mps;
  if ((double)ySpeed > -0.08 && (double)ySpeed < 0.08)
    ySpeed = 0.0_mps;
  if ((double)rot > -0.08 && (double)rot < 0.08)
    rot = 0.0_rad_per_s;

  auto states = m_kinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot, GetAngle())
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  m_kinematics.NormalizeWheelSpeeds(&states, kMaxSpeed);

  auto [front, bl, br] = states;

  m_front.SetDesiredState(front);
  m_backLeft.SetDesiredState(bl);
  m_backRight.SetDesiredState(br);
  
  frc::SmartDashboard::PutNumber("xSpeed", (double)xSpeed);
  frc::SmartDashboard::PutNumber("ySpeed", (double)ySpeed);
  frc::SmartDashboard::PutNumber("rot", (double)rot);

  frc::SmartDashboard::PutNumber("fr angle", (double)front.angle.Degrees());
  frc::SmartDashboard::PutNumber("fr speed", (double)front.speed);
  frc::SmartDashboard::PutNumber("bl angle", (double)bl.angle.Degrees());
  frc::SmartDashboard::PutNumber("bl speed", (double)bl.speed);
  frc::SmartDashboard::PutNumber("br angle", (double)br.angle.Degrees());
  frc::SmartDashboard::PutNumber("br speed", (double)br.speed);

}

void Drivetrain::UpdateOdometry() {
  m_odometry.Update(GetAngle(), m_front.GetState(),
                    m_backLeft.GetState(), m_backRight.GetState());
}
