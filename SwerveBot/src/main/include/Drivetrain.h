/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/AnalogGyro.h>
#include <frc/ADXRS450_Gyro.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <wpi/math>

#include "SwerveModule.h"

/**
 * Represents a swerve drive style drivetrain.
 */
class Drivetrain {
 public:
  Drivetrain() { m_gyro.Reset(); }

  /**
   * Get the robot angle as a Rotation2d.
   */
  frc::Rotation2d GetAngle() const {
    // Negating the angle because WPILib Gyros are CW positive.
    return frc::Rotation2d(units::degree_t(-m_gyro.GetAngle()));
  }

  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative);
  void UpdateOdometry();

  static constexpr units::meters_per_second_t kMaxSpeed =
      3.0_mps;  // 3 meters per second
  static constexpr units::radians_per_second_t kMaxAngularSpeed{
      wpi::math::pi};  // 1/2 rotation per second

 private:
 // front 8.5" == 0.2159m
  frc::Translation2d m_frontLocation{+0.2159_m, +0.0_m};
  // rear 7.25"  == 0.18415m
  frc::Translation2d m_backLeftLocation{-0.2159_m, +0.18415_m};
  frc::Translation2d m_backRightLocation{-0.2159_m, -0.18415_m};

// CAN ID's 3/5/7 Drive
// CAN ID's 4/6/8 turn
  SwerveModule m_front{3, 4, 0};
  SwerveModule m_backLeft{5, 6, 1};
  SwerveModule m_backRight{7, 8, 2};

  //frc::AnalogGyro m_gyro{0};
  frc::ADXRS450_Gyro m_gyro;

  frc::SwerveDriveKinematics<3> m_kinematics{
      m_frontLocation, m_backLeftLocation,
      m_backRightLocation};

  frc::SwerveDriveOdometry<3> m_odometry{m_kinematics, GetAngle()};
};
