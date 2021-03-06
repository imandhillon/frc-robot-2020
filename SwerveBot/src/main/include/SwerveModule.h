/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/Encoder.h>
#include <frc/PWMVictorSPX.h>
#include <frc/AnalogEncoder.h>
#include <frc/AnalogInput.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <wpi/math>

#include <rev/CANSparkMax.h>

class SwerveModule {
 public:
  SwerveModule(int driveMotorChannel, int turningMotorChannel, int turningEncoderAnalogInput);
  frc::SwerveModuleState GetState(); // const;
  void SetDesiredState(const frc::SwerveModuleState& state);

 private:
    // 3.5" diameter  ==  0.0889, radius = 0.04445
  static constexpr double kWheelRadius = 0.04445;
  static constexpr int kEncoderResolution = 4096;

  static constexpr auto kModuleMaxAngularVelocity =
      wpi::math::pi * 1_rad_per_s;  // radians per second
  static constexpr auto kModuleMaxAngularAcceleration =
      wpi::math::pi * 2_rad_per_s / 1_s;  // radians per second^2

  rev::CANSparkMax m_driveMotor;
  rev::CANSparkMax m_turningMotor;

  rev::CANEncoder m_driveEncoder;
  frc::AnalogInput m_turningInput;
  frc::AnalogEncoder m_turningEncoder;

/*
  frc::PWMVictorSPX m_driveMotor;
  frc::PWMVictorSPX m_turningMotor;

  frc::Encoder m_driveEncoder{0, 1};
  frc::Encoder m_turningEncoder{2, 3};
*/

  frc2::PIDController m_drivePIDController{1.0, 0, 0};
  frc::ProfiledPIDController<units::radians> m_turningPIDController{
      1.0,
      0.0,
      0.0,
      {kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration}};
};
