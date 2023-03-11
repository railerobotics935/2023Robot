// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

#include <frc/AnalogInput.h>
#include <frc/Encoder.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <frc/simulation/EncoderSim.h>
#include "ctre/phoenix.h"
#include "rev/CANSparkMax.h" 



class SwerveModule
{
public:
  SwerveModule(int driveMotorChannel, int turningMotorChannel, int turningEncoderChannel, double turningEncoderOffset);
  
  frc::SwerveModuleState GetState() const;
  frc::SwerveModulePosition GetPosition() const;
  void SetDesiredState(const frc::SwerveModuleState& state);


private:
  static constexpr double ANALOG_TO_RAD_FACTOR = 1.2566;     // 0 to 5.0 volt = 2PI rad

  static constexpr double kWheelRadius = 0.0508;
  static constexpr int kEncoderResolution = 42;
  static constexpr double kGearRatio = 6.75;

  static constexpr auto kModuleMaxAngularVelocity = 18.0 * std::numbers::pi * 1_rad_per_s;  // radians per second
  static constexpr auto kModuleMaxAngularAcceleration = 18.0 * std::numbers::pi * 2_rad_per_s / 1_s;  // radians per second^2

  double kTurningEncoderOffset;

  rev::CANSparkMax m_driveMotor;
  rev::CANSparkMax m_turningMotor;

  rev::SparkMaxRelativeEncoder m_driveEncoder;
  frc::AnalogInput m_turningEncoder;

  frc2::PIDController m_drivePIDController{1.0, 0.0, 0.0};

  frc::ProfiledPIDController<units::radians> m_turningPIDController{
      12.5, 190.0, 0.15,
      {kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration}};

  frc::SimpleMotorFeedforward<units::meters> m_driveFeedforward{0.5_V, 1_V / 1_mps};
  frc::SimpleMotorFeedforward<units::radians> m_turnFeedforward{0.5_V, 0.5_V / 1_rad_per_s};
};