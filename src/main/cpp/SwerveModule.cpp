// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveModule.h"


SwerveModule::SwerveModule(const int driveMotorChannel,
                           const int turningMotorChannel,
                           const int turningEncoderChannel)
    : m_driveMotor(driveMotorChannel, rev::CANSparkMax::MotorType::kBrushless),
      m_turningMotor(turningMotorChannel),
      m_driveEncoder(m_driveMotor.GetEncoder()),
      m_turningEncoder(turningEncoderChannel)
{
  // Set the distance per pulse for the drive encoder. We can simply use the
  // distance traveled for one rotation of the wheel divided by the encoder
  // resolution. Don't forget the gear ratio of the swerve drive module.
  m_driveEncoder.SetVelocityConversionFactor(2 * wpi::numbers::pi * kWheelRadius / (kGearRatio * kEncoderResolution));

  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.
  m_turningPIDController.EnableContinuousInput(-units::radian_t(wpi::numbers::pi), units::radian_t(wpi::numbers::pi));
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState& referenceState)
{
  // Optimize the reference state to avoid spinning further than 90 degrees
  const auto state = frc::SwerveModuleState::Optimize(
      referenceState, units::radian_t((m_turningEncoder.GetVoltage() * ANALOG_TO_RAD_FACTOR) - wpi::numbers::pi));

  // Calculate the drive output from the drive PID controller.
  const auto driveOutput = m_drivePIDController.Calculate(
      m_driveEncoder.GetVelocity(), state.speed.value());

  const auto driveFeedforward = m_driveFeedforward.Calculate(state.speed);

  // Calculate the turning motor output from the turning PID controller.
  const auto turnOutput = m_turningPIDController.Calculate(
      units::radian_t((m_turningEncoder.GetVoltage() * ANALOG_TO_RAD_FACTOR) - wpi::numbers::pi), state.angle.Radians());

  const auto turnFeedforward = m_turnFeedforward.Calculate(m_turningPIDController.GetSetpoint().velocity);

  // Set the motor outputs, cut off at very low intended speed
  if (fabs(state.speed.value()) > 0.02)
    m_driveMotor.SetVoltage(units::volt_t{driveOutput} + driveFeedforward);
  else
    m_driveMotor.SetVoltage(units::volt_t{0.0});

  m_turningMotor.SetVoltage(units::volt_t{turnOutput} + turnFeedforward);
}

frc::SwerveModuleState SwerveModule::GetState() const
{
  return {units::meters_per_second_t{m_driveEncoder.GetVelocity()},
          frc::Rotation2d(units::radian_t((m_turningEncoder.GetVoltage() * ANALOG_TO_RAD_FACTOR) - wpi::numbers::pi))};
}