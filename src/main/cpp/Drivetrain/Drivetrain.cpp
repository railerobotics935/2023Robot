// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drivetrain/Drivetrain.h"

Drivetrain::Drivetrain(){
  // Initialize shuffleboard communication
  auto nt_inst = nt::NetworkTableInstance::GetDefault();
  auto nt_table = nt_inst.GetTable("datatable");

  nte_fl_set_angle = nt_table->GetEntry("Swerve Drive/Front Left/Set Angle");
  nte_fr_set_angle = nt_table->GetEntry("Swerve Drive/Front Right/Set Angle");
  nte_bl_set_angle = nt_table->GetEntry("Swerve Drive/Back Left/Set Angle");
  nte_br_set_angle = nt_table->GetEntry("Swerve Drive/Back Right/Set Angle");
  nte_fl_set_speed = nt_table->GetEntry("Swerve Drive/Front Left/Set Speed");
  nte_fr_set_speed = nt_table->GetEntry("Swerve Drive/Front Right/Set Speed");
  nte_bl_set_speed = nt_table->GetEntry("Swerve Drive/Back Left/Set Speed");
  nte_br_set_speed = nt_table->GetEntry("Swerve Drive/Back Right/Set Speed");

  nte_fl_real_angle = nt_table->GetEntry("Swerve Drive/Front Left/Real Angle");
  nte_fr_real_angle = nt_table->GetEntry("Swerve Drive/Front Right/Real Angle");
  nte_bl_real_angle = nt_table->GetEntry("Swerve Drive/Back Left/Real Angle");
  nte_br_real_angle = nt_table->GetEntry("Swerve Drive/Back Right/Real Angle");
  nte_fl_real_speed = nt_table->GetEntry("Swerve Drive/Front Left/Real Speed");
  nte_fr_real_speed = nt_table->GetEntry("Swerve Drive/Front Right/Real Speed");
  nte_bl_real_speed = nt_table->GetEntry("Swerve Drive/Back Left/Real Speed");
  nte_br_real_speed = nt_table->GetEntry("Swerve Drive/Back Right/Real Speed");

  nte_fl_raw_encoder_voltage = nt_table->GetEntry("Swerve Drive/Front Left/Encoder Voltage");
  nte_fr_raw_encoder_voltage = nt_table->GetEntry("Swerve Drive/Front Right/Encoder Voltage");
  nte_bl_raw_encoder_voltage = nt_table->GetEntry("Swerve Drive/Back Left/Encoder Voltage");
  nte_br_raw_encoder_voltage = nt_table->GetEntry("Swerve Drive/Back Right/Encoder Voltage");

  nte_gyro_angle = nt_table->GetEntry("Swerve Drive/Gyro Angle");
  nte_robot_x = nt_table->GetEntry("Swerve Drive/Robot X");
  nte_robot_y = nt_table->GetEntry("Swerve Drive/Robot Y");
}

void Drivetrain::Periodic() {
  // Implementation of subsystem periodic method goes here.
  m_odometry.Update(GetHeading(),
                    {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                    m_backLeft.GetPosition(), m_backRight.GetPosition()});
}

void Drivetrain::GetGyroAngle()
{

}

void Drivetrain::Drive(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rot, bool fieldRelative)
{
  auto states = m_kinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.GetAngle())
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  m_kinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);

  auto [fl, fr, bl, br] = states;
  
  nte_fl_set_angle.SetDouble((double)fl.angle.Radians());
  nte_fr_set_angle.SetDouble((double)fr.angle.Radians());
  nte_bl_set_angle.SetDouble((double)bl.angle.Radians());
  nte_br_set_angle.SetDouble((double)br.angle.Radians());
  nte_fl_set_speed.SetDouble((double)fl.speed);
  nte_fr_set_speed.SetDouble((double)fr.speed);
  nte_bl_set_speed.SetDouble((double)bl.speed);
  nte_br_set_speed.SetDouble((double)br.speed);
  
  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_backLeft.SetDesiredState(bl);
  m_backRight.SetDesiredState(br);

  nte_fl_real_angle.SetDouble((double)m_frontLeft.GetState().angle.Radians());
  nte_fr_real_angle.SetDouble((double)m_frontRight.GetState().angle.Radians());
  nte_bl_real_angle.SetDouble((double)m_backLeft.GetState().angle.Radians());
  nte_br_real_angle.SetDouble((double)m_backRight.GetState().angle.Radians());
  nte_fl_real_speed.SetDouble((double)m_frontLeft.GetState().speed);
  nte_fr_real_speed.SetDouble((double)m_frontRight.GetState().speed);
  nte_bl_real_speed.SetDouble((double)m_backLeft.GetState().speed);
  nte_br_real_speed.SetDouble((double)m_backRight.GetState().speed);

m_odometry.Update(m_gyro.GetAngle(),
                    {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                     m_backLeft.GetPosition(), m_backRight.GetPosition()});
                     
  nte_gyro_angle.SetDouble((double)m_odometry.GetPose().Rotation().Radians());
  nte_robot_x.SetDouble((double)m_odometry.GetPose().X());
  nte_robot_y.SetDouble((double)m_odometry.GetPose().Y());
}

void Drivetrain::SetModuleStates(
    wpi::array<frc::SwerveModuleState, 4> desiredStates) {
  m_kinematics.DesaturateWheelSpeeds(&desiredStates, kMaxSpeed);

  m_frontLeft.SetDesiredState(desiredStates[0]);
  m_frontRight.SetDesiredState(desiredStates[1]);
  m_backLeft.SetDesiredState(desiredStates[2]);
  m_backRight.SetDesiredState(desiredStates[3]);
}

units::radian_t Drivetrain::GetHeading() const {
  units::radian_t returnValue = m_gyro.GetAngle();
  return returnValue;
}

void Drivetrain::UpdateNTE()
{
  nte_fl_real_angle.SetDouble((double)m_frontLeft.GetState().angle.Radians());
  nte_fr_real_angle.SetDouble((double)m_frontRight.GetState().angle.Radians());
  nte_bl_real_angle.SetDouble((double)m_backLeft.GetState().angle.Radians());
  nte_br_real_angle.SetDouble((double)m_backRight.GetState().angle.Radians());
  nte_fl_real_speed.SetDouble((double)m_frontLeft.GetState().speed);
  nte_fr_real_speed.SetDouble((double)m_frontRight.GetState().speed);
  nte_bl_real_speed.SetDouble((double)m_backLeft.GetState().speed);
  nte_br_real_speed.SetDouble((double)m_backRight.GetState().speed);

  nte_fl_raw_encoder_voltage.SetDouble(m_frontLeft.GetEncoderVoltage());
  nte_fr_raw_encoder_voltage.SetDouble(m_frontRight.GetEncoderVoltage());
  nte_bl_raw_encoder_voltage.SetDouble(m_backLeft.GetEncoderVoltage());
  nte_br_raw_encoder_voltage.SetDouble(m_backRight.GetEncoderVoltage());

  m_odometry.Update(m_gyro.GetAngle(),
                    {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                     m_backLeft.GetPosition(), m_backRight.GetPosition()});
                     
  nte_gyro_angle.SetDouble((double)m_odometry.GetPose().Rotation().Radians());
  nte_robot_x.SetDouble((double)m_odometry.GetPose().X());
  nte_robot_y.SetDouble((double)m_odometry.GetPose().Y());
  }

// Park method to lock the wheels rotation in an X shape, applying no power to the drive motors
void Drivetrain::Park()
{

  frc::SwerveModuleState fl;
  frc::SwerveModuleState fr;
  frc::SwerveModuleState bl;
  frc::SwerveModuleState br;

  fl.angle = frc::Rotation2d (units::radian_t(std::numbers::pi / 4));
  fr.angle = frc::Rotation2d (units::radian_t(-std::numbers::pi / 4));
  bl.angle = frc::Rotation2d (units::radian_t(-std::numbers::pi / 4));
  br.angle = frc::Rotation2d (units::radian_t(std::numbers::pi / 4));

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_backLeft.SetDesiredState(bl);
  m_backRight.SetDesiredState(br);
}

void Drivetrain::ZeroHeading() {
  m_gyro.Reset();
}

units::radians_per_second_t Drivetrain::GetTurnRate() {
  units::radians_per_second_t returnValue =  m_gyro.GetRate();
  return returnValue;
}

frc::Pose2d Drivetrain::GetPose() {
  return m_odometry.GetPose();
}

void Drivetrain::ResetOdometry(frc::Pose2d pose) {
  m_odometry.ResetPosition(
      GetHeading(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_backLeft.GetPosition(), m_backRight.GetPosition()},
      pose);
}

void Drivetrain::SetAnglePIDValues(double Kp, double Ki, double Kd, double offsetRadians)
{
  m_yawPID.SetPID(Kp, Ki, Kd);
  yawSetpoint = (double)GetPose().Rotation().Radians() + offsetRadians;
  m_yawPID.SetSetpoint(yawSetpoint);
}

frc2::CommandPtr Drivetrain::TestMethodCommand() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return this->RunOnce([this] {printf("Test Method Command has Ran\r\n");});
}