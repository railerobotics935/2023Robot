// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drivetrain.h"

Drivetrain::Drivetrain()
{
  m_gyro.Reset();
  m_gyro.SetYawAxis(frc::ADIS16470_IMU::IMUAxis::kZ);

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

  nte_fl_act_angle = nt_table->GetEntry("Swerve Drive/Front Left/Act Angle");
  nte_fr_act_angle = nt_table->GetEntry("Swerve Drive/Front Right/Act Angle");
  nte_bl_act_angle = nt_table->GetEntry("Swerve Drive/Back Left/Act Angle");
  nte_br_act_angle = nt_table->GetEntry("Swerve Drive/Back Right/Act Angle");
  nte_fl_act_speed = nt_table->GetEntry("Swerve Drive/Front Left/Act Speed");
  nte_fr_act_speed = nt_table->GetEntry("Swerve Drive/Front Right/Act Speed");
  nte_bl_act_speed = nt_table->GetEntry("Swerve Drive/Back Left/Act Speed");
  nte_br_act_speed = nt_table->GetEntry("Swerve Drive/Back Right/Act Speed");

  nte_gyro_angle = nt_table->GetEntry("Swerve Drive/Gyro Angle");
  nte_robot_x = nt_table->GetEntry("Swerve Drive/Robot X");
  nte_robot_y = nt_table->GetEntry("Swerve Drive/Robot Y");
}

void Drivetrain::ResetOdometry(const frc::Pose2d& pose)
{
  m_odometry.ResetPosition(pose, m_gyro.GetAngle());
}

void Drivetrain::ResetGyro()
{
  m_gyro.Reset();
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

  nte_fl_act_angle.SetDouble((double)m_frontLeft.GetState().angle.Radians());
  nte_fr_act_angle.SetDouble((double)m_frontRight.GetState().angle.Radians());
  nte_bl_act_angle.SetDouble((double)m_backLeft.GetState().angle.Radians());
  nte_br_act_angle.SetDouble((double)m_backRight.GetState().angle.Radians());
  nte_fl_act_speed.SetDouble((double)m_frontLeft.GetState().speed);
  nte_fr_act_speed.SetDouble((double)m_frontRight.GetState().speed);
  nte_bl_act_speed.SetDouble((double)m_backLeft.GetState().speed);
  nte_br_act_speed.SetDouble((double)m_backRight.GetState().speed);

  m_odometry.Update(m_gyro.GetAngle(), m_frontLeft.GetState(), m_frontRight.GetState(),
                    m_backLeft.GetState(), m_backRight.GetState());

  nte_gyro_angle.SetDouble((double)m_odometry.GetPose().Rotation().Radians());
  nte_robot_x.SetDouble((double)m_odometry.GetPose().X());
  nte_robot_y.SetDouble((double)m_odometry.GetPose().Y());
}

void Drivetrain::FaceTarget()
{
  rotPower = m_yawPID.Calculate((double)GetPose().Rotation().Radians());

  Drive(units::meters_per_second_t(0),units::meters_per_second_t(0), units::radians_per_second_t(rotPower), true);
}

void Drivetrain::SetAnglePIDValues(double Kp, double Ki, double Kd, double offsetRadians)
{
  m_yawPID.SetPID(Kp, Ki, Kd);
  yawSetpoint = (double)GetPose().Rotation().Radians() + offsetRadians;
  m_yawPID.SetSetpoint(yawSetpoint);
}
const frc::Pose2d& Drivetrain::GetPose()
{
  return m_odometry.GetPose();
}