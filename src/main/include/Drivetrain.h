// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

#include <frc/ADIS16470_IMU.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardLayout.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/controller/PIDController.h>

#include "SwerveModule.h"

/**
 * Represents a swerve drive style drivetrain.
 */
class Drivetrain
{
public:
  Drivetrain();

  void ResetOdometry(const frc::Pose2d& pose);
  void ResetGyro();
  void Drive(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed,
              units::radians_per_second_t rot, bool fieldRelative);
  void FaceTarget();
  void SetAnglePIDValues(double Kp, double Ki, double Kd, double offsetRadians);

  const frc::Pose2d& GetPose();

  static constexpr units::meters_per_second_t kMaxSpeed = 4.0_mps;  // 4 meters per second
  static constexpr units::radians_per_second_t kMaxAngularSpeed{3 * wpi::numbers::pi};  // 2 rotations per second

private:
  // Declaring all of the network table entrys
  nt::NetworkTableEntry nte_fl_set_angle;
  nt::NetworkTableEntry nte_fr_set_angle;
  nt::NetworkTableEntry nte_bl_set_angle;
  nt::NetworkTableEntry nte_br_set_angle;
  nt::NetworkTableEntry nte_fl_set_speed;
  nt::NetworkTableEntry nte_fr_set_speed;
  nt::NetworkTableEntry nte_bl_set_speed;
  nt::NetworkTableEntry nte_br_set_speed;
  
  nt::NetworkTableEntry nte_fl_act_angle;
  nt::NetworkTableEntry nte_fr_act_angle;
  nt::NetworkTableEntry nte_bl_act_angle;
  nt::NetworkTableEntry nte_br_act_angle;
  nt::NetworkTableEntry nte_fl_act_speed;
  nt::NetworkTableEntry nte_fr_act_speed;
  nt::NetworkTableEntry nte_bl_act_speed;
  nt::NetworkTableEntry nte_br_act_speed;

  nt::NetworkTableEntry nte_gyro_angle;
  nt::NetworkTableEntry nte_robot_x;
  nt::NetworkTableEntry nte_robot_y;

  // Righthand rule coordinate system: X positive to front, Y positive to left

  // NEED TO UPDATE

  frc::Translation2d m_frontLeftLocation{+0.324_m, +0.2675_m};
  frc::Translation2d m_frontRightLocation{+0.324_m, -0.2675_m};
  frc::Translation2d m_backLeftLocation{-0.324_m, +0.2675_m};
  frc::Translation2d m_backRightLocation{-0.324_m, -0.2675_m};

  // Creating the four swerve modules
  SwerveModule m_frontLeft{4, 0, 0};
  SwerveModule m_frontRight{5, 1, 1};
  SwerveModule m_backLeft{6, 2, 2};
  SwerveModule m_backRight{7, 3, 3};

  frc::ADIS16470_IMU m_gyro;

  // PID controller for changing robot rotation and other variables
  double rotPower;
  double yawSetpoint;
  double yawKp = 1.0;
  double yawKi = 0.0;
  double yawKd = 0.0;
  frc::PIDController m_yawPID{yawKp, yawKi, yawKd};
  
  frc::SwerveDriveKinematics<4> m_kinematics{m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation};
  frc::SwerveDriveOdometry<4> m_odometry{m_kinematics, m_gyro.GetAngle()};
};