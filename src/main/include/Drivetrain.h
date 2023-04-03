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

//  void ResetOdometry(const frc::Pose2d& pose);
  void ResetGyro();
  void Drive(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed,
              units::radians_per_second_t rot, bool fieldRelative);
  void UpdateNTE();
  void Park();

  void SetAnglePIDValues(double Kp, double Ki, double Kd, double offsetRadians);

  void GetGyroAngle();
  const frc::Pose2d& GetPose();


  static constexpr units::meters_per_second_t kMaxSpeed = 9.0_mps;  // 12 meters per second??
  static constexpr units::radians_per_second_t kMaxAngularSpeed{3 * std::numbers::pi};  // 3 rotations per second??

private:  // Declaring all of the network table entrys
  nt::NetworkTableEntry nte_fl_set_angle;
  nt::NetworkTableEntry nte_fr_set_angle;
  nt::NetworkTableEntry nte_bl_set_angle;
  nt::NetworkTableEntry nte_br_set_angle;
  nt::NetworkTableEntry nte_fl_set_speed;
  nt::NetworkTableEntry nte_fr_set_speed;
  nt::NetworkTableEntry nte_bl_set_speed;
  nt::NetworkTableEntry nte_br_set_speed;
  
  nt::NetworkTableEntry nte_fl_real_angle;
  nt::NetworkTableEntry nte_fr_real_angle;
  nt::NetworkTableEntry nte_bl_real_angle;
  nt::NetworkTableEntry nte_br_real_angle;
  nt::NetworkTableEntry nte_fl_real_speed;
  nt::NetworkTableEntry nte_fr_real_speed;
  nt::NetworkTableEntry nte_bl_real_speed;
  nt::NetworkTableEntry nte_br_real_speed;

  nt::NetworkTableEntry nte_fl_raw_encoder_voltage;
  nt::NetworkTableEntry nte_fr_raw_encoder_voltage;
  nt::NetworkTableEntry nte_bl_raw_encoder_voltage;
  nt::NetworkTableEntry nte_br_raw_encoder_voltage;

  nt::NetworkTableEntry nte_gyro_angle;
  nt::NetworkTableEntry nte_robot_x;
  nt::NetworkTableEntry nte_robot_y;

  // Righthand rule coordinate system: X positive to front, Y positive to left
  // Dimensions of the center of the wheels - 53 cm by 70.8  
  frc::Translation2d m_frontLeftLocation{+0.354_m, +0.265_m};
  frc::Translation2d m_frontRightLocation{+0.354_m, -0.265_m};
  frc::Translation2d m_backLeftLocation{-0.354_m, +0.265_m};
  frc::Translation2d m_backRightLocation{-0.354_m, -0.265_m};

  // Offsets in radians for the encoders. 
  double flEncoderOffset = 1.113; //0.939;
  double frEncoderOffset = 2.787; //2.605-180
  double blEncoderOffset = -0.155; //-0.234;
  double brEncoderOffset = -1.850; //-1.877;

  // Creating the four swerve modules
  SwerveModule m_frontLeft{11, 12, 0, flEncoderOffset};
  SwerveModule m_frontRight{9, 10, 1, frEncoderOffset};
  SwerveModule m_backLeft{19, 20, 2, blEncoderOffset};
  SwerveModule m_backRight{21, 2, 3, brEncoderOffset};

  frc::ADIS16470_IMU m_gyro;

  // PID controller for changing robot rotation and other variables
  double rotPower;
  double yawSetpoint;
  double yawKp = 1.0;
  double yawKi = 0.0;
  double yawKd = 0.0;
  frc::PIDController m_yawPID{yawKp, yawKi, yawKd};
  
  frc::SwerveDriveKinematics<4> m_kinematics{m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation};
  frc::SwerveDriveOdometry<4> m_odometry{m_kinematics, m_gyro.GetAngle(), {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_backLeft.GetPosition(), m_backRight.GetPosition()}};
};