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
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include "SwerveModule.h"

/**
 * Represents a swerve drive style drivetrain.
 */
class Drivetrain : public frc2::SubsystemBase {
public:
  Drivetrain();

//  void ResetOdometry(const frc::Pose2d& pose);
  void Periodic() override;

  void Drive(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed,
              units::radians_per_second_t rot, bool fieldRelative);
  void UpdateNTE();
  void Park();
  void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);
  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in radians, from -pi to pi
   */  units::radian_t GetHeading() const;

  void ZeroHeading();
  units::radians_per_second_t GetTurnRate();
  frc::Pose2d GetPose();
  void ResetOdometry(frc::Pose2d pose);

  void SetAnglePIDValues(double Kp, double Ki, double Kd, double offsetRadians);

  void GetGyroAngle();
  void ResetGyro();

  frc2::CommandPtr TestMethodCommand();

  // Righthand rule coordinate system: X positive to front, Y positive to left
  // Dimentions of the center of the wheeles - 53cm by 72cm
  units::meter_t kTrackWidth =
      0.72_m;  // Distance between centers of right and left wheels on robot
  units::meter_t kWheelBase =
      0.53_m;  // Distance between centers of front and back wheels on robot

  frc::SwerveDriveKinematics<4> m_kinematics{
      frc::Translation2d{kWheelBase / 2, kTrackWidth / 2},
      frc::Translation2d{kWheelBase / 2, -kTrackWidth / 2},
      frc::Translation2d{-kWheelBase / 2, kTrackWidth / 2},
      frc::Translation2d{-kWheelBase / 2, -kTrackWidth / 2}};

  static constexpr units::meters_per_second_t kMaxSpeed = 9.0_mps;  // 12 meters per second??
  static constexpr units::radians_per_second_t kMaxAngularSpeed{3 * std::numbers::pi};  // 3 rotations per second??

  static constexpr double deg_to_rad = 0.0174532;

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

  // Offsets in radians for the encoders. 
  double flEncoderOffset = 1.113; 
  double frEncoderOffset = 2.787; 
  double blEncoderOffset = -0.155;
  double brEncoderOffset = -1.850;
  
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
  
  frc::SwerveDriveOdometry<4>  m_odometry{m_kinematics, m_gyro.GetAngle(), {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
    m_backLeft.GetPosition(), m_backRight.GetPosition()}, frc::Pose2d{}};
};