// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Anything with the number 2 on the end has to do with the Custom Controller
#pragma once

#include <optional>

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardLayout.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <frc/XboxController.h>
#include <frc/MathUtil.h>
#include <frc/SerialPort.h>
#include <frc/GenericHID.h>
#include <frc/Servo.h>

#include "ArmFunctions.h"
#include "Drivetrain/Drivetrain.h"
#include "RobotContainer.h"


class Robot : public frc::TimedRobot {
public:
  void RobotInit() override;
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

  void SimulationInit() override;
  void SimulationPeriodic() override;
  
private:
  // Autonomous modes
  enum AutoState {kScoreCube, kScoreCone, kMobility, kEngageChargeStation, kEnd};
  AutoState currentAutoState = kScoreCube; // not currenlty high
  enum PathPlannerAutoState {kScore, kDrive, kStop};
  PathPlannerAutoState PPAutoState = kScore;
  frc::Pose2d initialPose2d;
  frc::Timer autoTimer;

  double ANALOG_TO_RAD_FACTOR = 1.2566;     // 0 to 5.0 volt = 2PI rad

  frc::XboxController m_driveController{0}; // one of the rc controllers
  frc::XboxController m_opController{1};

  // Configuration information
  bool commandAuto = false;
  bool powerArm = true;
  bool customArmController = true;
  bool exponentialDriveControl = true;


  bool controllerStartedNeutral = false;

  ArmFunctions m_arm;

  // Drive variables
  double driveJoystickAdjustedInputX = 0.0;
  double driveJoystickAdjustedInputY = 0.0;

  // Both arm inputs become negative as you push them forward
  double lowerArmSetpointAngle;
  double pushRodArmSetpointAngle;
  double lowerArmInputRange = -1.21;
  double pushRodArmInputRange = -1.23;

  // Range set to the controller also check the arm funcitons limit
  double workingLowerArmRange = 1.8;
  double workingPushRodArmRange = 2.0;

  // TEMPORARY FIX!!! COME BACK TO FIX
  double totalWristRange = std::numbers::pi; // 270 ish degrees

  // Oporator joystick trim adjustment
  double turretTrim = 0.14;
  double lowerArmTrim = -0.81;
  double pushRodArmTrim = -0.81;
  double wristTrim = 0.13;

  // Maximun range input for the 
  double lowerArmTrim2 = -0.25;
  double pushRodArmTrim2 = 0.07;
  double wristTrim2 = -0.05;

  // Maximun range input for the 
  double lowerArmInputRange2 = 0.88;
  double pushRodArmInputRange2 = 0.89;
  double wristInputRange2 = 1.53;

  double wristSetAngle = 0.0;

  bool automaticPlacing = false;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{2 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{2 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{2 / 1_s};

  bool fieldRelative = true;
  bool isParked = false;
  bool slowMode = false;

  // NetowrkTable Entries
  nt::NetworkTableEntry nte_lowerArmSetpointAngle;
  nt::NetworkTableEntry nte_pushRodArmSetpointAngle;
  nt::NetworkTableEntry nte_wirstSetpointAngle;
  nt::NetworkTableEntry nte_autoMode;

  void DriveWithJoystick(bool fieldRelative, bool slowMode);


  // command stuff for auto or something

  // Have it empty by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.
  std::optional<frc2::CommandPtr> m_autonomousCommand;

  RobotContainer m_container;

};
