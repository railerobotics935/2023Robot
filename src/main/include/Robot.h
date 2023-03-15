// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>

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
#include <frc/XboxController.h>
#include <frc/MathUtil.h>
#include <frc/SerialPort.h>
#include <frc/GenericHID.h>
#include <frc/Servo.h>

#include "ArmFunctions.h"
#include "Drivetrain.h"

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
  double ANALOG_TO_RAD_FACTOR = 1.2566;     // 0 to 5.0 volt = 2PI rad

  frc::XboxController m_driveController{0}; // one of the rc controllers
  frc::XboxController m_opController{1};

  bool setpointControler = true;
  bool customArmController = false;
  bool controllerStartedNeutral = false;

  // Servedrive and arm objects
  Drivetrain m_swerve;
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
  double workingLowerArmRange = 2.0;
  double workingPushRodArmRange = 1.855;
  double workingWristRange = std::numbers::pi;

  // Oporator joystick trim adjustment
  double turretTrim = 0.14;
  double lowerArmTrim = -0.81;
  double pushRodArmTrim = -0.81;
  double wristTrim = 0.13;

  // Oporator joystick trim adjustment for the custom arm controller
  double lowerArmTrim2 = 0.0;
  double pushRodArmTrim2 = 0.0;
  double wristTrim2 = 0.40;

  double lowerArmInputRange2 = -0.35;
  double pushRodArmInputRange2 = -0.29;
  double wristInputRange2 = -0.50;


  double wristSetAngle = 0.0;

  bool automaticPlacing = false;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{3 / 1_s};

  bool fieldRelative = true;
  bool isParked = false;

  // NetowrkTable Entries
  nt::NetworkTableEntry nte_lowerArmSetpointAngle;
  nt::NetworkTableEntry nte_pushRodArmSetpointAngle;
  nt::NetworkTableEntry nte_wirstSetpointAngle;

  void DriveWithJoystick(bool fieldRelative);

};
