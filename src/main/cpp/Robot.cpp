// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>
#include <ctre/phoenix/motorcontrol/InvertType.h>
#include <frc/MathUtil.h>
#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <networktables/NetworkTableEntry.h>
#include <frc/shuffleboard/ShuffleboardLayout.h>
#include <frc/shuffleboard/ShuffleboardTab.h>

#include "Robot.h"

void Robot::RobotInit() {}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {
  
  if (isParked)
  {
    m_swerve.Park();
  }
  else
  {
    // Drive
    DriveWithJoystick(fieldRelative);
  }

  // Button Inputs
  // Back Button
  if (m_driveController.GetRawButtonPressed(9))
    fieldRelative = !fieldRelative;
  
  // Red Button
  if (m_driveController.GetRawButtonPressed(3))
    isParked = !isParked;
  
  if (m_driveController.GetRawButtonPressed(4))
    m_swerve.ResetGyro();
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

void Robot::DriveWithJoystick(bool fieldRelative) {

    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    const auto xSpeed = -m_xspeedLimiter.Calculate(frc::ApplyDeadband(m_driveController.GetRawAxis(1), 0.05)) * Drivetrain::kMaxSpeed;
    
    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    const auto ySpeed = -m_yspeedLimiter.Calculate(
                          frc::ApplyDeadband(m_driveController.GetRawAxis(0), 0.05)) * Drivetrain::kMaxSpeed;


    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    const auto rot = -m_rotLimiter.Calculate(
                        frc::ApplyDeadband(m_driveController.GetRawAxis(2), 0.05)) * Drivetrain::kMaxAngularSpeed;


    m_swerve.Drive(xSpeed, ySpeed, rot, fieldRelative);
  }

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
