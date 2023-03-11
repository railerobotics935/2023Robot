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
#include "ArmFunctions.h"

void Robot::RobotInit() {}
void Robot::RobotPeriodic() {
  m_arm.UpdateNTE();
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {
  
  // Drive
  if (isParked)
  {
    m_swerve.Park();
  }
  else
  {
    DriveWithJoystick(fieldRelative);
  }

  // Button Inputs
  // Red Button
  if (m_driveController.GetRawButton(1))
    fieldRelative = false;
  else
    fieldRelative = true;
  
  // Red Button
  if (m_driveController.GetRawButton(2))
    isParked = true;
  else
    isParked = false;
  
  if (m_driveController.GetRawButtonPressed(3))
    m_swerve.ResetGyro();

  // Arm Control
  m_arm.SetTurretMotor(m_opController.GetRawAxis(0));
  m_arm.SetLowerArmMotor(0.3 * m_opController.GetRawAxis(1));
  m_arm.SetPushRodArmMotor(0.3 * m_opController.GetRawAxis(3));

  // Intake is button 8??????
  if (m_opController.GetRawButton(7))
    m_arm.SetIntakeMotor(1.0);
  else if (m_opController.GetRawButton(8))
    m_arm.SetIntakeMotor(-1.0);
  else
    m_arm.SetIntakeMotor(0.0);
  
  if (m_opController.GetRawButtonPressed(5))
    wristSetAngle = (wristSetAngle-0.2);
  if (m_opController.GetRawButtonPressed(6))
    wristSetAngle = (wristSetAngle+0.2);
  m_arm.SetWristServo(wristSetAngle);
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

void Robot::DriveWithJoystick(bool fieldRelative) {
/*

    // Calculate adjusted speed values from joystick imput.
    // find distance from center using x and y components of the joystick (radius)
    radiusOfJoystickInput = sqrt((m_driveController.GetRawAxis(0) * m_driveController.GetRawAxis(0))  
                                  + (m_driveController.GetRawAxis(1) * m_driveController.GetRawAxis(1)));
    
    // If the joystick is above the threshold, then start steadaly accerating it to max
    // else, use scaled down joystick imput.
    if (radiusOfJoystickInput > 0.8)
    {
      // X adjustments
      if (m_driveController.GetRawAxis(0) > 0)
      {
        maxSpeedAdditionX = maxSpeedAdditionX + (0.5 /(timeToMaxSpeed/0.02));
        if (maxSpeedAdditionX > 0.5)
          maxSpeedAdditionX = 0.5;
        driveJoystickAdjustedInputX = 0.5 + maxSpeedAdditionX;
      }
      else
      {
        maxSpeedAdditionX = maxSpeedAdditionX - (0.5 /(timeToMaxSpeed/0.02));
        if (maxSpeedAdditionX < -0.5)
          maxSpeedAdditionX = -0.5;
        driveJoystickAdjustedInputX = -0.5 + maxSpeedAdditionX;
      }
      // Y adjustements
      if (m_driveController.GetRawAxis(1) > 0)
      {
        maxSpeedAdditionY =maxSpeedAdditionY + (0.5 /(timeToMaxSpeed/0.02));
        if (maxSpeedAdditionY > 0.5)
          maxSpeedAdditionY = 0.5;
        driveJoystickAdjustedInputY = 0.5 + maxSpeedAdditionY;
      }
      else
      {
        maxSpeedAdditionY = maxSpeedAdditionY - (0.5 /(timeToMaxSpeed/0.02));
        if (maxSpeedAdditionY < -0.5)
          maxSpeedAdditionY = -0.5;
        driveJoystickAdjustedInputY = -0.5 + maxSpeedAdditionY;
      }
    }
    else
    {
      driveJoystickAdjustedInputX = (0.625 * frc::ApplyDeadband(m_driveController.GetRawAxis(0), 0.05));
      driveJoystickAdjustedInputY = (0.625 * frc::ApplyDeadband(m_driveController.GetRawAxis(1), 0.05));
      maxSpeedAdditionX = 0.0;
      maxSpeedAdditionY = 0.0;
    }
    */
    // Exponetal adjustment for the controller
  //  driveJoystickAdjustedInputX = ((1.25 * frc::ApplyDeadband(m_driveController.GetRawAxis(2), 0.05)) * (1.25 * frc::ApplyDeadband(m_driveController.GetRawAxis(2), 0.05)));
  //  driveJoystickAdjustedInputY = ((1.25 * frc::ApplyDeadband(m_driveController.GetRawAxis(4), 0.05)) * (1.25 * frc::ApplyDeadband(m_driveController.GetRawAxis(4), 0.05)));
//
  //  if ((1.25 * frc::ApplyDeadband(m_driveController.GetRawAxis(2), 0.05)) < 0)
  //    driveJoystickAdjustedInputX = -driveJoystickAdjustedInputX;
//
  //  if ((1.25 * frc::ApplyDeadband(m_driveController.GetRawAxis(4), 0.05)) < 0)
  //    driveJoystickAdjustedInputY = -driveJoystickAdjustedInputY;

    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    const auto xSpeed = -m_xspeedLimiter.Calculate((1.66667 * frc::ApplyDeadband(m_driveController.GetRawAxis(2), 0.12))) * Drivetrain::kMaxSpeed;    

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    const auto ySpeed = -m_yspeedLimiter.Calculate((1.66667 * frc::ApplyDeadband(m_driveController.GetRawAxis(4), 0.07))) * Drivetrain::kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW introllers return posits positive in
    // mathematics). Xbox coive values when you pull to
    // the right by default.
    const auto rot = -m_rotLimiter.Calculate((1.25 *frc::ApplyDeadband(m_driveController.GetRawAxis(0), 0.07))) * Drivetrain::kMaxAngularSpeed;

    m_swerve.Drive(xSpeed, ySpeed, rot, fieldRelative);
  }

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
