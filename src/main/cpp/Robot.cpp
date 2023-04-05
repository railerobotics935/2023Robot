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

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/Command.h>
#include <frc2/command/PrintCommand.h>
#include <frc2/command/RunCommand.h>

#include <pathplanner/lib/PathPlannerTrajectory.h>
#include <pathplanner/lib/commands/FollowPathWithEvents.h>

#include <pathplanner/lib/PathPlanner.h>
#include <pathplanner/lib/auto/SwerveAutoBuilder.h>
#include <pathplanner/lib/commands/PPSwerveControllerCommand.h>
#include <pathplanner/lib/commands/FollowPathWithEvents.h>

#include "Robot.h"
#include "ArmFunctions.h"

using namespace pathplanner;

void Robot::RobotInit() {
    // Initialize shuffleboard communication
  auto nt_inst = nt::NetworkTableInstance::GetDefault();
  auto nt_table = nt_inst.GetTable("datatable");
  nte_lowerArmSetpointAngle = nt_table->GetEntry("Arm/Lower Arm Setpoint Angle");
  nte_pushRodArmSetpointAngle = nt_table->GetEntry("Arm/Push Rod Arm Setpoint Angle");
  nte_wirstSetpointAngle = nt_table->GetEntry("Arm/Wrist Setpoint Angle");
}
void Robot::RobotPeriodic() {
  m_arm.UpdateNTE();
  m_swerve.UpdateNTE();
}

void Robot::AutonomousInit() 
{
  initialPose2d = m_swerve.GetPose();
  autoTimer.Reset();
  autoTimer.Start();
}
void Robot::AutonomousPeriodic() 
{
  if (oldAuto)
  {  // Switch for the Autonomous states
    switch (currentAutoState)
    {
    case kMobility:
      std::cout << "Mobility\r\n";
      if (initialPose2d.X() < (units::length::meter_t)3.0  && autoTimer.Get() < (units::time::second_t)2.0)
      {
        // add something to to check if you set a game peice or not.
        m_swerve.Drive((units::velocity::meters_per_second_t) -4.0, 
                      (units::velocity::meters_per_second_t) -0.5, 
                      (units::angular_velocity::radians_per_second_t)0.0, 
                      false);
      }
      else
      {
        currentAutoState = kEnd;
        autoTimer.Reset();
        autoTimer.Start();
      }
      break;
    case kEngageChargeStation:
      std::cout << "Engage Charge Station\r\n";
      // Move forward
      if (1 == 1)
        //nothing yet
      break;
    case kScoreCube:
      std:: cout << "Score Cube";
      if (autoTimer.Get() < (units::time::second_t)3.0)
      {
        m_arm.SetWristServo(0.0);
        if (autoTimer.Get() > (units::time::second_t)2.0)
          m_arm.SetIntakeMotor(-1.0);
      }
      else
      {
        currentAutoState = kEnd;
        autoTimer.Reset();
        autoTimer.Start();
      }
      break;
    case kScoreCone:
      std::cout << "Score Cone";
      if (autoTimer.Get() < (units::time::second_t)3.0)
        // Something
    case kEnd:
      std::cout << "End\r\n";
      m_arm.SetWristServo(0.0);
      m_swerve.Park();
      break;
    
    default:
      m_swerve.Park();
      std::cout << "Something went wrong...\r\n";
      break;
    }
  }
  else
  {
    // simple path planner auto
    // This will load the file "Example Path.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
    PathPlannerTrajectory examplePath = PathPlanner::loadPath("Test Drive Forward", PathConstraints((units::meters_per_second_t)4,(units::meters_per_second_squared_t)3));

    switch (PPAutoState)
    {
    case kScore:
      if (autoTimer.Get() < (units::time::second_t)3.0)
        m_arm.SetArmForHighCone();
      else if (autoTimer.Get() < (units::time::second_t)3.5)
        m_arm.SetIntakeMotor(-0.15);
      else
      {
        m_arm.SetArmToHome();
        autoTimer.Reset();
        autoTimer.Start();
        PPAutoState = kDrive;
      } 
      break;
    case kDrive:
      PathPlannerTrajectory::PathPlannerState state = examplePath.sample(autoTimer.Get()); 
      //driveRamsete.Calculate(m_swerve.GetPose(), examplePath.gett);

      break;
    default:
      m_swerve.Park();
      std::cout << "Something went wrong...\r\n";
      break;
    }
  }
}

void Robot::TeleopInit() {
  if (customArmController)
  {
    if (m_opController.GetRawAxis(0) - lowerArmTrim2 > -0.02 && m_opController.GetRawAxis(1) - pushRodArmTrim2 > -0.02)
      controllerStartedNeutral = true;
    else
      controllerStartedNeutral = false;
  }
  else
  {
    if (m_opController.GetRawAxis(2) + lowerArmTrim > -0.05 && m_opController.GetRawAxis(1) + pushRodArmTrim > -0.05)
      controllerStartedNeutral = true;
    else
      controllerStartedNeutral = false;
  }
}

void Robot::TeleopPeriodic() {
  
  // Drive
  if (isParked)
  {
    m_swerve.Park();
  }
  else
  {
    DriveWithJoystick(fieldRelative, slowMode);
  }

  // Left Switch
  if (m_driveController.GetRawButton(1))
    fieldRelative = true;
  else
    fieldRelative = false;
  
  // Left Back Swtich up
  if (m_driveController.GetRawButton(5))
    slowMode = true;
  else
    slowMode = false;

  // Right Switch
  if (m_driveController.GetRawButton(2))
    isParked = true;
  else
    isParked = false;
  
  // Red Reset button
  if (m_driveController.GetRawButtonPressed(3))
    m_swerve.ResetGyro();

  // Deafult is to run on setpoint control
  if (powerArm)
  {
    if (customArmController)
    {
      if (controllerStartedNeutral)
      {
        // Joystick to angle conversion axis 2 is lower arm. axis one is push rod arm
        lowerArmSetpointAngle =  ((m_opController.GetRawAxis(0) - lowerArmTrim2) / lowerArmInputRange2) * workingLowerArmRange;
        pushRodArmSetpointAngle = -((((m_opController.GetRawAxis(1) - pushRodArmTrim2) / pushRodArmInputRange2) * workingPushRodArmRange) - std::numbers::pi);
        m_arm.SetLowerArmAngle(lowerArmSetpointAngle);
        m_arm.SetPushRodArmRawAngle(pushRodArmSetpointAngle);
        nte_lowerArmSetpointAngle.SetDouble(lowerArmSetpointAngle);
        nte_pushRodArmSetpointAngle.SetDouble(pushRodArmSetpointAngle);
      }
      else
      {
        m_arm.SetLowerArmAngle(0.0);
        m_arm.SetPushRodArmRawAngle(3.1);
        if (m_opController.GetRawAxis(0) - lowerArmTrim2 > -0.02 && m_opController.GetRawAxis(1) - pushRodArmTrim2 > -0.02)
          controllerStartedNeutral = true;
        std::cout << "Set Joystick to Zero!\r\n";
      }
      
      // Turret Control
      wristSetAngle = ((-(m_opController.GetRawAxis(2) - wristTrim2)/wristInputRange2) * totalWristRange / 2);
 //     if (wristSetAngle > std::numbers::pi/2)
 //       wristSetAngle = std::numbers::pi/2;
 //     if (wristSetAngle < -std::numbers::pi/2)
 //       wristSetAngle = -std::numbers::pi/2;
      m_arm.SetWristServo(wristSetAngle);
      nte_wirstSetpointAngle.SetDouble(wristSetAngle);

      // Intake controll
      if (m_opController.GetRawButton(1))
      {
        m_arm.SetIntakeMotor(0.15);
      }
      else if (m_opController.GetRawButton(2))
        m_arm.SetIntakeMotor(-0.15);
      else
        m_arm.SetIntakeMotor(0.0);
    }
    else
    {
      if (controllerStartedNeutral)
      {
        // Joystick to angle conversion axis 2 is lower arm. axis one is push rod arm
        lowerArmSetpointAngle =  ((m_opController.GetRawAxis(2) + lowerArmTrim) / lowerArmInputRange) * workingLowerArmRange;
        pushRodArmSetpointAngle = -((((m_opController.GetRawAxis(1) + pushRodArmTrim) / pushRodArmInputRange) * workingPushRodArmRange) - std::numbers::pi);
        m_arm.SetLowerArmAngle(lowerArmSetpointAngle);
        m_arm.SetPushRodArmRawAngle(pushRodArmSetpointAngle);
        nte_lowerArmSetpointAngle.SetDouble(lowerArmSetpointAngle);
        nte_pushRodArmSetpointAngle.SetDouble(pushRodArmSetpointAngle);
      }
      else
      {
        m_arm.SetLowerArmAngle(0.0);
        m_arm.SetPushRodArmRawAngle(3.1);
        if (m_opController.GetRawAxis(2) + lowerArmTrim > -0.05 && m_opController.GetRawAxis(1) + pushRodArmTrim > -0.05)
          controllerStartedNeutral = true;
        std::cout << "Set Joystick to Zero!\r\n";
      }
      
      // Wrist input
      m_arm.SetWristServo(wristSetAngle);
      nte_wirstSetpointAngle.SetDouble(wristSetAngle);

      // Intake controll, Red button is one
      if (m_opController.GetRawButton(1))
      {
        wristSetAngle = wristSetAngle + 0.01;
        m_arm.SetIntakeMotor(1.0);
        std::cout << "1\r\n";
      }
      else if (m_opController.GetRawButton(4))
      {
        wristSetAngle = wristSetAngle - 0.01;
        m_arm.SetIntakeMotor(-1.0);
      }
      else
        m_arm.SetIntakeMotor(0.0);
    }
  }
  else
  {
  /*
  // Intake is button 8??????
  if (m_opController.GetRawButton(7))
    m_arm.SetIntakeMotor(1.0);
  else if (m_opController.GetRawButton(8))
    m_arm.SetIntakeMotor(-1.0);
  else
    m_arm.SetIntakeMotor(0.0);
  a
  if (m_opController.GetRawButtonPressed(5))
    wristSetAngle = (wristSetAngle - 0.2);
  if (m_opController.GetRawButtonPressed(6))
    wristSetAngle = (wristSetAngle + 0.2);
  m_arm.SetWristServo(wristSetAngle);

  */
  }
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

void Robot::DriveWithJoystick(bool fieldRelative, bool slowMode) {

  // If you are in slow mode, only use one quarter of the drive speed
  if (slowMode)
  {
    driveJoystickAdjustedInputX = 0.25 * (m_driveController.GetRawAxis(2) * m_driveController.GetRawAxis(2));
    driveJoystickAdjustedInputY = 0.25 * (m_driveController.GetRawAxis(4) * m_driveController.GetRawAxis(4));
  }
  else
  {    
  driveJoystickAdjustedInputX = (m_driveController.GetRawAxis(2) * m_driveController.GetRawAxis(2));
  driveJoystickAdjustedInputY = (m_driveController.GetRawAxis(4) * m_driveController.GetRawAxis(4));
  }
  if (m_driveController.GetRawAxis(2) < 0)
    driveJoystickAdjustedInputX = -driveJoystickAdjustedInputX;
  if (m_driveController.GetRawAxis(4) < 0)
    driveJoystickAdjustedInputY = -driveJoystickAdjustedInputY;

  // If I don't want to drive with expoential control
  if (!exponentialDriveControl)
  {
    if (slowMode)
    {
      driveJoystickAdjustedInputX = 0.25 * m_driveController.GetRawAxis(2);
      driveJoystickAdjustedInputY = 0.25 * m_driveController.GetRawAxis(4);
    }
    driveJoystickAdjustedInputX = m_driveController.GetRawAxis(2);
    driveJoystickAdjustedInputY = m_driveController.GetRawAxis(4);
  } 
  // Get the x speed. We are inverting this because Xbox controllers return
  // negative values when we push forward.
  const auto xSpeed = -m_xspeedLimiter.Calculate(frc::ApplyDeadband(driveJoystickAdjustedInputX, 0.02)) * Drivetrain::kMaxSpeed;    

  // Get the y speed or sideways/strafe speed. We are inverting this because
  // we want a positive value when we pull to the left. Xbox controllers
  // return positive values when you pull to the right by default.
  const auto ySpeed = -m_yspeedLimiter.Calculate(frc::ApplyDeadband(driveJoystickAdjustedInputY, 0.02)) * Drivetrain::kMaxSpeed;

  // Get the rate of angular rotation. We are inverting this because we want a
  // positive value when we pull to the left (remember, CCW introllers return posits positive in
  // mathematics). Xbox coive values when you pull to
  // the right by default.
  const auto rot = -m_rotLimiter.Calculate(frc::ApplyDeadband(m_driveController.GetRawAxis(0), 0.07)) * Drivetrain::kMaxAngularSpeed;

  m_swerve.Drive(xSpeed, ySpeed, rot, fieldRelative);
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
