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

void Robot::RobotInit() {
    // Initialize shuffleboard communication
  auto nt_inst = nt::NetworkTableInstance::GetDefault();
  auto nt_table = nt_inst.GetTable("datatable");
  nte_lowerArmSetpointAngle = nt_table->GetEntry("Arm/Lower Arm Setpoint Angle");
  nte_pushRodArmSetpointAngle = nt_table->GetEntry("Arm/Push Rod Arm Setpoint Angle");
}
void Robot::RobotPeriodic() {
  m_arm.UpdateNTE();
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  if (abs(m_opController.GetRawAxis(2) + lowerArmTrim) < 0.05 && abs(m_opController.GetRawAxis(1) + pushRodArmTrim) < 0.05)
    controllerStartedNeutral = true;
  else
    controllerStartedNeutral = false;
}
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

  // Deafult is to run on setpoint control
  if (setpointControler)
  {
    // Turret Control
    m_arm.SetTurretMotor(m_opController.GetRawAxis(4));

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
      if (abs(m_opController.GetRawAxis(2) + lowerArmTrim) < 0.05 && abs(m_opController.GetRawAxis(1) + pushRodArmTrim) < 0.05)
        controllerStartedNeutral = true;
      std::cout << "Set Joystick to Zero!\r\n";
    }
    
    // Wrist input
    wristSetAngle = wristSetAngle + ((m_opController.GetRawAxis(0) + wristTrim) * 0.8);
    m_arm.SetWristServo(wristSetAngle);

    // Intake controll, Red button is one
    if (m_opController.GetRawButton(1))
      m_arm.SetIntakeMotor(1.0);
    else if (m_opController.GetRawButton(4))
      m_arm.SetIntakeMotor(-1.0);
    else
      m_arm.SetIntakeMotor(0.0);

  }
  else
  {
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
    wristSetAngle = (wristSetAngle - 0.2);
  if (m_opController.GetRawButtonPressed(6))
    wristSetAngle = (wristSetAngle + 0.2);
  m_arm.SetWristServo(wristSetAngle);
  }
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
    const auto xSpeed = -m_xspeedLimiter.Calculate((1.66667 * frc::ApplyDeadband(m_driveController.GetRawAxis(2), 0.12))) * Drivetrain::kMaxSpeed;    

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    const auto ySpeed = -m_yspeedLimiter.Calculate((1.66667 * frc::ApplyDeadband(m_driveController.GetRawAxis(4), 0.07))) * Drivetrain::kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW introllers return posits positive in
    // mathematics). Xbox coive values when you pull to
    // the right by default.
    const auto rot = -m_rotLimiter.Calculate((1.25 * frc::ApplyDeadband(m_driveController.GetRawAxis(0), 0.07))) * Drivetrain::kMaxAngularSpeed;

    m_swerve.Drive(xSpeed, ySpeed, rot, fieldRelative);
  }

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
