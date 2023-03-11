
// Includes all of the funcitons of the arm, writst and turret


#pragma once

#include <iostream>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <rev/CANSparkMax.h>
#include <frc/DigitalInput.h>
#include <frc/AnalogPotentiometer.h>
#include <frc/Servo.h>
#include <frc/controller/PIDController.h>
#include "ctre/phoenix.h"

class ArmFunctions
{
public:
  ArmFunctions();
  void UpdateNTE();
  void SetLowerArmMotor(double percent);
  void SetPushRodArmMotor(double precent);
  void SetTurretMotor(double percent);
  void SetIntakeMotor(double precent);
  void SetWristServo(double angle);
  void SetTurretAngle(double angle);
  void SetLowerArmAngle(double angle);
  void SetPushRodArmAngle(double angle);
  double GetPushRodArmEncoder();
  double GetWristServoSensor();
  double GetPushRodArmAngle();
  double GetWristServoAngle();
  double GetLowerArmAngle();
  double GetTurretAngle();
  void SafteyArmStop();

private:
  // Lengths of the parts of the arm in meters
  double lengthOfLowerArm = 0.722;
  double lengthOfPushRodArm = 0.787;
  double lengthOfFullWrist = 28.15;

  // 3D translation of the center of the turret, at the height of the lower arm pivot point.
  double xTranslationOfArm = -0.089;
  double yTranslationOfArm = -0.325;
  double zTranslationOfArm = +0.287;

  double turretEncoderOffset = -0.972;
  double lowerArmEncoderOffset = 2.00 + (0.5 * std::numbers::pi); // measured at pi/2
  double pushRodArmEcoderOffset = 1.694 + (0.5 * std::numbers::pi); // measured at pi/2
  double wristServoOffset = 3.0;

  // Arm Limits
  double lowerArmMin = 0.13;
  double lowerArmMax = (1.5 * std::numbers::pi/3);
  double pushRodArmMin = (std::numbers::pi/2);
  double pushRodArmMax = 3.13;
  double turretLimit = (11 * std::numbers::pi / 180.0);

  // Arm Sensors
  frc::AnalogPotentiometer turretEncoder{4, (2.0 * std::numbers::pi), (-(std::numbers::pi + turretEncoderOffset))};
  frc::AnalogPotentiometer lowerArmEncoder{5, (2.0 * std::numbers::pi), (-(lowerArmEncoderOffset))};
  frc::AnalogPotentiometer pushRodArmEncoder{6, (2.0 * std::numbers::pi), (-(pushRodArmEcoderOffset))};
  frc::Servo wristServo{0};
  frc::DigitalInput intakeSwitch{0};

  // Motor Controller
  rev::CANSparkMax lowerArmMotor{13, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax pushRodArmMotor{17, rev::CANSparkMax::MotorType::kBrushless};
  VictorSPX turretMotor{14};
  VictorSPX intakeMotor{15};

  // PID controllers for the arm
  frc2::PIDController turretPID{1.0, 0.0, 0.0};
  frc2::PIDController lowerArmPID{1.0, 0.0, 0.0};
  frc2::PIDController pushRodArmPID{1.0, 0.0, 0.0};

  double turretOutput = 0.0;
  double lowerArmOutput = 0.0;
  double pushRodArmOutput = 0.0;

  // Network Tables
  nt::NetworkTableEntry nte_turretAngle;
  nt::NetworkTableEntry nte_lowerArmAngle;
  nt::NetworkTableEntry nte_pushRodArmEncoder;
  nt::NetworkTableEntry nte_pushRodArmAngle;
  nt::NetworkTableEntry nte_wristServoAngle;

};