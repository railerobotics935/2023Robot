
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
#include <numbers>
#include "ctre/phoenix.h"

class ArmFunctions
{
public:
  ArmFunctions();
  void UpdateNTE();

  void SetLowerArmMotor(double percent);
  void SetPushRodArmMotor(double percent);
  void SetTurretMotor(double percent);
  void SetIntakeMotor(double percent);
  void SetWristServo(double angle);
  void SetWristAngle(double angle);
  void SetTurretAngle(double angle);
  void SetLowerArmAngle(double angle);
  void SetPushRodArmAdjustedAngle(double angle);
  void SetPushRodArmRawAngle(double angle);

  void SetArmToHome();
  void SetArmForMidCone();
  void SetArmForHighCone();
  void SetArmForFloorCubeIntake();
  void SetArmForConeDrop();

  double GetPushRodArmEncoder();
  double GetWristServoSensor();
  double GetPushRodArmAngle();
  double GetWristServoAngle();
  double GetLowerArmAngle();
  double GetTurretAngle();

private:
  // Lengths of the parts of the arm in meters
  double lengthOfLowerArm = 0.722;
  double lengthOfPushRodArm = 0.787;
  double lengthOfFullWrist = 0.02815;

  // 3D translation of the center of the turret, at the height of the lower arm pivot point.
  double xTranslationOfArm = -0.089;
  double yTranslationOfArm = -0.325;
  double zTranslationOfArm = +0.287;

  // measured offsets DON'T TOUCH
  double turretEncoderOffset = -0.972;
  double lowerArmEncoderOffset = 2.00 + (0.5 * std::numbers::pi); // measured at pi/2
  double pushRodArmEcoderOffset = 3.428 + (0.5 * std::numbers::pi); // measured at pi/2
  double wristServoOffset = 0.46;

  // Arm Limits. the push rod arm is using the raw angle
  double lowerArmLimit = 1.7;
  double pushRodArmLimit = 0.85;
  double turretLimit = (2 * std::numbers::pi / 180.0);
  double wristServoRange = (std::numbers::pi * 3) / 2;
  double idleLowerArmThreshold = 0.19;
  double idlePushRodArmThreshold = 3.0415;
  bool lowerArmIdleMode = true;
  bool pushRodArmIdleMode = true;

  // Arm Sensors
  frc::AnalogPotentiometer turretEncoder{4, (2.0 * std::numbers::pi), (-(std::numbers::pi + turretEncoderOffset))};
  frc::AnalogPotentiometer lowerArmEncoder{5, (2.0 * std::numbers::pi), (-(lowerArmEncoderOffset))};
  frc::AnalogPotentiometer pushRodArmEncoder{6, (2.0 * std::numbers::pi), (-(pushRodArmEcoderOffset))};
  frc::Servo wristServo{0};

  // Motor Controller
  rev::CANSparkMax lowerArmMotor{13, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax pushRodArmMotor{17, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax turretMotor{14, rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax intakeMotor{5, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax intakeFollower{6, rev::CANSparkMax::MotorType::kBrushless};
  
  // PID controllers for the arm
  frc2::PIDController turretPID{15.0, 0.0, 0.0};
  frc2::PIDController lowerArmPID{25.0, 10.0, 1.0};
  frc2::PIDController pushRodArmPID{35.0, 15.0, 1.5};

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