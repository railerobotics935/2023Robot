
// Includes all of the funcitons of the arm, writst and turret


#pragma once

#include <iostream>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <rev/CANSparkMax.h>
#include <frc/DigitalInput.h>
#include <frc/AnalogPotentiometer.h>
#include <frc/Servo.h>
#include "ctre/phoenix.h"

class ArmFunctions
{
public:
  ArmFunctions();
  void UpdateNTE();
  void SetLowerArmMotor(double percent);
  void SetPushrodArmMotor(double precent);
  void SetTurretMotor(double percent);
  void SetIntakeMotor(double precent);
  double GetPushRodArmEncoder();
  double GetPushRodArmAngle();
  double GetLowerArmAngle();
  double GetTurretAngle();

private:

  double turretEncoderOffset = -0.972;
  double lowerArmEncoderOffset = 2.00 + (0.5 * std::numbers::pi); // measured at pi/2
  double pushrodArmEcoderOffset = 1.694 + (0.5 * std::numbers::pi); // measured at pi/2

  // Arm Sensors
  frc::AnalogPotentiometer turretEncoder{4, (2.0 * std::numbers::pi), (-(std::numbers::pi + turretEncoderOffset))};
  frc::AnalogPotentiometer lowerArmEncoder{5, (2.0 * std::numbers::pi), (-(lowerArmEncoderOffset))};
  frc::AnalogPotentiometer pushrodArmEncoder{6, (2.0 * std::numbers::pi), (-(pushrodArmEcoderOffset))};
  frc::Servo wristServo{0};
  frc::DigitalInput intakeSwitch{0};

  // Motor Controller
  rev::CANSparkMax lowerArmMotor{13, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax pushrodArmMotor{17, rev::CANSparkMax::MotorType::kBrushless};
  VictorSPX turretMotor{14};
  VictorSPX intakeMotor{15};


  // Network Tables
  nt::NetworkTableEntry nte_turretAngle;
  nt::NetworkTableEntry nte_lowerArmAngle;
  nt::NetworkTableEntry nte_pushRodArmEncoder;
  nt::NetworkTableEntry nte_pushRodArmAngle;

};