
#include "ArmFunctions.h"

ArmFunctions::ArmFunctions()
{
  // Initialize shuffleboard communication
  auto nt_inst = nt::NetworkTableInstance::GetDefault();
  auto nt_table = nt_inst.GetTable("datatable");
  nte_turretAngle = nt_table->GetEntry("Arm/Turret Angle");
  nte_lowerArmAngle = nt_table->GetEntry("Arm/Lower Arm Angle");
  nte_pushRodArmEncoder = nt_table->GetEntry("Debug/Push Rod Arm Encoder Angle");
  nte_pushRodArmAngle = nt_table->GetEntry("Arm/Push Rod Arm Angle");
  nte_wristServoAngle = nt_table->GetEntry("Arm/Wrist Angle");

  lowerArmMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  pushRodArmMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  intakeMotor.SetNeutralMode(NeutralMode::Brake);
}

// Updates Netowrk table entries, needs to be called every cycle for acurrate values
void ArmFunctions::UpdateNTE() 
{
  // Update Network Table Values
  nte_turretAngle.SetDouble(GetTurretAngle());
  nte_lowerArmAngle.SetDouble(GetLowerArmAngle());
  nte_pushRodArmEncoder.SetDouble(GetPushRodArmEncoder());
  nte_pushRodArmAngle.SetDouble(GetPushRodArmAngle());
  nte_wristServoAngle.SetDouble(GetWristServoAngle());
}

// Angle values in radians. All Angles are in standard positon. Looking at the left side of the robot, ccw is increasing, horizontal, inital side from left to right

// Returns the angle, in radians, of the wrist reletive to the robot
double ArmFunctions::GetLowerArmAngle()
{
  // invert it for ccw 
  return -lowerArmEncoder.Get();
}

// Returns the angle, in radians, of the push rod arm, reletive to the lower arm
double ArmFunctions::GetPushRodArmEncoder()
{
  // invert it for ccw 
  return -pushRodArmEncoder.Get();
}

// Returns the setpoint, in radians, of the wrist servo, reletive to the push rod arm
double ArmFunctions::GetWristServoSensor()
{
  return (wristServo.Get() * 2 * std::numbers::pi + wristServoOffset);
}

// Returns the angle, in radians, of the wrist reletive to the robot
double ArmFunctions::GetWristServoAngle()
{
  return (GetPushRodArmAngle() + GetWristServoSensor());
}

// Returns the angle, in radians, of the push rod arm reletive to the robot
double ArmFunctions::GetPushRodArmAngle()
{
  return (GetPushRodArmEncoder() + GetLowerArmAngle()); 
}

// Returns the angle, in radians, of the turret
double ArmFunctions::GetTurretAngle()
{
  return turretEncoder.Get();
}

// Postive values rotates ccw
void ArmFunctions::SetTurretMotor(double percent)
{
  turretMotor.Set(motorcontrol::ControlMode::PercentOutput, percent);
}

// Positive values rotates the arm ccw
void ArmFunctions::SetLowerArmMotor(double precent)
{
  lowerArmMotor.Set(precent);
  SafteyArmStop();
}

// Positive values rotates the arm cw
void ArmFunctions::SetPushRodArmMotor(double precent)
{
  pushRodArmMotor.Set(precent);
  SafteyArmStop();
}

// Sets power to motor
void ArmFunctions::SetIntakeMotor(double precent)
{
    intakeMotor.Set(motorcontrol::ControlMode::PercentOutput, precent);
}

// Sets wrist angle reletive to the push rod arm
void ArmFunctions::SetWristServo(double angle)
{
  wristServo.Set((angle + wristServoOffset)/(2*std::numbers::pi));
}

// Basic Saftey stop. If it goes further than specified limit, it sets power the other direction.
void ArmFunctions::SafteyArmStop()
{
  if (GetLowerArmAngle() > lowerArmMax)
    SetLowerArmMotor(0.3);
//  if (GetLowerArmAngle() < lowerArmMin)
//    SetLowerArmMotor(-0.3);

  // Remeber because the pushrod arm rotates the oposate direction, it looks inverted, but the angle reference is the same
//  if (GetPushRodArmEncoder() > pushRodArmMax)
//    SetPushRodArmMotor(0.3);
  if (GetPushRodArmEncoder() < pushRodArmMin)
    SetPushRodArmMotor(-0.3);

  
  if (GetTurretAngle() < -turretLimit)
    SetTurretMotor(-0.5);
  if (GetTurretAngle() > turretLimit)
    SetTurretMotor(0.5);
  
  
}