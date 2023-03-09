
#include "ArmFunctions.h"

ArmFunctions::ArmFunctions()
{
  // Initialize shuffleboard communication
  auto nt_inst = nt::NetworkTableInstance::GetDefault();
  auto nt_table = nt_inst.GetTable("datatable");
  nte_turretAngle = nt_table->GetEntry("Arm/Turret Angle");
  nte_lowerArmAngle = nt_table->GetEntry("Arm/Lower Arm Angle");
  nte_pushRodArmEncoder = nt_table->GetEntry("Debug/Pushrod Arm Encoder Angle");
  nte_pushRodArmAngle = nt_table->GetEntry("Arm/Pushrod Arm Angle");
  nte_wristServoAngle = nt_table->GetEntry("Arm/Wrist Angle");

  lowerArmMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  pushrodArmMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  intakeMotor.SetNeutralMode(NeutralMode::Brake);
}

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
// Returns the angle, in radians, of the pushrod arm, reletive to the lower arm
double ArmFunctions::GetPushRodArmEncoder()
{
  // invert it for ccw 
  return -pushrodArmEncoder.Get();
}
// Returns the setpoint, in radians, of the wrist servo, reletive to the pushrod arm
double ArmFunctions::GetWristServoSensor()
{
  return (wristServo.Get() * 2 * std::numbers::pi + wristServoOffset);
}
// Returns the angle, in radians, of the wrist reletive to the robot
double ArmFunctions::GetWristServoAngle()
{
  return (GetPushRodArmAngle() + GetWristServoSensor());
}
// Returns the angle, in radians, of the pushrod arm reletive to the robot
double ArmFunctions::GetPushRodArmAngle()
{
  return (GetPushRodArmEncoder() + GetLowerArmAngle()); 
}
// Returns the angle, in radians, of the turret
double ArmFunctions::GetTurretAngle()
{
  return turretEncoder.Get();
}

// Set basic motor power
void ArmFunctions::SetTurretMotor(double percent)
{
  turretMotor.Set(motorcontrol::ControlMode::PercentOutput, percent);
}
// Set basic motor power
void ArmFunctions::SetLowerArmMotor(double precent)
{
  lowerArmMotor.Set(precent);
}
// Set basic motor power
void ArmFunctions::SetPushrodArmMotor(double precent)
{
  pushrodArmMotor.Set(precent);
}
// Sets power to motor
void ArmFunctions::SetIntakeMotor(double precent)
{
  if(!intakeSwitch.Get())
    intakeMotor.Set(motorcontrol::ControlMode::PercentOutput, precent);
  else
    intakeMotor.Set(motorcontrol::ControlMode::PercentOutput, 0.0);
}
// Sets wrist angle reletive to the pushrod arm
void ArmFunctions::SetWristServo(double angle)
{
  wristServo.Set((angle/(2*std::numbers::pi)));
}