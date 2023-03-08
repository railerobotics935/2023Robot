
#include "ArmFunctions.h"

ArmFunctions::ArmFunctions()
{
  // Initialize shuffleboard communication
  auto nt_inst = nt::NetworkTableInstance::GetDefault();
  auto nt_table = nt_inst.GetTable("datatable");
  nte_turretAngle = nt_table->GetEntry("Arm/Turret Angle");
  nte_lowerArmAngle = nt_table->GetEntry("Arm/Lower Arm Angle");
  nte_pushRodArmEncoder = nt_table->GetEntry("Arm/Pushrod Arm Encoder Angle");
  nte_pushRodArmAngle = nt_table->GetEntry("Arm/Pushrod Arm Angle");
}

void ArmFunctions::UpdateNTE() 
{
  // Update Network Table Values
  nte_turretAngle.SetDouble(GetTurretAngle());
  nte_lowerArmAngle.SetDouble(GetLowerArmAngle());
  nte_pushRodArmEncoder.SetDouble(GetPushRodArmEncoder());
  nte_pushRodArmAngle.SetDouble(GetPushRodArmAngle());
}

// Angle values in radians. All Angles are in standard positon. Looking at the left side of the robot, ccw is increasing, horizontal, inital side from left to right
double ArmFunctions::GetLowerArmAngle()
{
  // invert it for ccw 
  return -lowerArmEncoder.Get();
}
double ArmFunctions::GetPushRodArmEncoder()
{
  // invert it for ccw 
  return -pushrodArmEncoder.Get();
}
double ArmFunctions::GetPushRodArmAngle()
{
  return (GetPushRodArmEncoder() + GetLowerArmAngle()); 
}
double ArmFunctions::GetTurretAngle()
{
  return turretEncoder.Get();
}

// Set basic motor power
void ArmFunctions::SetTurretMotor(double percent)
{
  turretMotor.Set(motorcontrol::ControlMode::PercentOutput, percent);
}

void ArmFunctions::SetLowerArmMotor(double precent)
{
  lowerArmMotor.Set(precent);
}

void ArmFunctions::SetPushrodArmMotor(double precent)
{
  pushrodArmMotor.Set(precent);
}
