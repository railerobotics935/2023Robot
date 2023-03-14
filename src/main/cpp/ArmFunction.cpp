
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
  if (GetTurretAngle() > turretLimit)
  {
    if (percent > 0.0)
      turretMotor.Set(motorcontrol::ControlMode::PercentOutput, percent);
    else
      SetTurretAngle(turretLimit);
  }
  else if (GetTurretAngle() < -turretLimit)
  {
    if (percent < 0.0)
      turretMotor.Set(motorcontrol::ControlMode::PercentOutput, percent);
    else
      SetTurretAngle(-turretLimit);
  }
  else
    turretMotor.Set(motorcontrol::ControlMode::PercentOutput, percent);
}

// Positive values rotates the arm ccw
void ArmFunctions::SetLowerArmMotor(double precent)
{
  lowerArmMotor.Set(precent);
}

// Positive values rotates the arm cw
void ArmFunctions::SetPushRodArmMotor(double precent)
{
  pushRodArmMotor.Set(precent);
}

// Sets power to motor
void ArmFunctions::SetIntakeMotor(double precent)
{
  intakeMotor.Set(motorcontrol::ControlMode::PercentOutput, precent);
}

// Sets wrist angle reletive to the push rod arm
void ArmFunctions::SetWristServo(double angle)
{
  wristServo.Set(((angle + wristServoOffset) * 180)/(std::numbers::pi));
}

// Sets turret to a specified angle ONLY input inbetween +- 0.1919
void ArmFunctions::SetTurretAngle(double angle)
{
  if (GetTurretAngle() > turretLimit)
  {
    if (angle < turretLimit)
      turretPID.SetSetpoint(angle);
    else
      turretPID.SetSetpoint(turretLimit);
  }
  else if (GetTurretAngle() < -turretLimit)
  {
    if (angle > -turretLimit)
      turretPID.SetSetpoint(angle);
    else
      turretPID.SetSetpoint(-turretLimit);
  }
  else
    turretPID.SetSetpoint(angle);
  
  turretOutput = turretPID.Calculate(GetTurretAngle());
  turretMotor.Set(ControlMode::Current, turretOutput);
}

// Sets lower arm position
void ArmFunctions::SetLowerArmAngle(double angle)
{
  // Double check the angle is ok
  if (GetLowerArmAngle() > lowerArmLimit)
  {
    if (angle < lowerArmLimit)
      lowerArmPID.SetSetpoint(angle);
    else
      lowerArmPID.SetSetpoint(lowerArmLimit);
  }
  else if (angle < idleLowerArmThreshold)
  {
    if (GetLowerArmAngle() < idleLowerArmThreshold + 0.2)
      lowerArmIdleMode = true;
    else
    {
      lowerArmPID.SetSetpoint(idleLowerArmThreshold);
      lowerArmIdleMode = false;
    }
  }
  else
  {
    lowerArmPID.SetSetpoint(angle);
    lowerArmIdleMode = false;
  }

  // If the idle mode is active, set the power to zero
  if (lowerArmIdleMode)
    lowerArmMotor.SetVoltage(units::volt_t{0.0});
  {
    lowerArmPID.SetSetpoint(angle);
    lowerArmOutput = lowerArmPID.Calculate(GetLowerArmAngle());
    lowerArmMotor.SetVoltage(units::volt_t{lowerArmOutput});
  }
}

// Sets push rod arm position
void ArmFunctions::SetPushRodArmRawAngle(double angle)
{
  // Double check the angle is ok
  if (GetPushRodArmEncoder() < pushRodArmLimit)
  {
    if (angle > pushRodArmLimit)
      pushRodArmPID.SetSetpoint(angle);
    else
      pushRodArmPID.SetSetpoint(pushRodArmLimit);
  }
  else if (angle > idlePushRodArmThreshold)
  {
    if (GetPushRodArmEncoder() > idlePushRodArmThreshold - 0.2)
      pushRodArmIdleMode = true;
    else
    {
      pushRodArmPID.SetSetpoint(idlePushRodArmThreshold);
      pushRodArmIdleMode = false;
    }
  }
  else
  {
    pushRodArmPID.SetSetpoint(angle);
    pushRodArmIdleMode = false;
  }

  // If the idle mode is active, set the power to zero
  if (pushRodArmIdleMode)
    pushRodArmMotor.SetVoltage(units::volt_t{0.0});
  {
    pushRodArmPID.SetSetpoint(angle);
    pushRodArmOutput = pushRodArmPID.Calculate(GetLowerArmAngle());
    pushRodArmMotor.SetVoltage(units::volt_t{pushRodArmOutput});
  }
}

/*
// Sets push rod arm position
void ArmFunctions::SetPushRodArmAdjustedAngle(double angle)
{
  pushRodArmPID.SetSetpoint(angle);
  pushRodArmOutput = pushRodArmPID.Calculate(GetPushRodArmAngle());
  pushRodArmMotor.SetVoltage(units::volt_t{pushRodArmOutput});
}
*/