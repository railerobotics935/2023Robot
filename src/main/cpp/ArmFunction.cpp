
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

  lowerArmMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  pushRodArmMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  intakeMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  intakeFollower.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  intakeFollower.Follow(intakeMotor, true);

  //intakeMotor.SetSmartCurrentLimit();

}

// Updates Netowrk table entries, needs to be called every cycle for acurrate values
void ArmFunctions::UpdateNTE() 
{
  // Update Network Table Values
  nte_turretAngle.SetDouble(GetTurretAngle());
  nte_lowerArmAngle.SetDouble(GetLowerArmAngle());
  nte_pushRodArmEncoder.SetDouble(GetPushRodArmEncoder());
  nte_pushRodArmAngle.SetDouble(GetPushRodArmAngle());
  nte_wristServoAngle.SetDouble(GetWristServoSensor());
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
  return (wristServo.Get() - wristServoOffset);
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
      turretMotor.Set(percent);
    else
      SetTurretAngle(turretLimit);
  }
  else if (GetTurretAngle() < -turretLimit)
  {
    if (percent < 0.0)
      turretMotor.Set(percent);
    else
      SetTurretAngle(-turretLimit);
  }
  else
    turretMotor.Set(percent);
}

// Positive values rotates the arm ccw
void ArmFunctions::SetLowerArmMotor(double percent)
{
  lowerArmMotor.Set(percent);
}

// Positive values rotates the arm cw
void ArmFunctions::SetPushRodArmMotor(double percent)
{
  pushRodArmMotor.Set(percent);
}

// Sets power to motor
void ArmFunctions::SetIntakeMotor(double percent) 
{
  intakeMotor.Set(percent);
}

// Sets wrist angle reletive to the push rod arm
void ArmFunctions::SetWristServo(double angle)
{
  wristServo.Set(wristServoOffset + (angle/wristServoRange));
}

// Sets wrist angle reletive to the robt
void ArmFunctions::SetWristAngle(double angle)
{
  //wristServo.Set(wristServoOffset + (angle/(std::numbers::pi*3)) + GetPushRodArmAngle());
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
  std::cout << "Setting Turret Output\r\n";
  turretOutput = turretPID.Calculate(GetTurretAngle());
  turretMotor.Set(turretOutput);
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
    if (GetLowerArmAngle() < idleLowerArmThreshold + 0.05)
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
  else
  {
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
    if ((-angle + std::numbers::pi) > (GetLowerArmAngle() + (std::numbers::pi/6)))
    {
      pushRodArmPID.SetSetpoint(-(GetLowerArmAngle() - (std::numbers::pi * 5/6)));
    }
    else if (angle > pushRodArmLimit)
      pushRodArmPID.SetSetpoint(angle);
    else
    {
      pushRodArmPID.SetSetpoint(pushRodArmLimit);
    }
  }
  else if (angle > idlePushRodArmThreshold)
  {
    if (GetPushRodArmEncoder() > idlePushRodArmThreshold - 0.05)
      pushRodArmIdleMode = true;
    else
    {
      pushRodArmPID.SetSetpoint(idlePushRodArmThreshold);
      pushRodArmIdleMode = false;
    }
  }
  else
  {
    if ((-angle + std::numbers::pi) > (GetLowerArmAngle() + (std::numbers::pi/6)))
    {
      pushRodArmPID.SetSetpoint(-(GetLowerArmAngle() - (std::numbers::pi * 5/6)));
    }
    else
      pushRodArmPID.SetSetpoint(angle);
      
    pushRodArmIdleMode = false;
  }

  // If the idle mode is active, set the power to zero
  if (pushRodArmIdleMode)
    pushRodArmMotor.SetVoltage(units::volt_t{0.0});
  else
  {
    pushRodArmOutput = -pushRodArmPID.Calculate(GetPushRodArmEncoder());
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

// All are preset positions for the arm
void ArmFunctions::SetArmToHome()
{
  SetLowerArmAngle(0.0);
  SetPushRodArmRawAngle(0.0);
  SetWristServo(0.75);
}

void ArmFunctions::SetArmForMidCone()
{
  SetLowerArmAngle(1.336);
  SetPushRodArmRawAngle(1.877);
  SetWristServo(0.0); // Horizontal
}

void ArmFunctions::SetArmForHighCone()
{
  // who knows...
  SetLowerArmAngle(1.8); // max
  SetPushRodArmRawAngle(1.1); // max
  SetWristServo(0.0); // horizontal to upish
}

void ArmFunctions::SetArmForFloorCubeIntake()
{
  SetLowerArmAngle(0.811);
  SetPushRodArmRawAngle(3.013);
  SetWristServo(0.3); // NOT MEASURED a little lower than horizontal
}

void ArmFunctions::SetArmForConeDrop()
{
  SetLowerArmAngle(0.0);
  SetPushRodArmRawAngle(0.0);
  SetWristServo(0.0);
}