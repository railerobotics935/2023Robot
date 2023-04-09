

#include "Commands/ArmCommands.h"

#include <frc2/command/Commands.h>

// Returns the command from arm functions for Auto
Arm::SetArmToHome::SetArmToHome(ArmFunctions* arm)
    : m_arm{arm} {
  AddRequirements(m_arm);

  m_arm->SetArmToHome();
}

Arm::SetArmForCubeIntake::SetArmForCubeIntake(ArmFunctions* arm)
    : m_arm{arm} {
  AddRequirements(m_arm);

  m_arm->SetArmForCubeIntake();   
}

Arm::SetArmForMid::SetArmForMid(ArmFunctions* arm)
    : m_arm{arm} {
  AddRequirements(m_arm);

  m_arm->SetArmForMid();   
}

Arm::SetArmForHigh::SetArmForHigh(ArmFunctions* arm)
    : m_arm{arm} {
  AddRequirements(m_arm);

  m_arm->SetArmForHigh();   
}

Arm::SetIntakeToIntake::SetIntakeToIntake(ArmFunctions* arm)
    : m_arm{arm} {
  AddRequirements(m_arm);

  m_arm->SetIntakeMotor(0.15);   
}

Arm::SetIntakeToOuttake::SetIntakeToOuttake(ArmFunctions* arm)
    : m_arm{arm} {
  AddRequirements(m_arm);

  m_arm->SetIntakeMotor(-0.15);   
}

Arm::SetIntakeOff::SetIntakeOff(ArmFunctions* arm)
    : m_arm{arm} {
  AddRequirements(m_arm);

  m_arm->SetIntakeMotor(0.0);   
}