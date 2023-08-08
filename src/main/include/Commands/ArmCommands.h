
#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Drivetrain/Drivetrain.h"
#include "ArmFunctions.h"

//#define ARM_COMMANDS

#ifdef ARM_COMMANDS
namespace ArmCommands {
/**
 * Example static factory for an autonomous command.
 */

} 
namespace Arm {
class SetArmToHome
    : public frc2::CommandHelper<frc2::CommandBase, SetArmToHome> {
  public:
  /**
   * Creates a new TestCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  explicit SetArmToHome(ArmFunctions* arm);

  private:
  ArmFunctions* m_arm;
};


class SetArmForCubeIntake
    : public frc2::CommandHelper<frc2::CommandBase, SetArmForCubeIntake> {
  public:
  /**
   * Creates a new TestCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  explicit SetArmForCubeIntake(ArmFunctions* arm);

  private:
  ArmFunctions* m_arm;
};


class SetArmForMid
    : public frc2::CommandHelper<frc2::CommandBase, SetArmForMid> {
  public:
  /**
   * Creates a new TestCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  explicit SetArmForMid(ArmFunctions* arm);

  private:
  ArmFunctions* m_arm;
};


class SetArmForHigh
    : public frc2::CommandHelper<frc2::CommandBase, SetArmForHigh> {
  public:
  /**
   * Creates a new TestCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  explicit SetArmForHigh(ArmFunctions* arm);

  private:
  ArmFunctions* m_arm;
};

class SetIntakeToIntake
    : public frc2::CommandHelper<frc2::CommandBase, SetIntakeToIntake> {
  public:
  /**
   * Creates a new TestCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  explicit SetIntakeToIntake(ArmFunctions* arm);

  private:
  ArmFunctions* m_arm;
};

class SetIntakeToOuttake
    : public frc2::CommandHelper<frc2::CommandBase, SetIntakeToOuttake> {
  public:
  /**
   * Creates a new TestCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  explicit SetIntakeToOuttake(ArmFunctions* arm);

  private:
  ArmFunctions* m_arm;
};

class SetIntakeOff
    : public frc2::CommandHelper<frc2::CommandBase, SetIntakeOff> {
  public:
  /**
   * Creates a new TestCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  explicit SetIntakeOff(ArmFunctions* arm);

  private:
  ArmFunctions* m_arm;
};
}
#endif
