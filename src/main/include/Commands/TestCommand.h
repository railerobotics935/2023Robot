

#pragma once


#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "DriveTrain/Drivetrain.h"

/**
 * An example command that uses an example subsystem.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class TestCommand
    : public frc2::CommandHelper<frc2::CommandBase, TestCommand> {
 public:
  /**
   * Creates a new TestCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  explicit TestCommand(Drivetrain* drivetrain);

 private:
  Drivetrain* m_drivetrain;
};


