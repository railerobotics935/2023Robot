

#include "Commands/Autos.h"

#include <frc2/command/Commands.h>

#include "Commands/TestCommand.h"

frc2::CommandPtr autos::TestAuto(Drivetrain* drivetrain) {
  return frc2::cmd::Sequence(drivetrain->TestMethodCommand(),
                             TestCommand(drivetrain).ToPtr());
}
