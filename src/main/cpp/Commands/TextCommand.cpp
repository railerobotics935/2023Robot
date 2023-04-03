


#include "Commands/TestCommand.h"

TestCommand::TestCommand(Drivetrain*drivetrain)
    : m_drivetrain{drivetrain} {
  // Register that this command requires the subsystem.
  AddRequirements(m_drivetrain);

  printf("TestCommand has ran \r\n");
}

