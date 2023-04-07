
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>

#include "Drivetrain/Drivetrain.h"

class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();
  void Swerve_Drive(units::meters_per_second_t xSpeed, 
                    units::meters_per_second_t ySpeed, 
                    units::radians_per_second_t rotSpeed, 
                    bool fieldRelative);
  void Swerve_UpdateNTE();
  void Swerve_Park();
  void Swerve_SetModuleStates();
  void Swerve_ZeroHeading();
  private:
  // The robot's subsystems
  Drivetrain m_drivetrain;

  // The chooser for the autonomous routines
  frc::SendableChooser<std::string> m_chooser;
  
  void ConfigureButtonBindings();

  frc2::PIDController XController{1.0, 0.0, 0.0};
  frc2::PIDController YController{1.0, 0.0, 0.0};
  frc2::PIDController RotController{1.0, 0.0, 0.0};
  
};


