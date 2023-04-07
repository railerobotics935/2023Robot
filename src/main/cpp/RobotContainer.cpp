#include "RobotContainer.h"

#include <utility>

#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <units/angle.h>
#include <units/velocity.h>

#include <pathplanner/lib/PathPlannerTrajectory.h>
#include <pathplanner/lib/PathPlanner.h>
#include <pathplanner/lib/commands/PPSwerveControllerCommand.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/RunCommand.h>

using namespace pathplanner;

RobotContainer::RobotContainer() {
  m_chooser.AddOption("Test Drive Forward", "TestDriveForward");
  m_chooser.AddOption("Test Balance", "TestBalance");
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  std::string autoPathName = m_chooser.GetSelected();
  // This will load the file "Example Path.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
  PathPlannerTrajectory examplePath = PathPlanner::loadPath(autoPathName, PathConstraints((units::meters_per_second_t)4,(units::meters_per_second_squared_t)3));

  frc2::CommandPtr swerveCommand{PPSwerveControllerCommand(
    examplePath,
    [this] () -> frc::Pose2d { return m_drivetrain.GetPose(); },
    m_drivetrain.m_kinematics,
    frc2::PIDController{XController},
    frc2::PIDController{YController},
    frc2::PIDController{RotController},
    [this] (std::array<frc::SwerveModuleState, 4U> states) { m_drivetrain.SetModuleStates(states); },
    {&m_drivetrain},
    true
  )};

  // Reset odometry to the starting pose of the trajectory.
  m_drivetrain.ResetOdometry(examplePath.getInitialPose());

  // no auto
  return std::move(swerveCommand)
          .BeforeStarting([this]() { m_drivetrain.Drive(0_mps, 0_mps, 0_rad_per_s, false); }, {});
}

// Helper funtions to get to the drivetrain.
void RobotContainer::Swerve_Drive(units::meters_per_second_t xSpeed, 
                                  units::meters_per_second_t ySpeed, 
                                  units::radians_per_second_t rotSpeed, 
                                  bool fieldRelative)
{
  m_drivetrain.Drive(xSpeed, ySpeed, rotSpeed, fieldRelative);
}

void RobotContainer::Swerve_UpdateNTE()
{
  m_drivetrain.UpdateNTE();
}

void RobotContainer::Swerve_Park()
{
  m_drivetrain.Park();
}

void RobotContainer::Swerve_ZeroHeading() 
{
  m_drivetrain.ZeroHeading();
}